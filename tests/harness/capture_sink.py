# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# The SD card, RTC and session-folder lifecycle shared by the capture sinks
# (bidir_capture_sink.py today; other sinks would add only what differs: their
# record format, what they write as provenance, and their meta fields).
#
# PicoBell Adalogger for Pico pinout (learn.adafruit.com/
# adafruit-picowbell-adalogger-for-pico/pinouts): SD card on SPI0
# (MISO=GPIO16, CS=GPIO17, SCK=GPIO18, MOSI=GPIO19), PCF8523 RTC on I2C
# (SDA=GPIO4, SCL=GPIO5, address 0x68). All four DShot channels in
# tests/harness/run_scenario.py sit on a contiguous GPIO6-9 block, clear of
# both this board's I2C pins and its SD card's GPIO16-19.
#
# Mirrors the SdSink lifecycle proven on the sister test rig (Flight-Benchy,
# src/telemetry/recorder.py: mount early/fail-fast, open a session directory
# once recording actually starts, unmount on close()).

import os
import struct
import time
from machine import Pin, SPI, I2C

import sdcard
from pcf8523 import PCF8523

SD_SCK = 18
SD_MOSI = 19
SD_MISO = 16
SD_CS = 17
RTC_SDA = 4
RTC_SCL = 5

SD_MOUNT = "/sd"
LOG_DIR = SD_MOUNT + "/dshot_captures"


class CaptureSinkBase:
    # struct format of one capture.bin record; each sink sets its own
    RECORD_FMT = None

    def __init__(self):
        """Mount the SD card and validate it is accessible.

        Raises OSError immediately if the card is missing or unreadable,
        giving the operator a clear signal before motors are armed.
        """
        cs_pin = Pin(SD_CS, Pin.OUT, value=1)
        spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
                  sck=Pin(SD_SCK), mosi=Pin(SD_MOSI), miso=Pin(SD_MISO))
        time.sleep_ms(250)
        self.sd = sdcard.SDCard(spi, cs_pin, baudrate=25_000_000)
        self.vfs = os.VfsFat(self.sd)
        os.mount(self.vfs, SD_MOUNT)

        self.i2c = I2C(0, sda=Pin(RTC_SDA), scl=Pin(RTC_SCL))
        self.rtc = PCF8523(self.i2c)

        self.run_dir = None
        self.file = None
        self.pack_buf = bytearray(struct.calcsize(self.RECORD_FMT))
        self.meta = {}

    def create_session_dir(self, meta):
        """Create the timestamped run directory and write its first meta.txt.

        `meta` becomes the session's key=value metadata (see finalize_meta()
        for updating it once the run ends).

        Raises OSError if the RTC hasn't been set yet (see scripts/set_rtc.py) -
        surfaced here rather than silently falling back to a fake timestamp,
        since a wrong session name is worse than a clear failure.
        """
        try:
            os.mkdir(LOG_DIR)
        except OSError:
            pass  # already exists

        dt = self.rtc.datetime()
        self.run_dir = "{}/{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
            LOG_DIR, dt[0], dt[1], dt[2], dt[4], dt[5], dt[6]
        )
        os.mkdir(self.run_dir)

        self.meta = meta
        self.write_meta()

    def open_capture(self):
        """Open capture.bin in the run directory for records."""
        self.file = open(self.run_dir + "/capture.bin", "wb")

    def write_meta(self):
        with open(self.run_dir + "/meta.txt", "w") as f:
            for key, value in self.meta.items():
                f.write("{}={}\n".format(key, value))

    def finalize_meta(self, outcome, fields):
        """Record how the run ended, plus its final counters, in meta.txt.

        Called before close() so a truncated capture.bin from an aborted run is
        unambiguous to the PC-side analyzer rather than merely inferable from
        the record count.
        """
        self.meta["outcome"] = outcome
        for key, value in fields.items():
            self.meta[key] = value
        self.write_meta()

    @property
    def path(self):
        """Return the run directory path (useful for diagnostics)."""
        return self.run_dir

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
        os.umount(SD_MOUNT)
