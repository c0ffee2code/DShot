# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# The SD card and session-folder lifecycle shared by the capture sinks
# (bidir_capture_sink.py today; other sinks would add only what differs: their
# record format, what they write as provenance, and their meta fields).
#
# PicoBell Adalogger for Pico pinout (learn.adafruit.com/
# adafruit-picowbell-adalogger-for-pico/pinouts): SD card on SPI0
# (MISO=GPIO16, CS=GPIO17, SCK=GPIO18, MOSI=GPIO19). The board's PCF8523 RTC is not used: the PC names
# every session (scripts/run_session.py, session_name.txt). All four DShot channels in
# tests/bench/dshot_protocol/run_scenario.py sit on a contiguous GPIO6-9 block, clear of
# the board's I2C pins (GPIO4-5) and its SD card's GPIO16-19.
#
# Mirrors the SdSink lifecycle proven on the sister test rig (Flight-Benchy,
# src/telemetry/recorder.py: mount early/fail-fast, open a session directory
# once recording actually starts, unmount on close()).

import os
import struct
import time
from machine import Pin, SPI

import sdcard

SD_SCK = 18
SD_MOSI = 19
SD_MISO = 16
SD_CS = 17

SD_MOUNT = "/sd"
LOG_DIR = SD_MOUNT + "/dshot_captures"


def session_name_from_pc():
    """The session name scripts/run_test.py / run_session.py chose (uploaded as session_name.txt). Raises
    OSError when there is none: every session is named by the PC, the Pico has no clock."""
    with open("session_name.txt") as f:
        name = f.read().strip()
    if not name:
        raise OSError("session_name.txt is empty")
    return name


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

        self.run_dir = None
        self.file = None
        self.pack_buf = bytearray(struct.calcsize(self.RECORD_FMT))
        self.meta = {}

    def create_session_dir(self, meta):
        """Create the session directory and write its first meta.txt.

        `meta` becomes the session's key=value metadata (see finalize_meta()
        for updating it once the run ends).

        The folder is named by the PC (session_name.txt, see session_name_from_pc()); no name raises OSError
        before anything is armed.
        """
        try:
            os.mkdir(LOG_DIR)
        except OSError:
            pass  # already exists

        self.run_dir = LOG_DIR + "/" + session_name_from_pc()
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
