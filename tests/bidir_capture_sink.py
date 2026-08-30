# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes raw bidir DShot capture records (as produced by
# tests/bidir_capture_runner.py's BidirCaptureRunner.drain()) to a
# timestamped session folder on the PicoBell Adalogger's SD card.
#
# PicoBell Adalogger for Pico pinout (learn.adafruit.com/
# adafruit-picowbell-adalogger-for-pico/pinouts): SD card on SPI0
# (MISO=GPIO16, CS=GPIO17, SCK=GPIO18, MOSI=GPIO19), PCF8523 RTC on I2C
# (SDA=GPIO4, SCL=GPIO5, address 0x68). GPIO4/5 previously carried two
# TX-only DShot channels in test_bidir_rx_capture.py - those moved to
# GPIO6/7 to free the I2C bus for this board.
#
# Mirrors the SdSink lifecycle proven on the sister test rig (Flight-Benchy,
# src/telemetry/recorder.py: mount early/fail-fast, open a session directory
# once recording actually starts, unmount on close()) - but without its
# preallocate/tmp-log dance: this record rate (~680/s x 22 bytes, see
# test_bidir_rx_capture.py's hardware-verified throughput) is low enough
# that plain buffered appends are fine. Revisit if power-loss protection
# mid-run becomes a real requirement.

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

_SD_MOUNT = "/sd"
_LOG_DIR = _SD_MOUNT + "/dshot_captures"

# ticks_us, throttle, word0, word1, word2, word3 - matches the tuple shape
# BidirCaptureRunner.drain() returns, so write_record(*record) works directly.
_RECORD_FMT = "<IH4I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)


class BidirCaptureSink:
    def __init__(self):
        """Mount the SD card and validate it is accessible.

        Raises OSError immediately if the card is missing or unreadable,
        giving the operator a clear signal before motors are armed.
        """
        cs_pin = Pin(SD_CS, Pin.OUT, value=1)
        spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
                  sck=Pin(SD_SCK), mosi=Pin(SD_MOSI), miso=Pin(SD_MISO))
        time.sleep_ms(250)
        self._sd = sdcard.SDCard(spi, cs_pin, baudrate=25_000_000)
        self._vfs = os.VfsFat(self._sd)
        os.mount(self._vfs, _SD_MOUNT)

        self._i2c = I2C(0, sda=Pin(RTC_SDA), scl=Pin(RTC_SCL))
        self._rtc = PCF8523(self._i2c)

        self._run_dir = None
        self._f = None
        self._pack_buf = bytearray(_RECORD_SIZE)

    def init_session(self, dshot_speed, rx_clock_hz):
        """Create a timestamped run directory and open the capture log.

        Raises OSError if the RTC hasn't been set yet (see scripts/set_rtc.py) -
        that's surfaced here rather than silently falling back to a fake
        timestamp, since a wrong session name is worse than a clear failure.
        """
        try:
            os.mkdir(_LOG_DIR)
        except OSError:
            pass  # already exists

        dt = self._rtc.datetime()
        self._run_dir = "{}/{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
            _LOG_DIR, dt[0], dt[1], dt[2], dt[4], dt[5], dt[6]
        )
        os.mkdir(self._run_dir)

        with open(self._run_dir + "/meta.txt", "w") as f:
            f.write("dshot_speed={}\n".format(dshot_speed))
            f.write("rx_clock_hz={}\n".format(rx_clock_hz))
            f.write("record_fmt={}\n".format(_RECORD_FMT))

        self._f = open(self._run_dir + "/capture.bin", "wb")

    @property
    def path(self):
        """Return the run directory path (useful for diagnostics)."""
        return self._run_dir

    def write_record(self, ticks_us, throttle, w0, w1, w2, w3):
        struct.pack_into(_RECORD_FMT, self._pack_buf, 0, ticks_us, throttle, w0, w1, w2, w3)
        self._f.write(self._pack_buf)

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        if self._f:
            self._f.flush()
            self._f.close()
            self._f = None
        os.umount(_SD_MOUNT)
