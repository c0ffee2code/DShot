# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes raw scenario capture records (as produced by
# tests/harness/scenario_runner.py's ScenarioRunner.drain()) to a
# timestamped session folder on the PicoBell Adalogger's SD card.
#
# PicoBell Adalogger for Pico pinout (learn.adafruit.com/
# adafruit-picowbell-adalogger-for-pico/pinouts): SD card on SPI0
# (MISO=GPIO16, CS=GPIO17, SCK=GPIO18, MOSI=GPIO19), PCF8523 RTC on I2C
# (SDA=GPIO4, SCL=GPIO5, address 0x68). All four DShot channels in
# tests/test_scenario_capture.py sit on a contiguous GPIO6-9 block, clear of
# both this board's I2C pins and its SD card's GPIO16-19.
#
# Mirrors the SdSink lifecycle proven on the sister test rig (Flight-Benchy,
# src/telemetry/recorder.py: mount early/fail-fast, open a session directory
# once recording actually starts, unmount on close()) - including copying
# the scenario's own JSON into the session folder for full provenance, the
# same move recorder.py makes with config.json/specification.json.

import os
import struct
import time
from machine import Pin, SPI, I2C

import sdcard
from pcf8523 import PCF8523
from dshot_pio import BIDIR_PROFILES

SD_SCK = 18
SD_MOSI = 19
SD_MISO = 16
SD_CS = 17
RTC_SDA = 4
RTC_SCL = 5

_SD_MOUNT = "/sd"
_LOG_DIR = _SD_MOUNT + "/dshot_captures"

# ticks_us, throttle0..3, then one 4-word GCR capture group per motor
# (motor0_w0..w3, motor1_w0..w3, motor2_w0..w3, motor3_w0..w3) - matches the
# 21-field tuple shape ScenarioRunner.drain() returns, so
# write_record(*record) works directly. A non-bidirectional motor's word
# group is always zero.
_RECORD_FMT = "<I4H16I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)

_COPY_CHUNK_SIZE = 512


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
        self._meta = {}

    def init_session(self, scenario, scenario_path):
        """Create a timestamped run directory and open the capture log.

        Copies `scenario_path`'s contents verbatim into the session folder
        as scenario.json - full provenance for the PC-side analyzer, same
        move as Flight-Benchy's SdSink.init_session() copying config.json.

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

        bidir_indices = ",".join(str(i) for i in scenario.bidir_indices)
        rx_clock_hz = BIDIR_PROFILES.get(scenario.dshot_speed) if scenario.bidir_indices else 0

        self._meta = {
            "dshot_speed": str(scenario.dshot_speed),
            "rx_clock_hz": str(rx_clock_hz),
            "record_fmt": _RECORD_FMT,
            "bidir_motor_indices": bidir_indices,
            "outcome": "running",
        }
        self._write_meta()

        with open(scenario_path, "rb") as src, open(self._run_dir + "/scenario.json", "wb") as dst:
            while True:
                chunk = src.read(_COPY_CHUNK_SIZE)
                if not chunk:
                    break
                dst.write(chunk)

        self._f = open(self._run_dir + "/capture.bin", "wb")

    def _write_meta(self):
        with open(self._run_dir + "/meta.txt", "w") as f:
            for key, value in self._meta.items():
                f.write("{}={}\n".format(key, value))

    def finalize(self, outcome, total_records, dropped, largest_gap_us):
        """Record how the run ended and its final device-side stats in meta.txt.

        Called before close() so a truncated capture.bin from an aborted run
        is unambiguous to the PC-side analyzer (not just inferable from
        record count), and so the analyzer can re-check the scenario's own
        `expect` thresholds against the actual on-device dropped/gap counts
        rather than recomputing an approximation from capture.bin alone.
        """
        self._meta["outcome"] = outcome
        self._meta["total_records"] = str(total_records)
        self._meta["dropped"] = str(dropped)
        self._meta["largest_gap_us"] = str(largest_gap_us)
        self._write_meta()

    @property
    def path(self):
        """Return the run directory path (useful for diagnostics)."""
        return self._run_dir

    def write_record(self, ticks_us, t0, t1, t2, t3,
                      m0w0, m0w1, m0w2, m0w3,
                      m1w0, m1w1, m1w2, m1w3,
                      m2w0, m2w1, m2w2, m2w3,
                      m3w0, m3w1, m3w2, m3w3):
        struct.pack_into(
            _RECORD_FMT, self._pack_buf, 0,
            ticks_us, t0, t1, t2, t3,
            m0w0, m0w1, m0w2, m0w3,
            m1w0, m1w1, m1w2, m1w3,
            m2w0, m2w1, m2w2, m2w3,
            m3w0, m3w1, m3w2, m3w3,
        )
        self._f.write(self._pack_buf)

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        if self._f:
            self._f.flush()
            self._f.close()
            self._f = None
        os.umount(_SD_MOUNT)
