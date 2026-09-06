# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes sampled raw records from tests/test_bidir_rx_stress.py's W4/W5
# saturation and starvation characterization runs to a timestamped session
# folder on the PicoBell Adalogger's SD card.
#
# Own copy of tests/harness/bidir_capture_sink.py's SD/RTC mount and
# session-directory lifecycle rather than a shared class: this is
# throwaway characterization data with a different, smaller record shape
# (one channel + a phase tag + a variable word count per record, not the
# fixed 4-motor record bidir_capture_sink.py writes). scripts/pull_captures.py
# fetches it unchanged - it only cares about the meta.txt/capture.bin/
# scenario.json filenames under one session directory, not their contents -
# but the permanent scenario engine's analyzer needs a schema-valid 4-motor
# scenario.json, which doesn't fit here; see scripts/analyze_bidir_stress_log.py
# instead.

import json
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

# ticks_us, channel, phase, word_count, w0..w3 (zero-padded past word_count -
# word_count itself is what marks a record as a partial/boundary group)
_RECORD_FMT = "<IBBB4I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)


class StressCaptureSink:
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

    def init_session(self, provenance):
        """Create a timestamped run directory and open the capture log.

        provenance: a plain dict of run parameters (channels, throttle,
        target frames, starvation settings) written to scenario.json for
        human/analyzer reference - nothing parses it against a schema,
        unlike the permanent scenario engine's capture sink.

        Raises OSError if the RTC hasn't been set yet (see scripts/set_rtc.py) -
        surfaced here rather than silently falling back to a fake
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

        self._meta = {"outcome": "running"}
        self._write_meta()

        with open(self._run_dir + "/scenario.json", "w") as f:
            json.dump(provenance, f)

        self._f = open(self._run_dir + "/capture.bin", "wb")

    def _write_meta(self):
        with open(self._run_dir + "/meta.txt", "w") as f:
            for key, value in self._meta.items():
                f.write("{}={}\n".format(key, value))

    def write_records(self, records):
        """Batch-write buffered (ticks_us, channel, phase, word_count, w0, w1, w2, w3)
        tuples in one pass, after the measurement loop has ended.

        test_bidir_rx_stress.py deliberately does zero file I/O while the
        loop it's characterizing is running - inline I/O there would pace
        the loop and the harness would end up measuring itself instead of
        the driver.
        """
        for ticks_us, channel, phase, word_count, w0, w1, w2, w3 in records:
            struct.pack_into(_RECORD_FMT, self._pack_buf, 0,
                              ticks_us, channel, phase, word_count, w0, w1, w2, w3)
            self._f.write(self._pack_buf)

    def finalize(self, outcome, meta_fields):
        """Record how the run ended and its final counters in meta.txt.

        meta_fields: flat dict of already-stringifiable counters (frames
        queued, complete/misaligned/partial counts, FIFO occupancy
        histogram, starvation aggregates) written as one key=value line
        each - meta.txt alone is enough to report results even before
        capture.bin's sampled words are decoded.
        """
        self._meta["outcome"] = outcome
        for key, value in meta_fields.items():
            self._meta[key] = value
        self._write_meta()

    @property
    def path(self):
        """Return the run directory path (useful for diagnostics)."""
        return self._run_dir

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        if self._f:
            self._f.flush()
            self._f.close()
            self._f = None
        os.umount(_SD_MOUNT)
