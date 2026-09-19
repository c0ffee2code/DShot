# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes raw scenario capture records (as produced by
# tests/harness/scenario_runner.py's ScenarioRunner.drain()) to a
# timestamped session folder on the PicoBell Adalogger's SD card. The SD/RTC
# mount and session-folder lifecycle are in capture_sink.py; this adds the
# scenario record format and copies the scenario's own JSON into the session
# folder for full provenance, the same move Flight-Benchy's recorder.py makes
# with config.json/specification.json.

import struct

from capture_sink import CaptureSinkBase
from dshot_pio import BIDIR_PROFILES

COPY_CHUNK_SIZE = 512


class BidirCaptureSink(CaptureSinkBase):
    # ticks_us, throttle0..3, then one 4-word GCR capture group per motor
    # (motor0_w0..w3, motor1_w0..w3, motor2_w0..w3, motor3_w0..w3) - matches the
    # 21-field tuple shape ScenarioRunner.drain() returns, so
    # write_record(*record) works directly. A non-bidirectional motor's word
    # group is always zero.
    RECORD_FMT = "<I4H16I"

    def init_session(self, scenario, scenario_path):
        """Create a timestamped run directory and open the capture log.

        Copies `scenario_path`'s contents verbatim into the session folder
        as scenario.json - full provenance for the PC-side analyzer.
        """
        bidir_indices = ",".join(str(i) for i in scenario.bidir_indices)
        rx_clock_hz = ((BIDIR_PROFILES.get(scenario.dshot_speed) or {}).get("rx_speed")
                       if scenario.bidir_indices else 0)

        self.create_session_dir({
            "dshot_speed": str(scenario.dshot_speed),
            "rx_clock_hz": str(rx_clock_hz),
            "record_fmt": self.RECORD_FMT,
            "bidir_motor_indices": bidir_indices,
            "outcome": "running",
        })

        with open(scenario_path, "rb") as src, open(self.run_dir + "/scenario.json", "wb") as dst:
            while True:
                chunk = src.read(COPY_CHUNK_SIZE)
                if not chunk:
                    break
                dst.write(chunk)

        self.open_capture()

    def finalize(self, outcome, total_records, dropped, largest_gap_us):
        """Record how the run ended and its final device-side stats in meta.txt.

        Lets the analyzer re-check the scenario's own `expect` thresholds
        against the actual on-device dropped/gap counts rather than
        recomputing an approximation from capture.bin alone.
        """
        self.finalize_meta(outcome, {
            "total_records": str(total_records),
            "dropped": str(dropped),
            "largest_gap_us": str(largest_gap_us),
        })

    def write_record(self, ticks_us, t0, t1, t2, t3,
                      m0w0, m0w1, m0w2, m0w3,
                      m1w0, m1w1, m1w2, m1w3,
                      m2w0, m2w1, m2w2, m2w3,
                      m3w0, m3w1, m3w2, m3w3):
        struct.pack_into(
            self.RECORD_FMT, self.pack_buf, 0,
            ticks_us, t0, t1, t2, t3,
            m0w0, m0w1, m0w2, m0w3,
            m1w0, m1w1, m1w2, m1w3,
            m2w0, m2w1, m2w2, m2w3,
            m3w0, m3w1, m3w2, m3w3,
        )
        self.file.write(self.pack_buf)
