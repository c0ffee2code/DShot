# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes raw scenario capture records (as produced by
# tests/harness/run_scenario.py from MotorGroup.raw_telemetry()) to a
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
    # (motor0_w0..w3, motor1_w0..w3, motor2_w0..w3, motor3_w0..w3). A motor
    # that had no new capture for this record, and a non-bidirectional motor, has
    # an all-zero word group.
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

    def finalize(self, outcome, total_records, missed, largest_gap_us, published):
        """Record how the run ended and its final device-side stats in meta.txt.

        `missed` is the number of captures the group published that the run
        never saw, and `published` maps each bidirectional motor's index to the
        sequence number of its last capture, so the analyzer can relate the
        records in capture.bin to what the ESC actually sent.
        """
        fields = {
            "total_records": str(total_records),
            "captures_missed": str(missed),
            "largest_gap_us": str(largest_gap_us),
        }
        for index in published:
            fields["motor" + str(index) + "_captures_published"] = str(published[index])
        self.finalize_meta(outcome, fields)

    def write_record(self, ticks_us, throttles, words):
        """Write one record: `throttles` is the 4 motors' throttle values, `words`
        the 4 motors' 4-word capture groups (a tuple of 4 ints each)."""
        fields = [ticks_us]
        fields.extend(throttles)
        for group in words:
            fields.extend(group)
        struct.pack_into(self.RECORD_FMT, self.pack_buf, 0, *fields)
        self.file.write(self.pack_buf)
