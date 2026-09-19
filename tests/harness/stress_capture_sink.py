# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes sampled raw records from tests/test_bidir_rx_stress.py's W4/W5
# saturation and starvation characterization runs to a timestamped session
# folder on the PicoBell Adalogger's SD card. The SD/RTC mount and
# session-folder lifecycle are in capture_sink.py; this adds a different,
# smaller record shape (one channel + a phase tag + a variable word count per
# record, not the fixed 4-motor record bidir_capture_sink.py writes).
# scripts/pull_captures.py fetches it unchanged - it only cares about the
# meta.txt/capture.bin/scenario.json filenames under one session directory, not
# their contents - but the permanent scenario engine's analyzer needs a
# schema-valid 4-motor scenario.json, which doesn't fit here; see
# scripts/analyze_bidir_stress_log.py instead.

import json
import struct

from capture_sink import CaptureSinkBase


class StressCaptureSink(CaptureSinkBase):
    # ticks_us, channel, phase, word_count, w0..w3 (zero-padded past word_count -
    # word_count itself is what marks a record as a partial/boundary group)
    RECORD_FMT = "<IBBB4I"

    def init_session(self, provenance):
        """Create a timestamped run directory and open the capture log.

        provenance: a plain dict of run parameters (channels, throttle,
        target frames, starvation settings) written to scenario.json for
        human/analyzer reference - nothing parses it against a schema,
        unlike the permanent scenario engine's capture sink.
        """
        self.create_session_dir({"outcome": "running"})

        with open(self.run_dir + "/scenario.json", "w") as f:
            json.dump(provenance, f)

        self.open_capture()

    def write_records(self, records):
        """Batch-write buffered (ticks_us, channel, phase, word_count, w0, w1, w2, w3)
        tuples in one pass, after the measurement loop has ended.

        test_bidir_rx_stress.py deliberately does zero file I/O while the
        loop it's characterizing is running - inline I/O there would pace
        the loop and the harness would end up measuring itself instead of
        the driver.
        """
        for ticks_us, channel, phase, word_count, w0, w1, w2, w3 in records:
            struct.pack_into(self.RECORD_FMT, self.pack_buf, 0,
                              ticks_us, channel, phase, word_count, w0, w1, w2, w3)
            self.file.write(self.pack_buf)

    def finalize(self, outcome, meta_fields):
        """Record how the run ended and its final counters in meta.txt.

        meta_fields: flat dict of already-stringifiable counters (frames
        queued, complete/misaligned/partial counts, FIFO occupancy
        histogram, starvation aggregates) written as one key=value line
        each - meta.txt alone is enough to report results even before
        capture.bin's sampled words are decoded.
        """
        self.finalize_meta(outcome, meta_fields)
