# scripts/analyze_bidir_capture_log.py on a synthetic session: the motor's hardware line
# and the shaft RPM it derives from the ESC's eRPM.

import contextlib
import importlib.util
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "pc"))
# The analyzer sits in scripts/ next to dshot_bidir_decode.py, which it imports by name
sys.path.insert(0, str(ROOT / "scripts"))

import fakes  # noqa: E402,F401
from test_scenario import valid_scenario  # noqa: E402


def load_analyzer():
    spec = importlib.util.spec_from_file_location(
        "analyze_bidir_capture_log", ROOT / "scripts" / "analyze_bidir_capture_log.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_session(directory, erpm, seconds=4):
    (directory / "scenario.json").write_text(json.dumps(valid_scenario()))
    (directory / "meta.txt").write_text(
        "outcome=completed\nrx_clock_hz=1\ndshot_speed=2400000\nbidir_motor_indices=0\n")
    # one record per millisecond, the motor-0 word carrying the eRPM directly
    records = b"".join(struct.pack("<I4H4I", 5_000 + i * 1000, 100, 0, 0, 0, erpm, 0, 0, 0)
                       for i in range(seconds * 1000))
    (directory / "capture.bin").write_bytes(records)


def analyze(directory):
    module = load_analyzer()
    # the real decoder needs real frames; here a word is simply its own eRPM
    module.analyze_frame = lambda word: {"full": word, "crc_ok": True, "erpm": float(word)}
    out = io.StringIO()
    argv = sys.argv
    sys.argv = ["analyze_bidir_capture_log.py", str(directory)]
    try:
        with contextlib.redirect_stdout(out):
            module.main()
    finally:
        sys.argv = argv
    return out.getvalue()


class ShaftRpmReportTest(unittest.TestCase):
    def test_the_report_names_the_motor_and_divides_erpm_by_pole_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), erpm=20400)
            text = analyze(Path(tmp))
        self.assertIn("Test Motor 1104, 7200 KV, 12 magnets (6 pole pairs)", text)
        self.assertIn("shaft RPM (eRPM / 6)", text)
        self.assertIn("3400", text)


if __name__ == "__main__":
    unittest.main()
