# scripts/analyze_bidir_capture_log.py on a synthetic sampled session: the motor's hardware line and the
# shaft RPM it derives from the ESC's eRPM, the refusal of anything that is not a sampled log, and the
# cross-check of the Pico's tally and failed flags against the decode on the PC.

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

FAILED = 0x80
TOKEN = 5 << 21


def load_analyzer():
    spec = importlib.util.spec_from_file_location(
        "analyze_bidir_capture_log", ROOT / "scripts" / "analyze_bidir_capture_log.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(ticks_us, frame, motor=0, failed=False):
    """One sampled record: motor 0's capture is the frame (with a token above it), the others none."""
    return struct.pack("<IB4H4I", ticks_us, motor | (FAILED if failed else 0), 100, 0, 0, 0,
                       TOKEN | frame if motor == 0 else 0, 0, 0, 0)


def write_session(directory, records, device=None, scenario=None):
    (directory / "scenario.json").write_text(json.dumps(scenario or valid_scenario()))
    meta = {"outcome": "completed", "rx_clock_hz": "1", "dshot_speed": "2400000", "bidir_motor_indices": "0",
            "capture_kind": "sampled", "sample_every": "10"}
    meta.update(device or {})
    (directory / "session.json").write_text(json.dumps({"session": directory.name, "device": meta}))
    (directory / "capture.bin").write_bytes(b"".join(records))


def analyze(directory, decoder=None):
    module = load_analyzer()
    # the real decoder needs real frames; here a frame is simply its own eRPM, and 0xFFFFF is a frame that fails
    module.analyze_frame = decoder or (lambda word: {"full": None if word == 0xFFFFF else word, "crc_ok": word != 0xFFFFF,
                                                     "erpm": float(word)})
    out = io.StringIO()
    argv = sys.argv
    sys.argv = ["analyze_bidir_capture_log.py", str(directory)]
    code = 0
    try:
        with contextlib.redirect_stdout(out):
            try:
                module.main()
            except SystemExit as stop:
                code = stop.code if stop.code is not None else 0
                if isinstance(stop.code, str):
                    out.write(stop.code)
                    code = 1
    finally:
        sys.argv = argv
    return out.getvalue(), code


class ShaftRpmReportTest(unittest.TestCase):
    def test_the_report_names_the_motor_and_divides_erpm_by_pole_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), [sample(5_000 + i * 1000, 20400) for i in range(30)])
            text, _ = analyze(Path(tmp))
        self.assertIn("Test Motor 1104, 7200 KV, 12 magnets (6 pole pairs)", text)
        self.assertIn("shaft RPM (eRPM / 6)", text)
        self.assertIn("3400", text)


class RefusalTest(unittest.TestCase):
    def test_a_log_that_is_not_sampled_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), [sample(1, 20400)], device={"capture_kind": "per-pass"})
            text, code = analyze(Path(tmp))
        self.assertNotEqual(0, code)
        self.assertIn("capture_kind", text)

    def test_a_session_without_the_kind_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), [sample(1, 20400)])
            meta = json.loads((Path(tmp) / "session.json").read_text())
            del meta["device"]["capture_kind"]
            (Path(tmp) / "session.json").write_text(json.dumps(meta))
            _, code = analyze(Path(tmp))
        self.assertNotEqual(0, code)

    def test_an_aborted_run_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), [sample(1, 20400)], device={"outcome": "aborted: RuntimeError: arming did not complete within 18000ms"})
            _, code = analyze(Path(tmp))
        self.assertNotEqual(0, code)


class CrossCheckTest(unittest.TestCase):
    def test_a_matching_tally_and_flags_pass(self):
        records = [sample(1000 * i, 20400) for i in range(25)] + [sample(30_000, 0xFFFFF, failed=True)]
        device = {"motor0_decode_sampled": "26", "motor0_decode_crc_ok": "25", "motor0_decode_crc_fail": "0",
                  "motor0_decode_invalid": "1", "samples_written": "26", "samples_failed": "1"}
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), records, device=device)
            text, code = analyze(Path(tmp))
        self.assertEqual(0, code, text)
        self.assertIn("failed sample at 30000 us: token 5 frame 0x0fffff (device flagged it: True)", text)

    def test_a_failed_flag_the_pc_does_not_reproduce_is_reported(self):
        records = [sample(1000 * i, 20400) for i in range(24)] + [sample(30_000, 20400, failed=True)]
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), records)
            text, code = analyze(Path(tmp))
        self.assertEqual(1, code)
        self.assertIn("failed flag disagrees", text)

    def test_a_tally_that_differs_from_the_devices_is_reported(self):
        records = [sample(1000 * i, 20400) for i in range(25)]
        device = {"motor0_decode_sampled": "25", "motor0_decode_crc_ok": "24", "motor0_decode_crc_fail": "1",
                  "motor0_decode_invalid": "0"}
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), records, device=device)
            text, code = analyze(Path(tmp))
        self.assertEqual(1, code)
        self.assertIn("device tallied crc_ok=24", text)

    def test_a_missing_record_is_reported(self):
        records = [sample(1000 * i, 20400) for i in range(25)]
        with tempfile.TemporaryDirectory() as tmp:
            write_session(Path(tmp), records, device={"samples_written": "26"})
            text, code = analyze(Path(tmp))
        self.assertEqual(1, code)
        self.assertIn("wrote 26 samples, the log holds 25", text)


if __name__ == "__main__":
    unittest.main()
