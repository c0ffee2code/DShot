# Tests of the ESC mock's fixture format (tests/bench/wingman/shared/esc_mock_format.py) and of tests/bench/wingman/tools/esc_mock_fixture.py,
# which builds the fixture from a recorded spy.bin. The wire streams are built here from known edges, with the same
# helpers test_spy_edge_log.py uses.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import esc_mock_format
import esc_mock_fixture
import gcr_decode
import run_session
import session_summary
import spy_check_am32
from datetime import datetime


def real_frame(data12):
    """The 21-bit reply frame for a 12-bit payload, built the way an ESC does (CRC, GCR, differential)."""
    number = (data12 << 4) | gcr_decode.crc_inverted(data12)
    bits = 0
    for shift in (12, 8, 4, 0):
        bits = (bits << 5) | gcr_decode.GCR_ENCODE_TABLE[(number >> shift) & 0xF]
    level, out = 0, 0
    for i in range(19, -1, -1):          # differential: a 1 bit flips the level, a 0 keeps it
        if (bits >> i) & 1:
            level ^= 1
        out = (out << 1) | level
    return out


class FormatTest(unittest.TestCase):
    def test_not_running_frame_is_what_the_driver_expects(self):
        self.assertEqual(real_frame(gcr_decode.AM32_NOT_RUNNING_DATA12), gcr_decode.AM32_NOT_RUNNING_FRAME)

    def test_format_not_running_frame_equals_the_drivers(self):
        self.assertEqual(esc_mock_format.NOT_RUNNING_FRAME, gcr_decode.AM32_NOT_RUNNING_FRAME)

    def test_real_frame_decodes(self):
        data12 = (2 << 9) | 100
        result = gcr_decode.analyze_frame(real_frame(data12))
        self.assertTrue(result["crc_ok"])
        self.assertEqual(result["data12"], data12)

    def test_pio_word_round_trip(self):
        for frame in (0, 1, gcr_decode.AM32_NOT_RUNNING_FRAME, real_frame(1234), (1 << 21) - 1):
            self.assertEqual(esc_mock_format.frame_from_pio_word(esc_mock_format.pio_word(frame)), frame)

    def test_pio_word_shifts_the_first_level_out_first(self):
        word = esc_mock_format.pio_word(0b0 << 20 | 0b101)       # marker 0 (line low), then zeros, then 1 0 1
        self.assertEqual(word >> 31, 0)                           # the marker level goes out first
        self.assertEqual((word >> 11) & 0b111, 0b101)             # the last three levels, as they are
        self.assertEqual(word & 0x7FF, 0)                         # the 11 unused low bits

    def test_fixture_round_trip(self):
        lines = [[real_frame(300 + i) for i in range(5)], [real_frame(400 + i) for i in range(5)]]
        self.assertEqual(esc_mock_format.parse_fixture(esc_mock_format.pack_fixture(lines)), lines)

    def test_unequal_lines_are_refused(self):
        with self.assertRaises(ValueError):
            esc_mock_format.pack_fixture([[1, 2], [3]])

    def test_damaged_fixture_is_refused(self):
        data = esc_mock_format.pack_fixture([[real_frame(300)]])
        with self.assertRaises(ValueError):
            esc_mock_format.parse_fixture(data[:-1])
        with self.assertRaises(ValueError):
            esc_mock_format.parse_fixture(b"XXXX" + data[4:])


class ProvenanceTest(unittest.TestCase):
    def test_the_sidecar_says_which_session_the_replies_came_from(self):
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "2026-11-01_11-36-52"
            folder.mkdir()
            (folder / "session.json").write_text(json.dumps({"scenario": "s.json", "git_commit": "abc", "load": "props"}), encoding="utf-8")
            info = esc_mock_fixture.provenance(folder, 1024, 5000, [52632.4, 53004.6], built="2026-11-01")
        self.assertEqual({"source_session": "2026-11-01_11-36-52", "source_scenario": "s.json", "source_git_commit": "abc",
                          "source_load": "props", "built": "2026-11-01", "replies_per_line": 1024, "taken_after_ms": 5000,
                          "median_erpm_per_line": [52632, 53005]}, info)

    def test_a_session_folder_without_a_manifest_still_gives_a_sidecar(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            info = esc_mock_fixture.provenance(tmp, 8, 0, [1.0], built="2026-11-01")
        self.assertIsNone(info["source_git_commit"])


class SessionTest(unittest.TestCase):
    def test_manifest_marks_an_esc_mock_session(self):
        mock = run_session.build_manifest("s", "a.json", True, [], None, None, datetime(2026, 1, 1), "none", esc_mock={"fixture": "f.bin", "source": {"source_session": "s"}})
        self.assertEqual(mock["esc_mock"], {"fixture": "f.bin", "source": {"source_session": "s"}})
        real = run_session.build_manifest("s", "a.json", True, [], None, None, datetime(2026, 1, 1), "none")
        self.assertIsNone(real["esc_mock"])

    def results(self):
        R = spy_check_am32.Result
        return [R("first reply comes only after the bidirectional latch (101 frames)", spy_check_am32.WARN, "early", "", esc_only=True),
                R("the ESC's arming silence comes before our first nonzero throttle", spy_check_am32.WARN, "none", "", esc_only=True),
                R("reply integrity", spy_check_am32.FAIL, "bad crc", "")]

    def test_esc_behaviour_checks_do_not_apply_to_a_mock_session(self):
        summary = session_summary.check_summary(self.results(), esc_mock=True)
        self.assertEqual(summary["counts"], {spy_check_am32.NA: 2, spy_check_am32.FAIL: 1})
        self.assertEqual([n["check"] for n in summary["notable"]], ["reply integrity"])

    def test_a_real_session_still_reports_them(self):
        summary = session_summary.check_summary(self.results())
        self.assertEqual(summary["counts"], {spy_check_am32.WARN: 2, spy_check_am32.FAIL: 1})


if __name__ == "__main__":
    unittest.main()
