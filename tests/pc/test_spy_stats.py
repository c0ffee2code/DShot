# Tests of tests/bench/wingman/tools/spy_stats.py: the numbers and per-frame series behind tests/bench/wingman/tools/spy_analyze.py.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import spy_stats


def record(start_units, throttle=60, reply=None, bits=None):
    """One spy_edge_log.analyse() record with a 16-bit frame of fixed timing (periods 16.667 units)."""
    bits = bits if bits is not None else [1, 0] * 8
    return {
        "start": start_units,
        "edges": 32,
        "tx": {"ok": True, "throttle": throttle, "crc_ok": True, "periods": [16.667] * 15,
               "lows": [12.5 if b else 6.25 for b in bits], "bits": bits},
        "reply": reply,
    }


def reply(crc_ok=True, not_running=False, erpm=23000, delay=300):
    return {"crc_ok": crc_ok, "not_running": not_running, "erpm": erpm, "delay": delay}


class SummaryTests(unittest.TestCase):
    def test_summary_values(self):
        s = spy_stats.summary([1, 2, 3, 4, 100])
        self.assertEqual((s["n"], s["min"], s["median"], s["max"]), (5, 1, 3, 100))
        self.assertAlmostEqual(s["mean"], 22.0)

    def test_empty_summary_has_no_values(self):
        self.assertIsNone(spy_stats.summary([])["median"])


class FrameTableTests(unittest.TestCase):
    def test_spacing_and_frame_length_in_microseconds(self):
        rows = spy_stats.frame_table([record(0), record(10450), record(20900)])
        self.assertIsNone(rows[0]["spacing_us"])
        self.assertAlmostEqual(rows[1]["spacing_us"], 1045.0)
        self.assertAlmostEqual(rows[0]["frame_us"], 25.0, places=2)
        self.assertAlmostEqual(rows[0]["low1_us"], 1.25)
        self.assertAlmostEqual(rows[0]["low0_us"], 0.625)
        self.assertAlmostEqual(rows[2]["t_ms"], 2.09)

    def test_groups_without_a_frame_are_skipped_and_do_not_break_spacing(self):
        broken = {"start": 5000, "edges": 3, "tx": None, "reply": None}
        rows = spy_stats.frame_table([record(0), broken, record(10450)])
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[1]["spacing_us"], 1045.0)

    def test_reply_status(self):
        rows = spy_stats.frame_table([
            record(0), record(1, reply=reply()), record(2, reply=reply(crc_ok=False, erpm=None)),
            record(3, reply=reply(not_running=True, erpm=917))])
        self.assertEqual([r["reply"] for r in rows], ["none", "run", "bad", "idle"])
        self.assertEqual(rows[1]["erpm"], 23000)
        self.assertIsNone(rows[2]["erpm"])
        self.assertIsNone(rows[3]["erpm"])           # the not-running reply is not a speed
        self.assertAlmostEqual(rows[1]["delay_us"], 30.0)


class HiccupTests(unittest.TestCase):
    def test_a_stalled_frame_is_found_and_quiet_ones_are_not(self):
        starts = [i * 10450 for i in range(50)]
        starts[30:] = [s + 3000 for s in starts[30:]]           # one 300us stall at frame 30
        rows = spy_stats.frame_table([record(s) for s in starts])
        found = spy_stats.hiccups(rows, 50)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][1], 1345.0)

    def test_window_keeps_only_the_asked_stretch(self):
        rows = spy_stats.frame_table([record(i * 10450) for i in range(100)])
        kept = spy_stats.window(rows, 20, 30)
        self.assertTrue(all(20 <= r["t_ms"] <= 30 for r in kept))
        self.assertGreater(len(kept), 5)

    def test_summaries_skip_fields_without_values(self):
        labels = [label for label, _ in spy_stats.summaries(spy_stats.frame_table([record(0), record(10450)]))]
        self.assertIn("frame spacing (us)", labels)
        self.assertNotIn("reply eRPM", labels)


if __name__ == "__main__":
    unittest.main()
