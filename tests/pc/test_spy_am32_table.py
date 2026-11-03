# Tests of tests/bench/wingman/tools/spy_am32_table.py: the per-recording cells and the table that lays runs side by side.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
sys.path.insert(0, str(ROOT / "tests" / "pc"))

import spy_am32_table
import test_spy_check_am32 as helpers


def lines(count=4, **kwargs):
    return [spy_am32_table.line_metrics(helpers.recording([0] * 120 + [60] * 40, **kwargs), False) for _ in range(count)]


class CellTests(unittest.TestCase):
    def test_a_clean_recording_has_no_mismatches_and_positive_margins(self):
        cell = spy_am32_table.combine(lines())
        self.assertEqual(cell["speed"], 600)
        self.assertEqual((cell["mismatches"], cell["outside"], cell["crc_bad"], cell["commands"]), (0, 0, 0, 0))
        self.assertGreater(cell["margin_zero"], 0.1)
        self.assertGreater(cell["margin_one"], 0.1)
        self.assertLess(cell["worst_pct"], 6.25)
        self.assertEqual(cell["status"]["bit"], "PASS")

    def test_cells_take_the_worst_line(self):
        good = lines(3)
        bad = spy_am32_table.line_metrics(helpers.recording([0] * 150, mutate={20: {"zero_low": 9.0}}), False)
        cell = spy_am32_table.combine(good + [bad])
        self.assertGreater(cell["mismatches"], 0)
        self.assertEqual(cell["status"]["bit"], "FAIL")

    def test_pace_comes_from_the_pooled_spacings(self):
        cell = spy_am32_table.combine(lines(2))
        self.assertAlmostEqual(cell["spacing"]["median"], 1045.0, places=1)


class TableTests(unittest.TestCase):
    def test_table_has_one_column_per_run_and_the_limits(self):
        cells = [spy_am32_table.combine(lines()), spy_am32_table.combine(lines())]
        text = spy_am32_table.table(600, ["run 1", "run 2"], cells)
        self.assertIn("DSHOT600 - 2 recording(s), 4 lines each", text)
        self.assertIn("run 1", text)
        self.assertIn("run 2", text)
        self.assertIn("bits AM32 would read differently from ours", text)
        self.assertIn("< 6.25", text)
        self.assertIn("frame spacing, median (us)", text)
        self.assertIn("1045.0", text)


if __name__ == "__main__":
    unittest.main()
