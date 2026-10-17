# scripts/run_scenarios.py's arming-time summary: how much room a batch of arms
# had. The clean/reboot verdict itself is judged against the bound; this pins
# what "marginal" and the median/extremes mean, so a batch is read correctly.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from run_scenarios import (arming_time_summary, fields, run_record,  # noqa: E402
                           CLEAN_ARM_THRESHOLD_MS, MARGINAL_BAND_MS)


class ArmingTimeSummaryTest(unittest.TestCase):
    def test_an_empty_batch_has_no_summary(self):
        self.assertIsNone(arming_time_summary([]))

    def test_min_median_max(self):
        room = arming_time_summary([2030, 2026, 2100])
        self.assertEqual((room["min"], room["median"], room["max"], room["n"]),
                         (2026, 2030, 2100, 3))

    def test_median_of_an_even_count_is_the_midpoint(self):
        self.assertEqual(arming_time_summary([2000, 2100, 2200, 2300])["median"], 2150)

    def test_arms_near_the_bound_are_marginal(self):
        edge = CLEAN_ARM_THRESHOLD_MS - MARGINAL_BAND_MS
        room = arming_time_summary([2030, edge - 1, edge, CLEAN_ARM_THRESHOLD_MS - 1])
        self.assertEqual(room["marginal"], 2)

    def test_a_late_arm_is_not_marginal(self):
        # at or over the bound it is a KPI failure, counted as reboot elsewhere
        room = arming_time_summary([CLEAN_ARM_THRESHOLD_MS, 4600])
        self.assertEqual(room["marginal"], 0)
        self.assertEqual(room["max"], 4600)

    def test_explicit_bound_and_band(self):
        room = arming_time_summary([900, 1000, 1100], bound_ms=1200, band_ms=250)
        self.assertEqual(room["marginal"], 2)


class RecordTest(unittest.TestCase):
    """Lines are split on whitespace, then on "=": no prose parsing."""

    def parse(self, line):
        label, *pairs = line.split(" ")
        return label, dict(p.split("=", 1) for p in pairs)

    def test_a_pass_carries_separate_fields(self):
        line = run_record(3, 20, "a.json", "pass", "clean", 2027, "2026-10-13_21-15-01", 3, 0, 0)
        label, f = self.parse(line)
        self.assertEqual(label, "RUN")
        self.assertEqual((f["verdict"], f["outcome"], f["armed_ms"]), ("pass", "clean", "2027"))
        self.assertEqual(f["margin_ms"], str(CLEAN_ARM_THRESHOLD_MS - 2027))
        self.assertEqual((f["pass_so_far"], f["fail_so_far"]), ("3", "0"))

    def test_a_late_arm_is_a_fail_with_its_time_and_no_margin(self):
        _, f = self.parse(run_record(1, 5, "a.json", "fail", "reboot", 4600, "s", 0, 1, 0))
        self.assertEqual((f["verdict"], f["armed_ms"]), ("fail", "4600"))
        self.assertNotIn("margin_ms", f)

    def test_unknown_values_are_left_out(self):
        _, f = self.parse(run_record(1, 5, "a.json", "fail", "refused", None, None, 0, 1, 0))
        self.assertNotIn("armed_ms", f)
        self.assertNotIn("session", f)

    def test_fields_keeps_order_and_skips_none(self):
        self.assertEqual(fields("X", a=1, b=None, c="z"), "X a=1 c=z")


if __name__ == "__main__":
    unittest.main()
