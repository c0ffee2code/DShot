# scripts/run_optical_rpm_modes.py's verdict: when the four DShot modes agree on the
# motor's real speed and when they do not.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from run_optical_rpm_modes import judge  # noqa: E402


def runs(**by_mode):
    return [(mode, rpm, "") for mode, values in by_mode.items() for rpm in values]


class JudgeTest(unittest.TestCase):
    def test_modes_that_agree_pass(self):
        lines, failures = judge(runs(a=[3385, 3380], b=[3393, 3393], c=[3388, 3379], d=[3388, 3366]))
        self.assertEqual(failures, [])
        self.assertIn("all modes: 3384", lines[0])

    def test_a_mode_far_from_the_rest_fails(self):
        _, failures = judge(runs(a=[3400, 3400], b=[3400, 3400], c=[3400, 3400], d=[3000, 3000]))
        self.assertEqual(len(failures), 1)
        self.assertIn("the modes differ by", failures[0])

    def test_the_limit_is_on_the_gap_between_fastest_and_slowest_mode(self):
        # means 1000 and 1030: a gap of 30 on an overall mean of 1015 is 2.96%
        _, failures = judge(runs(a=[1000], b=[1030]), tolerance_pct=3.0)
        self.assertEqual(failures, [])
        _, failures = judge(runs(a=[1000], b=[1040]), tolerance_pct=3.0)
        self.assertEqual(len(failures), 1)

    def test_a_run_without_a_figure_is_a_failure_even_if_the_rest_agree(self):
        all_runs = runs(a=[3400], b=[3400]) + [("c", None, "no optical RPM printed (arming failed)")]
        _, failures = judge(all_runs)
        self.assertEqual(failures, ["c: no optical RPM printed (arming failed)"])

    def test_no_figures_at_all_fails_without_a_table(self):
        lines, failures = judge([("a", None, "timed out")])
        self.assertEqual(lines, [])
        self.assertEqual(failures, ["a: timed out"])


if __name__ == "__main__":
    unittest.main()
