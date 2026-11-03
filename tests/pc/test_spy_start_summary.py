# Tests of the start classifier in tests/bench/wingman/tools/spy_start_summary.py: a start is judged against what the throttle that was
# sent normally reads, not against one fixed number (throttle 60 reads about 8,400 eRPM, throttle 215 about 52,600).

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import spy_start_summary as start_summary


def start(throttle, median):
    """The part of a start summary that classify() reads: the 0.5-1.5 s window's median."""
    return {"throttle": throttle, "windows": [(0, 100, None), (500, 1500, median)]}


class ClassifyTest(unittest.TestCase):
    def test_throttle_60_is_judged_exactly_as_before(self):
        self.assertEqual("STALL", start_summary.classify(start(60, 2999)))
        self.assertEqual("normal", start_summary.classify(start(60, 3001)))
        self.assertEqual("normal", start_summary.classify(start(60, 8400)))
        self.assertEqual("normal", start_summary.classify(start(60, 11999)))
        self.assertEqual("PLATEAU", start_summary.classify(start(60, 12001)))

    def test_a_healthy_throttle_215_start_is_normal(self):
        self.assertEqual("normal", start_summary.classify(start(215, 52632)))
        self.assertEqual("normal", start_summary.classify(start(215, 49700)))

    def test_throttle_215_stall_and_plateau_are_relative_to_its_own_reading(self):
        self.assertEqual("STALL", start_summary.classify(start(215, 945)))
        self.assertEqual("STALL", start_summary.classify(start(215, 18000)))
        self.assertEqual("PLATEAU", start_summary.classify(start(215, 80000)))

    def test_no_median_is_no_start_and_an_unknown_throttle_has_no_reference(self):
        self.assertEqual("no start", start_summary.classify(start(215, None)))
        self.assertEqual("no reference", start_summary.classify(start(130, 30000)))

    def test_the_old_limits_are_kept_as_ratios_of_the_throttle_60_reading(self):
        self.assertAlmostEqual(3000, start_summary.STALL_RATIO * start_summary.NORMAL_ERPM[60])
        self.assertAlmostEqual(12000, start_summary.PLATEAU_RATIO * start_summary.NORMAL_ERPM[60])


def rows_of(*pieces):
    """pieces: (reply state, from ms, to ms, throttle) -> one row per ms."""
    rows = []
    for state, a, b, throttle in pieces:
        rows += [{"t_ms": float(t), "reply": state, "throttle": throttle} for t in range(a, b)]
    return rows


class FinalCutTest(unittest.TestCase):
    def test_replies_that_stay_not_running_with_the_throttle_on_are_a_cut_at_its_start(self):
        rows = rows_of(("run", 0, 300, 150), ("idle", 300, 2000, 150))
        self.assertEqual(300, start_summary.final_cut_after_ms(rows, 0))

    def test_a_brief_not_running_stretch_followed_by_running_is_not_a_cut(self):
        rows = rows_of(("run", 0, 90, 150), ("idle", 90, 115, 150), ("run", 115, 4000, 150))
        self.assertIsNone(start_summary.final_cut_after_ms(rows, 0))

    def test_the_idle_after_the_throttle_returns_to_zero_is_not_a_cut(self):
        rows = rows_of(("run", 0, 4000, 215), ("idle", 4000, 6000, 0))
        self.assertIsNone(start_summary.final_cut_after_ms(rows, 0))

    def test_a_cut_shorter_than_the_minimum_is_not_final_yet(self):
        rows = rows_of(("run", 0, 300, 150), ("idle", 300, 600, 150))
        self.assertIsNone(start_summary.final_cut_after_ms(rows, 0))

    def test_a_motor_that_never_ran_has_no_cut(self):
        self.assertIsNone(start_summary.final_cut_after_ms(rows_of(("idle", 0, 3000, 150)), 0))


def run_rows(pieces):
    """pieces: (from ms, to ms, eRPM or None for not running, throttle) -> one row per ms with the fields spin_up() reads."""
    rows = []
    for a, b, erpm, throttle in pieces:
        rows += [{"t_ms": float(t), "reply": "idle" if erpm is None else "run", "erpm": erpm, "throttle": throttle} for t in range(a, b)]
    return rows


class SpinUpTest(unittest.TestCase):
    def test_spin_up_is_the_time_to_80_percent_of_the_motors_own_settled_speed(self):
        rows = run_rows([(0, 100, 3000, 215), (100, 4500, 48000, 215)])
        self.assertEqual((100.0, 48000), start_summary.spin_up(rows, 0))

    def test_it_holds_for_other_props_without_any_number_written_in_the_tool(self):
        rows = run_rows([(0, 250, 5000, 215), (250, 4500, 30000, 215)])        # a slower motor: 80% of 30,000 = 24,000
        self.assertEqual((250.0, 30000), start_summary.spin_up(rows, 0))

    def test_a_slow_start_is_long(self):
        rows = run_rows([(0, 1000, 2500, 215), (1000, 4500, 47000, 215)])
        self.assertEqual((1000.0, 47000), start_summary.spin_up(rows, 0))

    def test_a_lone_spike_is_not_speed(self):
        rows = run_rows([(0, 300, 3000, 215), (300, 302, 48000, 215), (302, 600, 3000, 215), (600, 4500, 48000, 215)])
        self.assertEqual((600.0, 48000), start_summary.spin_up(rows, 0))

    def test_a_motor_cut_before_it_settled_has_no_spin_up(self):
        rows = run_rows([(0, 300, 3000, 215), (300, 4500, None, 215)])
        self.assertEqual((None, None), start_summary.spin_up(rows, 0))

    def test_a_start_glitch_reading_is_not_speed(self):
        rows = run_rows([(0, 20, 30_000_000, 215), (20, 100, 3000, 215), (100, 4500, 48000, 215)])
        self.assertEqual((100.0, 48000), start_summary.spin_up(rows, 0))


if __name__ == "__main__":
    unittest.main()
