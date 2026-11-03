# Tests of scripts/start_kpi.py: the start KPI's per-run block (all four motors spun up and held, peers agree, nothing derails, spin-up within the limit, whether
# the run counts or never started), the one-field verdict for a batch RUN line, and the totals over many sessions.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import start_kpi


def line(gpio, spin_up=100.0, cut=None, settled=48000, hold_outside=0.1, step=215, max_throttle=215, mode="bidirectional", commanded=True):
    return {"mode": mode, "gpio": gpio,
            "facts": {"commanded": commanded, "spin_up_ms": spin_up, "cut_after_ms": cut, "settled_erpm": settled, "hold_outside_pct": hold_outside,
                      "step_throttle": step, "max_throttle": max_throttle}}


GOOD = [line(6, 117.0), line(7, 115.0, settled=47800), line(8, 89.0, settled=48400), line(9, 160.0, settled=48100)]
PROP = start_kpi.KPI_PROPELLER


def block(lines=None, props=None, esc_mock=False):
    return start_kpi.start_block(GOOD if lines is None else lines, props if props is not None else [PROP] * 4, esc_mock)


def summary(lines=None, props=None, esc_mock=False, valid=True):
    lines = GOOD if lines is None else lines
    return {"start": block(lines, props, esc_mock), "spy": {"valid": valid, "lines": lines}}


class StartBlockTest(unittest.TestCase):
    def test_the_limit_is_half_a_second(self):
        self.assertEqual(500, start_kpi.SPIN_UP_LIMIT_MS)

    def test_a_run_where_all_four_spun_up_in_time_counts_and_names_the_slowest(self):
        start = block()
        self.assertEqual("in_scope", start["status"])
        self.assertTrue(start["all_spun_up"])
        self.assertTrue(start["spin_up_within_limit"])
        self.assertTrue(start["start_ok"])
        self.assertEqual((160.0, 9), (start["slowest_spin_up_ms"], start["slowest_gpio"]))
        self.assertIn("slowest spin-up 160 ms (GPIO 9), within the 500 ms limit   [counts toward the start KPI]", start_kpi.start_text(start))

    def test_a_slow_start_is_over_the_limit_but_still_spun_up(self):
        start = block(GOOD[:3] + [line(9, 1052.0, settled=48100)])
        self.assertTrue(start["all_spun_up"])
        self.assertFalse(start["spin_up_within_limit"])
        self.assertFalse(start["start_ok"])
        self.assertIn("OVER the 500 ms limit", start_kpi.start_text(start))

    def test_exactly_at_the_limit_is_within_it(self):
        self.assertTrue(block(GOOD[:3] + [line(9, 500.0, settled=48100)])["spin_up_within_limit"])

    def test_a_motor_that_never_settled_or_was_cut_is_not_spun_up(self):
        for bad in (line(9, spin_up=None, settled=None), line(9, spin_up=120.0, cut=2400.0)):
            with self.subTest(bad=bad):
                start = block(GOOD[:3] + [bad])
                self.assertFalse(start["all_spun_up"])
                self.assertIsNone(start["spin_up_within_limit"])
                self.assertEqual([9], start["not_spun_up_or_cut"])
                self.assertIn("NOT all spun up and held - GPIO 9", start_kpi.start_text(start))

    def test_a_motor_stuck_at_the_low_reading_is_not_spun_up_just_because_it_settled_there(self):
        stuck = line(9, spin_up=0.0, settled=2500)                  # 2.5k eRPM for the whole run, never cut: its own spin-up reads 0 ms
        start = block(GOOD[:3] + [stuck])
        self.assertFalse(start["all_spun_up"])
        self.assertEqual([9], start["peer_outliers"])
        self.assertEqual([9], start["not_spun_up_or_cut"])
        self.assertFalse(start["start_ok"])
        self.assertIn("settled speed far from the other motors: GPIO 9", start_kpi.start_text(start))

    def test_peers_within_the_tolerance_agree(self):
        near = [line(6, settled=48000), line(7, settled=48000 * 1.14), line(8, settled=48000 * 0.86), line(9, settled=48000)]
        self.assertEqual([], block(near)["peer_outliers"])
        far = [line(6, settled=48000), line(7, settled=48000), line(8, settled=48000), line(9, settled=48000 * 1.2)]
        self.assertEqual([9], block(far)["peer_outliers"])

    def test_a_motor_that_stalls_after_spinning_up_without_a_cut_does_not_hold(self):
        derailing = line(9, hold_outside=12.5, settled=48100)
        start = block(GOOD[:3] + [derailing])
        self.assertEqual([9], start["derailed"])
        self.assertFalse(start["all_spun_up"])
        self.assertIn("did not hold: GPIO 9", start_kpi.start_text(start))
        self.assertTrue(block(GOOD[:3] + [line(9, hold_outside=0.9, settled=48100)])["all_spun_up"])       # brief dips are fine

    def test_the_esc_mock_other_props_other_throttle_and_other_motor_counts_are_outside_the_kpi(self):
        self.assertIn("ESC mock", block(esc_mock=True)["outside_the_scope_because"][0])
        other = block(props=[PROP, PROP, PROP, "none"])
        self.assertEqual("out_of_scope", other["status"])
        self.assertIn("not all Gemfan 2218 3-Blade", other["outside_the_scope_because"][0])
        self.assertEqual("out_of_scope", block(props=[None] * 4)["status"])
        self.assertEqual("out_of_scope", block(GOOD[:2], props=[PROP] * 2)["status"])
        for throttle in (149, 150, 170):                            # the stuck-rotor experiments and the reversed-order variants
            with self.subTest(throttle=throttle):
                run = [line(g, step=throttle, max_throttle=throttle) for g in (6, 7, 8, 9)]
                start = block(run)
                self.assertEqual("out_of_scope", start["status"])
                self.assertIn("throttle step is not 0 -> 215", start["outside_the_scope_because"][0])
        ramp = [line(g, step=60, max_throttle=215) for g in (6, 7, 8, 9)]
        self.assertEqual("out_of_scope", block(ramp)["status"])

    def test_a_kpi_run_that_never_reached_the_step_is_not_started_not_out_of_scope(self):
        never = [line(g, spin_up=None, settled=None, step=None, max_throttle=0, commanded=False) for g in (6, 7, 8, 9)]
        start = block(never)
        self.assertEqual("not_started", start["status"])
        self.assertFalse(start["in_kpi_scope"])
        self.assertIn("never reached the throttle step (0 of 4 motors commanded)", start_kpi.start_text(start))

    def test_a_run_that_commanded_nothing_with_other_props_is_out_of_scope(self):
        start = block([line(6, None, commanded=False)], props=["none"] * 4)
        self.assertEqual("out_of_scope", start["status"])


class RunFieldTest(unittest.TestCase):
    def test_ok_slow_fail_not_started_out_of_scope_and_unknown(self):
        self.assertEqual(("ok", 160.0), start_kpi.run_field(summary()))
        self.assertEqual(("slow", 1052.0), start_kpi.run_field(summary(GOOD[:3] + [line(9, 1052.0, settled=48100)])))
        self.assertEqual(("fail", 117.0), start_kpi.run_field(summary([line(6, 117.0), line(7, None, settled=None), line(8, 90.0), line(9, 100.0)])))
        never = [line(g, spin_up=None, settled=None, step=None, max_throttle=0, commanded=False) for g in (6, 7, 8, 9)]
        self.assertEqual(("not_started", None), start_kpi.run_field(summary(never)))
        self.assertEqual(("out_of_scope", 160.0), start_kpi.run_field(summary(esc_mock=True)))
        self.assertEqual(("unknown", None), start_kpi.run_field({}))
        self.assertEqual(("unknown", None), start_kpi.run_field(None))


class LowerBoundTest(unittest.TestCase):
    def test_it_is_the_exact_one_sided_bound_the_arming_evidence_uses(self):
        self.assertAlmostEqual(0.972, start_kpi.lower_bound(104, 104), places=3)       # "104 of 104 support about 97%"
        self.assertAlmostEqual(0.970, start_kpi.lower_bound(100, 100), places=3)       # "100 support about 97%"
        self.assertAlmostEqual(0.990, start_kpi.lower_bound(300, 300), places=3)       # "about 300 support 99%"
        self.assertAlmostEqual(0.899, start_kpi.lower_bound(28, 28), places=3)
        self.assertAlmostEqual(0.792, start_kpi.lower_bound(26, 28), places=3)
        self.assertIsNone(start_kpi.lower_bound(0, 0))
        self.assertEqual(0.0, start_kpi.lower_bound(0, 10))


class AggregateTest(unittest.TestCase):
    def batch(self):
        slow = GOOD[:3] + [line(9, 1052.0, settled=48100)]
        failed = [line(6, 100.0), line(7, 100.0), line(8, 100.0), line(9, None, settled=None)]
        never = [line(g, spin_up=None, settled=None, step=None, max_throttle=0, commanded=False) for g in (6, 7, 8, 9)]
        return ([summary()] * 6 + [summary(slow), summary(failed), summary(esc_mock=True), summary(never), summary(valid=False), {"verdict": "pass"}])

    def test_the_run_is_the_unit_and_every_other_session_is_counted_apart(self):
        agg = start_kpi.aggregate(self.batch())
        self.assertEqual((12, 8), (agg["sessions"], agg["runs"]))
        self.assertEqual((1, 1, 1, 1), (agg["out_of_scope"], agg["not_started"], agg["instrument_problem"], agg["skipped_no_start_block"]))
        self.assertEqual(7, agg["all_spun_up"])
        self.assertEqual(6, agg["start_ok"])
        self.assertEqual(1, agg["runs_with_a_start_over_the_limit"])
        self.assertEqual(87.5, agg["all_spun_up_pct"])

    def test_a_not_started_run_is_reported_both_ways(self):
        agg = start_kpi.aggregate(self.batch())
        self.assertEqual(75.0, agg["start_ok_pct"])                                   # 6 of 8 runs
        self.assertEqual(round(100 * 6 / 9, 1), agg["start_ok_if_not_started_fail_pct"])   # 6 of 8 + 1 not started

    def test_the_bound_per_run_is_the_one_sided_exact_one(self):
        agg = start_kpi.aggregate([summary()] * 28)
        self.assertEqual(89.9, agg["all_spun_up_lower95_pct"])
        self.assertEqual(89.9, agg["start_ok_lower95_pct"])

    def test_per_motor_numbers_sit_beside_the_run_figure(self):
        agg = start_kpi.aggregate(self.batch())
        nine = agg["per_gpio"][9]
        self.assertEqual((8, 1, 1), (nine["starts"], nine["not_spun_up"], nine["over_limit"]))
        self.assertEqual(160.0, nine["median_ms"])
        self.assertEqual(1052.0, nine["max_ms"])
        self.assertEqual(0, agg["per_gpio"][6]["not_spun_up"])

    def test_the_printed_lines_use_the_key_value_style(self):
        lines = start_kpi.format_lines(start_kpi.aggregate(self.batch()))
        self.assertTrue(lines[0].startswith("START_KPI runs=8 start_ok=6 start_ok_pct=75 "))
        self.assertIn("spin_up_limit_ms=500 not_started=1", lines[0])
        self.assertIn("instrument_problem=1", lines[0])
        self.assertEqual(5, len(lines))
        self.assertTrue(lines[4].startswith("SPIN_UP gpio=9 starts=8 not_spun_up=1 median_ms=160 "))

    def test_an_empty_batch_prints_dashes_not_a_crash(self):
        lines = start_kpi.format_lines(start_kpi.aggregate([]))
        self.assertIn("runs=0 start_ok=0 start_ok_pct=-", lines[0])


if __name__ == "__main__":
    unittest.main()
