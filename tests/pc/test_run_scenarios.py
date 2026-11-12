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

    def test_the_start_kpi_of_the_run_is_two_more_fields(self):
        _, f = self.parse(run_record(1, 5, "a.json", "pass", "clean", 2112, "s", 1, 0, 0, "pass", "slow", 1052.4))
        self.assertEqual((f["start"], f["spin_up_max_ms"]), ("slow", "1052"))
        _, g = self.parse(run_record(1, 5, "a.json", "pass", "clean", 2112, "s", 1, 0, 0, "pass"))
        self.assertNotIn("start", g)

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


class SessionVerdictTotalsTest(unittest.TestCase):
    def test_a_batch_of_passing_arms_still_shows_the_failed_sessions(self):
        import run_scenarios
        totals = run_scenarios.session_verdict_totals(["pass", "fail", "unknown", "pass", None, "fail"])
        self.assertEqual({"runs": 6, "pass": 2, "fail": 2, "unknown": 1, "no_session": 1}, totals)
        line = run_scenarios.fields("SESSION_VERDICTS", **totals)
        self.assertEqual("SESSION_VERDICTS runs=6 pass=2 fail=2 unknown=1 no_session=1", line)

    def test_the_batch_end_prints_the_totals_and_keeps_the_arming_lines(self):
        import io
        import json
        import tempfile
        from contextlib import redirect_stdout
        import run_scenarios
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(__import__("shutil").rmtree, folder, True)
        (folder / "s1").mkdir()
        (folder / "s1" / "summary.json").write_text(json.dumps({"verdict": "fail"}), encoding="utf-8")
        output = "Uploaded 3, failed 0.\nArmed after 2100ms.\nOutcome: completed\n"
        old_session, old_captures = run_scenarios.run_session.run_session, run_scenarios.run_session.CAPTURES
        run_scenarios.run_session.run_session = lambda *a, **k: ("s1", output, 0, {"verdict": "fail"})
        run_scenarios.run_session.CAPTURES = folder
        self.addCleanup(setattr, run_scenarios.run_session, "run_session", old_session)
        self.addCleanup(setattr, run_scenarios.run_session, "CAPTURES", old_captures)
        old = sys.argv
        sys.argv = ["run_scenarios.py", "--scenario", str(Path(__file__)), "--rounds", "2", "--load", "none"]
        self.addCleanup(setattr, sys, "argv", old)
        out = io.StringIO()
        with redirect_stdout(out):
            run_scenarios.main()
        text = out.getvalue()
        self.assertIn("verdict=pass outcome=clean", text)           # the arming outcome keeps its meaning
        self.assertIn("session_verdict=fail", text)
        self.assertIn("RESULT scenario=test_run_scenarios.py attempts=2 pass=2 fail=0", text)
        self.assertIn("SESSION_VERDICTS runs=2 pass=0 fail=2 unknown=0 no_session=0", text)


class BatchFailureTest(unittest.TestCase):
    def patch_session(self, fn):
        import run_scenarios
        self.addCleanup(setattr, run_scenarios.run_session, "run_session", run_scenarios.run_session.run_session)
        run_scenarios.run_session.run_session = fn
        return run_scenarios

    def test_a_run_session_that_gives_up_is_an_error_not_a_crash(self):
        def give_up(*a, **k):
            raise SystemExit("the observer never reported that it is recording")
        outcome, session, output, verdict = self.patch_session(give_up).run_once("x.json")
        self.assertEqual("error", outcome)
        self.assertIn("observer never reported", output)

    def test_a_wingman_that_did_not_start_is_an_error_not_a_deploy_failure_and_is_not_retried(self):
        calls = []

        def wingman_missing(*a, **k):
            calls.append(1)
            return "s1", "the observer never reported that it is recording\n", 1, {
                "verdict": "fail", "start_failure": {"who": "wingman", "message": "x"}}
        run_scenarios = self.patch_session(wingman_missing)
        outcome, session, output, verdict = run_scenarios.run_attempt("x.json")
        self.assertEqual(("error", "s1", "fail"), (outcome, session, verdict))
        self.assertEqual(1, len(calls))

    def test_a_main_pico_that_did_not_start_keeps_its_deploy_classification(self):
        def upload_failed(*a, **k):
            return "s1", "Uploaded 0, failed 17.\n", 1, {"verdict": "fail", "start_failure": {"who": "main_pico", "message": "x"}}
        outcome, _, _, verdict = self.patch_session(upload_failed).run_once("x.json")
        self.assertEqual(("deploy_failed", "fail"), (outcome, verdict))

    def test_the_batch_reports_such_attempts_as_failed_sessions_not_as_no_session(self):
        import io
        import json
        import tempfile
        from contextlib import redirect_stdout
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(__import__("shutil").rmtree, folder, True)
        (folder / "s1").mkdir()
        (folder / "s1" / "summary.json").write_text(json.dumps({"verdict": "fail"}), encoding="utf-8")
        run_scenarios = self.patch_session(lambda *a, **k: ("s1", "no wingman\n", 1, {
            "verdict": "fail", "start_failure": {"who": "wingman", "message": "x"}}))
        self.addCleanup(setattr, run_scenarios.run_session, "CAPTURES", run_scenarios.run_session.CAPTURES)
        run_scenarios.run_session.CAPTURES = folder
        old = sys.argv
        sys.argv = ["run_scenarios.py", "--scenario", str(Path(__file__)), "--rounds", "2", "--load", "none"]
        self.addCleanup(setattr, sys, "argv", old)
        out = io.StringIO()
        with redirect_stdout(out):
            run_scenarios.main()
        text = out.getvalue()
        self.assertIn("outcome=error session=s1", text)
        self.assertIn("session_verdict=fail", text)
        self.assertIn("SESSION_VERDICTS runs=2 pass=0 fail=2 unknown=0 no_session=0", text)

    def test_the_batch_stops_after_repeated_problems(self):
        import io
        from contextlib import redirect_stdout
        calls = []

        def give_up(*a, **k):
            calls.append(1)
            raise SystemExit("boom")
        run_scenarios = self.patch_session(give_up)
        scenario = Path(__file__)
        out = io.StringIO()
        old = sys.argv
        sys.argv = ["run_scenarios.py", "--scenario", str(scenario), "--rounds", "10", "--load", "none"]
        self.addCleanup(setattr, sys, "argv", old)
        with redirect_stdout(out):
            run_scenarios.main()
        self.assertEqual(run_scenarios.MAX_CONSECUTIVE_PROBLEMS, len(calls))
        self.assertIn("STOPPED", out.getvalue())


if __name__ == "__main__":
    unittest.main()
