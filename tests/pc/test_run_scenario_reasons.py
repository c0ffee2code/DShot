# The bail-out reasons of tests/bench/dshot_protocol/run_scenario.py (ADR-008): each states the measured
# value against its limit and nothing else, and the readers on the PC still pick the text up. The script
# runs a scenario when imported and needs the Pico's modules, so only its definitions are executed here.

import ast
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "bench"))
sys.path.insert(0, str(ROOT / "scripts"))

import fakes  # noqa: E402,F401
from decode_tally import OverspeedWatch, SpinWatch  # noqa: E402
import session_summary  # noqa: E402
import run_scenarios  # noqa: E402

WANTED = {"ABORTED", "NOT_RUNNING_LIMIT_MS", "BailOutError", "abort_reason", "check_no_real_erpm", "check_expect",
          "overspeed_reason", "arming_reason"}


def load_definitions():
    source = (ROOT / "tests" / "bench" / "dshot_protocol" / "run_scenario.py").read_text(encoding="utf-8")
    keep = []
    for node in ast.parse(source).body:
        names = [t.id for t in node.targets if isinstance(t, ast.Name)] if isinstance(node, ast.Assign) else [getattr(node, "name", None)]
        if any(name in WANTED for name in names):
            keep.append(node)
    namespace = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), "run_scenario.py", "exec"), namespace)
    return namespace


RS = load_definitions()


def reason_of(function, *args):
    try:
        function(*args)
    except Exception as error:
        return RS["abort_reason"](error)
    raise AssertionError("no bail-out")


class BailOutReasonTest(unittest.TestCase):
    def test_spin_bail_out_states_time_against_limit(self):
        watches = [SpinWatch(RS["NOT_RUNNING_LIMIT_MS"]) for _ in range(4)]
        throttles = [215, 0, 0, 0]
        RS["check_no_real_erpm"]([0], watches, throttles, 0)
        self.assertEqual("motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms",
                         reason_of(RS["check_no_real_erpm"], [0], watches, throttles, 2004))

    def test_overspeed_bail_out_states_value_against_limit(self):
        watch = OverspeedWatch()
        watch.note_throttle(215, 0)
        for _ in range(5):
            watch.note_erpm(91000.0)
        self.assertEqual("motor 2 throttle=215: 91000 eRPM in 5 decodes in a row > limit=64500 eRPM",
                         RS["overspeed_reason"](2, 215, 91000.0, watch))

    def test_max_gap_and_record_rate_keep_their_text(self):
        self.assertEqual("largest_gap=163.2ms > max_gap_ms=150",
                         reason_of(RS["check_expect"], {"max_gap_ms": 150}, 163200, 10, 1000))
        self.assertEqual("record rate=412.5/s < min_record_rate_hz=450",
                         reason_of(RS["check_expect"], {"min_record_rate_hz": 450}, 0, 8250, 20000))

    def test_the_runners_own_bail_outs_carry_no_exception_type_but_a_core_1_error_does(self):
        self.assertEqual("x", RS["abort_reason"](RS["BailOutError"]("x")))
        self.assertEqual("ValueError: a b", RS["abort_reason"](ValueError("a\nb".replace("\n", " "))))
        self.assertEqual("OSError: a b", RS["abort_reason"](OSError("a\r\nb")).replace("  ", " "))

    def test_a_refused_arming_is_still_recognised_by_run_scenarios_and_the_run_log_reader(self):
        reason = RS["arming_reason"](18000, [None])
        self.assertEqual("arming did not complete within 18000ms; per motor (replying_for_ms, last_reply_ms_ago), None = no reply: [None]",
                         reason)
        self.assertTrue(run_scenarios.REFUSED_RE.search("Outcome: " + RS["ABORTED"] + reason))
        traceback = "Traceback (most recent call last):\n  File \"run_scenario.py\", line 1\nBailOutError: " + reason + "\n"
        self.assertEqual((None, "BailOutError: " + reason), session_summary.parse_run_log(traceback))

    def test_the_summary_shows_the_reason_without_a_type_prefix(self):
        outcome = RS["ABORTED"] + "motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms"
        proof = session_summary.proof_finished({"outcome": outcome, "error": None}, False)
        self.assertEqual("failed", proof["status"])
        self.assertIn("the Pico aborted the run: motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms", str(proof))
        self.assertNotIn("Error", str(proof))


if __name__ == "__main__":
    unittest.main()
