# scripts/arming_stats.py's listening periods, on synthetic capture timelines:
# the BUG-002 bisection is judged by these counts, so what counts as accepted,
# rejected or undecided is pinned here. A timeline is one (time_ms, label,
# throttle) per capture, as classify_reply_timeline.timeline() makes them.

import sys
import unittest
from pathlib import Path

import fakes  # noqa: F401  puts driver/ on sys.path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from arming_stats import listening_periods  # noqa: E402

TICK_MS = 0.75


def captures(label, start_ms, end_ms):
    """One capture every TICK_MS from start_ms up to end_ms, all with `label`."""
    count = int((end_ms - start_ms) / TICK_MS)
    return [(start_ms + k * TICK_MS, label, 0) for k in range(count)]


def summary(events):
    return [(kind, outcome) for kind, outcome, _, _ in listening_periods(events)]


class ListeningPeriodsTest(unittest.TestCase):
    def test_accepted_at_first_contact(self):
        events = captures("echo", 0, 75) + captures("stop", 75, 3000)
        self.assertEqual(summary(events), [("first contact", "accepted")])

    def test_rejected_at_first_contact_then_accepted_after_the_reset(self):
        events = (captures("echo", 0, 1857) + captures("low", 1857, 2457)
                  + captures("echo", 2457, 2540) + captures("stop", 2540, 5000))
        self.assertEqual(summary(events), [("first contact", "rejected"),
                                           ("after reset", "accepted")])

    def test_rejection_is_timed_from_the_period_start(self):
        events = (captures("echo", 0, 1857) + captures("low", 1857, 2457)
                  + captures("echo", 2457, 2540) + captures("stop", 2540, 5000))
        _, _, start, reset_after = listening_periods(events)[0]
        self.assertEqual(start, 0.0)
        self.assertAlmostEqual(reset_after, 1857, delta=TICK_MS)

    def test_mid_tune_at_arm_counts_as_after_tune(self):
        events = captures("low", 0, 400) + captures("echo", 400, 480) + captures("stop", 480, 3000)
        self.assertEqual(summary(events), [("after tune", "accepted")])

    def test_a_log_ending_inside_a_period_is_undecided(self):
        events = captures("echo", 0, 1857) + captures("low", 1857, 2457) + captures("echo", 2457, 3500)
        self.assertEqual(summary(events), [("first contact", "rejected"),
                                           ("after reset", "undecided")])

    def test_a_late_first_reply_is_not_an_accept(self):
        # Replies starting a second in are not the ESC's bidirectional latch
        events = captures("echo", 0, 1000) + captures("stop", 1000, 3000)
        self.assertEqual(summary(events), [("first contact", "undecided")])


if __name__ == "__main__":
    unittest.main()
