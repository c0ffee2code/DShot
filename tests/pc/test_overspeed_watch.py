# tests/bench/decode_tally.py's OverspeedWatch: the bench's over-speed guard, which stops a run when a
# motor reports far more eRPM than its commanded throttle gives.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))

import fakes  # noqa: E402,F401
from decode_tally import OverspeedWatch  # noqa: E402


def feed(watch, throttle, erpms, start_ms=0, step_ms=21):
    """Feed one decode per step (every 20th reply of a motor answering once per 1045us period is about 21ms apart); returns the index of the first reading that tripped it, or None."""
    for i, erpm in enumerate(erpms):
        watch.note_throttle(throttle, start_ms + i * step_ms)
        if watch.note_erpm(erpm):
            return i
    return None


class OverspeedWatchTest(unittest.TestCase):
    def test_a_healthy_motor_never_trips(self):
        # this bench: throttle 60 reads about 8,400 eRPM (at most 12,500), throttle 250 about 62,000
        self.assertIsNone(feed(OverspeedWatch(), 60, [8400, 12500, 8300] * 20))
        self.assertIsNone(feed(OverspeedWatch(), 250, [62000, 70000, 61000] * 20))

    def test_a_motor_running_far_above_its_throttle_trips_after_five_decodes(self):
        # the 2026-10-19 channel 4 start: throttle 60, median 23,292 eRPM
        self.assertEqual(feed(OverspeedWatch(), 60, [23000] * 10), 4)

    def test_one_stray_reading_does_not_trip(self):
        # BUG-009's single 30,000,000 eRPM reading
        self.assertIsNone(feed(OverspeedWatch(), 60, [8400] * 5 + [30_000_000] + [8400] * 5))

    def test_the_count_restarts_after_a_normal_reading(self):
        readings = [23000] * 4 + [8400] + [23000] * 4
        self.assertIsNone(feed(OverspeedWatch(), 60, readings))

    def test_a_motor_that_is_not_commanded_is_never_judged(self):
        self.assertIsNone(feed(OverspeedWatch(), 0, [60000] * 20))

    def test_a_motor_slowed_down_is_given_time_to_coast(self):
        watch = OverspeedWatch()
        self.assertIsNone(feed(watch, 250, [62000] * 5))
        # the throttle drops to 60 but the motor is still at 40,000 eRPM: not judged against 60 yet
        self.assertIsNone(feed(watch, 60, [40000] * 10, start_ms=100))
        # two seconds later it must have slowed down
        self.assertEqual(feed(watch, 60, [40000] * 10, start_ms=2500), 4)

    def test_the_limit_follows_the_highest_recent_throttle(self):
        watch = OverspeedWatch()
        watch.note_throttle(60, 0)
        self.assertEqual(watch.limit_erpm(), 18000)
        watch.note_throttle(250, 10)
        self.assertEqual(watch.limit_erpm(), 75000)


if __name__ == "__main__":
    unittest.main()
