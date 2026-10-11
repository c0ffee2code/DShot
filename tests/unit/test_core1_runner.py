# Core1Runner's pacing (tests/harness/core1_runner.py), exercised by calling
# loop() directly rather than through a real thread - loop() has no _thread
# dependency of its own, only start()/stop() do.
#
# BUG-002 context (bug-reports/BUG-002-...md's Betaflight comparison): a
# positive interval_us holds a running deadline, anchored to the previous one,
# so a slow call is absorbed into that one interval rather than shifting every
# later call by the same amount. There used to be a second mode here (a fixed
# sleep added AFTER each call, so a slow call's cost compounded into every
# later gap) - removed 2026-10-09 by the user's explicit direction: every
# constant tuned for it needed re-tuning the moment anything changed update()'s
# own cost, even though it was, for a while, the only configuration that ever
# reached 100% clean arms on the bench. These tests prove the one remaining
# mechanism's behavior on the fake clock, not on real timing.

import sys
import unittest
from pathlib import Path

HARNESS = Path(__file__).resolve().parents[1] / "harness"
sys.path.insert(0, str(HARNESS))

import fakes  # noqa: E402,F401
from fakes import Clock  # noqa: E402
from core1_runner import Core1Runner  # noqa: E402

INTERVAL_US = 1000


class SchedulingTestCase(unittest.TestCase):
    def setUp(self):
        Clock.reset()

    def run_calls(self, runner, count, work_us=None):
        """Run loop() for exactly `count` calls, recording utime.ticks_us() at
        each call. `work_us`, if given, is the simulated cost of one call -
        either a single int (every call costs the same) or a list indexed by
        call number (0-based) for a per-call cost."""
        import utime
        timestamps = []
        calls = [0]

        def fake_update():
            timestamps.append(utime.ticks_us())
            calls[0] += 1
            if work_us is not None:
                cost = work_us[calls[0] - 1] if isinstance(work_us, list) else work_us
                Clock.advance_us(cost)
            if calls[0] >= count:
                runner.running = False

        runner.update = fake_update
        runner.running = True
        runner.loop()
        return timestamps

    def gaps(self, timestamps):
        return [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]

    def test_keeps_period_regular_when_calls_are_cheap(self):
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        timestamps = self.run_calls(runner, 5, work_us=0)
        self.assertEqual(self.gaps(timestamps), [INTERVAL_US] * 4)
        self.assertEqual(runner.schedule_behind_count, 0)

    def test_absorbs_a_slow_call_without_padding_or_lingering_shift(self):
        # Call 2 (index 1) takes 5x the interval. The gap spanning it can't be
        # shorter than the call itself (that's physically what "slow" means) -
        # what this mechanism buys is: no permanent phase shift carried into
        # later calls once it catches back up. It does land on the next
        # period boundary after the call finishes, not the instant it
        # finishes - that's deliberate (2026-10-07 fix): firing the very next
        # call with zero gap instead would be a burst, which is what this
        # mode exists to avoid. See BUG-002.md's "deadline underperforms the
        # pad" entry for why that distinction is not cosmetic - it measurably
        # hurt arming reliability on the bench.
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        slow_call = 5 * INTERVAL_US
        timestamps = self.run_calls(runner, 4, work_us=[0, slow_call, 0, 0])
        gaps = self.gaps(timestamps)
        self.assertEqual(gaps[0], INTERVAL_US)
        # Lands on the next period boundary after the call, not at the
        # instant it finishes - one extra interval beyond the call's own
        # cost, never a zero/near-zero gap
        self.assertEqual(gaps[1], slow_call + INTERVAL_US)
        self.assertEqual(gaps[2], INTERVAL_US)  # no lingering shift
        # The one real gap in this mechanism (see the user's own review): it
        # can only absorb a slow call by falling behind and resyncing, which
        # is silent on its own - this counter is what makes it observable.
        self.assertEqual(runner.schedule_behind_count, 1)

    def test_schedule_behind_count_counts_every_resync_not_just_the_first(self):
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        slow_call = 2 * INTERVAL_US
        self.run_calls(runner, 5, work_us=[0, slow_call, 0, slow_call, 0])
        self.assertEqual(runner.schedule_behind_count, 2)

    def test_behind_at_us_records_absolute_timestamp_of_each_fall_behind(self):
        # BUG-002, 2026-10-08: testing whether rare fall-behind events land
        # inside AM32's narrow early-boot window needs to know WHEN each one
        # happened, not just that it happened.
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        self.run_calls(runner, 3, work_us=[0, 2500, 0])
        self.assertEqual(runner.schedule_behind_count, 1)
        # Clock starts at 0 (setUp): call 1 at t=0 (cost 0) -> sleeps to
        # t=1000. Call 2 at t=1000, cost 2500 -> clock reaches 3500, which is
        # where "fell behind" is detected and recorded.
        self.assertEqual(runner.behind_at_us[0], 3500)

    def test_behind_at_us_stops_recording_once_capacity_is_full_but_count_keeps_going(self):
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        runner.BEHIND_AT_CAPACITY = 2
        from array import array
        runner.behind_at_us = array('I', [0, 0])
        slow_call = 2 * INTERVAL_US
        costs = [0, slow_call, 0, slow_call, 0, slow_call, 0]  # 3 fall-behinds, capacity 2
        self.run_calls(runner, len(costs), work_us=costs)
        self.assertEqual(runner.schedule_behind_count, 3)
        self.assertEqual(len(runner.behind_at_us), 2)  # unchanged - no overflow, no error

    def test_never_fires_a_zero_gap_burst_after_falling_behind(self):
        # The actual bug this file's 2026-10-07 fix closes: resyncing to
        # "now" instead of skipping forward by whole periods made the call
        # right after every fall-behind a back-to-back, zero-gap burst -
        # found on the bench as this mechanism scoring far worse than the
        # (now-removed) pad mode that never bursts (BUG-002.md's "deadline
        # underperforms the pad"). No gap may ever be shorter than one
        # interval, no matter how many calls fall behind or by how much.
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        costs = [0, 3 * INTERVAL_US, 0, int(1.5 * INTERVAL_US), 0, 10 * INTERVAL_US, 0]
        timestamps = self.run_calls(runner, len(costs), work_us=costs)
        for gap in self.gaps(timestamps):
            self.assertGreaterEqual(gap, INTERVAL_US)

    def test_holds_exact_period_through_small_per_call_jitter(self):
        # Real per-call cost (RX drain, bookkeeping) varies call to call but
        # normally stays well under the interval - this mechanism must hide
        # all of that variability from the achieved spacing, not just a single
        # large outlier.
        runner = Core1Runner(None, interval_us=INTERVAL_US)
        costs = [0, 200, 400, 100, 300, 0]
        timestamps = self.run_calls(runner, len(costs), work_us=costs)
        self.assertEqual(self.gaps(timestamps), [INTERVAL_US] * (len(costs) - 1))

    def test_zero_interval_means_no_fixed_pacing_at_all(self):
        # MotorGroup.UPDATE_INTERVAL_US's own meaning: back to back, as fast
        # as update() returns - no deadline, so nothing can fall behind.
        runner = Core1Runner(None, interval_us=0)
        costs = [0, 200, 5000, 100, 0]
        timestamps = self.run_calls(runner, len(costs), work_us=costs)
        # gap i is exactly call i's own cost - nothing added, nothing absorbed
        self.assertEqual(self.gaps(timestamps), costs[:-1])
        self.assertEqual(runner.schedule_behind_count, 0)

    def test_zero_interval_does_not_hang(self):
        # Real regression, found by review 2026-10-08, before it ever ran on
        # the bench: fixed_schedule defaulting to True (2026-10-07, before
        # that mode was removed entirely 2026-10-09) made this reachable by
        # any scenario that leaves core1_interval_us unset (bins_off.json,
        # the CLAUDE.md usage example both do). The deadline branch's inner
        # catch-up loop never advances next_due when interval_us is 0, so
        # without a guard this test never returns - it would hang the whole
        # suite, not just fail an assertion.
        runner = Core1Runner(None, interval_us=0)
        timestamps = self.run_calls(runner, 5, work_us=0)
        self.assertEqual(len(timestamps), 5)


if __name__ == "__main__":
    unittest.main()
