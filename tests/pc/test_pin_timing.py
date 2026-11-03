# tests/bench/pin_timing.py: frame timing from a pin recording's edge list, on synthetic edges.
# The self-test (tests/bench/pio_and_cores/test_pin_sampler.py) runs the same functions on the Pico.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))

import pin_timing as pt  # noqa: E402

PERIOD = 1045


def frame(t0, reply=False):
    """One transmitter frame: 16 alternating edges 1.7us apart, the first one falling; optionally the ESC's
    reply 55us after the last edge (21 edges 1.4us apart)."""
    edges = [(t0 + int(i * 1.7), i % 2) for i in range(16)]
    if reply:
        start = edges[-1][0] + 55
        edges += [(start + int(i * 1.4), i % 2) for i in range(21)]
    return edges


def train(count, offset=0, skip=(), reply=False):
    edges = []
    for k in range(count):
        if k not in skip:
            edges += frame(offset + k * PERIOD, reply)
    return edges


class FrameStartsTest(unittest.TestCase):
    def test_every_frame_start_is_found_once(self):
        starts, odd = pt.frame_starts(train(20))
        self.assertEqual(starts, [k * PERIOD for k in range(20)])
        self.assertEqual(odd, 0)

    def test_a_reply_after_the_frame_does_not_count_as_another_frame(self):
        starts, _ = pt.frame_starts(train(20, reply=True))
        self.assertEqual(len(starts), 20)

    def test_a_burst_that_starts_rising_is_counted_as_odd(self):
        edges = [(0, 1), (2, 0), (4, 1)] + train(3, offset=PERIOD)
        _, odd = pt.frame_starts(edges)
        self.assertEqual(odd, 1)


class DurationTest(unittest.TestCase):
    def test_the_frame_length_stops_at_the_reply_pause(self):
        durations = pt.frame_durations(train(10, reply=True))
        self.assertEqual(set(durations), {25})


class SpacingTest(unittest.TestCase):
    def test_a_steady_train_has_the_period_as_its_mean_and_no_odd_gaps(self):
        stats = pt.spacing_stats(pt.frame_starts(train(50))[0], PERIOD)
        self.assertAlmostEqual(stats["mean_us"], PERIOD)
        self.assertEqual((stats["short"], stats["long"]), (0, 0))
        self.assertEqual(stats["p99_dev_us"], 0)

    def test_a_missing_frame_is_a_long_gap(self):
        stats = pt.spacing_stats(pt.frame_starts(train(50, skip={20}))[0], PERIOD)
        self.assertEqual(stats["long"], 1)
        self.assertEqual(stats["max_us"], 2 * PERIOD)

    def test_a_frame_squeezed_next_to_another_is_a_short_gap(self):
        edges = train(10) + frame(3 * PERIOD + 300)
        stats = pt.spacing_stats(pt.frame_starts(sorted(edges))[0], PERIOD)
        self.assertEqual(stats["short"], 1)

    def test_one_wobbling_frame_moves_the_deviation_but_hardly_the_mean(self):
        edges = train(50)
        shifted = [(t + 40, level) if PERIOD * 25 <= t < PERIOD * 26 else (t, level) for t, level in edges]
        stats = pt.spacing_stats(pt.frame_starts(shifted)[0], PERIOD)
        self.assertEqual(stats["p99_dev_us"], 40)
        self.assertLess(abs(stats["mean_us"] - PERIOD), 2)

    def test_too_few_frames_give_no_mean(self):
        self.assertIsNone(pt.spacing_stats([0], PERIOD)["mean_us"])


class PhaseTest(unittest.TestCase):
    def test_a_channel_that_starts_300us_later_in_every_pass_has_that_offset_every_time(self):
        a = pt.frame_starts(train(30))[0]
        b = pt.frame_starts(train(30, offset=300))[0]
        self.assertEqual(set(pt.phase_offsets(a, b)), {300})

    def test_nothing_pairs_up_when_the_other_channel_starts_first(self):
        self.assertEqual(pt.phase_offsets([1000, 2045], [10, 20]), [])


if __name__ == "__main__":
    unittest.main()
