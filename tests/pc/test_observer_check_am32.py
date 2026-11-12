# Tests of tests/bench/wingman/tools/observer_check_am32.py: every check must pass on a well-formed recording and fail
# when the thing it guards is broken. Recordings are built here as edge lists (units of 0.1us) and put
# through observer_edge_log.analyse, exactly as a real one is.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import observer_check_am32 as check
import observer_edge_log
import gcr_decode

PERIOD = 16.667
ONE_LOW = 12.5
ZERO_LOW = 6.25
SPACING = 10450                     # 1045us


def word_for(throttle):
    value = throttle << 1
    return (value << 4) | ((~(value ^ (value >> 4) ^ (value >> 8))) & 0xF)


def frame_edges(throttle, start, zero_low=ZERO_LOW, word=None):
    word = word_for(throttle) if word is None else word
    edges = []
    t = start
    for bit in range(15, -1, -1):
        low = ONE_LOW if (word >> bit) & 1 else zero_low
        edges.append((t, 0))
        edges.append((t + low, 1))
        t += PERIOD
    return edges


def reply_edges(frame, start):
    bit = 13.333
    edges = []
    level = 1
    for k in range(21):
        b = (frame >> (20 - k)) & 1
        if b != level:
            edges.append((start + k * bit, b))
            level = b
    if level == 0:
        edges.append((start + 21 * bit, 1))
    return edges


def recording(throttles, starts=None, replies=None, mutate=None):
    """throttles: one value per frame; starts: frame start times (default 1045us apart);
    replies: {frame index: 21-bit reply frame}; mutate: {frame index: dict of frame_edges keyword arguments}."""
    edges = []
    for i, throttle in enumerate(throttles):
        start = starts[i] if starts else 100 + i * SPACING
        edges += frame_edges(throttle, start, **((mutate or {}).get(i, {})))
        if replies and i in replies:
            edges += reply_edges(replies[i], start + 16 * PERIOD + 300)
    return observer_edge_log.analyse(edges)


def by_name(results):
    return {r.name.split(" ")[0] + " " + r.name.split(" ")[1]: r for r in results}


def result(results, fragment):
    matches = [r for r in results if fragment in r.name]
    assert len(matches) == 1, (fragment, [r.name for r in results])
    return matches[0]


NOT_RUNNING = gcr_decode.AM32_NOT_RUNNING_FRAME


class InputSideTests(unittest.TestCase):
    def good(self, **kwargs):
        return recording([0] * 120 + [60] * 40, **kwargs)

    def test_a_clean_recording_passes_every_input_check(self):
        results = check.run_checks(self.good(), from_first_frame=True)
        for fragment in ("reads every bit", "frame length is inside", "classify as DSHOT600", "throttle values", "no gap"):
            self.assertEqual(result(results, fragment).status, check.PASS, fragment)

    def test_a_pulse_over_the_threshold_flips_a_bit(self):
        # every 0-bit of frame 20 gets a 0.90us pulse: above AM32's span/32 threshold (~0.82us) but below the midpoint
        # our own decoder uses (0.94us), so the two readers disagree
        records = recording([0] * 150, mutate={20: {"zero_low": 9.0}})
        self.assertEqual(result(check.run_checks(records), "reads every bit").status, check.FAIL)

    def test_a_stretched_frame_leaves_the_learned_window(self):
        records = recording([0] * 150, mutate={30: {"word": 0xFFFF}})      # all ones: 15 periods + a long last pulse
        records[30]["tx_edges"] = [(t + (i * 1.2 if i > 15 else 0), lv) for i, (t, lv) in enumerate(records[30]["tx_edges"])]
        status = result(check.run_checks(records), "frame length is inside").status
        self.assertEqual(status, check.FAIL)

    def test_command_values_are_flagged(self):
        records = recording([0] * 40 + [20] * 3)
        self.assertEqual(result(check.run_checks(records), "throttle values").status, check.FAIL)

    def test_signal_loss_thresholds(self):
        starts = [100 + i * SPACING for i in range(60)]
        starts[30:] = [s + 6_000_000 for s in starts[30:]]                  # a 600ms gap
        status = result(check.run_checks(recording([0] * 60, starts=starts)), "no gap").status
        self.assertEqual(status, check.WARN)
        starts[30:] = [s + 20_000_000 for s in starts[30:]]                 # 2s more
        status = result(check.run_checks(recording([0] * 60, starts=starts)), "no gap").status
        self.assertEqual(status, check.FAIL)

    def test_arming_gate(self):
        early = recording([0] * 30 + [60] * 10)                              # throttle after 31ms
        self.assertEqual(result(check.run_checks(early, from_first_frame=True), "to arm").status, check.FAIL)
        self.assertEqual(result(check.run_checks(early, from_first_frame=False), "to arm").status, check.INFO)
        late = recording([0] * 1100 + [60] * 10)
        self.assertEqual(result(check.run_checks(late, from_first_frame=True), "to arm").status, check.PASS)

    def test_unidirectional_has_no_reply_checks(self):
        results = check.run_checks(self.good(), unidirectional=True)
        self.assertEqual(results[-1].status, check.NA)


class ReplySideTests(unittest.TestCase):
    def with_replies(self, answered, count=400, from_first_frame=True):
        replies = {i: NOT_RUNNING for i in answered}
        return check.run_checks(recording([0] * count, replies=replies), from_first_frame=from_first_frame)

    def test_exactly_the_latch_and_the_arming_silence_are_marked_as_the_real_ESC_own_behaviour(self):
        results = self.with_replies(range(110, 400))
        marked = sorted(r.name for r in results if r.esc_only)
        self.assertEqual(2, len(marked))
        self.assertTrue(any("bidirectional latch" in name for name in marked))
        self.assertTrue(any("arming silence" in name for name in marked))
        self.assertFalse(any(r.esc_only for r in check.run_checks(recording([0] * 60), unidirectional=True)))

    def test_latch_needs_101_frames(self):
        late = self.with_replies(range(110, 400))
        self.assertEqual(result(late, "bidirectional latch").status, check.PASS)
        early = self.with_replies(range(50, 400))
        self.assertEqual(result(early, "bidirectional latch").status, check.WARN)
        self.assertEqual(result(self.with_replies(range(50, 400), from_first_frame=False), "bidirectional latch").status, check.INFO)

    def test_the_arming_tune_gap_is_explained_and_other_silence_is_not(self):
        answered = [i for i in range(110, 1000) if not 200 <= i < 487]
        results = check.run_checks(recording([0] * 1000, replies={i: NOT_RUNNING for i in answered}), from_first_frame=True)
        coverage = result(results, "an answer to every frame")
        self.assertEqual(coverage.status, check.PASS, coverage.detail)
        self.assertIn("arming tune", coverage.detail)
        answered = [i for i in range(110, 1000) if not 200 <= i < 300]       # 100ms: matches nothing in the source
        results = check.run_checks(recording([0] * 1000, replies={i: NOT_RUNNING for i in answered}), from_first_frame=True)
        self.assertEqual(result(results, "an answer to every frame").status, check.WARN)

    def test_reply_integrity(self):
        results = self.with_replies(range(110, 400))
        self.assertEqual(result(results, "CRC-valid").status, check.PASS)
        broken = {i: NOT_RUNNING for i in range(110, 400)}
        broken[200] = NOT_RUNNING ^ 0b10000                                    # one flipped line level
        results = check.run_checks(recording([0] * 400, replies=broken), from_first_frame=True)
        self.assertIn(result(results, "CRC-valid").status, (check.WARN, check.FAIL))

    def test_a_reply_bit_time_off_the_f421_figure_is_a_warning(self):
        results = self.with_replies(range(110, 400))                           # the test builds 1.3333us bits, not F421's 1.2833
        timing = result(results, "reply bit time")
        self.assertEqual(timing.status, check.WARN)
        self.assertIn("esc_buddy", timing.detail)

    def test_the_f421_reply_bit_time_passes_at_both_speeds(self):
        for speed, units in ((600, 12.83), (300, 25.67)):
            records = [{"reply": {"bit_units": units, "delay": 231}} for _ in range(20)]
            timing = check.check_reply_timing(records, speed)
            self.assertEqual(timing.status, check.PASS, speed)
            self.assertAlmostEqual(timing.values["off_pct"], 0, delta=0.1)

    def test_start_and_stuck_rotor_report(self):
        throttles = [0] * 300 + [60] * 700
        replies = {i: NOT_RUNNING for i in range(110, 1000)}
        running = (3 << 9) | 325
        number = (running << 4) | gcr_decode.crc_inverted(running)
        symbols = 0
        for shift in (12, 8, 4, 0):
            symbols = (symbols << 5) | gcr_decode.GCR_ENCODE_TABLE[(number >> shift) & 0xF]
        level = 0
        frame = 0
        for k in range(19, -1, -1):
            level ^= (symbols >> k) & 1
            frame = (frame << 1) | level
        for i in range(310, 700):
            replies[i] = frame
        results = check.run_checks(recording(throttles, replies=replies), from_first_frame=True)
        text = result(results, "stuck-rotor").detail
        self.assertIn("first running reply", text)
        self.assertIn("not-running", text)                                      # it went back to idle at frame 700


if __name__ == "__main__":
    unittest.main()
