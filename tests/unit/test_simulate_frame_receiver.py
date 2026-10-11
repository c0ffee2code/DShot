# scripts/simulate_frame_receiver.py's raw-sample reconstruction math
# (sample_cycle, find_edges, reconstruct_frame) - moved here 2026-10-11 along
# with the code itself, from driver/gcr_decode.py's old analyze_capture()
# path, once its one production caller (tools/calibrate_bidir_rx.py, via the
# now-removed sample receiver) was deleted. This machinery has no production
# caller any more; it backs scripts/simulate_frame_receiver.py's own PC-side
# verification of the frame receiver's PIO program, and is exercised here as
# plain Python (no `fakes`/hardware stand-ins needed - it was always pure
# integer math, never hardware-dependent itself).
#
# Two kinds of evidence, same as before the move:
#   - Real captures taken on the bench, with the numbers an independent PC-side
#     decoder (scripts/dshot_bidir_decode.py) produced for them.
#   - A synthetic reply: any 16-bit value is GCR-encoded, turned into the pin's
#     waveform and sampled the way the old sample receiver sampled it (a
#     sample every 2 cycles, 2 extra cycles at each 32-sample seam), then
#     decoded. That covers every value, not only the few the bench happened
#     to produce, and the trailing bits that merge into the idle level.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "driver"))
sys.path.insert(0, str(ROOT / "scripts"))

import fakes  # noqa: F401  puts driver/ on sys.path for utime etc.
from gcr_decode import decode, check_crc, decode_result, FRAME_LENGTH_BITS
from simulate_frame_receiver import sample_cycle, find_edges, reconstruct_frame
from dshot_profiles import BIDIR_PROFILES, DSHOT_SPEEDS

# The GCR code AM32 uses (specification/AM32.md#11), written out here so the
# encoder in this file does not share a table with the decoder under test.
GCR_CODE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]

# (capture words, 16-bit reply number, eRPM) as decoded by scripts/dshot_bidir_decode.py.
# Captured on the bench with the profiles' tuned receiver clocks.
REAL_DSHOT300 = [
    ([0x1f00ff8, 0x78003f, 0xfc001fff, 0xffffffff], 55593, 2332.1),
    ([0x1ff00780, 0x783ffe0, 0x1fe0fff, 0xffffffff], 21291, 49019.6),
    ([0x1ff00f80, 0x787ffc1, 0xfc01ffff, 0xffffffff], 21306, 48859.9),
    ([0x1ff0f07f, 0x7fc3fe, 0x3c001fff, 0xffffffff], 30160, 21490.0),
    ([0x1ff0f07f, 0x807fc001, 0xc1e00fff, 0xffffffff], 30100, 21739.1),
    ([0x1ff0f87f, 0x7c3fe0, 0x3fe00fff, 0xffffffff], 30085, 21802.3),
    ([0xf0f807, 0x87803c1e, 0x3e000fff, 0xffffffff], 65520, 917.3),
]
REAL_DSHOT600 = [
    ([0xf007f8, 0x7ffc01f, 0xc001ffff, 0xffffffff], 55970, 2200.7),
    ([0x1ff00f80, 0x787ffc0, 0x3fe1fff, 0xffffffff], 21291, 49019.6),
    ([0x1ff00f80, 0x787ffc1, 0xfc01ffff, 0xffffffff], 21306, 48859.9),
    ([0x1ff0f80, 0x7f83c01, 0xc001ffff, 0xffffffff], 45634, 6421.2),
    ([0x1ff0f87f, 0x807fc01f, 0xc01e0fff, 0xffffffff], 30119, 21676.3),
    ([0x1ff0f87f, 0x807fc001, 0xc1e00fff, 0xffffffff], 30100, 21739.1),
    ([0x1ff0f87f, 0x807c3fe0, 0x3fe00fff, 0xffffffff], 30085, 21802.3),
]

PROFILES = {
    "DSHOT300": BIDIR_PROFILES[DSHOT_SPEEDS.DSHOT300],
    "DSHOT600": BIDIR_PROFILES[DSHOT_SPEEDS.DSHOT600],
}


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return plain ^ 0xF


def reply_number(data12):
    """The 16-bit number AM32 sends: 12 data bits and the inverted CRC."""
    return (data12 << 4) | crc_inverted(data12)


def synthetic_capture(number, period, edge_offset=0.0):
    """
    The 4 capture words for a reply carrying `number`, at `period` receiver cycles
    per bit. The marker edge falls `edge_offset` cycles before sample 0.
    """
    code = 0
    for shift in (12, 8, 4, 0):
        code = (code << 5) | GCR_CODE[(number >> shift) & 0xF]
    # The 20 code bits are the transitions of the line: a 1 flips the level. The
    # marker bit, first, is low.
    levels = [0]
    level = 0
    for shift in range(19, -1, -1):
        level ^= (code >> shift) & 1
        levels.append(level)

    words = []
    for word_index in range(4):
        word = 0
        for i in range(32):
            cycles_since_edge = sample_cycle(word_index * 32 + i) - 1 + edge_offset
            bit = int(cycles_since_edge // period)
            word = (word << 1) | (levels[bit] if bit < len(levels) else 1)  # idle is high
        words.append(word)
    return words


def analyze_capture(words, rx_clock_hz, expected_ratio):
    """
    Test-only reassembly of the old driver pipeline (find_edges ->
    reconstruct_frame -> decode_result), now that it is no longer one
    production function - each piece lives wherever it is actually used, and
    this helper just chains them the way the removed analyze_capture() did,
    for tests that want that whole-pipeline result shape.
    """
    edges = find_edges(words)
    if not edges:
        return None
    result = decode_result(reconstruct_frame(words, edges, expected_ratio))
    period_us = expected_ratio / rx_clock_hz * 1_000_000
    result["period_cycles"] = expected_ratio
    result["period_us"] = period_us
    result["bitrate_bps"] = int(1_000_000 / period_us) if period_us else None
    return result


class SampleTimingTest(unittest.TestCase):
    def test_samples_are_2_cycles_apart_with_2_extra_at_each_seam(self):
        self.assertEqual(sample_cycle(0), 1)
        self.assertEqual(sample_cycle(1), 3)
        self.assertEqual(sample_cycle(31), 63)
        self.assertEqual(sample_cycle(32), 67)  # 66 cycles per pass of 32
        self.assertEqual(sample_cycle(127), 3 * 66 + 1 + 31 * 2)


class FindEdgesTest(unittest.TestCase):
    def test_a_constant_line_has_no_edges(self):
        self.assertEqual(find_edges([0xFFFFFFFF] * 4), [])

    def test_the_first_sample_has_nothing_before_it(self):
        # sample 0 is 0, then all ones: one edge, at sample 1
        self.assertEqual(find_edges([0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]), [1])

    def test_edges_across_word_and_half_word_boundaries(self):
        # 1 x16 then 0 x16 (edge at 16), 0 -> 1 across the word seam (edge at 32),
        # and a last sample that differs (edge at 127)
        words = [0xFFFF0000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE]
        self.assertEqual(find_edges(words), [16, 32, 127])

    def test_alternating_samples_give_an_edge_at_every_sample(self):
        self.assertEqual(find_edges([0x55555555] * 4), list(range(1, 128)))

    def test_edges_come_out_in_ascending_order(self):
        words = [0x0F0F0F0F, 0xF0F0F0F0, 0x00FF00FF, 0xFF00FF00]
        edges = find_edges(words)
        self.assertEqual(edges, sorted(edges))


class RealCaptureTest(unittest.TestCase):
    def check_all(self, name, captures):
        profile = PROFILES[name]
        for words, number, erpm in captures:
            result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
            self.assertTrue(result["crc_ok"], hex(words[0]))
            self.assertEqual(result["full"], number)
            self.assertAlmostEqual(result["erpm"], erpm, places=1)

    def test_dshot300_captures_from_the_bench(self):
        self.check_all("DSHOT300", REAL_DSHOT300)

    def test_dshot600_captures_from_the_bench(self):
        self.check_all("DSHOT600", REAL_DSHOT600)

    def test_the_reported_bit_period_is_the_profiles(self):
        profile = PROFILES["DSHOT300"]
        result = analyze_capture(REAL_DSHOT300[0][0], profile["rx_speed"], profile["expected_ratio"])
        self.assertEqual(result["period_cycles"], profile["expected_ratio"])
        self.assertAlmostEqual(result["period_us"], profile["expected_ratio"] / profile["rx_speed"] * 1e6)

    def test_matches_analyze_frame_on_the_same_reply(self):
        """The reconstruction math here and gcr_decode.analyze_frame() must
        agree on a real reply, since dshot_bidir_rx_frame hands the CPU
        exactly the frame this file's reconstruct_frame() would also build
        from the same raw samples."""
        from gcr_decode import analyze_frame
        for words, number, erpm in REAL_DSHOT300:
            profile = PROFILES["DSHOT300"]
            edges = find_edges(words)
            frame = reconstruct_frame(words, edges, profile["expected_ratio"])

            result = analyze_frame(frame)

            self.assertTrue(result["crc_ok"], hex(words[0]))
            self.assertEqual(result["full"], number)
            self.assertAlmostEqual(result["erpm"], erpm, places=1)
            self.assertTrue(result["marker_ok"])


class SyntheticRoundTripTest(unittest.TestCase):
    def check(self, name, values, edge_offset):
        profile = PROFILES[name]
        for data12 in values:
            words = synthetic_capture(reply_number(data12), profile["expected_ratio"], edge_offset)
            result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
            self.assertIsNotNone(result, "%s data12=%d" % (name, data12))
            self.assertTrue(result["crc_ok"], "%s data12=%d" % (name, data12))
            self.assertEqual(result["data12"], data12)

    def test_every_12_bit_value_at_both_speeds(self):
        for name in PROFILES:
            self.check(name, range(4096), 0.0)

    def test_the_marker_edge_anywhere_within_the_first_sample_interval(self):
        # The edge is seen at a sample, so it can be up to 2 cycles earlier than
        # sample 0 says; every 13th value keeps this quick
        for name in PROFILES:
            for edge_offset in (0.5, 1.0, 1.5):
                self.check(name, range(0, 4096, 13), edge_offset)

    def test_a_bit_period_a_percent_off_still_decodes(self):
        # reconstruct_frame() rounds each run to the nearest whole number of
        # bits at the given period - no search - so a period that's slightly
        # off from the real one still decodes as long as rounding doesn't
        # cross a boundary.
        for name in PROFILES:
            profile = PROFILES[name]
            for scale in (0.99, 1.01):
                for data12 in range(0, 4096, 29):
                    words = synthetic_capture(reply_number(data12), profile["expected_ratio"] * scale)
                    result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
                    self.assertTrue(result and result["crc_ok"] and result["data12"] == data12,
                                    "%s scale %s data12=%d" % (name, scale, data12))

    def test_the_erpm_is_sixty_million_over_the_period(self):
        profile = PROFILES["DSHOT300"]
        exponent, mantissa = 2, 0x123  # period 0x123 << 2 = 1164 us
        data12 = (exponent << 9) | mantissa
        words = synthetic_capture(reply_number(data12), profile["expected_ratio"])
        result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
        self.assertAlmostEqual(result["erpm"], 60_000_000 / (mantissa << exponent))

    def test_a_zero_period_has_no_erpm(self):
        profile = PROFILES["DSHOT300"]
        words = synthetic_capture(reply_number(0), profile["expected_ratio"])
        result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
        self.assertTrue(result["crc_ok"])
        self.assertIsNone(result["erpm"])


class UnusableCaptureTest(unittest.TestCase):
    def test_a_dead_line_gives_none(self):
        profile = PROFILES["DSHOT300"]
        for words in ([0] * 4, [0xFFFFFFFF] * 4):
            self.assertIsNone(analyze_capture(words, profile["rx_speed"], profile["expected_ratio"]))

    def test_a_reply_with_a_corrupted_bit_is_not_reported_valid(self):
        profile = PROFILES["DSHOT300"]
        words = list(REAL_DSHOT300[2][0])
        words[1] ^= 0x00010000  # one sample flipped inside the frame
        result = analyze_capture(words, profile["rx_speed"], profile["expected_ratio"])
        self.assertFalse(result is not None and result["crc_ok"] and result["full"] == REAL_DSHOT300[2][1])


if __name__ == "__main__":
    unittest.main()
