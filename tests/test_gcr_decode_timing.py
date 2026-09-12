# On-device timing micro-benchmark for gcr_decode.analyze_capture() on the
# fixed-ratio decode path (estimate_bit_period_fixed) - the brute-force
# sweep (estimate_bit_period) this benchmark used to exercise was retired
# from driver/gcr_decode.py 2026-09-12 once both DSHOT300 and DSHOT600 had
# a verified fixed ratio (see decision/ADR-002-bidirectional-dshot.md's
# fixed-ratio RX sampling section) - there is no sweep left on-device to
# benchmark any more. No ESC, no motor, no arming - this only needs a Pico
# connected, not the bench hardware-testing gate.
#
# expected_ratio values below are hardcoded, not imported live from
# driver/dshot_profiles.py - this benchmark's whole job is a fixed, stable
# number to compare across runs, and a live read would let a future
# retune silently change what got measured without this file's own numbers
# (or the ADR's) being updated to match.
#
# Every group in both datasets below is CRC-valid: with ratio_tolerance=0.0
# (both live profiles use it), estimate_bit_period_fixed never calls
# _period_score at all, so raw_samples/find_edges/reconstruct_bits, not
# period search, dominate cost regardless of validity - unlike the old
# sweep benchmark, there's no reason to seek out an invalid group here.
# Groups are still picked with a spread of edge counts to keep those
# stages' cost representative.

import gcr_decode
from gcr_decode import analyze_capture
import utime

ITERATIONS_PER_GROUP = 50

# DSHOT300, K=9 - see driver/dshot_profiles.py's
# BIDIR_PROFILES[DSHOT_SPEEDS.DSHOT300], captures/2026-09-12_13-04-36.
RX_CLOCK_HZ_DSHOT300 = 3_375_000
EXPECTED_RATIO_DSHOT300 = 8.7069

GROUPS_DSHOT300 = [
    ([0x1f00ff8, 0x78003f, 0xfc001fff, 0xffffffff], True),
    ([0x1ff00780, 0x783ffe0, 0x1fe0fff, 0xffffffff], True),
    ([0x1ff00f80, 0x787ffc1, 0xfc01ffff, 0xffffffff], True),
    ([0x1ff0f07f, 0x7fc3fe, 0x3c001fff, 0xffffffff], True),
    ([0x1ff0f07f, 0x807fc001, 0xc1e00fff, 0xffffffff], True),
    ([0x1ff0f87f, 0x7c3fe0, 0x3fe00fff, 0xffffffff], True),
    ([0xf0f807, 0x87803c1e, 0x3e000fff, 0xffffffff], True),
]

# DSHOT600, K=9 - see driver/dshot_profiles.py's
# BIDIR_PROFILES[DSHOT_SPEEDS.DSHOT600], captures/2026-09-12_16-01-39.
RX_CLOCK_HZ_DSHOT600 = 6_750_000
EXPECTED_RATIO_DSHOT600 = 8.7129

GROUPS_DSHOT600 = [
    ([0xf007f8, 0x7ffc01f, 0xc001ffff, 0xffffffff], True),
    ([0x1ff00f80, 0x787ffc0, 0x3fe1fff, 0xffffffff], True),
    ([0x1ff00f80, 0x787ffc1, 0xfc01ffff, 0xffffffff], True),
    ([0x1ff0f80, 0x7f83c01, 0xc001ffff, 0xffffffff], True),
    ([0x1ff0f87f, 0x807fc01f, 0xc01e0fff, 0xffffffff], True),
    ([0x1ff0f87f, 0x807fc001, 0xc1e00fff, 0xffffffff], True),
    ([0x1ff0f87f, 0x807c3fe0, 0x3fe00fff, 0xffffffff], True),
]


def stage_breakdown(words, rx_clock_hz, expected_ratio, label, iterations=10):
    """Bracket each pipeline stage separately to find which one actually
    dominates cost - the whole-pipeline number alone can't tell an
    expensive linear scan in one stage from another."""
    raw_us, edges_us, period_us, bits_us = [], [], [], []
    for _ in range(iterations):
        t0 = utime.ticks_us()
        samples = gcr_decode.raw_samples(words)
        t1 = utime.ticks_us()
        edges = gcr_decode.find_edges(samples)
        t2 = utime.ticks_us()
        period = gcr_decode.estimate_bit_period_fixed(edges, expected_ratio, 0.0)
        t3 = utime.ticks_us()
        gcr_decode.reconstruct_bits(samples, edges, period)
        t4 = utime.ticks_us()
        raw_us.append(utime.ticks_diff(t1, t0))
        edges_us.append(utime.ticks_diff(t2, t1))
        period_us.append(utime.ticks_diff(t3, t2))
        bits_us.append(utime.ticks_diff(t4, t3))

    total = sum(raw_us) + sum(edges_us) + sum(period_us) + sum(bits_us)
    print("  [{}] edges={} raw_samples={:.0f}us find_edges={:.0f}us "
          "estimate_bit_period_fixed={:.0f}us ({:.0f}%) reconstruct_bits={:.0f}us ({:.0f}%)".format(
              label, len(edges),
              sum(raw_us) / iterations, sum(edges_us) / iterations,
              sum(period_us) / iterations, sum(period_us) / total * 100,
              sum(bits_us) / iterations, sum(bits_us) / total * 100))


def bench_one(label, groups, rx_clock_hz, expected_ratio):
    print("=== GCR Decode Timing ({}) ===".format(label))
    print("{} groups x {} iterations each".format(len(groups), ITERATIONS_PER_GROUP))
    print()

    all_us = []
    for words, expect_valid in groups:
        times_us = []
        for _ in range(ITERATIONS_PER_GROUP):
            start = utime.ticks_us()
            result = analyze_capture(words, rx_clock_hz, expected_ratio, 0.0)
            elapsed = utime.ticks_diff(utime.ticks_us(), start)
            times_us.append(elapsed)

        got_valid = result is not None and result["crc_ok"]
        match = "OK" if got_valid == expect_valid else "MISMATCH"
        min_us = min(times_us)
        max_us = max(times_us)
        mean_us = sum(times_us) / len(times_us)
        print("  expect_valid={} got_valid={} [{}]  min={}us mean={:.1f}us max={}us".format(
            expect_valid, got_valid, match, min_us, mean_us, max_us))
        all_us.extend(times_us)

    print()
    overall_min = min(all_us)
    overall_max = max(all_us)
    overall_mean = sum(all_us) / len(all_us)
    print("Overall: min={}us mean={:.1f}us max={}us".format(
        overall_min, overall_mean, overall_max))
    implied_rate = 1_000_000 / overall_max
    print("Implied sustainable rate at worst-case cost: {:.0f} decodes/s".format(implied_rate))
    print()
    return overall_min, overall_mean, overall_max


def test_gcr_decode_timing():
    print("=== GCR Decode Stage Breakdown ===")
    stage_breakdown(GROUPS_DSHOT300[0][0], RX_CLOCK_HZ_DSHOT300, EXPECTED_RATIO_DSHOT300, "DSHOT300 K=9")
    stage_breakdown(GROUPS_DSHOT600[0][0], RX_CLOCK_HZ_DSHOT600, EXPECTED_RATIO_DSHOT600, "DSHOT600 K=9")
    print()

    d300_min, d300_mean, d300_max = bench_one(
        "DSHOT300@3.375MHz, K=9", GROUPS_DSHOT300, RX_CLOCK_HZ_DSHOT300, EXPECTED_RATIO_DSHOT300)
    d600_min, d600_mean, d600_max = bench_one(
        "DSHOT600@6.75MHz, K=9", GROUPS_DSHOT600, RX_CLOCK_HZ_DSHOT600, EXPECTED_RATIO_DSHOT600)

    print("=== Summary ===")
    print("DSHOT300 (K=9): min={}us mean={:.1f}us max={}us".format(d300_min, d300_mean, d300_max))
    print("DSHOT600 (K=9): min={}us mean={:.1f}us max={}us".format(d600_min, d600_mean, d600_max))
    print("=== Test Complete ===")


test_gcr_decode_timing()
