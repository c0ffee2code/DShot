# On-device timing micro-benchmark for gcr_decode.analyze_capture() - the
# real open question the on-device telemetry validity check depends on:
# is the 250-iteration bit-period sweep (estimate_bit_period) viable as
# MicroPython bytecode on this hardware? No ESC, no motor, no arming -
# this only needs a Pico connected, not the bench hardware-testing gate.
#
# Groups below are real hardware captures (channel 1, DSHOT300,
# rx_clock_hz=4_000_000) pulled from captures/2026-09-06_21-03-50,
# captures/2026-09-07_19-45-56 and captures/2026-09-07_19-49-33 - a mix of
# CRC-valid and CRC-invalid groups, since estimate_bit_period's inner loop
# scales with edge count and the invalid groups here have denser edges
# (more bit transitions), giving a more realistic worst-case max than
# valid groups alone would.

import gcr_decode
from gcr_decode import analyze_capture
import utime

RX_CLOCK_HZ = 4_000_000
ITERATIONS_PER_GROUP = 50

GROUPS = [
    ([0x7c0f80, 0x3e0f800f, 0x7c1f000, 0x1fffffff], True),
    ([0x7fe0f8, 0x1ff83ff, 0x7c1ff83, 0xffffffff], True),
    ([0x7ff07c, 0x1ff81ff, 0x7c1ffc1, 0xffffffff], True),
    ([0x7fe0fc, 0x1ff83ff, 0x7c1ff83, 0xffffffff], True),
    ([0x78f3e78f, 0x7cf1e7cf, 0x3cf820c1, 0x7ffffff], False),
    ([0x7cf1e7cf, 0x3cf9e3cf, 0x3c783041, 0x3ffffff], False),
    ([0x78f1e78f, 0x3cf1e3cf, 0x3c7820c1, 0x3ffffff], False),
]


def stage_breakdown(words, label, iterations=10):
    """Bracket each pipeline stage separately to find which one actually
    dominates cost - the whole-pipeline number alone can't tell an
    oversized period sweep from an expensive linear scan elsewhere."""
    raw_us, edges_us, period_us, bits_us = [], [], [], []
    for _ in range(iterations):
        t0 = utime.ticks_us()
        samples = gcr_decode.raw_samples(words)
        t1 = utime.ticks_us()
        edges = gcr_decode.find_edges(samples)
        t2 = utime.ticks_us()
        period = gcr_decode.estimate_bit_period(edges)
        t3 = utime.ticks_us()
        gcr_decode.reconstruct_bits(samples, edges, period)
        t4 = utime.ticks_us()
        raw_us.append(utime.ticks_diff(t1, t0))
        edges_us.append(utime.ticks_diff(t2, t1))
        period_us.append(utime.ticks_diff(t3, t2))
        bits_us.append(utime.ticks_diff(t4, t3))

    total = sum(raw_us) + sum(edges_us) + sum(period_us) + sum(bits_us)
    print("  [{}] edges={} raw_samples={:.0f}us find_edges={:.0f}us "
          "estimate_bit_period={:.0f}us ({:.0f}%) reconstruct_bits={:.0f}us ({:.0f}%)".format(
              label, len(edges),
              sum(raw_us) / iterations, sum(edges_us) / iterations,
              sum(period_us) / iterations, sum(period_us) / total * 100,
              sum(bits_us) / iterations, sum(bits_us) / total * 100))


def test_gcr_decode_timing():
    print("=== GCR Decode Stage Breakdown ===")
    stage_breakdown(GROUPS[0][0], "valid")
    stage_breakdown(GROUPS[4][0], "invalid")
    print()

    print("=== GCR Decode Timing ===")
    print("{} groups x {} iterations each".format(len(GROUPS), ITERATIONS_PER_GROUP))
    print()

    all_us = []
    for words, expect_valid in GROUPS:
        times_us = []
        for _ in range(ITERATIONS_PER_GROUP):
            start = utime.ticks_us()
            result = analyze_capture(words, RX_CLOCK_HZ)
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
    print("=== Test Complete ===")


test_gcr_decode_timing()
