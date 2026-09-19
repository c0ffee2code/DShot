# Benchmark: where the telemetry decode spends its time and memory
#
# Purpose: decoding one capture takes 10-20ms on the Pico and is the application's
# main cost. This times each stage of gcr_decode.analyze_capture() on real
# captures and counts the heap each one allocates, because allocation feeds the
# garbage collector, and a collection on either core stalls both (see
# bench_loop_gaps.py). Needs no motor, no ESC and no signal.

import gc
import utime

import gcr_decode

# DSHOT300, K=9 - real captures, see tests/test_gcr_decode_timing.py
RX_CLOCK_HZ = 3_375_000
EXPECTED_RATIO = 8.7069
GROUPS = [
    [0x1f00ff8, 0x78003f, 0xfc001fff, 0xffffffff],
    [0x1ff00780, 0x783ffe0, 0x1fe0fff, 0xffffffff],
    [0x1ff00f80, 0x787ffc1, 0xfc01ffff, 0xffffffff],
    [0x1ff0f07f, 0x7fc3fe, 0x3c001fff, 0xffffffff],
    [0x1ff0f07f, 0x807fc001, 0xc1e00fff, 0xffffffff],
    [0x1ff0f87f, 0x7c3fe0, 0x3fe00fff, 0xffffffff],
    [0xf0f807, 0x87803c1e, 0x3e000fff, 0xffffffff],
]
REPEATS = 20


def measure(label, fn):
    """Average microseconds and heap bytes per call over every group."""
    total_us = 0
    total_bytes = 0
    calls = 0
    for words in GROUPS:
        for _ in range(REPEATS):
            # collect first, so the timed call cannot trigger a collection, and
            # keep the collector off while measuring how much the call allocates
            gc.collect()
            gc.disable()
            m0 = gc.mem_alloc()
            t0 = utime.ticks_us()
            fn(words)
            dt = utime.ticks_diff(utime.ticks_us(), t0)
            total_bytes += gc.mem_alloc() - m0
            gc.enable()
            total_us += dt
            calls += 1
    print("  %-40s %8.0f us   %7.0f bytes allocated" % (label, total_us / calls, total_bytes / calls))
    return total_us / calls


def main():
    print("=== Decode cost and allocation benchmark (no hardware) ===")
    print("Averages over %d real DSHOT300 captures x %d repeats" % (len(GROUPS), REPEATS))
    print()

    print("Stages, each fed its real input:")
    prepared = []
    for words in GROUPS:
        edges = gcr_decode.find_edges(words)
        period = gcr_decode.estimate_bit_period_fixed(edges, EXPECTED_RATIO, 0.0)
        frame = gcr_decode.reconstruct_frame(words, edges, period)
        full = gcr_decode.decode(frame)
        prepared.append((words, edges, period, frame, full))
    lookup = {id(p[0]): p for p in prepared}

    measure("find_edges(words)", lambda w: gcr_decode.find_edges(w))
    measure("estimate_bit_period_fixed(edges)", lambda w: gcr_decode.estimate_bit_period_fixed(lookup[id(w)][1], EXPECTED_RATIO, 0.0))
    measure("reconstruct_frame(words, edges, period)", lambda w: gcr_decode.reconstruct_frame(w, lookup[id(w)][1], lookup[id(w)][2]))
    measure("decode(frame)", lambda w: gcr_decode.decode(lookup[id(w)][3]))
    measure("check_crc(full)", lambda w: gcr_decode.check_crc(lookup[id(w)][4]))
    measure("(lookup overhead of this harness)", lambda w: lookup[id(w)])
    print()

    print("Whole pipeline:")
    total = measure("analyze_capture(words, ...)", lambda w: gcr_decode.analyze_capture(w, RX_CLOCK_HZ, EXPECTED_RATIO))
    print()

    edges_counts = [len(p[1]) for p in prepared]
    print("edges per capture: %s" % edges_counts)
    print("at that cost, one decode budget of %.1f ms allows about %.0f decodes/s per core" % (total / 1000, 1_000_000 / total))
    print("=== Benchmark Complete ===")


main()
