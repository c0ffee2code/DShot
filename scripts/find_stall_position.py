# find_stall_position.py - where in an arming.bin log did the largest gap
# between captures fall, relative to the first sign of ESC trouble (a motor 0
# word of exactly 0)?
#
# BUG-002 diagnostic (2026-10-03): written when meta.txt's arming_call_max_us
# recorded the size of the biggest gap during arming but not when it happened,
# so there was no way to tell a stall that *caused* a reset from one that
# merely rode along with an already-long, already-failing run. meta.txt now
# also has arming_call_max_at_us/arming_call_max_at_ms_since_arm (same session,
# measured.py) for any run captured after that change and before 2026-10-06 -
# check those first. This script stays useful for sessions captured before
# measured.py was retired, and for the inter-CAPTURE gap specifically (a
# different thing from the inter-UPDATE-CALL gap the meta.txt field measures:
# this one reflects when a new reply showed up at all, not just how often
# Core 1's loop ran).
#
# arming.bin logging itself was retired 2026-10-06 (BUG-002: real per-tick
# overhead on the exact loop being measured). A session captured after that
# date has no arming.bin for this script to read.
#
# Usage: python scripts/find_stall_position.py captures/<session>/arming.bin [more...]

import struct
import sys

RECORD_FMT = "<I4H4I"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
TICKS_MASK = 0x3FFFFFFF  # MicroPython ticks_us() wraps at 2**30, same as fakes.py


def ticks_diff(a, b):
    diff = (a - b) & TICKS_MASK
    return diff - (TICKS_MASK + 1) if diff >= (TICKS_MASK + 1) // 2 else diff

def analyze(path):
    """Print the biggest inter-record gap in `path`, where it falls (by record
    index and elapsed time), and whether it comes before or after the first
    record where motor 0's word is exactly 0 (a reset-in-progress signature -
    see "A genuinely unpowered ESC" in BUG-002.md for why 0 means this and not
    an unpowered ESC, in context)."""
    with open(path, "rb") as f:
        data = f.read()
    n = len(data) // RECORD_SIZE
    records = []
    for i in range(n):
        rec = struct.unpack_from(RECORD_FMT, data, i * RECORD_SIZE)
        ticks_us = rec[0]
        words = rec[5:9]  # 4 motor words
        records.append((ticks_us, words))

    if n < 2:
        print("  too few records:", n)
        return

    t0 = records[0][0]
    tN = records[-1][0]
    total_span = ticks_diff(tN, t0)

    max_gap = 0
    max_gap_idx = 0
    for i in range(1, n):
        gap = ticks_diff(records[i][0], records[i-1][0])
        if gap > max_gap:
            max_gap = gap
            max_gap_idx = i

    gap_time_us = ticks_diff(records[max_gap_idx][0], t0)
    pct_through = 100.0 * gap_time_us / total_span if total_span else 0

    # find first index where motor 0's word is 0 (zero/"low" classification - reset signature)
    first_zero_m0 = None
    for i, (t, words) in enumerate(records):
        if words[0] == 0:
            first_zero_m0 = i
            break
    first_zero_time = None
    if first_zero_m0 is not None:
        first_zero_time = ticks_diff(records[first_zero_m0][0], t0)

    print("  records=%d total_span_ms=%.1f max_gap_us=%d at record %d/%d (%.1f%% through, t=%.1fms)" % (
        n, total_span/1000, max_gap, max_gap_idx, n, pct_through, gap_time_us/1000))
    if first_zero_time is not None:
        print("  first motor0-zero-word record at t=%.1fms (record %d)" % (first_zero_time/1000, first_zero_m0))
        print("  stall %s first zero-word event" % ("BEFORE" if gap_time_us < first_zero_time else "AFTER"))
    else:
        print("  no motor0 zero-word record found in this log")

if __name__ == "__main__":
    for path in sys.argv[1:]:
        print(path)
        analyze(path)
