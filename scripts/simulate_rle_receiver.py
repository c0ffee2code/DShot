"""
PC-side model of driver/dshot_pio.py's dshot_bidir_rx_rle, replayed on real captures.

The run-length receiver is a PIO program; this checks its logic and timing
before it runs on hardware. For each CRC-valid reply in a stored capture session
it rebuilds the pin's waveform from the raw 128 samples, runs the program's
instruction list on a small PIO model (1 instruction per cycle, delay slots,
the 2-cycle input synchroniser) with the receiver clock set as the driver sets
it, and compares the frame the model pushes with the one
gcr_decode.reconstruct_frame() builds from the same samples.

PROGRAM below is hand-transcribed from dshot_bidir_rx_rle: change one and the
other has to follow. Run from the project root:
  python scripts/simulate_rle_receiver.py

The waveform comes from samples 2 cycles apart at the capture clock, so each
edge is only known to about +-1 capture cycle (about +-1.8 receiver cycles) -
the model sees more edge jitter than the real signal has.
"""

import random
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "driver"))

import gcr_decode
from dshot_profiles import BIDIR_PROFILES, DSHOT_SPEEDS, RLE_CYCLES_PER_BIT

RECORD_FMT = "<I4H16I"
FRAME_BITS = 21
SYNC_DELAY = 2  # cycles the pin's level takes to reach the state machine

# (session directory, DShot speed). Sessions were captured at that speed's rx_speed.
SESSIONS = [
    ("2026-09-12_13-04-36", DSHOT_SPEEDS.DSHOT300),
    ("2026-09-12_18-19-38", DSHOT_SPEEDS.DSHOT300),
    ("2026-09-19_12-44-44", DSHOT_SPEEDS.DSHOT300),
    ("2026-09-12_16-01-39", DSHOT_SPEEDS.DSHOT600),
]

# Addresses match the assembled program. Each entry is (op, argument, delay).
PROGRAM = [
    ("wait_low", None, 0),   # 0  wait(0, pin, 0): the marker edge (the earlier steps are not modelled)
    ("set_y", 20, 0),        # 1
    ("set_x", 0, 0),         # 2  flip_low
    ("jmp", 12, 0),          # 3  -> low
    ("set_x", 0, 0),         # 4  flip_high
    ("jmp_pin", 7, 0),       # 5  high: -> high_count
    ("jmp", 2, 0),           # 6  -> flip_low
    ("jmp_xdec", 5, 0),      # 7  high_count: -> high
    ("jmp", 14, 0),          # 8  -> emit
    ("set_x", 4, 1),         # 9  again
    ("jmp_pin", 5, 0),       # 10 -> high
    ("nop", None, 0),        # 11
    ("jmp_pin", 4, 0),       # 12 low: -> flip_high
    ("jmp_xdec", 12, 0),     # 13 -> low
    ("in_pin", None, 0),     # 14 emit
    ("jmp_ydec", 9, 0),      # 15 -> again (falls off the end after the 21st read)
]


def waveform(words):
    """Returns level(t): the pin at capture-clock cycle t, the marker edge at t=0."""
    samples = [(w >> i) & 1 for w in words for i in range(31, -1, -1)]
    cycles = [gcr_decode.sample_cycle(i) for i in range(len(samples))]

    def level(t):
        if t < 0:
            return 1
        if t < cycles[0] - 1:
            return 0
        lo, hi = 0, len(samples) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if cycles[mid] - 1 <= t:
                lo = mid
            else:
                hi = mid - 1
        return samples[lo]

    return level


def run(level, capture_cycles_per_cycle, phase, max_cycles=2000):
    """Runs PROGRAM; returns the pushed frame, or None if it never pushed one."""
    x = y = isr = count = pc = cycle = 0

    def pin():
        return level((cycle - SYNC_DELAY) * capture_cycles_per_cycle - phase)

    while cycle < max_cycles:
        op, arg, delay = PROGRAM[pc]
        next_pc = pc + 1
        if op == "wait_low":
            if pin():
                cycle += 1
                continue
        elif op == "set_x":
            x = arg
        elif op == "set_y":
            y = arg
        elif op == "jmp":
            next_pc = arg
        elif op == "jmp_pin":
            if pin():
                next_pc = arg
        elif op == "jmp_xdec":
            if x:
                next_pc = arg
            x = (x - 1) & 0xFFFFFFFF
        elif op == "jmp_ydec":
            if y:
                next_pc = arg
            y = (y - 1) & 0xFFFFFFFF
        elif op == "in_pin":
            isr = ((isr << 1) | pin()) & 0xFFFFFFFF
            count += 1
            if count == FRAME_BITS:
                return isr
        cycle += 1 + delay
        pc = next_pc
        if pc >= len(PROGRAM):
            return None
    return None


def load(session):
    size = struct.calcsize(RECORD_FMT)
    data = (ROOT / "captures" / session / "capture.bin").read_bytes()
    for offset in range(0, len(data) - size + 1, size):
        record = struct.unpack_from(RECORD_FMT, data, offset)
        words = record[5:9]  # motor 0's group
        if any(words):
            yield words


def replay(session, dshot_speed, mistune, rng):
    """Returns (CRC-valid replies in the capture, how many the model rebuilt identically)."""
    ratio = BIDIR_PROFILES[dshot_speed]["expected_ratio"]
    # capture cycles per receiver cycle; mistune skews it as a wrongly tuned clock would
    scale = ratio / RLE_CYCLES_PER_BIT * (1 + mistune)
    valid = identical = 0
    for words in load(session):
        edges = gcr_decode.find_edges(words)
        if len(edges) < 2:
            continue
        reference = gcr_decode.reconstruct_frame(words, edges, ratio)
        number = gcr_decode.decode(reference)
        if number is None or gcr_decode.check_crc(number)[0] is None:
            continue
        valid += 1
        frame = run(waveform(words), scale, rng.random() * scale)
        if frame == reference:
            identical += 1
    return valid, identical


def main():
    rng = random.Random(1)
    print("Identical-frame rate by receiver clock error (0% = tuned exactly to the profile)")
    print("%-8s" % "error" + "".join("%-26s" % (s + (" (300)" if v == DSHOT_SPEEDS.DSHOT300 else " (600)"))
                                     for s, v in SESSIONS) + "all")
    for mistune in (-0.05, -0.03, -0.015, 0.0, 0.015, 0.03, 0.05):
        row = "%+5.1f%%  " % (mistune * 100)
        total_valid = total_same = 0
        for session, speed in SESSIONS:
            valid, same = replay(session, speed, mistune, rng)
            total_valid += valid
            total_same += same
            row += "%-26s" % ("%d/%d" % (same, valid))
        print(row + "%.2f%%" % (100.0 * total_same / total_valid))


main()
