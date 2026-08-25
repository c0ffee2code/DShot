"""
Throwaway offline analysis - NOT part of the driver or its test suite.

Decodes the densely, uniformly-oversampled raw captures from
tests/test_bidir_rx_raw.py against the CURRENT dshot_bidir_rx design (see
decision/ADR-002-bidirectional-dshot.md's "RX redesign: unslotted dense
oversampling" section for the full history - including a first, disproven
frame-length hypothesis this script used to assume).

dshot_bidir_rx makes NO assumption about the real GCR bit period. It just
samples the pin every 2 PIO cycles (0.5us at the 4MHz rx_speed),
continuously, for 128 samples - covering the marker bit, the 20 real GCR
data bits, and idle tail, all in one flat un-slotted stream. This script's
job is exactly what the PIO program deliberately no longer does: figure out
where the real bit boundaries are and what the real bit period is, from the
raw waveform itself.

Each real reply produces FOUR 32-bit words (in_shiftdir=SHIFT_LEFT,
push_thresh=32): the OLDEST sample in each word is at bit31, the NEWEST at
bit0 (same MSB-first-in-time convention the driver has always used). Words
concatenate in capture order, giving 128 samples in time order - but NOT
perfectly uniformly spaced: dshot_bidir_rx's sample loop is a nested 4x32
structure (a flat 128-iteration loop is illegal - `set`'s immediate is a
5-bit field, max 31), which costs 2 extra PIO cycles at each of the 3
"outer pass" boundaries (every 32 samples) versus the normal 2-cycles/
sample gap within a pass. This is fully deterministic - see
dshot_bidir_rx's comment for the exact instruction accounting - so this
script tracks each sample's exact absolute CYCLE position (not just its
index) rather than assuming uniform spacing.

VERIFIED on hardware 2026-08-23: 17/17 captures CRC-valid, eRPM rising
monotonically across throttle steps 100/200/300 (~21.6k -> ~48.8k ->
~76.1k eRPM, tight clustering within each throttle group, exponent
stepping 3->2->1 exactly as a shrinking commutation period should). See
ADR-002 for the full result.

Method:
1. Reconstruct the 128-sample time series with exact per-sample cycle
   positions.
2. Find the marker's rising edge (0->1, end of the marker bit - the marker
   is captured directly now, unlike the old slotted design, so this is a
   real measurement, not an assumption) and estimate the real bit period
   (in cycles) from edge-to-edge gaps (sweeping candidate periods, since
   the true period is generally NOT an integer number of cycles).
3. Reconstruct the actual bit sequence via RUN-LENGTH decoding (each run of
   N cycles between edges contributes round(N/period) bits of that run's
   value) - NOT by resampling at fixed offsets. Run-length reconstruction
   is immune to the accumulated-phase-error problem that defeated every
   earlier design in this ADR: a resampler's window walks out of phase
   with the real signal bit-by-bit as it goes deeper into the frame, while
   run-length reconstruction only cares about each individual run's
   duration relative to the period, so errors don't accumulate across the
   frame.
4. The frame is marker (1 bit, always 0) + 20 differentially-encoded data
   bits = 21 bits total - NOT marker + a separate "seed" bit + 20 data (22
   total), which was this script's first hypothesis after mis-reading
   AM32's `gcr[]` array construction, and which a from-scratch simulation
   validated (187-200/200) before ever touching hardware. That simulation
   was still worth running - it caught two real bugs (an illegal `set`
   immediate, and an off-by-one in the resample window) - but its 22-bit
   frame assumption was itself wrong, disproven empirically (2/17 vs
   17/17) once real hardware data was available. Don't cite the simulation
   as evidence for a 22-bit frame.
5. Trailing data bits whose value matches idle (1) merge invisibly into
   the idle run - there's no edge between them, so run-length
   reconstruction alone can under-count by 1-2 bits (seen on hardware as
   19-20 reconstructed bits instead of the true 21). This is NOT a bug to
   chase further: since the frame length (21) is known, pad the
   reconstruction with idle-value (1) bits up to 21 before decoding. This
   padding step is load-bearing - without it, decoding fails even though
   everything else is correct.
"""

CAPTURES = [
    (100, [0x0ffc1f03, 0xfe007ff0, 0x07fe007e, 0x1fffffff]),
    (100, [0x0ffc1f83, 0xfe007ff0, 0x07c1ff81, 0xffffffff]),
    (100, [0x0ffc1f83, 0xfe007ff0, 0x07fe007e, 0x1fffffff]),
    (100, [0x0ffc1f07, 0xfe007fe0, 0x0fc1ff83, 0xffffffff]),
    (100, [0x0ffc1f03, 0xfe007fe0, 0x07c1ff83, 0xffffffff]),
    (200, [0x0ffc1f07, 0xfe007fe0, 0x07c1ff83, 0xffffffff]),
    (200, [0x0ffc007c, 0x000f83ff, 0xf8000ffe, 0x1fffffff]),
    (200, [0x0ffc00f8, 0x000f83ff, 0xf83ff003, 0xffffffff]),
    (200, [0x0ffc007c, 0x000f83ff, 0x003e0f81, 0xffffffff]),
    (200, [0x0ffc00fc, 0x000f83ff, 0xf8000ffc, 0x1fffffff]),
    (200, [0x0ffc00fc, 0x000f83e0, 0xffc00f80, 0x3fffffff]),
    (300, [0x0ffc007c, 0x000f83ff, 0xf83ff001, 0xffffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xffc00f80, 0x1fffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
    (300, [0x0fffe0f8, 0x3ff0001f, 0xf83ff07c, 0x3fffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
]

GCR_ENCODE_TABLE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]
GCR_DECODE_TABLE = {symbol: nibble for nibble, symbol in enumerate(GCR_ENCODE_TABLE)}

MOTOR_POLES = 14  # AM32 EEPROM default (wiki.am32.ca) - unverified for this specific ESC
RX_CLOCK_HZ = 4_000_000  # see dshot_pio.py's DShotPIO.__init__
FRAME_LENGTH_BITS = 21  # marker (1) + 20 differentially-encoded data bits - see module docstring

# Exact per-sample cycle position for dshot_bidir_rx's nested 4-outer x
# 32-inner sample loop (see dshot_bidir_rx's comment for the instruction
# accounting this derives from): pass p in 0..3, inner index i in 0..31,
# global sample index = p*32+i. Within a pass, samples are 2 cycles apart;
# each pass after the first costs 66 cycles total (1 for `set(y,31)` + 32*2
# for the inner loop + 1 for `jmp(x_dec)`), and the first sample of a pass
# lands 1 cycle after that pass's `set(y,31)`.
PASS_LENGTH_SAMPLES = 32
CYCLES_PER_PASS = 66
CYCLES_PER_SAMPLE_WITHIN_PASS = 2


def sample_cycle(global_index):
    p, i = divmod(global_index, PASS_LENGTH_SAMPLES)
    return p * CYCLES_PER_PASS + 1 + i * CYCLES_PER_SAMPLE_WITHIN_PASS


def crc_plain(data12):
    return (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF


def crc_inverted(data12):
    return (~crc_plain(data12)) & 0xF


def raw_samples(words):
    """
    Flatten 4 32-bit words into a 128-entry list of (cycle, bit) pairs, in
    time order (oldest first), using the exact non-uniform cycle positions
    of dshot_bidir_rx's nested sample loop - see module comment.
    """
    bits = []
    for word in words:
        for bit_pos in range(31, -1, -1):
            bits.append((word >> bit_pos) & 1)
    return [(sample_cycle(idx), bit) for idx, bit in enumerate(bits)]


def find_edges(samples):
    """Returns list of cycle positions where the value changes between consecutive samples."""
    edges = []
    for i in range(1, len(samples)):
        if samples[i][1] != samples[i - 1][1]:
            edges.append(samples[i][0])
    return edges


def estimate_bit_period(edges):
    """
    Estimate the real (possibly fractional) bit period in cycles from
    edge-to-edge gaps. Each gap is (close to) an integer multiple of the
    true period, since edges only occur where consecutive logical bits
    differ - runs of repeated bits produce no edge. An integer-only
    estimate (e.g. the smallest recurring gap) is not good enough: the true
    period is generally NOT an integer number of cycles, and rounding it to
    the nearest integer reintroduces exactly the accumulating error this
    redesign exists to avoid. So this sweeps fractional candidates and picks
    the one that best explains all observed gaps as integer multiples of
    itself (least total squared residual after rounding each gap/period to
    the nearest integer multiple).
    """
    if len(edges) < 2:
        return None
    gaps = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
    # Relative residual (residual/period)^2, NOT absolute - an absolute
    # metric is unboundedly biased toward small periods (any gap is
    # trivially "close" to some multiple of a tiny period). This bug was
    # made and caught earlier against the old slotted design's data - see
    # ADR-002 - and is worth guarding against here too.
    best_period = None
    best_score = None
    p = 6.0
    while p <= 16.0:
        score = 0.0
        for g in gaps:
            n = max(1, round(g / p))
            residual = (g - n * p) / p
            score += residual * residual
        if best_score is None or score < best_score:
            best_score = score
            best_period = p
        p += 0.04
    return best_period


def value_at_cycle(samples, target_cycle):
    """Bit value of the sample whose cycle position is nearest target_cycle."""
    best = min(samples, key=lambda s: abs(s[0] - target_cycle))
    return best[1]


def reconstruct_bits(samples, edges, period):
    """
    Reconstruct the bit sequence via run-length decoding rather than fixed-
    offset resampling - see module docstring for why this is immune to the
    accumulated-phase-error problem that defeated earlier designs. Returns
    bits starting with the marker bit itself, up to (not including) the
    final idle-merged run, padded with idle-value (1) bits up to
    FRAME_LENGTH_BITS if the true trailing bits merged into idle (see
    docstring point 5) - this padding is load-bearing, not a fallback.
    """
    start_cycle = samples[0][0]
    boundaries = [start_cycle] + edges
    bits = []
    for i in range(len(boundaries) - 1):
        seg_start = boundaries[i]
        seg_end = boundaries[i + 1]
        length = seg_end - seg_start
        # sample just after the boundary itself, not exactly on it
        val = value_at_cycle(samples, seg_start + 0.1)
        n = max(1, round(length / period))
        bits.extend([val] * n)
    while len(bits) < FRAME_LENGTH_BITS:
        bits.append(1)
    return bits


def decode(bits):
    """
    Differential-decode + GCR table lookup. bits[0] is the marker (always
    0); bits[1:21] are the 20 differentially-encoded real data bits, XORed
    against the immediately preceding bit - prev seeded from the marker's
    own value (0). This is the frame model verified on hardware (17/17
    CRC-valid) - see module docstring for why an earlier "marker + separate
    seed bit + 20 data" (22-bit) model was tried first and disproven.
    """
    data_bits = bits[1:21]
    if len(data_bits) < 20:
        return None
    prev = 0
    decoded_bits = []
    for b in data_bits:
        decoded_bits.append(b ^ prev)
        prev = b
    decoded20 = 0
    for db in decoded_bits:
        decoded20 = (decoded20 << 1) | db
    nibbles = []
    for shift in (15, 10, 5, 0):
        symbol = (decoded20 >> shift) & 0x1F
        nibble = GCR_DECODE_TABLE.get(symbol)
        if nibble is None:
            return None
        nibbles.append(nibble)
    return (nibbles[0] << 12) | (nibbles[1] << 8) | (nibbles[2] << 4) | nibbles[3]


def check_crc(dshot_full_number):
    crc = dshot_full_number & 0xF
    data12 = (dshot_full_number >> 4) & 0xFFF
    for name, expected in (("plain", crc_plain(data12)), ("inverted", crc_inverted(data12))):
        if crc == expected:
            return name, data12
    return None, data12


def analyze_capture(words):
    samples = raw_samples(words)
    edges = find_edges(samples)
    if not edges:
        return None
    period = estimate_bit_period(edges)
    if period is None:
        return None
    bits = reconstruct_bits(samples, edges, period)
    full = decode(bits)
    period_us = period / RX_CLOCK_HZ * 1_000_000
    return {
        "period_cycles": period,
        "period_us": period_us,
        "bitrate_bps": int(1_000_000 / period_us) if period_us else None,
        "full": full,
    }


def main():
    if not CAPTURES:
        print("No captures pasted in yet - see module docstring. Nothing to analyze.")
        return

    total = 0
    symbol_valid = 0
    crc_valid = 0
    for throttle, words in CAPTURES:
        total += 1
        result = analyze_capture(words)
        if result is None:
            print(f"throttle={throttle:4d}: could not analyze (no edges)")
            continue
        print(f"throttle={throttle:4d} period={result['period_cycles']:.2f} cycles "
              f"({result['period_us']:.2f}us, {result['bitrate_bps']:,} bps)", end="  ")
        full = result["full"]
        if full is None:
            print("invalid GCR symbols")
            continue
        symbol_valid += 1
        hit, data12 = check_crc(full)
        if hit:
            crc_valid += 1
            mantissa = data12 & 0x1FF
            exponent = (data12 >> 9) & 0x7
            period_us = mantissa << exponent
            erpm = None if period_us == 0 else 60_000_000 / period_us
            rpm = None if erpm is None else erpm / (MOTOR_POLES / 2)
            erpm_s = "n/a" if erpm is None else f"{erpm:.0f}"
            rpm_s = "n/a" if rpm is None else f"{rpm:.0f}"
            print(f"CRC-OK({hit:8s}) mantissa={mantissa:4d} exponent={exponent} "
                  f"eRPM={erpm_s:>7s} RPM={rpm_s:>7s}")
        else:
            print(f"symbols valid, CRC FAIL (got 0x{full & 0xF:x}, "
                  f"plain=0x{crc_plain(data12):x}, inverted=0x{crc_inverted(data12):x})")

    print()
    print(f"{symbol_valid}/{total} symbol-valid, {crc_valid}/{total} CRC-valid")


if __name__ == "__main__":
    main()
