"""
On-device bidirectional DShot GCR telemetry decoder - MicroPython, mirrors
scripts/dshot_bidir_decode.py (the PC-side reference) function-for-function.
The two are kept in sync by scripts/verify_gcr_decode_port.py, a permanent
regression check, not a one-off port verification - re-run it whenever
either file changes.

Decodes the densely, uniformly-oversampled raw captures produced by
dshot_bidir_rx (see decision/ADR-002-bidirectional-dshot.md's "RX redesign:
unslotted dense oversampling" section for the full history).

dshot_bidir_rx makes NO assumption about the real GCR bit period. It just
samples the pin every 2 PIO cycles (0.5us at the 4MHz rx_speed),
continuously, for 128 samples - covering the marker bit, the 20 real GCR
data bits, and idle tail, all in one flat un-slotted stream. This module's
job is to figure out where the real bit boundaries are and what the real
bit period is, from the raw waveform itself.

Each real reply produces FOUR 32-bit words (in_shiftdir=SHIFT_LEFT,
push_thresh=32): the OLDEST sample in each word is at bit31, the NEWEST at
bit0. Words concatenate in capture order, giving 128 samples in time order -
but NOT perfectly uniformly spaced: dshot_bidir_rx's sample loop is a nested
4x32 structure, which costs 2 extra PIO cycles at each of the 3 "outer pass"
boundaries (every 32 samples) versus the normal 2-cycles/sample gap within a
pass. This is fully deterministic, so this module tracks each sample's exact
absolute CYCLE position (not just its index) rather than assuming uniform
spacing.

Unlike the reference script, this module's check_crc() accepts ONLY the
inverted CRC polarity, not both. This is a deliberate difference, not a
missed port: tallying every CRC-valid capture pulled from real hardware so
far (714 groups, 2026-09-06/07) came back 100% inverted, 0% plain, matching
what AM32's own firmware source produces. Accepting only the polarity the
hardware actually uses halves the false-accept probability of the 4-bit CRC
check (1/16 instead of 2/16) - the reference script keeps accepting both
because it is a PC-side exploration tool where a stray "plain" hit would
itself be diagnostic; this module exists specifically to be the driver's
validity gate, so it doesn't get that latitude.

Method:
1. Reconstruct the 128-sample time series with exact per-sample cycle
   positions.
2. Find the marker's rising edge and estimate the real bit period (in
   cycles) from edge-to-edge gaps (sweeping candidate periods, since the
   true period is generally NOT an integer number of cycles).
3. Reconstruct the actual bit sequence via RUN-LENGTH decoding (each run of
   N cycles between edges contributes round(N/period) bits of that run's
   value) - NOT by resampling at fixed offsets, which would accumulate
   phase error deeper into the frame.
4. The frame is marker (1 bit, always 0) + 20 differentially-encoded data
   bits = 21 bits total.
5. Trailing data bits whose value matches idle (1) merge invisibly into the
   idle run, so run-length reconstruction alone can under-count by 1-2
   bits; pad with idle-value (1) bits up to FRAME_LENGTH_BITS before
   decoding - load-bearing, not a fallback.
"""

GCR_ENCODE_TABLE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]
GCR_DECODE_TABLE = {symbol: nibble for nibble, symbol in enumerate(GCR_ENCODE_TABLE)}

FRAME_LENGTH_BITS = 21  # marker (1) + 20 differentially-encoded data bits - see module docstring

# Exact per-sample cycle position for dshot_bidir_rx's nested 4-outer x
# 32-inner sample loop: pass p in 0..3, inner index i in 0..31, global
# sample index = p*32+i. Within a pass, samples are 2 cycles apart; each
# pass after the first costs 66 cycles total, and the first sample of a
# pass lands 1 cycle after that pass's set(y,31). Unchanged across
# DSHOT_SPEED/rx_clock_hz - only the clock frequency that converts cycles
# to seconds changes, which is why analyze_capture() takes rx_clock_hz as
# a parameter instead of hardcoding it.
PASS_LENGTH_SAMPLES = 32
CYCLES_PER_PASS = 66
CYCLES_PER_SAMPLE_WITHIN_PASS = 2


def sample_cycle(global_index):
    p, i = divmod(global_index, PASS_LENGTH_SAMPLES)
    return p * CYCLES_PER_PASS + 1 + i * CYCLES_PER_SAMPLE_WITHIN_PASS


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return (~plain) & 0xF


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
    """
    Returns a list of (cycle, sample_index) pairs where the value changes
    between consecutive samples. Carrying sample_index (not just the cycle
    position) lets reconstruct_bits read the transitioned-to value directly
    by indexing `samples`, instead of re-searching for the nearest sample -
    this is the same sample find_edges already looked at to detect the
    transition, no reason to look it up a second time. Replaces the old
    value_at_cycle() linear scan (O(128) per lookup, called once per
    reconstructed bit) with an O(1) index read - see
    decision/ADR-002-bidirectional-dshot.md for the measured on-device
    timing impact.
    """
    edges = []
    for i in range(1, len(samples)):
        if samples[i][1] != samples[i - 1][1]:
            edges.append((samples[i][0], i))
    return edges


# Measured band (see estimate_bit_period's docstring): 798 CRC-valid
# captures across 3 real sessions gave period_cycles in [9.64, 10.52],
# mean 10.30, std 0.13. Margin here is generous relative to that spread,
# not the bare observed min/max.
#
# These are NOT bare 8.5/12.0: the reference script's sweep starts at 6.0
# and repeatedly adds 0.04, and float addition doesn't associate cleanly
# with multiplication - 6.0 + 62*0.04 is not bit-identical to what 62
# real += 0.04 steps from 6.0 produce. Picking values off that reference
# sequence itself (its values at steps 63 and 150) keeps this sweep's
# grid points bit-for-bit aligned with the reference's, so
# scripts/verify_gcr_decode_port.py's float-tolerance period_cycles check
# isn't comparing two subtly different grids.
SEARCH_RANGE_START = 8.51999999999999
SEARCH_RANGE_END = 11.999999999999917


def estimate_bit_period(edges):
    """
    Estimate the real (possibly fractional) bit period in cycles from
    edge-to-edge gaps. Sweeps fractional candidates and picks the one that
    best explains all observed gaps as integer multiples of itself (least
    total squared residual after rounding each gap/period to the nearest
    integer multiple) - an integer-only estimate would reintroduce the
    accumulating error this run-length approach exists to avoid.

    Inner loop computes q = g/p once and reuses it for both the rounded
    multiple and the residual (residual = (g - n*p)/p is algebraically
    g/p - n = q - n) - one division instead of a division, a multiply, and
    a second division.

    SEARCH_RANGE below is a measured band, not a guess: tallying
    period_cycles across every CRC-valid capture ever pulled on this rig
    (798 groups across 3 sessions, 2026-09-06/07) gives min=9.64,
    max=10.52, mean=10.30, std=0.13 - the true period sits in a narrow
    pocket, not anywhere in the original 6-16 range that pocket was
    carved from. Narrowing the sweep to that pocket (with real margin
    either side, not the bare min/max) cuts the candidate count from 250
    to ~35 while still finding the same answer: replaying the full
    original 6-16 sweep against the narrowed one on all 798 CRC-valid
    groups gives 0 disagreements (offline, no MicroPython involved -
    see scripts/verify_gcr_decode_port.py, which now treats disagreement
    confined to already-CRC-invalid groups as expected rather than a
    porting bug, since garbage input has no "right" period to recover).

    A two-phase coarse-then-fine version of this sweep was tried and
    reverted (2026-09-07): a 0.5-step coarse pass followed by a 0.04-step
    fine pass within +-0.5 of the coarse winner gave the wrong period for
    511/2040 real captures - the residual-vs-period surface isn't
    well-behaved enough at 0.5-step granularity for a coarse pass to
    reliably land in the right neighborhood. The narrowed-range sweep
    below is a different kind of change - it doesn't search worse, it
    just searches less territory, all of which is grounded in measured
    data rather than picked by a two-stage search - so it doesn't carry
    the same failure mode.

    This band is specific to this rig's hardware (this ESC, this DSHOT
    speed, whatever thermal state it was in across these sessions) - if
    the ESC, wiring, or DSHOT variant changes, re-tally period_cycles
    against fresh captures before trusting SEARCH_RANGE still covers it,
    and re-run scripts/verify_gcr_decode_port.py either way.
    """
    if len(edges) < 2:
        return None
    gaps = [edges[i + 1][0] - edges[i][0] for i in range(len(edges) - 1)]
    # Relative residual (residual/period)^2, NOT absolute - an absolute
    # metric is unboundedly biased toward small periods (any gap is
    # trivially "close" to some multiple of a tiny period).
    best_period = None
    best_score = None
    p = SEARCH_RANGE_START
    while p <= SEARCH_RANGE_END:
        score = 0.0
        for g in gaps:
            q = g / p
            n = max(1, round(q))
            residual = q - n
            score += residual * residual
        if best_score is None or score < best_score:
            best_score = score
            best_period = p
        p += 0.04
    return best_period


def reconstruct_bits(samples, edges, period):
    """
    Reconstruct the bit sequence via run-length decoding rather than fixed-
    offset resampling - see module docstring point 3. Returns bits starting
    with the marker bit itself, padded with idle-value (1) bits up to
    FRAME_LENGTH_BITS if the true trailing bits merged into idle (docstring
    point 5) - this padding is load-bearing, not a fallback.

    Each boundary's value is read directly via the sample index find_edges
    already carries (samples[idx][1]) rather than re-searching for the
    nearest sample - see find_edges' docstring.
    """
    boundaries = [(samples[0][0], 0)] + edges
    bits = []
    for i in range(len(boundaries) - 1):
        seg_start, idx = boundaries[i]
        seg_end = boundaries[i + 1][0]
        length = seg_end - seg_start
        val = samples[idx][1]
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
    own value (0).
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
    """
    Only the inverted polarity is accepted - see module docstring for why
    (714/714 real CRC-valid captures pulled from hardware so far are
    inverted; accepting plain too would just double the false-accept rate
    of a 4-bit CRC for a polarity this hardware has never actually produced).
    """
    crc = dshot_full_number & 0xF
    data12 = (dshot_full_number >> 4) & 0xFFF
    if crc == crc_inverted(data12):
        return "inverted", data12
    return None, data12


def analyze_capture(words, rx_clock_hz):
    """
    Full pipeline from 4 raw 32-bit capture words to a decoded result.

    Returns None if no edges were found at all (dead line). Otherwise
    returns a dict with:
      period_cycles, period_us, bitrate_bps - the measured GCR bit timing
      full           - the raw 16-bit DShot number (12-bit data + 4-bit CRC),
                        or None if GCR symbol lookup failed
      crc_ok         - True if `full`'s CRC matched (inverted polarity only)
      crc_kind       - "inverted" on a CRC hit, else None
      data12         - the 12-bit payload (mantissa + exponent), if decoded
      erpm           - electrical RPM, or None if not decodable/CRC-invalid
    """
    samples = raw_samples(words)
    edges = find_edges(samples)
    if not edges:
        return None
    period = estimate_bit_period(edges)
    if period is None:
        return None
    bits = reconstruct_bits(samples, edges, period)
    full = decode(bits)
    period_us = period / rx_clock_hz * 1_000_000
    result = {
        "period_cycles": period,
        "period_us": period_us,
        "bitrate_bps": int(1_000_000 / period_us) if period_us else None,
        "full": full,
        "crc_ok": False,
        "crc_kind": None,
        "data12": None,
        "erpm": None,
    }
    if full is None:
        return result
    crc_kind, data12 = check_crc(full)
    result["crc_kind"] = crc_kind
    result["data12"] = data12
    result["crc_ok"] = crc_kind is not None
    if crc_kind is not None:
        mantissa = data12 & 0x1FF
        exponent = (data12 >> 9) & 0x7
        eperiod_us = mantissa << exponent
        result["erpm"] = None if eperiod_us == 0 else 60_000_000 / eperiod_us
    return result
