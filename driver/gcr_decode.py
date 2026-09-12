"""
On-device bidirectional DShot GCR telemetry decoder - MicroPython, mirrors
scripts/dshot_bidir_decode.py (the PC-side reference) function-for-function
for the fixed-ratio decode path (the two intentionally diverge on the
brute-force sweep - see estimate_bit_period_fixed's docstring below and
decision/ADR-002-bidirectional-dshot.md's "Fixed-ratio RX sampling retune"
section). scripts/verify_gcr_decode_port.py is the permanent regression
check keeping the shared path in sync - re-run it whenever either file
changes.

Decodes the densely, uniformly-oversampled raw captures produced by
dshot_bidir_rx (see decision/ADR-002-bidirectional-dshot.md's "RX redesign:
unslotted dense oversampling" section for the full history).

dshot_bidir_rx makes NO assumption about the real GCR bit period at the PIO
level - it just samples the pin every 2 PIO cycles, continuously, for 128
samples, covering the marker bit, the 20 real GCR data bits, and idle tail,
all in one flat un-slotted stream. What real bit period those cycles work
out to is a per-profile tuned constant (driver/dshot_profiles.py's
BIDIR_PROFILES, one per DShot speed, each measured and verified on real
hardware - see the ADR section above), not searched for at decode time -
this module's job is turning that raw waveform into bits using the tuned
constant, not discovering the period fresh from every capture.

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
2. Find the marker's rising edge and use the calling profile's tuned bit
   period (a fixed constant in cycles, generally NOT an integer -
   estimate_bit_period_fixed, see its own docstring).
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


def _period_score(gaps, p):
    """
    Shared residual-scoring math for estimate_bit_period and
    estimate_bit_period_fixed: how well would rounding every gap to the
    nearest integer multiple of candidate period p explain the observed
    gaps, as a total relative (residual/period)^2 score - see
    estimate_bit_period's docstring for why relative, not absolute, and
    for the q=g/p reuse that avoids a redundant division.
    """
    score = 0.0
    for g in gaps:
        q = g / p
        n = max(1, round(q))
        residual = q - n
        score += residual * residual
    return score


def estimate_bit_period_fixed(edges, expected_ratio, tolerance=0.0):
    """
    Betaflight-style fixed-ratio period estimate: when rx_speed has been
    tuned so the nominal samples-per-bit ratio is a known constant, there
    is no need to search for the real period at all - see decision/
    ADR-002-bidirectional-dshot.md's fixed-ratio RX sampling section for
    the density/margin tradeoff this depends on, and for why the tuned
    ratio is measured on real hardware before being trusted, not derived
    from the nominal protocol bitrate alone (real ESC oscillators drift a
    few percent off nominal).

    tolerance=0.0 (default): returns expected_ratio directly - a bare
    fixed divisor, no search, the fastest possible path (this is
    Betaflight's own technique: `(run_length + 1) / K` in `dshot_bitbang_
    decode.c`, just written as a function here instead of inline).

    tolerance>0.0: sweeps a narrow band [expected_ratio-tolerance,
    expected_ratio+tolerance] in 0.04 steps via _period_score, for when
    real hardware data shows the tuned rate still needs some margin
    against drift rather than being trusted as a bare constant. Every
    live DShot speed's profile so far (see driver/dshot_profiles.py) uses
    tolerance=0.0 - both DSHOT300 and DSHOT600 measured tight enough
    (std well under the 0.5-cycle rounding boundary) that a bare divisor
    reproduces the old brute-force sweep's answer exactly on every
    CRC-valid capture checked (see decision/ADR-002-bidirectional-
    dshot.md's fixed-ratio RX sampling section) - the tolerance>0.0 path
    exists for a future profile whose measured spread doesn't clear that
    bar as cleanly.

    len(edges) < 2 returns None - analyze_capture treats that as a
    dead-line signal, not something to paper over.
    """
    if len(edges) < 2:
        return None
    if tolerance <= 0.0:
        return expected_ratio
    gaps = [edges[i + 1][0] - edges[i][0] for i in range(len(edges) - 1)]
    best_period = None
    best_score = None
    p = expected_ratio - tolerance
    end = expected_ratio + tolerance
    while p <= end:
        score = _period_score(gaps, p)
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


def analyze_capture(words, rx_clock_hz, expected_ratio, ratio_tolerance=0.0):
    """
    Full pipeline from 4 raw 32-bit capture words to a decoded result.

    expected_ratio/ratio_tolerance feed estimate_bit_period_fixed - every
    DShot speed this driver supports has a tuned profile in
    driver/dshot_profiles.py's BIDIR_PROFILES with a real, measured
    expected_ratio, so this is a required argument, not optional: there is
    no brute-force sweep fallback any more (retired 2026-09-12 once both
    DSHOT300 and DSHOT600 had a verified fixed ratio - see decision/
    ADR-002-bidirectional-dshot.md's fixed-ratio RX sampling section).
    scripts/dshot_bidir_decode.py, the PC-side reference, deliberately
    keeps its own brute-force sweep permanently for analyzing captures
    from any era/rate - only this on-device module dropped it.

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
    period = estimate_bit_period_fixed(edges, expected_ratio, ratio_tolerance)
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
