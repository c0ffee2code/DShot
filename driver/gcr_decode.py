"""
On-device bidirectional DShot GCR telemetry decoder - MicroPython, mirrors
scripts/dshot_bidir_decode.py (the PC-side reference) function-for-function
for the fixed-ratio decode path. The two differ only in how the bit period is
found: this module takes it from a tuned profile, the reference can also search
for it. scripts/verify_gcr_decode_port.py checks that the shared path stays in
sync - re-run it whenever either file changes.

Decodes the raw captures produced by dshot_bidir_rx: a dense, uniform sampling
of the pin covering the marker bit, the 20 GCR data bits and the idle tail.
The real bit period those samples work out to is a per-DShot-speed constant
(driver/dshot_profiles.py's BIDIR_PROFILES), not something this module
searches for on every capture - searching is too slow to run on the Pico.
This module turns the raw waveform into bits using that constant.

Each real reply produces FOUR 32-bit words (in_shiftdir=SHIFT_LEFT,
push_thresh=32): the OLDEST sample in each word is at bit31, the NEWEST at
bit0. Words concatenate in capture order, giving 128 samples in time order -
but NOT perfectly uniformly spaced: dshot_bidir_rx's sample loop is a nested
4x32 structure, which costs 2 extra PIO cycles at each of the 3 "outer pass"
boundaries (every 32 samples) versus the normal 2-cycles/sample gap within a
pass. This is fully deterministic, so this module tracks each sample's exact
absolute CYCLE position (not just its index) rather than assuming uniform
spacing.

Unlike the reference script, check_crc() here accepts ONLY the inverted CRC
polarity. That is deliberate: every CRC-valid capture from real hardware has
been inverted, matching AM32's firmware source, and accepting the plain
polarity as well would double the false-accept probability of the 4-bit CRC
(2/16 instead of 1/16) for a polarity the hardware never produces. The
reference script accepts both because a stray plain hit is diagnostic there;
this module is the driver's validity gate and has no such use for it.

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
    by indexing `samples`, instead of re-searching for the nearest sample:
    find_edges has already looked at exactly that sample to detect the
    transition, and a search per reconstructed bit is too slow on the Pico.
    """
    edges = []
    for i in range(1, len(samples)):
        if samples[i][1] != samples[i - 1][1]:
            edges.append((samples[i][0], i))
    return edges


def period_score(gaps, p):
    """
    How well rounding every gap to the nearest integer multiple of candidate
    period p explains the observed gaps, as a total (residual/period)^2 score
    (lower is better). The residual is relative to the period so that
    candidates of different sizes are comparable. q = g/p is computed once and
    reused for both the rounded multiple and the residual, because division is
    expensive here.
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
    Fixed-ratio bit period estimate (the technique Betaflight's bidirectional
    DShot decoder uses). rx_speed is tuned so a reply bit spans a known,
    measured number of RX cycles, so the period does not have to be searched
    for on every capture. The ratio is measured on hardware rather than derived
    from the protocol's nominal bit rate, because ESC oscillators run a few
    percent off nominal (see driver/dshot_profiles.py).

    tolerance=0.0 (default): returns expected_ratio directly - a bare fixed
    divisor, the cheapest path. It is valid when the profile's measured spread
    stays well inside half a cycle, so rounding run lengths to bits is
    unambiguous.

    tolerance>0.0: searches [expected_ratio-tolerance, expected_ratio+tolerance]
    in 0.04 steps via period_score, for a profile whose spread is too wide to
    trust as a bare constant. No current profile needs it.

    Fewer than 2 edges returns None: analyze_capture treats that as a dead
    line rather than guessing.
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
        score = period_score(gaps, p)
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
    Only the inverted polarity is accepted - see the module docstring for why.
    """
    crc = dshot_full_number & 0xF
    data12 = (dshot_full_number >> 4) & 0xFFF
    if crc == crc_inverted(data12):
        return "inverted", data12
    return None, data12


def analyze_capture(words, rx_clock_hz, expected_ratio, ratio_tolerance=0.0):
    """
    Full pipeline from 4 raw 32-bit capture words to a decoded result.

    expected_ratio/ratio_tolerance feed estimate_bit_period_fixed. They are
    required: every supported DShot speed has a measured profile in
    driver/dshot_profiles.py's BIDIR_PROFILES, so there is no search fallback
    on the device. The PC-side reference (scripts/dshot_bidir_decode.py) keeps
    its own search for analysing captures taken at any rate.

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
