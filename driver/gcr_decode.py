"""
On-device bidirectional DShot GCR telemetry decoder - MicroPython.

Two entry points, one per receiver in driver/dshot_pio.py. analyze_frame()
decodes the frame receiver's (dshot_bidir_rx_rle) already run-length-
reconstructed 21-bit frame - the path BidirectionalDShot uses.
analyze_capture() decodes the sample receiver's (dshot_bidir_rx) raw
oversampled waveform instead, doing the run-length reconstruction here on
the CPU; it backs the standalone calibration tool that measures
driver/dshot_profiles.py's BIDIR_PROFILES expected_ratio for a new ESC unit,
since the frame receiver has no period search of its own.

Both share a fixed-ratio decode path with scripts/dshot_bidir_decode.py (the
PC-side reference), differing only in how the bit period is found: this
module takes it from a tuned profile (BIDIR_PROFILES), the reference can
also search for it. scripts/verify_gcr_decode_port.py checks the shared path
stays in sync - re-run it whenever either file changes.

analyze_capture()'s raw input: each real reply produces FOUR 32-bit words
(in_shiftdir=SHIFT_LEFT, push_thresh=32) - the OLDEST sample in each word is
at bit31, the NEWEST at bit0. Words concatenate in capture order, giving 128
samples in time order - but NOT perfectly uniformly spaced: dshot_bidir_rx's
sample loop is a nested 4x32 structure, which costs 2 extra PIO cycles at
each of the 3 "outer pass" boundaries (every 32 samples) versus the normal
2-cycles/sample gap within a pass. This is fully deterministic, so a
sample's position is computed as its exact CYCLE (sample_cycle()), not
assumed from its index.

Unlike the reference script, check_crc() here accepts ONLY the inverted CRC
polarity: every CRC-valid capture from real hardware has been inverted,
matching AM32's firmware source, and accepting the plain polarity as well
would double the false-accept probability of the 4-bit CRC (2/16 instead of
1/16) for a polarity the hardware never produces. The reference script
accepts both because a stray plain hit is diagnostic there; this module is
the driver's validity gate and has no such use for it.

analyze_capture()'s reconstruction method:
1. Find the edges - the samples whose value differs from the one before - by
   XOR-ing each half-word with itself shifted by one, instead of looking at all
   128 samples one by one. Only about a dozen samples are edges.
2. Rebuild the bit sequence via RUN-LENGTH decoding: each run of N cycles
   between edges contributes round(N/period) bits of that run's value, using the
   calling profile's tuned bit period (a fixed constant in cycles, generally NOT
   an integer). NOT by resampling at fixed offsets, which would accumulate phase
   error deeper into the frame.
3. The frame is marker (1 bit, always 0) + 20 differentially-encoded data bits =
   21 bits. Trailing data bits whose value matches idle (1) merge invisibly into
   the idle run, so run-length reconstruction alone can under-count by 1-2 bits;
   pad with idle-value (1) bits up to FRAME_LENGTH_BITS - load-bearing, not a
   fallback.
4. Differential-decode, then look each 5-bit group up in the GCR table.

Every step works on plain integers, not on lists of per-sample tuples: a
garbage collection on either core pauses both, so heap churn here shows up
as gaps in the command loop - see ADR-002 for the measured cost this design
avoids.
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

# Which bit a value with exactly one bit set has, for the 16-bit halves
# find_edges() scans. (This MicroPython has no int.bit_length().)
BIT_POSITION = {1 << bit: bit for bit in range(16)}


def sample_cycle(global_index):
    p, i = divmod(global_index, PASS_LENGTH_SAMPLES)
    return p * CYCLES_PER_PASS + 1 + i * CYCLES_PER_SAMPLE_WITHIN_PASS


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return (~plain) & 0xF


def find_edges(words):
    """
    Returns the indices, in ascending order, of the samples whose value differs
    from the previous sample's (samples numbered 0-127 in time order, the top
    bit of the first word being sample 0).

    Each word is scanned in two 16-bit halves so no intermediate value passes 30
    bits: a larger integer is a heap object, and this runs once per capture.
    Within a half, XOR with the half shifted by one marks the transitions, and
    clearing the lowest set bit visits them, newest sample first - hence the sort.
    """
    edges = []
    previous = 0  # the last sample of the previous half
    base = 0
    for word in words:
        for half in (word >> 16, word & 0xFFFF):
            changed = half ^ ((half >> 1) | (previous << 15))
            if base == 0:
                changed &= 0x7FFF  # sample 0 has nothing before it
            while changed:
                lowest = changed & -changed
                edges.append(base + 15 - BIT_POSITION[lowest])
                changed ^= lowest
            previous = half & 1
            base += 16
    edges.sort()
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

    `edges` is find_edges()' result.

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
    cycles = [sample_cycle(e) for e in edges]
    gaps = [cycles[i + 1] - cycles[i] for i in range(len(cycles) - 1)]
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


def reconstruct_frame(words, edges, period):
    """
    Rebuild the frame by run-length decoding (see the module docstring), as an
    integer of FRAME_LENGTH_BITS bits with the marker bit at the top.

    Each run between two edges lasts (cycle of its end - cycle of its start) and
    holds the value of the sample it starts at, which is the one just after the
    previous edge (sample 0 for the first run). The idle tail after the last
    edge is not a run: it carries no bits. If the runs come to fewer than
    FRAME_LENGTH_BITS, the missing trailing bits are idle-value (1) ones that
    merged into the tail - this padding is load-bearing, not a fallback - and
    if they come to more, only the first FRAME_LENGTH_BITS count.
    """
    frame = 0
    count = 0
    start_cycle = 1  # sample_cycle(0)
    value_index = 0
    for edge in edges:
        # sample_cycle(edge), written out because this runs for every edge
        end_cycle = (edge >> 5) * CYCLES_PER_PASS + 1 + (edge & 31) * CYCLES_PER_SAMPLE_WITHIN_PASS
        n = round((end_cycle - start_cycle) / period)
        if n < 1:
            n = 1
        if (words[value_index >> 5] >> (31 - (value_index & 31))) & 1:
            frame = (frame << n) | ((1 << n) - 1)
        else:
            frame <<= n
        count += n
        if count >= FRAME_LENGTH_BITS:
            return frame >> (count - FRAME_LENGTH_BITS)
        start_cycle = end_cycle
        value_index = edge
    missing = FRAME_LENGTH_BITS - count
    return (frame << missing) | ((1 << missing) - 1)


def decode(frame):
    """
    Differential-decode + GCR table lookup. `frame` is reconstruct_frame()'s
    result: its top bit is the marker (always 0) and the 20 bits below it are the
    real data bits, each XORed against the bit before it, the first against the
    marker's own value (0) - which is data ^ (data >> 1) over those 20 bits.
    Returns the 16-bit DShot number (12-bit data + 4-bit CRC), or None if a
    5-bit group is not a valid GCR symbol.
    """
    data = frame & 0xFFFFF
    decoded20 = data ^ (data >> 1)
    number = 0
    for shift in (15, 10, 5, 0):
        nibble = GCR_DECODE_TABLE.get((decoded20 >> shift) & 0x1F)
        if nibble is None:
            return None
        number = (number << 4) | nibble
    return number


def check_crc(dshot_full_number):
    """
    Only the inverted polarity is accepted - see the module docstring for why.
    """
    crc = dshot_full_number & 0xF
    data12 = (dshot_full_number >> 4) & 0xFFF
    if crc == crc_inverted(data12):
        return "inverted", data12
    return None, data12


def decode_result(frame):
    """
    Shared tail of analyze_capture() and analyze_frame(): decode a
    reconstructed FRAME_LENGTH_BITS-bit frame (marker at the top) into a
    result dict, without the period fields - each caller's own timing
    information, if it has any, goes in those.

    marker_ok is the frame's own top bit read back as 0, the reply's fixed
    start level. decode() itself never checks it (the differential decode
    folds the marker's fixed value in implicitly), but a false marker is a
    sign of a garbled first bit, worth surfacing next to the CRC check rather
    than dropping it silently.

    Returns a dict with:
      full      - the raw 16-bit DShot number (12-bit data + 4-bit CRC), or
                  None if GCR symbol lookup failed
      marker_ok - True if the frame's marker bit read back as 0
      crc_ok    - True if `full`'s CRC matched (inverted polarity only)
      crc_kind  - "inverted" on a CRC hit, else None
      data12    - the 12-bit payload (mantissa + exponent), if decoded
      erpm      - electrical RPM, or None if not decodable/CRC-invalid
    """
    full = decode(frame)
    result = {
        "full": full,
        "marker_ok": (frame >> (FRAME_LENGTH_BITS - 1)) == 0,
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


def analyze_capture(words, rx_clock_hz, expected_ratio, ratio_tolerance=0.0):
    """
    Full pipeline from 4 raw 32-bit capture words (dshot_bidir_rx's output) to
    a decoded result: find the edges, estimate the bit period, run-length
    reconstruct the frame, then decode_result().

    expected_ratio/ratio_tolerance feed estimate_bit_period_fixed. They are
    required: every supported DShot speed has a measured profile in
    driver/dshot_profiles.py's BIDIR_PROFILES, so there is no search fallback
    on the device. The PC-side reference (scripts/dshot_bidir_decode.py) keeps
    its own search for analysing captures taken at any rate.

    Returns None if no edges were found at all (dead line). Otherwise returns
    decode_result()'s dict plus the measured GCR bit timing: period_cycles,
    period_us, bitrate_bps.
    """
    edges = find_edges(words)
    if not edges:
        return None
    period = estimate_bit_period_fixed(edges, expected_ratio, ratio_tolerance)
    if period is None:
        return None
    result = decode_result(reconstruct_frame(words, edges, period))
    period_us = period / rx_clock_hz * 1_000_000
    result["period_cycles"] = period
    result["period_us"] = period_us
    result["bitrate_bps"] = int(1_000_000 / period_us) if period_us else None
    return result


def analyze_frame(frame):
    """
    Full pipeline for one frame the frame receiver (dshot_bidir_rx_rle, the
    only receiver BidirectionalDShot has) already reconstructed in hardware:
    the same FRAME_LENGTH_BITS-bit integer, marker at the top, that
    reconstruct_frame() builds from raw samples. Skips find_edges(),
    estimate_bit_period_fixed() and reconstruct_frame() entirely - the
    receiver did that work in the state machine, not the CPU.

    Same result shape as analyze_capture(), with period_cycles, period_us and
    bitrate_bps all None: the frame receiver's bit period is fixed by its
    clock divider (driver/dshot_profiles.py's rle_rx_speed()), not measured
    per capture, so there is nothing to report there.
    """
    result = decode_result(frame)
    result["period_cycles"] = None
    result["period_us"] = None
    result["bitrate_bps"] = None
    return result
