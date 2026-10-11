"""
On-device bidirectional DShot GCR telemetry decoder - MicroPython.

One entry point: analyze_frame() decodes the frame receiver's
(dshot_bidir_rx_frame, see driver/dshot_pio.py) already run-length-
reconstructed 21-bit frame - the path BidirectionalDShot uses. There used to
be a second, analyze_capture(), for the older sample receiver's raw
oversampled waveform (reconstructed on the CPU) - removed 2026-10-11 along
with that receiver and the one standalone tool that used it
(tools/calibrate_bidir_rx.py was the only caller of either). The raw-sample
reconstruction math (find_edges, reconstruct_frame, sample_cycle,
estimate_bit_period_fixed) moved to scripts/simulate_frame_receiver.py,
which is PC-side and was the only other caller left.

decode()/check_crc()/decode_result() are shared between analyze_frame() and
scripts/dshot_bidir_decode.py (the PC-side reference) and
scripts/simulate_frame_receiver.py. scripts/verify_gcr_decode_port.py's
verify_frame_parity() checks analyze_frame() stays in sync with the
reference - re-run it whenever either file changes.

Unlike the reference script, check_crc() here accepts ONLY the inverted CRC
polarity: every CRC-valid capture from real hardware has been inverted
(798/798 on this bench), matching AM32's firmware source - AM32 inverts its
reply CRC unconditionally (see specification/AM32.md) - and accepting the
plain polarity as well would double the false-accept probability of the
4-bit CRC (2/16 instead of 1/16) for a polarity the hardware never produces.
The reference script accepts both because a stray plain hit is diagnostic
there; this module is the driver's validity gate and has no such use for it.
scripts/verify_am32_reply.py independently confirms the whole GCR table
against AM32's own encoder: it reproduces make_dshot_package() line for
line and decodes all 2,304 distinct payloads AM32 can emit, with 0
mismatches against this module.

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

FRAME_LENGTH_BITS = 21  # marker (1) + 20 differentially-encoded data bits

# AM32's fixed "motor not running" payload (mantissa 0x1FF, exponent 0b111 -
# all 12 data bits set): sent whether the ESC is armed-but-stopped or fully
# disarmed, so a CRC-valid reply of exactly this value proves nothing about
# arm state, only that the motor isn't turning right now (see
# specification/AM32.md#9).
AM32_NOT_RUNNING_DATA12 = 0xFFF

# That payload as dshot_bidir_rx_frame pushes it: the whole 21-bit frame (marker,
# then 0xFFF and its inverted CRC in GCR). The only word a stopped AM32 ever
# replies with, so MotorGroup's arming gate recognises it with one integer compare
# instead of a decode (bug-reports/BUG-003). Our own throttle-0 frame, captured
# when the ESC is silent, never produces it.
AM32_NOT_RUNNING_FRAME = 0x52951


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return (~plain) & 0xF


def decode(frame):
    """
    Differential-decode + GCR table lookup. `frame` is a FRAME_LENGTH_BITS-bit
    integer: its top bit is the marker (always 0) and the 20 bits below it are the
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
    Shared tail of analyze_frame() and the PC-side reference's equivalent:
    decode a reconstructed FRAME_LENGTH_BITS-bit frame (marker at the top)
    into a result dict, without the period fields - each caller's own timing
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
      not_running - True if data12 is AM32's fixed not-running sentinel
                  (AM32_NOT_RUNNING_DATA12), else False; None if CRC-invalid.
                  Deliberately not folded into erpm: erpm keeps reporting the
                  same ~917 either way, so callers that already threshold on
                  eRPM magnitude are unaffected, while callers that need an
                  exact answer (not a fuzzy eRPM range) can check this instead.

    Known caveat, not necessarily a decode bug here: a CRC-valid frame has
    been seen reporting a wildly implausible erpm (data12=2, i.e. 30,000,000)
    shortly after a motor's throttle crosses from zero to nonzero. AM32's own
    source has an eRPM moving-average array that is zero-initialized at boot;
    one re-read of this project's pinned commit found a gating condition
    (`old_routine`, default-on at boot) under which the array can stay
    untouched until the motor's first real commutation, so the first reply
    computed after "running" flips could read it as genuinely all-zero. A
    separate hypothesis for *why* this never reproduces at a different
    throttle (a polling-rate difference) was checked against this project's
    own measured numbers and ruled out - the gap is still unexplained. Treat
    the whole mechanism as a candidate, not confirmed: it already reversed
    once between two source reads, and this driver's own TX-echo
    capture path has not been ruled out as a competing explanation either. See
    bug-reports/BUG-009-motors-sometimes-start-rough.md for the full,
    still-open account - do not treat this as settled.
    """
    full = decode(frame)
    result = {
        "full": full,
        "marker_ok": (frame >> (FRAME_LENGTH_BITS - 1)) == 0,
        "crc_ok": False,
        "crc_kind": None,
        "data12": None,
        "erpm": None,
        "not_running": None,
    }
    if full is None:
        return result
    crc_kind, data12 = check_crc(full)
    result["crc_kind"] = crc_kind
    result["data12"] = data12
    result["crc_ok"] = crc_kind is not None
    if crc_kind is not None:
        result["not_running"] = data12 == AM32_NOT_RUNNING_DATA12
        mantissa = data12 & 0x1FF
        exponent = (data12 >> 9) & 0x7
        eperiod_us = mantissa << exponent
        result["erpm"] = None if eperiod_us == 0 else 60_000_000 / eperiod_us
    return result


def analyze_frame(frame):
    """
    Full pipeline for one frame the frame receiver (dshot_bidir_rx_frame, the
    only receiver BidirectionalDShot has) already reconstructed in hardware:
    a FRAME_LENGTH_BITS-bit integer, marker at the top. The receiver did that
    reconstruction work in the state machine, not the CPU.

    period_cycles, period_us and bitrate_bps are always None in the returned
    dict: the frame receiver's bit period is fixed by its clock divider
    (driver/dshot_profiles.py's frame_rx_speed()), not measured per capture,
    so there is nothing to report there.
    """
    result = decode_result(frame)
    result["period_cycles"] = None
    result["period_us"] = None
    result["bitrate_bps"] = None
    return result
