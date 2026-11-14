"""
Decoder for the bidirectional DShot reply: GCR, CRC and eRPM from the 21-bit
frame dshot_bidir_rx_frame pushes. analyze_frame() is the entry point.

Pure Python with no hardware imports: the driver decodes on the Pico with it,
and PC tools and tests import it too. Works on plain integers, so a decode
allocates no per-bit objects. Only AM32's reply is accepted: inverted CRC.
Wire format: specification/DSHOT.md, specification/AM32.md. Design: ADR-002.
"""

GCR_ENCODE_TABLE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]
GCR_DECODE_TABLE = {symbol: nibble for nibble, symbol in enumerate(GCR_ENCODE_TABLE)}

FRAME_LENGTH_BITS = 21  # marker (1) + 20 differentially-encoded data bits

# AM32's "motor not running" payload, all 12 bits set; sent armed or disarmed,
# so it says nothing about arm state (specification/AM32.md#9)
AM32_NOT_RUNNING_DATA12 = 0xFFF

# That payload as dshot_bidir_rx_frame pushes it, so the reply gate needs one
# compare, not a decode. A capture of the driver's own throttle-0 frame
# never produces it.
AM32_NOT_RUNNING_FRAME = 0x52951


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return (~plain) & 0xF


def decode(frame):
    """
    Differential decode and GCR lookup of a FRAME_LENGTH_BITS-bit frame,
    marker (0) on top. Each of the 20 bits below is the XOR of a data bit with
    the one before it, the first with the marker.

    Returns the 16-bit DShot number (12-bit data, 4-bit CRC), or None if a
    5-bit group is not a GCR symbol.
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
    Return (crc_ok, data12): crc_ok is True when the CRC is AM32's inverted
    one. AM32 always inverts; accepting the plain CRC too would double the
    chance that a corrupt frame passes.
    """
    crc = dshot_full_number & 0xF
    data12 = (dshot_full_number >> 4) & 0xFFF
    return crc == crc_inverted(data12), data12


def analyze_frame(frame):
    """
    Decode one frame as dshot_bidir_rx_frame pushes it, a
    FRAME_LENGTH_BITS-bit integer with the marker on top, into a dict:

      full        - the 16-bit DShot number, or None if a GCR lookup failed
      marker_ok   - the marker bit read 0; False hints at a garbled first bit
      crc_ok      - the CRC matched (inverted polarity only)
      data12      - the 12-bit payload (mantissa, exponent), if decoded
      erpm        - electrical RPM, or None if not decoded or CRC-invalid
      not_running - data12 is AM32_NOT_RUNNING_DATA12; None if CRC-invalid.
                    erpm still reads about 917 for it.

    A CRC-valid reply can still carry an implausible eRPM: AM32 sends
    data12=2 (30,000,000 eRPM) in some replies right after a start.
    """
    full = decode(frame)
    result = {
        "full": full,
        "marker_ok": (frame >> (FRAME_LENGTH_BITS - 1)) == 0,
        "crc_ok": False,
        "data12": None,
        "erpm": None,
        "not_running": None,
    }
    if full is None:
        return result
    crc_ok, data12 = check_crc(full)
    result["data12"] = data12
    result["crc_ok"] = crc_ok
    if crc_ok:
        result["not_running"] = data12 == AM32_NOT_RUNNING_DATA12
        mantissa = data12 & 0x1FF
        exponent = (data12 >> 9) & 0x7
        eperiod_us = mantissa << exponent
        result["erpm"] = None if eperiod_us == 0 else 60_000_000 / eperiod_us
    return result
