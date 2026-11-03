# The replay fixture's file format and the word the ESC mock's state machine shifts out. Pure Python, no hardware
# imports: it runs on the wingman (esc_mock.py) and on the PC (tests/bench/wingman/tools/esc_mock_fixture.py and its tests).
#
# Fixture file: header "<4sBBH" = b"ESCM", version, number of lines, frames per line; then, per line, that many
# little-endian uint32. Each uint32 is one reply as the main Pico's receiver reads it: the 21 line levels, first bit
# on top (the integer gcr_decode.analyze_frame() takes).

import struct

MAGIC = b"ESCM"
VERSION = 1
HEADER = "<4sBBH"
HEADER_SIZE = struct.calcsize(HEADER)
REPLY_BITS = 21
NOT_RUNNING_FRAME = 0x52951      # AM32's fixed "motor not running" reply (gcr_decode.AM32_NOT_RUNNING_FRAME; a test keeps them equal)


def pio_word(frame):
    """The 32-bit word the mock's `out pins, 1` loop shifts out, first bit from bit 31: the 21 line levels at the top of
    the word, as they are (1 = driven high, 0 = driven low)."""
    return (frame & ((1 << REPLY_BITS) - 1)) << (32 - REPLY_BITS)


def frame_from_pio_word(word):
    """Inverse of pio_word()."""
    return (word >> (32 - REPLY_BITS)) & ((1 << REPLY_BITS) - 1)


def pack_fixture(lines):
    """lines: one list of 21-bit frames per line, all the same length -> the fixture file's bytes."""
    count = len(lines[0])
    for frames in lines:
        if len(frames) != count:
            raise ValueError("every line needs the same number of frames")
    out = struct.pack(HEADER, MAGIC, VERSION, len(lines), count)
    for frames in lines:
        for frame in frames:
            if frame < 0 or frame >> REPLY_BITS:
                raise ValueError("a frame must fit in %d bits" % REPLY_BITS)
        out += struct.pack("<%dI" % count, *frames)
    return out


def fixture_line_count(header):
    """How many lines of recorded replies a fixture holds, from its first HEADER_SIZE bytes."""
    magic, version, line_count, _ = struct.unpack_from(HEADER, header, 0)
    if magic != MAGIC or version != VERSION:
        raise ValueError("not an ESC mock fixture (magic %r, version %d)" % (magic, version))
    return line_count


def parse_fixture(data):
    """The fixture file's bytes -> a list with one list of 21-bit frames per line."""
    magic, version, line_count, count = struct.unpack_from(HEADER, data, 0)
    if magic != MAGIC or version != VERSION:
        raise ValueError("not an ESC mock fixture (magic %r, version %d)" % (magic, version))
    if len(data) != HEADER_SIZE + 4 * line_count * count:
        raise ValueError("the fixture file has the wrong size")
    lines = []
    for index in range(line_count):
        lines.append(list(struct.unpack_from("<%dI" % count, data, HEADER_SIZE + 4 * count * index)))
    return lines
