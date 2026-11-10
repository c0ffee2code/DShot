"""
esc_buddy_image.py - the firmware image of an ESC flash (PC only): an Intel HEX file made into the exact bytes that go to the
F421's application area, 0x1000..0x7BFF, the way the AM32 configurator makes them (src/flash.ts parseHex / fillImage: every byte
not in the file is 0xFF), plus the checks that it is a firmware for this ESC and not something else.
"""

import json
import re

FLASH_BASE = 0x08000000          # where the F421's flash sits in the MCU's address space; the HEX file uses these addresses
FLASH_START = 0x1000             # esc_buddy_protocol.FLASH_START / FLASH_END: the application area
FLASH_END = 0x7C00
NAME_OFFSET = 0x7BE0 - FLASH_START     # the 32-byte firmware file name AM32 places just below the settings page
NAME_LENGTH = 32
RAM_TOP = 0x20008000             # the F421 has less; anything above this in the stack pointer word is not a vector table


def parse_hex(text):
    """Intel HEX text -> {absolute address: byte}. Raises ValueError for a bad checksum, a record the configurator would
    mishandle, or a file without its end-of-file record."""
    data = {}
    upper = 0
    ended = False
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if ended:
            raise ValueError("line %d: data after the end-of-file record" % number)
        if not line.startswith(":"):
            raise ValueError("line %d: not an Intel HEX record" % number)
        try:
            record = bytes.fromhex(line[1:])
        except ValueError:
            raise ValueError("line %d: not hexadecimal" % number) from None
        if len(record) < 5 or len(record) != 5 + record[0]:
            raise ValueError("line %d: wrong record length" % number)
        if sum(record) & 0xFF:
            raise ValueError("line %d: checksum" % number)
        count, address, kind, body = record[0], (record[1] << 8) | record[2], record[3], record[4:4 + record[0]]
        if kind == 0x00:
            for i, byte in enumerate(body):
                data[upper + address + i] = byte
        elif kind == 0x01:
            ended = True
        elif kind == 0x02:
            if count != 2 or int.from_bytes(body, "big") != 0:
                raise ValueError("line %d: extended segment address record" % number)
        elif kind == 0x04:
            if count != 2:
                raise ValueError("line %d: extended linear address record" % number)
            upper = int.from_bytes(body, "big") << 16
        elif kind in (0x03, 0x05):
            pass                      # entry point: irrelevant to what is flashed
        else:
            raise ValueError("line %d: unknown record type %d" % (number, kind))
    if not ended:
        raise ValueError("no end-of-file record")
    return data


def image_from_hex(text):
    """The application area as 0x7C00 - 0x1000 bytes (27,648), 0xFF where the file has nothing. Refuses a file with data outside
    that area (the bootloader below, the settings page above), an empty one, and one whose first words are no vector table."""
    data = parse_hex(text)
    if not data:
        raise ValueError("the file holds no data")
    low, high = min(data), max(data)
    if low < FLASH_BASE + FLASH_START or high >= FLASH_BASE + FLASH_END:
        raise ValueError("data at 0x%08X..0x%08X is outside the application area 0x%08X..0x%08X" % (
            low, high, FLASH_BASE + FLASH_START, FLASH_BASE + FLASH_END - 1))
    image = bytearray(b"\xff" * (FLASH_END - FLASH_START))
    for address, byte in data.items():
        image[address - FLASH_BASE - FLASH_START] = byte
    return check_image(bytes(image))


def check_image(image):
    """The image itself if it is the right size and looks like firmware (vector table, file name), else ValueError."""
    if len(image) != FLASH_END - FLASH_START:
        raise ValueError("%d bytes, not the %d of the application area" % (len(image), FLASH_END - FLASH_START))
    stack = int.from_bytes(image[0:4], "little")
    reset = int.from_bytes(image[4:8], "little")
    if not 0x20000000 < stack <= RAM_TOP:
        raise ValueError("the first word 0x%08X is no stack pointer: not a firmware image" % stack)
    if not reset & 1 or not FLASH_BASE + FLASH_START <= reset < FLASH_BASE + FLASH_END:
        raise ValueError("the reset vector 0x%08X is not in the application area: not a firmware image" % reset)
    if firmware_name(image) is None:
        raise ValueError("no firmware file name at 0x%04X" % (FLASH_START + NAME_OFFSET))
    return image


def firmware_name(image_or_bytes):
    """The file name AM32 stores in its image (NAME_LENGTH bytes ending in a zero), e.g. 'SKYSTARS_KM55_V2_F421', or None when
    what is there is not such a name. Takes the whole image or just the 32 bytes read from the ESC."""
    raw = bytes(image_or_bytes)
    if len(raw) > NAME_LENGTH:
        raw = raw[NAME_OFFSET:NAME_OFFSET + NAME_LENGTH]
    name = raw.split(b"\x00")[0]
    if re.fullmatch(rb"[A-Z0-9_]+", name):
        return name.decode("ascii")
    return None


def image_from_dump(text):
    """The image in a `esc_buddy_run.py --dump-firmware` report (key "firmware_hex")."""
    try:
        return check_image(bytes.fromhex(json.loads(text)["firmware_hex"]))
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError("not a firmware dump: %s" % error) from None


def load(path):
    """An image from an AM32 .hex file, or from a firmware dump (.json) made by --dump-firmware."""
    with open(path, "r", encoding="ascii", errors="strict") as handle:
        text = handle.read()
    return image_from_dump(text) if str(path).lower().endswith(".json") else image_from_hex(text)
