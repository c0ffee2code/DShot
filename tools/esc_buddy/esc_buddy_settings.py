"""
esc_buddy_settings.py - decode an AM32 settings block (the "EEPROM" the bootloader serves at the settings address) into
named values. Pure Python, PC only.

Layout: EEPROM layout version 2 (byte 1), as AM32 firmware 2.16 stores it - bytes 5-16 hold the firmware name, settings start
at byte 17. Field meanings and encodings as the AM32 wiki's "Open ESC EEPROM Format" page and the configurator show them;
checked against the configurator's own display of the KM55A2's settings (2026-11-04, tools/esc_buddy/dump/). A block of
another layout version is refused rather than guessed at.
"""

SUPPORTED_LAYOUTS = (2,)


def on_off(value):
    return "on" if value else "off"


def advance(value):
    if value <= 3:
        return "%.1f deg" % (value * 7.5)          # old format: 7.5 degree steps
    if 10 <= value <= 42:
        return "%.1f deg" % ((value - 10) * 0.9375)  # new format
    return "invalid (%d)" % value


def limit(maximum, unit, scale=1):
    def decode(value):
        return "%d %s" % (value * scale, unit) if 0 < value <= maximum else "disabled"
    return decode


# (offset, name, decode)
FIELDS = [
    (0, "boot byte", lambda v: "%d (1 = firmware present)" % v),
    (1, "EEPROM layout version", str),
    (2, "bootloader version", str),
    (17, "direction reversed", on_off),
    (18, "3D mode (bi-direction)", on_off),
    (19, "sinusoidal startup", on_off),
    (20, "complementary PWM", on_off),
    (21, "variable PWM", on_off),
    (22, "stuck rotor protection", on_off),
    (23, "timing advance", advance),
    (24, "PWM frequency", lambda v: "%d kHz" % v),
    (25, "startup power", lambda v: "%d %%" % v),
    (26, "motor KV", lambda v: str(v * 40 + 20)),
    (27, "motor poles", str),
    (28, "brake on stop", on_off),
    (29, "stall protection", on_off),
    (30, "beeper volume", str),
    (31, "30 ms interval telemetry", on_off),
    (32, "servo low threshold", lambda v: "%d us" % (v * 2 + 750)),
    (33, "servo high threshold", lambda v: "%d us" % (v * 2 + 1750)),
    (34, "servo neutral", lambda v: "%d us" % (1374 + v)),
    (35, "servo dead band", str),
    (36, "low voltage cut off", on_off),
    (37, "low voltage threshold", lambda v: "%d (x0.01 V per cell)" % (250 + v)),
    (38, "car type reverse braking", on_off),
    (39, "use hall sensors", on_off),
    (40, "sine mode range", lambda v: "%d %%" % v),
    (41, "brake strength", str),
    (42, "running brake level", str),
    (43, "temperature limit", lambda v: "%d C" % v if 70 <= v <= 140 else "disabled"),
    (44, "current limit", limit(100, "A", 2)),
    (45, "sine mode power", str),
    (46, "input type", lambda v: {0: "auto", 1: "dshot", 2: "servo", 3: "serial", 4: "edt arm"}.get(v, str(v))),
    (47, "auto timing advance", on_off),
]


def firmware(block):
    return "%d.%d" % (block[3], block[4]), bytes(block[5:17]).decode("ascii", "replace").rstrip()


def decode(block):
    """[(offset, name, raw byte, decoded text)] for a settings block of a supported layout; ValueError otherwise."""
    block = bytes(block)
    if len(block) < 48:
        raise ValueError("settings block has %d bytes, need at least 48" % len(block))
    if block[1] not in SUPPORTED_LAYOUTS:
        raise ValueError("EEPROM layout version %d is not decoded here (supported: %s)" % (block[1], SUPPORTED_LAYOUTS))
    return [(offset, name, block[offset], text(block[offset])) for offset, name, text in FIELDS]


def as_dict(block):
    """One ESC's settings block as JSON-ready data: firmware version and name, every decoded field (byte offset, raw
    value, decoded text), and the whole block in hex so nothing is lost to the decoding."""
    version, name = firmware(block)
    return {
        "firmware": {"version": version, "name": name},
        "settings": {field: {"byte": offset, "raw": raw, "value": text} for offset, field, raw, text in decode(block)},
        "raw_hex": bytes(block).hex(),
    }
