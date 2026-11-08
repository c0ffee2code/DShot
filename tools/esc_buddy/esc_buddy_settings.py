"""
esc_buddy_settings.py - decode an AM32 settings block (the "EEPROM" the bootloader serves at the settings address) into
named values. Pure Python, PC only.

Layout: EEPROM layout version 2 (byte 1), as AM32 firmware 2.16 stores it - bytes 5-16 hold the firmware name, settings start
at byte 17. Field meanings and encodings as the AM32 wiki's "Open ESC EEPROM Format" page and the configurator show them;
checked against the configurator's own display of the KM55A2's settings (owner's screenshot, 2026-11-04). A block of
another layout version is refused rather than guessed at.
"""

import math

import esc_buddy_protocol as protocol

SUPPORTED_LAYOUTS = (2,)


def on_off(value):
    return "on" if value else "off"


def advance(value):
    # AM32 2.16 loadEEpromSettings: a value below 4 is 7.5 degree steps, anything else is replaced by level 2 (15 degrees)
    if value <= 3:
        return "%.1f deg" % (value * 7.5)
    return "invalid (%d), firmware uses 15.0 deg" % value


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
    (44, "current limit", limit(99, "A", 2)),
    (45, "sine mode power", str),
    (46, "input type", lambda v: {0: "auto", 1: "dshot", 2: "servo", 3: "serial", 4: "edt arm"}.get(v, str(v))),
    (47, "auto timing advance", on_off),
]


# --- writing (the only fields the buddy may change) ---
# Bytes 0-16 (boot byte, layout, bootloader version, firmware version and name) and the last 8 bytes of the block are never
# changed by the buddy: the configurator blanks the name and pads the tail when it saves (recorded 2026-11-05, README), so a
# block is only ever the one just read with named fields replaced.
PROTECTED = tuple(range(protocol.PROTECTED_HEAD)) + tuple(
    range(protocol.SETTINGS_SIZE - protocol.PROTECTED_TAIL, protocol.SETTINGS_SIZE))
KV_MIN, KV_STEP = 20, 40            # AM32 stores (KV - 20) / 40 in one byte
POLES_MIN, POLES_MAX = 2, 32        # AM32 2.16 main.c:830-832 divides by (32 / motor_poles), which is zero above 32; 2.16 uses poles / 2 elsewhere, so even


def encode_kv(text):
    kv = int(text)
    raw, rest = divmod(kv - KV_MIN, KV_STEP)
    if rest or not 0 <= raw <= 255:
        below = KV_MIN + KV_STEP * max(0, min(255, (kv - KV_MIN) // KV_STEP))
        raise ValueError("motor KV %d cannot be stored: values go in steps of %d from %d (nearest: %d or %d)" % (
            kv, KV_STEP, KV_MIN, below, below + KV_STEP))
    return raw


def encode_poles(text):
    poles = int(text)
    if poles % 2 or not POLES_MIN <= poles <= POLES_MAX:
        raise ValueError("motor poles %d: need an even number from %d to %d" % (poles, POLES_MIN, POLES_MAX))
    return poles


BEEP_MAX = 11                       # AM32 2.16 loadEEpromSettings: a volume above 11 is replaced by 5


def encode_beeper_volume(text):
    volume = int(text)
    if not 0 <= volume <= BEEP_MAX:
        raise ValueError("beeper volume %d: need 0 to %d" % (volume, BEEP_MAX))
    return volume


def encode_flag(text):
    # the firmware reads a flag as on only when the byte is exactly 0x01, so only 0 and 1 are ever written
    word = text.strip().lower()
    if word in ("on", "1"):
        return 1
    if word in ("off", "0"):
        return 0
    raise ValueError("%r: need on or off" % text)


def integer_in(name, low, high, step=1, scale=1, can_disable=False):
    """Encoder for a field the firmware accepts only from `low` to `high` (in the field's own unit); the stored byte is
    value / scale. Everything outside is refused: the firmware ignores or replaces it (AM32 2.16 loadEEpromSettings).
    `can_disable`: the word "disabled" stores 0, which the firmware ignores, so the field keeps its default of no limit
    (2.16 main.c: TEMPERATURE_LIMIT 255 = off, use_current_limit 0) - the value the vendor stored and `--read` calls disabled."""
    def encode(text):
        if can_disable and text.strip().lower() in ("disabled", "off"):
            return 0
        value = int(text)
        if not low <= value <= high or (value - low) % step or value % scale:
            raise ValueError("%s %d: need %d to %d%s" % (name, value, low, high, "" if step == 1 else " in steps of %d" % step))
        return value // scale
    return encode


def encode_cell_voltage(text):
    # stored as hundredths of a volt above 2.50 V; the firmware comment gives the range 2.5 to 3.5 V per cell
    volts = float(text)
    if not math.isfinite(volts):
        raise ValueError("low voltage threshold %s: need 2.50 to 3.50 V per cell, in steps of 0.01" % text)
    hundredths = round(volts * 100)
    if abs(volts * 100 - hundredths) > 1e-6 or not 250 <= hundredths <= 350:
        raise ValueError("low voltage threshold %s: need 2.50 to 3.50 V per cell, in steps of 0.01" % text)
    return hundredths - 250


# name -> (offset, encoder from the text given on the command line to the stored byte); names are FIELDS names.
# Not writable on purpose: direction, 3D mode, complementary PWM, hall sensors, input type, servo fields, car reverse braking
# (they change what the throttle means or are outside this bench) and running brake level (it changes the dead time).
WRITABLE = {
    "motor KV": (26, encode_kv),
    "motor poles": (27, encode_poles),
    "beeper volume": (30, encode_beeper_volume),
    # start-up and running behaviour
    "sinusoidal startup": (19, encode_flag),
    "variable PWM": (21, encode_flag),
    "timing advance": (23, integer_in("timing advance level", 0, 3)),     # level x 7.5 degrees
    "auto timing advance": (47, encode_flag),
    "PWM frequency": (24, integer_in("PWM frequency", 8, 48)),          # kHz
    "startup power": (25, integer_in("startup power", 50, 150)),        # percent
    "sine mode range": (40, integer_in("sine mode range", 5, 25)),      # percent throttle
    "sine mode power": (45, integer_in("sine mode power", 1, 10)),
    "brake on stop": (28, encode_flag),
    "brake strength": (41, integer_in("brake strength", 1, 10)),        # drag brake
    # protections that can stop a running motor
    "stuck rotor protection": (22, encode_flag),
    "stall protection": (29, encode_flag),
    "low voltage cut off": (36, encode_flag),
    "low voltage threshold": (37, encode_cell_voltage),
    "temperature limit": (43, integer_in("temperature limit", 70, 140, can_disable=True)),  # degrees C, or disabled
    "current limit": (44, integer_in("current limit", 2, 198, step=2, scale=2, can_disable=True)),  # amps (stored / 2), or disabled
}


def apply_changes(block, changes):
    """The settings block with `changes` ({field name: value text}) applied; ValueError for a field that may not be written
    or a value that cannot be stored. Never touches PROTECTED bytes. Returns (new block, [(field, offset, old raw, new raw)])."""
    new = bytearray(block)
    done = []
    for name, text in changes.items():
        if name not in WRITABLE:
            raise ValueError("%r cannot be written by the buddy (writable: %s)" % (name, ", ".join(sorted(WRITABLE))))
        offset, encode = WRITABLE[name]
        assert offset not in PROTECTED
        try:
            raw = encode(text)
        except ValueError as error:
            raise ValueError(str(error)) from None
        done.append((name, offset, new[offset], raw))
        new[offset] = raw
    return bytes(new), done


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
