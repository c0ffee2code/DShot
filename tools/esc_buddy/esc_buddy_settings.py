"""
esc_buddy_settings.py - decode an AM32 settings block (the "EEPROM" the bootloader serves at the settings address) into
named values, and encode the few fields the buddy may write. Pure Python, PC only.

Layout: EEPROM layout version 4 (byte 1), 192 bytes, as AM32 2.22 stores it (Inc/eeprom.h; loadEEpromSettings in Src/main.c gives
the accepted ranges, because the firmware ignores or replaces anything else). Bytes 0-4 are the boot byte, layout, bootloader
version and firmware version; the firmware name is no longer in the block (it sits at 0x7BE0, outside it). Older layouts are not
decoded (owner, 2026-11-09: firmware and layout moved together, no backward compatibility); a block of another layout version is
refused rather than guessed at.
"""

import math

import esc_buddy_protocol as protocol

SUPPORTED_LAYOUTS = (4,)


def on_off(value):
    return "on" if value else "off"


def advance(value):
    # AM32 2.22 loadEEpromSettings: 10-42 is the runtime advance plus 10 (0-32 steps); anything else is not a stored 2.22 value
    # and the firmware uses 16 steps. The degrees follow from how 2.22 converted the old 2.16 level 2 (15 degrees = 16 steps):
    # one step is about 30/32 degree, not checked against the configurator's display.
    if 10 <= value <= 42:
        return "%d steps (about %.1f deg)" % (value - 10, (value - 10) * 30 / 32)
    return "invalid (%d), firmware uses 16 steps" % value


def limit(maximum, unit, scale=1):
    def decode(value):
        return "%d %s" % (value * scale, unit) if 0 < value <= maximum else "disabled"
    return decode


def minimum_duty(value):
    # loadEEpromSettings: 1-50 becomes value * 10 on the firmware's 0-2000 throttle scale (0.5 % per step, up to 25 %); anything else is 0
    return "%.1f %%" % (value * 0.5) if 1 <= value <= 50 else "0 %% (raw %d: firmware uses none)" % value


def choice(names):
    """Decoder for a field that holds one of a few named states (the configurator's radio buttons)."""
    def decode(value):
        return names.get(value, "%d (unknown)" % value)
    return decode


PWM_TYPES = {0: "fixed", 1: "variable", 2: "by RPM"}                       # configurator "PWM Type"
LOW_VOLTAGE_MODES = {0: "off", 1: "cell based", 2: "absolute"}              # configurator "Low voltage cut off"
BRAKE_MODES = {0: "off", 1: "brake on stop", 2: "active brake"}             # configurator brake on stop radio
DEMAG_LEVELS = {0: "off", 1: "coast only", 2: "1/32 power cut", 3: "1/16 power cut", 4: "1/8 power cut"}    # Inc/eeprom.h comment


# (offset, name, decode)
FIELDS = [
    (0, "boot byte", lambda v: "%d (1 = firmware may start)" % v),
    (1, "EEPROM layout version", str),
    (2, "bootloader version", str),
    (5, "ramp rate", lambda v: "%.1f %% duty cycle per ms" % (v * 0.1)),          # configurator: 0.1 to 20, display factor 0.1
    (6, "minimum duty cycle", minimum_duty),
    (7, "disable stick calibration", on_off),
    (8, "absolute voltage cutoff", lambda v: "%.1f V" % (v * 0.5)),               # configurator: 1 to 100, display factor 0.5 V; used when low voltage cut off is absolute
    (9, "current limit P gain", str),
    (10, "current limit I gain", str),
    (11, "current limit D gain", str),
    (12, "active brake power", lambda v: "%d %%" % v),                              # configurator: 0 to 5
    (13, "brake on zero throttle", str),
    (14, "demag compensation", choice(DEMAG_LEVELS)),
    (17, "direction reversed", on_off),
    (18, "3D mode (bi-direction)", on_off),
    (19, "sinusoidal startup", on_off),
    (20, "complementary PWM", on_off),
    (21, "PWM type", choice(PWM_TYPES)),
    (22, "stuck rotor protection", on_off),
    (23, "timing advance steps", advance),
    (24, "PWM frequency", lambda v: "%d kHz" % v),
    (25, "startup power", lambda v: "%d %%" % v),
    (26, "motor KV", lambda v: str(v * 40 + 20)),
    (27, "motor poles", str),
    (28, "brake on stop", choice(BRAKE_MODES)),
    (29, "stall protection", on_off),
    (30, "beeper volume", str),
    (31, "30 ms interval telemetry", on_off),
    (32, "servo low threshold", lambda v: "%d us" % (v * 2 + 750)),
    (33, "servo high threshold", lambda v: "%d us" % (v * 2 + 1750)),
    (34, "servo neutral", lambda v: "%d us" % (1374 + v)),
    (35, "servo dead band", str),
    (36, "low voltage cut off", choice(LOW_VOLTAGE_MODES)),
    (37, "low voltage threshold", lambda v: "%.2f V per cell" % ((250 + v) / 100)),
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


# --- writing (the only fields the buddy may change) ---
# Bytes 0-4 (boot byte, layout, bootloader version, firmware version) and the last 16 bytes of the block (the CAN settings) are never
# changed by a settings write: a block is only ever the one just read with named fields replaced. (The firmware itself rewrites
# the layout and version bytes on its first boot after a flash; the buddy does not.)
PROTECTED = tuple(range(protocol.PROTECTED_HEAD)) + tuple(
    range(protocol.SETTINGS_SIZE - protocol.PROTECTED_TAIL, protocol.SETTINGS_SIZE))
KV_MIN, KV_STEP = 20, 40            # AM32 stores (KV - 20) / 40 in one byte
POLES_MIN, POLES_MAX = 2, 36        # the configurator allows 2 to 36; 2.22 loadEEpromSettings guards the 32 / poles division; kept even


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


BEEP_MAX = 11                       # loadEEpromSettings: a volume above 11 is replaced by 5


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


def integer_in(name, low, high, step=1, scale=1, can_disable=False, disabled_raw=0, offset=0):
    """Encoder for a field the firmware accepts only from `low` to `high` (in the field's own unit); the stored byte is
    value / scale + offset. Everything outside is refused: the firmware ignores or replaces it (loadEEpromSettings).
    `can_disable`: the word "disabled" stores `disabled_raw`, a value the firmware treats as no limit."""
    def encode(text):
        if can_disable and text.strip().lower() in ("disabled", "off"):
            return disabled_raw
        value = int(text)
        if not low <= value <= high or (value - low) % step or value % scale:
            raise ValueError("%s %d: need %d to %d%s" % (name, value, low, high, "" if step == 1 else " in steps of %d" % step))
        return value // scale + offset
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


def encode_choice(names):
    """Encoder for a named state: the name (any case) or its number."""
    by_name = {name.lower(): raw for raw, name in names.items()}

    def encode(text):
        word = text.strip().lower()
        if word in by_name:
            return by_name[word]
        if word.isdigit() and int(word) in names:
            return int(word)
        raise ValueError("%r: need one of %s" % (text, ", ".join(names[raw] for raw in sorted(names))))
    return encode


def encode_ramp_rate(text):
    # percent duty cycle per ms, 0.1 to 20 in steps of 0.1 (the configurator's range); stored in tenths
    rate = float(text)
    if not math.isfinite(rate) or not 0.1 <= rate <= 20 or abs(rate * 10 - round(rate * 10)) > 1e-6:
        raise ValueError("ramp rate %s: need 0.1 to 20 %% duty cycle per ms in steps of 0.1" % text)
    return round(rate * 10)


def encode_absolute_voltage(text):
    # volts in steps of 0.5, 0.5 to 50 (the configurator's range); stored in half volts; used when low voltage cut off is absolute
    volts = float(text)
    if not math.isfinite(volts) or not 0.5 <= volts <= 50 or abs(volts * 2 - round(volts * 2)) > 1e-6:
        raise ValueError("absolute voltage cutoff %s: need 0.5 to 50 V in steps of 0.5" % text)
    return round(volts * 2)


def encode_minimum_duty(text):
    # percent in steps of 0.5, 0 to 25; stored as half-percent steps (0 = none)
    percent = float(text)
    if not math.isfinite(percent) or not 0 <= percent <= 25 or abs(percent * 2 - round(percent * 2)) > 1e-6:
        raise ValueError("minimum duty cycle %s: need 0 to 25 %% in steps of 0.5" % text)
    return round(percent * 2)


# name -> (offset, encoder from the text given on the command line to the stored byte); names are FIELDS names.
# Not writable on purpose: direction, 3D mode, complementary PWM, hall sensors, input type, servo fields, car reverse braking
# (they change what the throttle means or are outside this bench), running brake level (it changes the dead time), and the fields
# whose meaning is not settled (brake on zero throttle: the configurator does not show it and the source gives no value list).
WRITABLE = {
    "motor KV": (26, encode_kv),
    "motor poles": (27, encode_poles),
    "beeper volume": (30, encode_beeper_volume),
    # start-up and running behaviour
    "sinusoidal startup": (19, encode_flag),
    "PWM type": (21, encode_choice(PWM_TYPES)),
    "timing advance steps": (23, integer_in("timing advance steps", 0, 32, offset=10)),     # steps of 30/32 degree, stored plus 10
    "auto timing advance": (47, encode_flag),
    "PWM frequency": (24, integer_in("PWM frequency", 8, 144)),         # kHz
    "startup power": (25, integer_in("startup power", 50, 150)),        # percent
    "minimum duty cycle": (6, encode_minimum_duty),                     # percent
    "ramp rate": (5, encode_ramp_rate),                                 # percent duty cycle per ms
    "disable stick calibration": (7, encode_flag),
    "demag compensation": (14, encode_choice(DEMAG_LEVELS)),
    "sine mode range": (40, integer_in("sine mode range", 5, 25)),      # percent throttle
    "sine mode power": (45, integer_in("sine mode power", 1, 10)),
    "brake on stop": (28, encode_choice(BRAKE_MODES)),
    "active brake power": (12, integer_in("active brake power", 0, 5)),          # percent
    "brake strength": (41, integer_in("brake strength", 1, 10)),        # drag brake
    # protections that can stop a running motor
    "stuck rotor protection": (22, encode_flag),
    "stall protection": (29, encode_flag),
    "low voltage cut off": (36, encode_choice(LOW_VOLTAGE_MODES)),
    "absolute voltage cutoff": (8, encode_absolute_voltage),
    "current limit P gain": (9, integer_in("current limit P gain", 0, 255)),
    "current limit I gain": (10, integer_in("current limit I gain", 0, 255)),
    "current limit D gain": (11, integer_in("current limit D gain", 0, 255)),
    "low voltage threshold": (37, encode_cell_voltage),
    "temperature limit": (43, integer_in("temperature limit", 70, 140, can_disable=True, disabled_raw=255)),  # degrees C, or disabled (255)
    "current limit": (44, integer_in("current limit", 2, 200, step=2, scale=2, can_disable=True)),  # amps (stored / 2), or disabled (0)
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
    """The firmware version stored in the block, e.g. '2.22' (the firmware writes it itself at boot)."""
    return "%d.%d" % (block[3], block[4])


def decode(block):
    """[(offset, name, raw byte, decoded text)] for a settings block of a supported layout; ValueError otherwise."""
    block = bytes(block)
    if len(block) < 48:
        raise ValueError("settings block has %d bytes, need at least 48" % len(block))
    if block[1] not in SUPPORTED_LAYOUTS:
        raise ValueError("EEPROM layout version %d is not decoded here (supported: %s)" % (block[1], SUPPORTED_LAYOUTS))
    return [(offset, name, block[offset], text(block[offset])) for offset, name, text in FIELDS]


def as_dict(block):
    """One ESC's settings block as JSON-ready data: firmware version, every decoded field (byte offset, raw value,
    decoded text), and the whole block in hex so nothing is lost to the decoding."""
    return {
        "firmware": {"version": firmware(block)},
        "settings": {field: {"byte": offset, "raw": raw, "value": text} for offset, field, raw, text in decode(block)},
        "raw_hex": bytes(block).hex(),
    }
