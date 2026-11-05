# Tests of the AM32 settings decoder (tools/esc_buddy/esc_buddy_settings.py) against a real settings block: the
# Skystars KM55A2's, read 2026-11-04 (all four ESCs identical), and the values the AM32 configurator displayed for it.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "esc_buddy"))

import esc_buddy_settings as s

KM55A2 = bytes.fromhex(
    "01020102104b4d35352046342020202020000000010101021864370e0000050080808032003200000f0a0a8d66060000") + b"\xff" * 136

# what the configurator showed for this block (owner's screenshot, tools/esc_buddy/dump/)
SHOWN = {
    "motor KV": "2220", "motor poles": "14", "startup power": "100 %", "timing advance": "15.0 deg",
    "PWM frequency": "24 kHz", "variable PWM": "on", "complementary PWM": "on", "stuck rotor protection": "on",
    "stall protection": "off", "use hall sensors": "off", "30 ms interval telemetry": "off", "auto timing advance": "off",
    "beeper volume": "5", "temperature limit": "disabled", "current limit": "disabled", "low voltage cut off": "off",
    "low voltage threshold": "300 (x0.01 V per cell)", "sinusoidal startup": "off", "sine mode range": "15 %",
    "sine mode power": "6", "brake on stop": "off", "car type reverse braking": "off", "brake strength": "10",
    "running brake level": "10", "servo low threshold": "1006 us", "servo high threshold": "2006 us",
    "servo neutral": "1502 us", "servo dead band": "50", "input type": "auto",
}


class Decode(unittest.TestCase):
    def test_matches_what_the_configurator_showed(self):
        decoded = {name: text for _, name, _, text in s.decode(KM55A2)}
        for name, shown in SHOWN.items():
            self.assertEqual(decoded[name], shown, name)

    def test_firmware_version_and_name(self):
        self.assertEqual(s.firmware(KM55A2), ("2.16", "KM55 F4"))

    def test_other_layouts_are_refused(self):
        block = bytearray(KM55A2)
        block[1] = 3
        with self.assertRaises(ValueError):
            s.decode(bytes(block))

    def test_short_block_is_refused(self):
        with self.assertRaises(ValueError):
            s.decode(KM55A2[:40])

    def test_as_dict_is_json_ready_and_keeps_the_raw_block(self):
        import json
        data = json.loads(json.dumps(s.as_dict(KM55A2)))
        self.assertEqual(data["firmware"], {"version": "2.16", "name": "KM55 F4"})
        self.assertEqual(data["settings"]["motor KV"], {"byte": 26, "raw": 55, "value": "2220"})
        self.assertEqual(bytes.fromhex(data["raw_hex"]), KM55A2)


if __name__ == "__main__":
    unittest.main()
