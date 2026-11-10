# Tests of the AM32 settings decoder (tools/esc_buddy/esc_buddy_settings.py) against a real layout 4 settings block: the
# Skystars KM55A2's after AM32 2.22 had been flashed on it and had migrated its own settings page (spare board, read 2026-11-09,
# all four ESCs identical; motor KV 7180 and 14 poles are the values set on the bench, the rest are what the firmware stored).

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "esc_buddy"))

import esc_buddy_settings as s

KM55A2_2_22 = bytes.fromhex(
    "0104010216a001000a64006400000000000000000101011a1864b30e0000010080808032003200000f0a0aff66060000") + b"\xff" * 144

# the values in plain units; the encodings come from AM32 2.22's loadEEpromSettings (comments in esc_buddy_settings.py)
SHOWN = {
    "boot byte": "1 (1 = firmware may start)", "EEPROM layout version": "4", "bootloader version": "1",
    "ramp rate": "16.0 % duty cycle per ms", "minimum duty cycle": "0.5 %", "disable stick calibration": "off", "absolute voltage cutoff": "5.0 V",
    "current limit P gain": "100", "current limit I gain": "0", "current limit D gain": "100", "active brake power": "0 %",
    "brake on zero throttle": "0", "demag compensation": "off",
    "motor KV": "7180", "motor poles": "14", "startup power": "100 %", "timing advance steps": "16 steps (about 15.0 deg)",
    "PWM frequency": "24 kHz", "PWM type": "variable", "complementary PWM": "on", "stuck rotor protection": "on",
    "stall protection": "off", "use hall sensors": "off", "30 ms interval telemetry": "off", "auto timing advance": "off",
    "beeper volume": "1", "temperature limit": "disabled", "current limit": "disabled", "low voltage cut off": "off",
    "low voltage threshold": "3.00 V per cell", "sinusoidal startup": "off", "sine mode range": "15 %",
    "sine mode power": "6", "brake on stop": "off", "car type reverse braking": "off", "brake strength": "10",
    "running brake level": "10", "servo low threshold": "1006 us", "servo high threshold": "2006 us",
    "servo neutral": "1502 us", "servo dead band": "50", "input type": "auto", "direction reversed": "off",
    "3D mode (bi-direction)": "off",
}


class Decode(unittest.TestCase):
    def test_every_decoded_field_is_checked_against_the_bench_block(self):
        decoded = {name: text for _, name, _, text in s.decode(KM55A2_2_22)}
        self.assertEqual(decoded, SHOWN)

    def test_firmware_version(self):
        self.assertEqual(s.firmware(KM55A2_2_22), "2.22")

    def test_other_layouts_are_refused(self):
        for layout in (2, 3, 5):
            block = bytearray(KM55A2_2_22)
            block[1] = layout
            with self.assertRaises(ValueError) as caught:
                s.decode(bytes(block))
            self.assertIn("layout version %d" % layout, str(caught.exception))

    def test_short_block_is_refused(self):
        with self.assertRaises(ValueError):
            s.decode(KM55A2_2_22[:40])

    def test_as_dict_is_json_ready_and_keeps_the_raw_block(self):
        import json
        data = json.loads(json.dumps(s.as_dict(KM55A2_2_22)))
        self.assertEqual(data["firmware"], {"version": "2.22"})
        self.assertEqual(data["settings"]["motor KV"], {"byte": 26, "raw": 179, "value": "7180"})
        self.assertEqual(bytes.fromhex(data["raw_hex"]), KM55A2_2_22)

    def test_timing_advance_steps_and_values_outside_10_to_42(self):
        self.assertEqual(s.advance(10), "0 steps (about 0.0 deg)")
        self.assertEqual(s.advance(42), "32 steps (about 30.0 deg)")
        for invalid in (0, 2, 9, 43):
            self.assertIn("firmware uses 16 steps", s.advance(invalid))

    def test_the_three_state_fields_show_their_names_and_flag_an_unknown_value(self):
        by_name = dict((name, text) for _, name, text in s.FIELDS)
        self.assertEqual([by_name["PWM type"](v) for v in (0, 1, 2)], ["fixed", "variable", "by RPM"])
        self.assertEqual([by_name["low voltage cut off"](v) for v in (0, 1, 2)], ["off", "cell based", "absolute"])
        self.assertEqual([by_name["brake on stop"](v) for v in (0, 1, 2)], ["off", "brake on stop", "active brake"])
        self.assertEqual(by_name["demag compensation"](4), "1/8 power cut")
        for name in ("PWM type", "low voltage cut off", "brake on stop"):
            self.assertIn("unknown", by_name[name](7))

    def test_ramp_rate_absolute_voltage_and_active_brake_in_plain_units(self):
        by_name = dict((name, text) for _, name, text in s.FIELDS)
        self.assertEqual(by_name["ramp rate"](1), "0.1 % duty cycle per ms")
        self.assertEqual(by_name["ramp rate"](200), "20.0 % duty cycle per ms")
        self.assertEqual((by_name["absolute voltage cutoff"](1), by_name["absolute voltage cutoff"](100)), ("0.5 V", "50.0 V"))
        self.assertEqual(by_name["active brake power"](5), "5 %")

    def test_minimum_duty_is_half_percent_steps_and_zero_outside_1_to_50(self):
        self.assertEqual((s.minimum_duty(1), s.minimum_duty(50)), ("0.5 %", "25.0 %"))
        self.assertIn("0 %", s.minimum_duty(0))
        self.assertIn("0 %", s.minimum_duty(51))

    def test_current_limit_and_temperature_limit_show_disabled_outside_the_accepted_range(self):
        by_name = dict((name, text) for _, name, text in s.FIELDS)
        self.assertEqual(by_name["current limit"](100), "200 A")
        self.assertEqual((by_name["current limit"](0), by_name["current limit"](101)), ("disabled", "disabled"))
        self.assertEqual((by_name["temperature limit"](70), by_name["temperature limit"](140)), ("70 C", "140 C"))
        self.assertEqual((by_name["temperature limit"](69), by_name["temperature limit"](255)), ("disabled", "disabled"))


if __name__ == "__main__":
    unittest.main()
