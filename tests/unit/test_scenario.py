# The scenario files and the code that loads them (tests/harness/scenario.py and
# throttle_profile.py). Both are plain Python, so a scenario that is wrong is
# caught here on a PC instead of on the bench, before anything is armed.

import sys
import unittest
from pathlib import Path

HARNESS = Path(__file__).resolve().parents[1] / "harness"
sys.path.insert(0, str(HARNESS))

import fakes  # noqa: E402,F401
from scenario import build_scenario, load_scenario  # noqa: E402
from throttle_profile import ThrottleProfile  # noqa: E402

SCENARIOS = sorted((HARNESS / "scenarios").glob("*.json"))


def valid_scenario():
    """A minimal valid scenario: motor 0 bidirectional alone on PIO0 (its frame
    receiver fills the block), three idle motors on the free PIO2 block. Every
    field here is one build_scenario() requires outright - there is no
    "leave it out and get a default" case to fall back on (see scenario.py's
    _require())."""
    idle = [{"type": "hold", "throttle": 0, "duration_ms": 1000}]
    return {
        "dshot_speed": "DSHOT300",
        "duration_ms": 1000,
        "arm_duration_ms": 3000,
        "status_interval_ms": 5000,
        "poll_ms": 10,
        "decode_every": 20,
        "gc_every_ms": 0,
        "expect": {},
        "motors": [
            {"pin": 6, "sm_id": 0, "bidirectional": True, "rx_sm_id": 1,
             "profile": [{"type": "hold", "throttle": 100, "duration_ms": 1000}]},
            {"pin": 7, "sm_id": 8, "bidirectional": False, "profile": idle},
            {"pin": 8, "sm_id": 9, "bidirectional": False, "profile": idle},
            {"pin": 9, "sm_id": 10, "bidirectional": False, "profile": idle},
        ],
    }


class ScenarioFilesTest(unittest.TestCase):
    def test_there_are_scenarios_to_check(self):
        self.assertTrue(SCENARIOS)

    def test_every_scenario_loads(self):
        for path in SCENARIOS:
            with self.subTest(path.name):
                scenario = load_scenario(str(path))
                self.assertEqual(len(scenario.motors), 4)
                self.assertGreater(scenario.duration_ms, 0)

    def test_every_scenario_arms_long_enough_and_names_the_bench_pins(self):
        # 2000ms matches AM32_SOURCE_VERIFICATION.md finding 1's computed floor
        # (>1s zero-throttle gate + 600ms startup tune + margin), not an
        # empirically re-verified safe value - see BUG-002 for evidence that
        # even more than this can still leave the ESC unarmed on some runs.
        for path in SCENARIOS:
            with self.subTest(path.name):
                scenario = load_scenario(str(path))
                self.assertGreaterEqual(scenario.arm_duration_ms, 2000)
                self.assertEqual(sorted(m.pin for m in scenario.motors), [6, 7, 8, 9])

    def test_every_scenario_stays_under_30s(self):
        # Keeps this checked-in set fast to iterate on (2026-09-27 decision) - a
        # deliberately long soak run is a different kind of tool, not a member
        # of this set, if one gets added back later.
        for path in SCENARIOS:
            with self.subTest(path.name):
                scenario = load_scenario(str(path))
                self.assertLessEqual(scenario.duration_ms, 30000)


class ScenarioValidationTest(unittest.TestCase):
    def test_the_minimal_scenario_is_valid(self):
        scenario = build_scenario(valid_scenario())
        self.assertEqual(scenario.bidir_indices, [0])
        self.assertEqual(scenario.dshot_speed, 2_400_000)

    def rejected(self, change, message=None):
        data = valid_scenario()
        change(data)
        with self.assertRaises(ValueError) as caught:
            build_scenario(data)
        if message:
            self.assertIn(message, str(caught.exception))

    def test_unknown_speed(self):
        self.rejected(lambda d: d.update(dshot_speed="DSHOT150"), "unknown dshot_speed")

    def test_wrong_motor_count(self):
        self.rejected(lambda d: d["motors"].pop(), "exactly 4 motors")

    def test_duplicate_pin(self):
        self.rejected(lambda d: d["motors"][1].update(pin=6), "duplicate pin")

    def test_duplicate_state_machine(self):
        self.rejected(lambda d: d["motors"][1].update(sm_id=0), "duplicate sm_id")

    def test_a_receiver_id_that_another_motor_uses(self):
        self.rejected(lambda d: d["motors"][1].update(sm_id=1), "duplicate sm_id")

    def test_bidirectional_needs_a_receiver_id(self):
        self.rejected(lambda d: d["motors"][0].pop("rx_sm_id"), "rx_sm_id is required")

    def test_receiver_id_must_follow_the_transmitter(self):
        self.rejected(lambda d: d["motors"][0].update(rx_sm_id=3), "sm_id + 1")

    def test_receiver_must_stay_in_the_transmitters_pio_block(self):
        def change(d):
            d["motors"][0].update(sm_id=3, rx_sm_id=4)
            d["motors"][3].update(sm_id=6)
        self.rejected(change, "must share a PIO block")

    def test_receiver_id_on_a_unidirectional_motor(self):
        self.rejected(lambda d: d["motors"][1].update(rx_sm_id=3), "bidirectional is false")

    def test_arming_frame_gap_defaults_to_back_to_back(self):
        self.assertEqual(build_scenario(valid_scenario()).arming_frame_gap_us, 0)

    def test_arming_frame_gap_is_read(self):
        data = valid_scenario()
        data["arming_frame_gap_us"] = 300
        self.assertEqual(build_scenario(data).arming_frame_gap_us, 300)

    def test_negative_arming_frame_gap(self):
        self.rejected(lambda d: d.update(arming_frame_gap_us=-1), "arming_frame_gap_us")

    def test_arming_class_bin_width_defaults_on(self):
        self.assertEqual(build_scenario(valid_scenario()).arming_class_bin_width_us, 100000)

    def test_arming_class_bin_width_is_read(self):
        data = valid_scenario()
        data["arming_class_bin_width_us"] = 0
        self.assertEqual(build_scenario(data).arming_class_bin_width_us, 0)

    def test_negative_arming_class_bin_width(self):
        self.rejected(lambda d: d.update(arming_class_bin_width_us=-1), "arming_class_bin_width_us")

    def test_a_unidirectional_motor_cannot_share_a_bidirectional_motors_pio_block(self):
        self.rejected(lambda d: d["motors"][1].update(sm_id=2), "shares PIO block")

    def test_two_bidirectional_motors_may_share_one_pio_block(self):
        # Identical programs are loaded once per block (ADR-002's layout table) -
        # only mixing with a unidirectional motor's different program is rejected.
        data = valid_scenario()
        data["motors"][1].update(bidirectional=True, sm_id=2, rx_sm_id=3,
                                  profile=[{"type": "hold", "throttle": 100, "duration_ms": 1000}])
        scenario = build_scenario(data)
        self.assertEqual(scenario.bidir_indices, [0, 1])

    def test_every_motor_needs_a_profile(self):
        self.rejected(lambda d: d["motors"][1].pop("profile"), "profile is required")

    def test_every_motor_needs_bidirectional_stated_explicitly(self):
        self.rejected(lambda d: d["motors"][1].pop("bidirectional"), "missing required field: bidirectional")

    def test_profile_must_last_as_long_as_the_scenario(self):
        self.rejected(lambda d: d["motors"][1]["profile"][0].update(duration_ms=500), "!= scenario duration_ms")

    def test_a_crc_threshold_needs_a_bidirectional_motor(self):
        self.rejected(lambda d: d.update(expect={"min_crc_valid_pct": {"1": 99.0}}), "not declared bidirectional")

    def test_a_median_erpm_threshold_needs_a_bidirectional_motor(self):
        self.rejected(lambda d: d.update(expect={"min_median_erpm": {"1": 10000}}), "not declared bidirectional")

    def test_thresholds_for_a_bidirectional_motor_are_accepted(self):
        data = valid_scenario()
        data["expect"] = {"min_crc_valid_pct": {"0": 99.0}, "min_median_erpm": {"0": 10000}}
        self.assertEqual(build_scenario(data).expect["min_median_erpm"], {"0": 10000})

    def test_decode_every_is_required_but_can_be_turned_off(self):
        self.rejected(lambda d: d.pop("decode_every"), "missing required field: decode_every")
        data = valid_scenario()
        data["decode_every"] = 0
        self.assertEqual(build_scenario(data).decode_every, 0)

    def test_decode_every_must_be_a_non_negative_whole_number(self):
        for bad in (-1, 2.5, "20"):
            data = valid_scenario()
            data["decode_every"] = bad
            with self.assertRaises(ValueError):
                build_scenario(data)

    def test_gc_every_ms_is_required_and_must_be_a_non_negative_whole_number(self):
        self.rejected(lambda d: d.pop("gc_every_ms"), "missing required field: gc_every_ms")
        data = valid_scenario()
        data["gc_every_ms"] = 100
        self.assertEqual(build_scenario(data).gc_every_ms, 100)
        for bad in (-5, 1.5, "100"):
            data["gc_every_ms"] = bad
            with self.assertRaises(ValueError):
                build_scenario(data)

    def test_no_field_is_silently_defaulted(self):
        """
        This is a test suite, not an application - a scenario JSON that omits
        a setting is a scenario mistake and must fail loudly, never fall back
        to a guessed value. See scenario.py's _require().
        """
        for field in ("arm_duration_ms", "status_interval_ms", "poll_ms", "expect"):
            self.rejected(lambda d, field=field: d.pop(field), "missing required field: " + field)


class ThrottleProfileTest(unittest.TestCase):
    def test_hold(self):
        profile = ThrottleProfile([{"type": "hold", "throttle": 100, "duration_ms": 1000},
                                   {"type": "hold", "throttle": 300, "duration_ms": 500}])
        self.assertEqual(profile.total_duration_ms, 1500)
        self.assertEqual([profile.throttle_at(t) for t in (0, 999, 1000, 1499)], [100, 100, 300, 300])

    def test_ramp_moves_in_equal_steps(self):
        profile = ThrottleProfile([{"type": "hold", "throttle": 100, "duration_ms": 100},
                                   {"type": "ramp", "to": 200, "step": 25, "duration_ms": 400}])
        # 4 steps of 25 every 100 ms after the hold
        self.assertEqual([profile.throttle_at(t) for t in (100, 199, 200, 300, 400, 500)],
                         [100, 100, 125, 150, 175, 200])

    def test_ramp_down(self):
        profile = ThrottleProfile([{"type": "hold", "throttle": 200, "duration_ms": 100},
                                   {"type": "ramp", "to": 100, "step": 50, "duration_ms": 200}])
        self.assertEqual([profile.throttle_at(t) for t in (100, 200, 300)], [200, 150, 100])

    def test_repeat_expands_its_inner_segments(self):
        profile = ThrottleProfile([{"type": "repeat", "duration_ms": 400, "segments": [
            {"type": "hold", "throttle": 100, "duration_ms": 100},
            {"type": "hold", "throttle": 200, "duration_ms": 100}]}])
        self.assertEqual(profile.total_duration_ms, 400)
        self.assertEqual([profile.throttle_at(t) for t in (0, 100, 200, 300)], [100, 200, 100, 200])

    def test_a_profile_starts_at_zero_when_it_does_not_say_otherwise(self):
        profile = ThrottleProfile([{"type": "ramp", "to": 100, "step": 100, "duration_ms": 100}])
        self.assertEqual(profile.throttle_at(0), 0)
        self.assertEqual(profile.throttle_at(100), 100)

    def rejected(self, segments, message):
        with self.assertRaises(ValueError) as caught:
            ThrottleProfile(segments)
        self.assertIn(message, str(caught.exception))

    def test_unknown_segment_type(self):
        self.rejected([{"type": "wobble"}], "unknown segment type")

    def test_ramp_step_must_be_positive(self):
        self.rejected([{"type": "ramp", "to": 100, "step": 0, "duration_ms": 100}], "must be positive")

    def test_ramp_must_divide_evenly_into_steps(self):
        self.rejected([{"type": "ramp", "to": 100, "step": 30, "duration_ms": 100}], "does not divide evenly")

    def test_ramp_duration_must_divide_evenly_into_steps(self):
        self.rejected([{"type": "ramp", "to": 100, "step": 50, "duration_ms": 101}], "does not divide evenly")

    def test_repeat_must_fit_its_duration_exactly(self):
        self.rejected([{"type": "repeat", "duration_ms": 350, "segments": [
            {"type": "hold", "throttle": 1, "duration_ms": 100}]}], "not an exact multiple")

    def test_repeat_of_nothing_is_rejected(self):
        self.rejected([{"type": "repeat", "duration_ms": 100, "segments": [
            {"type": "hold", "throttle": 1, "duration_ms": 0}]}], "zero duration")

    def test_hold_throttle_out_of_range_is_rejected(self):
        self.rejected([{"type": "hold", "throttle": 2048, "duration_ms": 100}], "0..2047")
        self.rejected([{"type": "hold", "throttle": -1, "duration_ms": 100}], "0..2047")

    def test_ramp_target_out_of_range_is_rejected(self):
        self.rejected([{"type": "ramp", "to": 2048, "step": 1, "duration_ms": 100}], "0..2047")
        self.rejected([{"type": "ramp", "to": -1, "step": 1, "duration_ms": 100}], "0..2047")


if __name__ == "__main__":
    unittest.main()
