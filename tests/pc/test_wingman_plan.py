# Tests of tests/bench/wingman/tools/wingman_plan.py, which reads a scenario and says what the wingman has to
# do: which lines to spy on, which to play the ESC on, which start line, how long to wait. The real scenario files are test input.

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import start_line
import wingman_plan

SCENARIOS = ROOT / "tests" / "bench" / "dshot_protocol" / "scenarios"


def load(name):
    return json.loads((SCENARIOS / name).read_text(encoding="utf-8"))


def scenario(pins=(6, 7, 8, 9), bidirectional=(True, True, True, True), speed="DSHOT600", duration=20000):
    data = {"dshot_speed": speed, "motors": [{"pin": p, "bidirectional": b} for p, b in zip(pins, bidirectional)]}
    if duration is not None:
        data["duration_ms"] = duration
    return data


class PlanFromRealScenariosTest(unittest.TestCase):
    def test_the_bidirectional_scenario(self):
        plan = wingman_plan.plan_from_scenario(load("four_channel_bidirectional_600.json"), esc_mock=True)
        self.assertEqual([6, 7, 8, 9], plan.spy_pins)
        self.assertEqual([True] * 4, plan.bidirectional)
        self.assertEqual(start_line.START_PIN, plan.start_line_pin)
        self.assertEqual("DSHOT600", plan.dshot_speed)
        self.assertEqual(20 + wingman_plan.RUN_MARGIN_S, plan.run_seconds)
        self.assertEqual([6, 7, 8, 9], plan.mock.pins)
        self.assertEqual([0, 1, 2, 3], plan.mock.lines)
        self.assertEqual("esc_mock_replies_dshot600.bin", plan.mock.fixture)

    def test_the_unidirectional_scenario_is_spied_but_not_mocked(self):
        data = load("four_channel_unidirectional_600.json")
        plan = wingman_plan.plan_from_scenario(data)
        self.assertEqual([6, 7, 8, 9], plan.spy_pins)
        self.assertEqual([False] * 4, plan.bidirectional)
        self.assertIsNone(plan.mock)
        with self.assertRaisesRegex(ValueError, "none"):
            wingman_plan.plan_from_scenario(data, esc_mock=True)


class FixtureProvenanceTest(unittest.TestCase):
    def test_the_plan_carries_where_the_recorded_replies_came_from(self):
        plan = wingman_plan.plan_from_scenario(load("four_channel_bidirectional_600.json"), esc_mock=True)
        entry = plan.mock.manifest_entry()
        self.assertEqual("esc_mock_replies_dshot600.bin", entry["fixture"])
        for key in ("source_session", "source_git_commit", "built", "replies_per_line", "median_erpm_per_line"):
            self.assertIn(key, entry["source"])

    def test_replies_without_a_provenance_file_are_refused(self):
        import shutil
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            shutil.copy(wingman_plan.BOARD_DIR / "esc_mock_replies_dshot600.bin", tmp)
            saved = wingman_plan.BOARD_DIR
            wingman_plan.BOARD_DIR = Path(tmp)
            try:
                with self.assertRaisesRegex(ValueError, "provenance"):
                    wingman_plan.plan_from_scenario(scenario(), esc_mock=True)
            finally:
                wingman_plan.BOARD_DIR = saved


class PinsComeFromTheScenarioTest(unittest.TestCase):
    def test_other_pins_are_followed(self):
        plan = wingman_plan.plan_from_scenario(scenario(pins=(10, 11, 12, 13)), esc_mock=True)
        self.assertEqual([10, 11, 12, 13], plan.spy_pins)
        self.assertEqual([10, 11, 12, 13], plan.mock.pins)

    def test_fewer_motors_fewer_lines(self):
        plan = wingman_plan.plan_from_scenario(scenario(pins=(6, 9), bidirectional=(True, True)))
        self.assertEqual([6, 9], plan.spy_pins)

    def test_the_mock_plays_only_the_bidirectional_lines_and_keeps_their_positions(self):
        plan = wingman_plan.plan_from_scenario(scenario(bidirectional=(False, True, False, True)), esc_mock=True)
        self.assertEqual([6, 7, 8, 9], plan.spy_pins)                 # the spy watches every line
        self.assertEqual([7, 9], plan.mock.pins)
        self.assertEqual([1, 3], plan.mock.lines)                      # motor 1 plays fixture line 1, motor 3 line 3

    def test_a_scenario_without_duration_ms_gets_the_margin_alone(self):
        plan = wingman_plan.plan_from_scenario(scenario(duration=None))
        self.assertEqual(wingman_plan.RUN_MARGIN_S, plan.run_seconds)


class RefusalsTest(unittest.TestCase):
    def refuses(self, data, words, **kwargs):
        with self.assertRaisesRegex(ValueError, words):
            wingman_plan.plan_from_scenario(data, **kwargs)

    def test_no_motors(self):
        self.refuses({"dshot_speed": "DSHOT600", "motors": []}, "no motors")
        self.refuses({"dshot_speed": "DSHOT600"}, "no motors")

    def test_too_many_motors(self):
        self.refuses(scenario(pins=(6, 7, 8, 9, 10), bidirectional=(True,) * 5), "at most 4")

    def test_two_motors_on_one_pin(self):
        self.refuses(scenario(pins=(6, 6), bidirectional=(True, True)), "already uses")

    def test_a_motor_on_the_start_line(self):
        self.refuses(scenario(pins=(6, start_line.START_PIN), bidirectional=(True, True)), "start line")

    def test_a_motor_without_a_pin_or_a_direction(self):
        self.refuses({"dshot_speed": "DSHOT600", "motors": [{"bidirectional": True}]}, "no whole-number pin")
        self.refuses({"dshot_speed": "DSHOT600", "motors": [{"pin": 6}]}, "bidirectional")

    def test_missing_or_unknown_speed(self):
        data = scenario()
        del data["dshot_speed"]
        self.refuses(data, "dshot_speed")
        self.refuses(scenario(speed="DSHOT1200"), "dshot_speed")

    def test_dshot300_is_spied_but_the_mock_refuses_it(self):
        data = scenario(speed="DSHOT300")
        self.assertEqual("DSHOT300", wingman_plan.plan_from_scenario(data).dshot_speed)
        self.refuses(data, "DSHOT600 only", esc_mock=True)

    def test_a_bad_duration(self):
        self.refuses(scenario(duration=-5), "duration_ms")

    def test_the_plan_does_not_change_the_scenario_it_reads(self):
        data = scenario()
        before = copy.deepcopy(data)
        wingman_plan.plan_from_scenario(data, esc_mock=True)
        self.assertEqual(before, data)


if __name__ == "__main__":
    unittest.main()
