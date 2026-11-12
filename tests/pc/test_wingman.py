# Tests of the PC side of the wingman that touches no hardware: tests/bench/wingman/tools/wingman_board.py (finding the wingman
# and refusing the main Pico), wingman_run.esc_mock_code (the program the launcher sends to the board) and
# run_session.release_wingman / its use when an ESC mock session ends badly.

import ast
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import run_session
import wingman_board
import wingman_plan
import wingman_run


class FakePort:
    def __init__(self, serial_number, device):
        self.serial_number = serial_number
        self.device = device


class FakeProcess:
    def __init__(self, running=True):
        self.running = running
        self.killed = False

    def poll(self):
        return None if self.running else 0

    def kill(self):
        self.killed = True
        self.running = False

    def communicate(self, timeout=None):
        return "", ""


class PatchedTest(unittest.TestCase):
    def patch(self, obj, name, value):
        saved = getattr(obj, name)
        setattr(obj, name, value)
        self.addCleanup(setattr, obj, name, saved)


class FindBoardTest(PatchedTest):
    def test_the_wingman_is_found_by_id_and_the_main_pico_never(self):
        self.patch(wingman_board.list_ports, "comports", lambda: [
            FakePort(wingman_board.MAIN_ID.upper(), "COM10"), FakePort("other", "COM3"), FakePort(wingman_board.WINGMAN_ID, "COM7")])
        self.assertEqual("COM7", wingman_board.find_port())

    def test_only_the_main_pico_connected_means_no_wingman(self):
        self.patch(wingman_board.list_ports, "comports", lambda: [FakePort(wingman_board.MAIN_ID, "COM10")])
        self.assertIsNone(wingman_board.find_port())

    def reports(self, board_id):
        class Result:
            stdout = board_id + "\n"
            stderr = ""
        self.patch(wingman_board, "mpremote", lambda *a, **k: Result())

    def test_check_board_refuses_the_main_pico(self):
        self.reports(wingman_board.MAIN_ID)
        with self.assertRaises(SystemExit) as raised:
            wingman_board.check_board("COM10")
        self.assertIn("MAIN Pico", str(raised.exception))

    def test_check_board_refuses_an_unknown_board_and_accepts_the_wingman(self):
        self.reports("0123456789abcdef")
        with self.assertRaises(SystemExit):
            wingman_board.check_board("COM3")
        self.reports(wingman_board.WINGMAN_ID)
        wingman_board.check_board("COM7")


def plan(bidirectional=(True, True, True, True), pins=(6, 7, 8, 9), esc_mock=True):
    data = {"dshot_speed": "DSHOT600", "duration_ms": 20000,
            "motors": [{"pin": p, "bidirectional": b} for p, b in zip(pins, bidirectional)]}
    return wingman_plan.plan_from_scenario(data, esc_mock=esc_mock)


class EscMockCodeTest(unittest.TestCase):
    def test_the_program_is_valid_python_and_keeps_the_mock_safe(self):
        code = wingman_run.esc_mock_code(plan())
        ast.parse(code)
        self.assertIn("esc_mock.start(pins=(6,7,8,9,), fixture='esc_mock_replies_dshot600.bin', lines=(0,1,2,3,))", code)
        self.assertIn("configure_pins=False", code)             # the observer must not hand the pins back to plain GPIO
        self.assertIn("tick=esc_mock.guard((6,7,8,9,))", code)
        self.assertLess(code.index("esc_mock.start"), code.index("observer.run"))     # the mock first
        self.assertTrue(code.rstrip().endswith("mock.stop()"))
        self.assertLess(code.index("finally:"), code.index("mock.stop()"))      # stopped even if the observer raises

    def test_a_mixed_scenario_mocks_only_the_bidirectional_line_and_spies_on_all(self):
        code = wingman_run.esc_mock_code(plan(bidirectional=(True, False, False, False)))
        ast.parse(code)
        self.assertIn("esc_mock.start(pins=(6,), fixture=", code)
        self.assertIn("lines=(0,))", code)
        self.assertIn("observer.run(pins=(6,7,8,9,), sync_pin=15", code)
        self.assertIn("tick=esc_mock.guard((6,))", code)
        for pin in (7, 8, 9, 15):                                    # lines the mock does not play are made inputs by hand
            self.assertIn("    Pin(%d, Pin.IN)" % pin, code)
        self.assertNotIn("    Pin(6, Pin.IN)", code)                  # the mock's own pin is the mock's

    def test_other_pins_appear_in_the_program_and_nothing_else_does(self):
        code = wingman_run.esc_mock_code(plan(pins=(10, 11, 12, 13)))
        self.assertIn("pins=(10,11,12,13,)", code)
        self.assertNotIn("6,7,8,9", code)

    def test_the_observer_only_program(self):
        code = wingman_run.observer_code(plan(esc_mock=False, pins=(7, 9), bidirectional=(False, False)))
        ast.parse(code)
        self.assertIn("observer.run(pins=(7,9,), sync_pin=15, wait_ms=150000)", code)
        self.assertNotIn("esc_mock", code)


class ReleaseWingmanTest(PatchedTest):
    def setUp(self):
        self.resets = []
        self.patch(run_session.wingman_board, "find_port", lambda: "COM7")
        self.patch(run_session.wingman_board, "reset", lambda port: self.resets.append(port))

    def test_a_running_launcher_is_killed_and_the_board_reset(self):
        process = FakeProcess(running=True)
        run_session.release_wingman(process)
        self.assertTrue(process.killed)
        self.assertEqual(["COM7"], self.resets)

    def test_an_ended_launcher_is_left_alone_but_the_board_is_still_reset(self):
        process = FakeProcess(running=False)
        run_session.release_wingman(process)
        self.assertFalse(process.killed)
        self.assertEqual(["COM7"], self.resets)

    def test_no_launcher_and_no_board_is_fine(self):
        self.patch(run_session.wingman_board, "find_port", lambda: None)
        run_session.release_wingman(None)
        self.assertEqual([], self.resets)


class MockSessionEndsBadlyTest(PatchedTest):
    """The deploy fails, so run_session kills the launcher; a killed launcher runs no cleanup, so run_session must reset the board."""

    def run_it(self, esc_mock):
        import subprocess
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        scenario = ROOT / "tests" / "bench" / "dshot_protocol" / "scenarios" / "four_channel_bidirectional_600.json"
        self.process = FakeProcess(running=True)
        self.resets = []
        self.patch(run_session, "CAPTURES", Path(tmp.name) / "captures")
        self.patch(run_session, "start_observer", lambda *a, **k: (self.process, []))
        self.patch(run_session.time, "sleep", lambda s: None)
        self.patch(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 1, stdout="Uploaded 3, failed 2.", stderr=""))
        self.patch(run_session, "collect_sd_session", lambda *a, **k: False)
        self.patch(run_session.session_summary, "write_summary", lambda folder: {"verdict": "unknown"})
        self.patch(run_session.wingman_board, "find_port", lambda: "COM7")
        self.patch(run_session.wingman_board, "reset", lambda port: self.resets.append(port))
        run_session.run_session(scenario, observer=True, echo=False, esc_mock=esc_mock)

    def test_the_board_is_reset_after_a_mock_session_whose_launcher_was_killed(self):
        self.run_it(esc_mock=True)
        self.assertTrue(self.process.killed)
        self.assertEqual(["COM7"], self.resets)

    def test_a_plain_observer_session_does_not_reset_the_board(self):
        self.run_it(esc_mock=False)
        self.assertEqual([], self.resets)


if __name__ == "__main__":
    unittest.main()
