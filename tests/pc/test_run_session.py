import json
import re
import sys
import tempfile
import unittest
from unittest import mock
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import run_session
import run_test

SESSION_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")


class SessionNameTest(unittest.TestCase):
    def test_name_has_the_session_folder_shape(self):
        self.assertEqual("2026-10-29_21-09-19", run_test.new_session_name(datetime(2026, 10, 29, 21, 9, 19)))
        self.assertRegex(run_test.new_session_name(), SESSION_NAME)


class ManifestTest(unittest.TestCase):
    def test_manifest_records_what_was_run(self):
        started = datetime(2026, 10, 29, 21, 9, 19)
        m = run_session.build_manifest("s", "a.json", True, ["a.json"], "abc", True, started)
        self.assertEqual("s", m["session"])
        self.assertEqual("2026-10-29T21:09:19", m["started"])
        self.assertTrue(m["observer"])
        self.assertTrue(m["git_tree_dirty"])
        self.assertIsNotNone(m["wingman_pico_id"])

    def test_manifest_records_the_load_on_the_shafts(self):
        m = run_session.build_manifest("s", "a.json", True, [], None, None, datetime(2026, 1, 1), "Gemfan D2.2, 1 g each")
        self.assertEqual("Gemfan D2.2, 1 g each", m["load"])

    def test_a_device_script_without_the_wingman_has_no_wingman_id(self):
        m = run_session.build_manifest("s", "a.json", False, [], None, None, datetime(2026, 1, 1))
        self.assertFalse(m["observer"])
        self.assertIsNone(m["wingman_pico_id"])

    def test_manifest_is_written_as_json_into_the_session_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "2026-10-29_21-09-19"
            run_session.write_manifest(folder, {"session": "x", "observer": False})
            self.assertEqual({"session": "x", "observer": False}, json.loads((folder / "session.json").read_text()))


class ExitCodeTest(unittest.TestCase):
    def test_only_a_pass_exits_zero(self):
        self.assertEqual(0, run_session.exit_code(0, "pass"))
        self.assertEqual(1, run_session.exit_code(0, "fail"))
        self.assertEqual(2, run_session.exit_code(0, "unknown"))

    def test_a_non_zero_run_test_exits_non_zero_whatever_the_verdict(self):
        for verdict in ("pass", "fail", "unknown"):
            self.assertEqual(1, run_session.exit_code(3, verdict))
            self.assertEqual(1, run_session.exit_code(-1, verdict))

    def test_the_function_still_returns_run_tests_own_code_for_the_batch_runner(self):
        import subprocess
        saved = (subprocess.run, run_session.collect_sd_session, run_session.session_summary.write_summary, run_session.CAPTURES)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        run_session.CAPTURES = Path(tmp.name) / "captures"
        subprocess.run = lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout="Uploaded 5, failed 0.", stderr="")
        run_session.collect_sd_session = lambda *a, **k: False
        run_session.session_summary.write_summary = lambda folder: {"verdict": "fail"}
        try:
            _, _, code, summary = run_session.run_session(Path("probe_script.py"), echo=False)
        finally:
            subprocess.run, run_session.collect_sd_session, run_session.session_summary.write_summary, run_session.CAPTURES = saved
        self.assertEqual(0, code)
        self.assertEqual("fail", summary["verdict"])


class FakeWingman:
    def poll(self):
        return 0

    def communicate(self, timeout=None):
        return "end: valid=1", None

    def kill(self):
        pass


class FakePlan:
    mock = None
    run_seconds = 1


class ScenarioCopyTest(unittest.TestCase):
    def run_with_fakes(self, scenario_name):
        """run_session() with the bench replaced by stand-ins; returns the session folder."""
        import subprocess
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        scenario = Path(tmp.name) / scenario_name
        scenario.write_text('{"duration_ms": 1}', encoding="utf-8")
        saved = (run_session.CAPTURES, subprocess.run, run_session.collect_sd_session, run_session.session_summary.write_summary)
        run_session.CAPTURES = Path(tmp.name) / "captures"
        subprocess.run = lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout="Uploaded 5, failed 0.\n=== Scenario Capture ===", stderr="")
        run_session.collect_sd_session = lambda *a, **k: False
        run_session.session_summary.write_summary = lambda folder: {"verdict": "unknown"}
        try:
            if scenario_name.endswith(".py"):
                session, _, _, _ = run_session.run_session(scenario, echo=False)
            else:
                with mock.patch.object(run_session, "start_observer", return_value=(FakeWingman(), [])), \
                        mock.patch.object(run_session.wingman_plan, "plan_from_scenario", return_value=FakePlan()), \
                        mock.patch.object(run_session.time, "sleep"):
                    session, _, _, _ = run_session.run_session(scenario, echo=False)
        finally:
            run_session.CAPTURES, subprocess.run, run_session.collect_sd_session, run_session.session_summary.write_summary = saved
        return run_session_folder(tmp.name, session)

    def test_a_scenario_run_keeps_its_own_copy_of_the_scenario_in_the_session(self):
        folder = self.run_with_fakes("my_scenario.json")
        self.assertEqual('{"duration_ms": 1}', (folder / "scenario.json").read_text(encoding="utf-8"))

    def test_a_device_script_run_has_no_scenario_copy(self):
        folder = self.run_with_fakes("probe_script.py")
        self.assertFalse((folder / "scenario.json").exists())


class WingmanAlwaysRecordsTest(unittest.TestCase):
    def test_a_scenario_cannot_be_run_without_the_wingman(self):
        with tempfile.TemporaryDirectory() as tmp:
            scenario = Path(tmp) / "s.json"
            scenario.write_text('{"duration_ms": 1}', encoding="utf-8")
            with self.assertRaises(SystemExit) as stop:
                run_session.run_session(scenario, observer=False, echo=False)
            self.assertIn("always recorded", str(stop.exception.code))

    def test_the_option_that_left_the_wingman_out_is_gone(self):
        with mock.patch.object(sys, "argv", ["run_session.py", "s.json", "--load", "none", "--no-observe"]), \
                mock.patch("sys.stderr"):
            with self.assertRaises(SystemExit) as stop:
                run_session.main()
        self.assertEqual(2, stop.exception.code)

    def test_a_scenario_does_not_start_when_the_wingman_never_records(self):
        import subprocess
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        scenario = Path(tmp.name) / "s.json"
        scenario.write_text('{"duration_ms": 1}', encoding="utf-8")
        calls = []
        with mock.patch.object(run_session, "CAPTURES", Path(tmp.name) / "captures"), \
                mock.patch.object(run_session, "start_observer", side_effect=run_session.WingmanStartError("never recorded", "")), \
                mock.patch.object(run_session.session_summary, "git_commit", return_value=None), \
                mock.patch.object(run_session.wingman_plan, "plan_from_scenario", return_value=FakePlan()), \
                mock.patch.object(run_session, "git_state", return_value=(None, None)), \
                mock.patch.object(subprocess, "run", side_effect=lambda *a, **k: calls.append(a)):
            _, _, _, summary = run_session.run_session(scenario, echo=False)
        self.assertEqual([], calls)
        self.assertEqual("fail", summary["verdict"])

    def test_a_device_script_is_recorded_only_when_asked(self):
        import subprocess
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        started = []
        for asked in (False, True):
            with mock.patch.object(run_session, "CAPTURES", Path(tmp.name) / "captures"), \
                    mock.patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0, stdout="Uploaded 5, failed 0.", stderr="")), \
                    mock.patch.object(run_session, "collect_sd_session", return_value=False), \
                    mock.patch.object(run_session.session_summary, "write_summary", return_value={"verdict": "unknown"}), \
                    mock.patch.object(run_session, "start_observer", side_effect=lambda *a: started.append(a) or (FakeWingman(), [])), \
                    mock.patch.object(run_session.wingman_plan, "plan_from_scenario", return_value=FakePlan()), \
                    mock.patch.object(run_session.time, "sleep"):
                config = Path(tmp.name) / "c.json"
                config.write_text("{}", encoding="utf-8")
                run_session.run_session(Path("probe_script.py"), observer=asked, echo=False, config=config)
        self.assertEqual(1, len(started))


class StartFailureTest(unittest.TestCase):
    """A session whose wingman or main Pico did not start still ends with run.log and a failed summary.json (ADR-008, amendment 2026-11-12 (2))."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.scenario = Path(self.tmp.name) / "s.json"
        self.scenario.write_text('{"duration_ms": 1, "motors": []}', encoding="utf-8")
        self.launched = []
        self.released = []
        self.pulled = []

    def run_it(self, wingman_lines=(), test_output=None, test_code=1, plan_error=False, esc_mock=False):
        """run_session() on a scenario with the boards replaced by stand-ins. test_output None: the wingman launcher prints
        wingman_lines and ends; else the wingman records and run_test.py prints test_output. Returns (session, output, code, summary, folder)."""
        import io
        import subprocess

        class Launcher:
            def __init__(self, lines):
                self.stdout = io.StringIO("".join(line + "\n" for line in lines))

            def kill(self):
                pass

        def fake_run(*a, **k):
            self.launched.append(a)
            return subprocess.CompletedProcess(a, test_code, stdout=test_output, stderr="")

        patches = [mock.patch.object(run_session, "CAPTURES", Path(self.tmp.name) / "captures"),
                   mock.patch.object(run_session, "git_state", return_value=(None, None)),
                   mock.patch.object(run_session, "release_wingman", side_effect=self.released.append),
                   mock.patch.object(run_session, "collect_sd_session", side_effect=lambda *a, **k: self.pulled.append(a)),
                   mock.patch.object(run_session.time, "sleep"),
                   mock.patch.object(run_session.session_summary, "git_commit", return_value=None),
                   mock.patch.object(subprocess, "run", side_effect=fake_run)]
        if plan_error:
            patches.append(mock.patch.object(run_session.wingman_plan, "plan_from_scenario", side_effect=ValueError("unsupported pin")))
        else:
            patches.append(mock.patch.object(run_session.wingman_plan, "plan_from_scenario", return_value=FakePlan()))
        if test_output is None:
            patches.append(mock.patch.object(subprocess, "Popen", side_effect=lambda *a, **k: Launcher(wingman_lines)))
        else:
            patches.append(mock.patch.object(run_session, "start_observer", return_value=(FakeWingman(), [])))
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        session, output, code, summary = run_session.run_session(self.scenario, echo=False, esc_mock=esc_mock)
        return session, output, code, summary, run_session_folder(self.tmp.name, session)

    def test_a_wingman_that_is_not_connected_fails_the_session_and_the_main_pico_is_not_touched(self):
        _, output, code, summary, folder = self.run_it(["The wingman Pico (ID 5d70) is not connected."])
        self.assertEqual([], self.launched)         # run_test.py never launched: nothing uploaded, nothing run
        self.assertEqual([], self.pulled)
        self.assertEqual(1, code)
        self.assertEqual("fail", summary["verdict"])
        self.assertEqual(["the wingman did not start: the observer never reported that it is recording (The wingman Pico (ID 5d70) is not connected.)"],
                         summary["reasons"])
        self.assertIn("is not connected", (folder / "run.log").read_text(encoding="utf-8"))
        self.assertEqual("wingman", json.loads((folder / "session.json").read_text(encoding="utf-8"))["start_failure"]["who"])
        self.assertEqual("fail", json.loads((folder / "summary.json").read_text(encoding="utf-8"))["verdict"])
        self.assertEqual(1, run_session.exit_code(code, summary["verdict"]))

    def test_a_mock_session_whose_wingman_did_not_start_still_resets_the_board(self):
        self.run_it(["not connected"], esc_mock=True)
        self.assertEqual([None], self.released)         # no launcher to stop, the board is reset

    def test_a_wingman_that_prints_nothing_still_leaves_a_reason(self):
        _, _, _, summary, folder = self.run_it([])
        self.assertIn("it printed nothing", summary["reasons"][0])
        self.assertTrue((folder / "run.log").is_file())

    def test_a_scenario_the_wingman_cannot_serve_fails_the_session_inside_its_folder(self):
        _, _, _, summary, folder = self.run_it(plan_error=True)
        self.assertEqual([], self.launched)
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("the wingman did not start: the wingman cannot serve", summary["reasons"][0])
        self.assertIn("unsupported pin", (folder / "run.log").read_text(encoding="utf-8"))

    def test_an_upload_that_failed_means_the_main_pico_did_not_start(self):
        out = "Deploying to Pico on COM10...\n  FAIL     driver/dshot_pio.py: could not open port\n\nUploaded 0, failed 17.\n"
        _, _, code, summary, folder = self.run_it(test_output=out)
        self.assertEqual(1, len(self.launched))
        self.assertEqual([], self.pulled)       # the card is not asked after a run that never started
        self.assertEqual("fail", summary["verdict"])
        self.assertEqual(["the main Pico did not start: the upload to it did not complete (FAIL     driver/dshot_pio.py: could not open port)"],
                         summary["reasons"])
        self.assertEqual("main_pico", json.loads((folder / "session.json").read_text(encoding="utf-8"))["start_failure"]["who"])
        self.assertEqual(out, (folder / "run.log").read_text(encoding="utf-8"))
        self.assertEqual(1, run_session.exit_code(code, summary["verdict"]))

    def test_a_mock_session_whose_main_pico_did_not_start_releases_the_wingman(self):
        self.run_it(test_output="Uploaded 0, failed 1.\n", esc_mock=True)
        self.assertEqual([FakeWingman], [type(p) for p in self.released])

    def test_a_scenario_that_never_began_means_the_main_pico_did_not_start(self):
        out = "Uploaded 19, failed 0.\nRunning run_scenario.py on COM10 (live output)...\nfailed to access COM10 (it may be in use by another program)\n"
        _, _, _, summary, _ = self.run_it(test_output=out)
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("the main Pico did not start: the scenario never began on it", summary["reasons"][0])
        self.assertIn("failed to access COM10", summary["reasons"][0])

    def test_a_run_that_began_is_not_a_start_failure_even_without_the_pico_record(self):
        out = "Uploaded 19, failed 0.\n=== Scenario Capture ===\nOSError: SD card\n"
        self.assertIsNone(run_session.main_pico_start_failure(out, False))
        _, _, _, summary, folder = self.run_it(test_output=out)
        self.assertIsNone(json.loads((folder / "session.json").read_text(encoding="utf-8"))["start_failure"])
        self.assertEqual("unknown", summary["verdict"])
        self.assertEqual(1, len(self.pulled))

    def test_a_device_script_is_judged_by_its_upload_alone(self):
        self.assertIsNone(run_session.main_pico_start_failure("Uploaded 19, failed 0.\nhello\n", True))
        self.assertIn("did not complete", run_session.main_pico_start_failure("MISSING test script x", True))


def run_session_folder(tmp, session):
    return Path(tmp) / "captures" / session


if __name__ == "__main__":
    unittest.main()
