import json
import re
import sys
import tempfile
import unittest
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
        self.assertTrue(m["spy"])
        self.assertTrue(m["git_tree_dirty"])
        self.assertIsNotNone(m["wingman_pico_id"])

    def test_manifest_records_the_load_on_the_shafts(self):
        m = run_session.build_manifest("s", "a.json", True, [], None, None, datetime(2026, 1, 1), "Gemfan D2.2, 1 g each")
        self.assertEqual("Gemfan D2.2, 1 g each", m["load"])

    def test_no_spy_means_no_wingman_id(self):
        m = run_session.build_manifest("s", "a.json", False, [], None, None, datetime(2026, 1, 1))
        self.assertFalse(m["spy"])
        self.assertIsNone(m["wingman_pico_id"])

    def test_manifest_is_written_as_json_into_the_session_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "2026-10-29_21-09-19"
            run_session.write_manifest(folder, {"session": "x", "spy": False})
            self.assertEqual({"session": "x", "spy": False}, json.loads((folder / "session.json").read_text()))


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
        subprocess.run = lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout="Uploaded 5, failed 0.", stderr="")
        run_session.collect_sd_session = lambda *a, **k: False
        run_session.session_summary.write_summary = lambda folder: {"verdict": "unknown"}
        try:
            session, _, _, _ = run_session.run_session(scenario, spy=False, echo=False)
        finally:
            run_session.CAPTURES, subprocess.run, run_session.collect_sd_session, run_session.session_summary.write_summary = saved
        return run_session_folder(tmp.name, session)

    def test_a_scenario_run_keeps_its_own_copy_of_the_scenario_in_the_session(self):
        folder = self.run_with_fakes("my_scenario.json")
        self.assertEqual('{"duration_ms": 1}', (folder / "scenario.json").read_text(encoding="utf-8"))

    def test_a_device_script_run_has_no_scenario_copy(self):
        folder = self.run_with_fakes("probe_script.py")
        self.assertFalse((folder / "scenario.json").exists())


def run_session_folder(tmp, session):
    return Path(tmp) / "captures" / session


if __name__ == "__main__":
    unittest.main()
