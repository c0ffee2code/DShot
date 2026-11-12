import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import session_files


class SessionFilesTest(unittest.TestCase):
    def folder(self, files):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-10-30_09-00-00"
        folder.mkdir()
        for name, text in files.items():
            (folder / name).write_text(text, encoding="utf-8")
        return folder

    def test_parse_meta_strips_and_ignores_noise(self):
        self.assertEqual({"a": "1", "motor0_name": "X Y"}, session_files.parse_meta(" a = 1 \nmotor0_name=X Y\nnoise\n"))

    def test_new_session_reads_the_device_section(self):
        folder = self.folder({"session.json": json.dumps({"session": "s", "device": {"outcome": "aborted: RuntimeError: x"}})})
        self.assertEqual({"outcome": "aborted: RuntimeError: x"}, session_files.load_device_meta(folder))
        self.assertTrue(session_files.is_session(folder))

    def test_folder_without_the_pico_record_is_not_a_session(self):
        self.assertFalse(session_files.is_session(self.folder({"signal_observation.bin": "x"})))
        self.assertFalse(session_files.is_session(self.folder({"session.json": json.dumps({"session": "s"})})))
        self.assertEqual({}, session_files.load_device_meta(self.folder({"signal_observation.bin": "x"})))

    def test_an_esc_mock_session_is_not_evidence(self):
        mock = self.folder({"session.json": json.dumps({"session": "s", "esc_mock": {"fixture": "f.bin"}, "device": {}})})
        real = self.folder({"session.json": json.dumps({"session": "s", "esc_mock": None, "device": {}})})
        self.assertFalse(session_files.is_evidence(mock))
        self.assertTrue(session_files.is_evidence(real))
        self.assertTrue(session_files.is_evidence(self.folder({"signal_observation.bin": "x"})))      # no session.json: nothing marks it a mock

    def test_a_leftover_meta_txt_is_not_read(self):
        folder = self.folder({"meta.txt": "outcome=completed\n"})
        self.assertEqual({}, session_files.load_device_meta(folder))
        self.assertFalse(session_files.is_session(folder))

    def test_broken_session_json_is_not_a_session(self):
        folder = self.folder({"session.json": "{not json"})
        self.assertEqual({}, session_files.load_device_meta(folder))
        self.assertFalse(session_files.is_session(folder))

    def test_fold_moves_meta_txt_into_the_existing_manifest(self):
        folder = self.folder({"session.json": json.dumps({"session": "s", "observer": True}), "meta.txt": "outcome=completed\nx=1\n"})
        self.assertTrue(session_files.fold_device_meta(folder))
        manifest = json.loads((folder / "session.json").read_text())
        self.assertEqual({"outcome": "completed", "x": "1"}, manifest["device"])
        self.assertTrue(manifest["observer"])
        self.assertFalse((folder / "meta.txt").exists())

    def test_fold_creates_a_manifest_when_there_is_none(self):
        folder = self.folder({"meta.txt": "outcome=aborted: RuntimeError: x\n"})
        self.assertTrue(session_files.fold_device_meta(folder))
        manifest = json.loads((folder / "session.json").read_text())
        self.assertEqual(folder.name, manifest["session"])
        self.assertEqual("aborted: RuntimeError: x", manifest["device"]["outcome"])

    def test_fold_without_meta_txt_does_nothing(self):
        folder = self.folder({"session.json": json.dumps({"session": "s"})})
        self.assertFalse(session_files.fold_device_meta(folder))
        self.assertNotIn("device", json.loads((folder / "session.json").read_text()))

    def test_fold_twice_is_harmless(self):
        folder = self.folder({"meta.txt": "outcome=aborted: RuntimeError: x\n"})
        session_files.fold_device_meta(folder)
        self.assertFalse(session_files.fold_device_meta(folder))
        self.assertEqual("aborted: RuntimeError: x", session_files.load_device_meta(folder)["outcome"])
        self.assertTrue(session_files.is_session(folder))


if __name__ == "__main__":
    unittest.main()
