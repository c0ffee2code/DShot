import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
import session_files
import session_summary
import spy_check_am32


def record(start, throttle, reply=None, crc_ok=True):
    """A spy_edge_log.analyse() record with just the fields spy_stats.frame_table() reads."""
    bits = [1, 0] * 8
    return {"start": start, "edges": 32, "tx_edges": None, "reply": reply,
            "tx": {"throttle": throttle, "crc_ok": crc_ok, "bits": bits, "lows": [12] * 16, "periods": [17] * 16}}


def reply(erpm=None, not_running=False, crc_ok=True, delay=230):
    return {"crc_ok": crc_ok, "not_running": not_running, "erpm": erpm, "delay": delay}


DEVICE_COMPLETED = json.dumps({"device": {"outcome": "completed"}})
DEVICE_FAILED = json.dumps({"device": {"outcome": "failed"}})


class ParseTest(unittest.TestCase):
    def test_meta_is_key_value_lines(self):
        self.assertEqual({"outcome": "failed", "total_records": "0"}, session_files.parse_meta("outcome=failed\ntotal_records=0\nnoise\n"))

    def test_run_log_gives_arming_time_and_last_error(self):
        text = "Armed after 2137ms.\n  | RuntimeError: first\nTraceback\nRuntimeError: not-running failsafe tripped\n"
        self.assertEqual((2137, "RuntimeError: not-running failsafe tripped"), session_summary.parse_run_log(text))

    def test_run_log_without_either(self):
        self.assertEqual((None, None), session_summary.parse_run_log("nothing here\n"))

    def test_lines_map_to_gpio_pins_and_pins_to_scenario_motors(self):
        self.assertEqual({0: 6, 1: 7}, session_summary.pins_of_stream("pins=6,7 sync_pin=15 marker=v2"))
        scenario = {"motors": [{"pin": 6, "bidirectional": True}, {"pin": 7, "bidirectional": False}]}
        self.assertTrue(session_summary.motors_by_pin(scenario)[6]["bidirectional"])
        self.assertFalse(session_summary.motors_by_pin(scenario)[7]["bidirectional"])


class LineFactsTest(unittest.TestCase):
    def test_a_line_that_spun_and_kept_answering(self):
        records = [record(i * 10450, 100, reply(erpm=8000.0)) for i in range(100)]
        facts = session_summary.line_facts(records, True)
        self.assertEqual(100, facts["frames"])
        self.assertEqual(100.0, facts["answered_pct"])
        self.assertEqual(100, facts["replies_real_erpm"])
        self.assertEqual(0, facts["not_running_after_first_real_erpm"])
        self.assertTrue(facts["commanded"])
        self.assertEqual(8000, facts["median_erpm"])

    def test_not_running_after_a_real_erpm_is_counted(self):
        records = [record(i * 10450, 100, reply(erpm=8000.0)) for i in range(10)]
        records += [record((10 + i) * 10450, 100, reply(not_running=True)) for i in range(60)]
        facts = session_summary.line_facts(records, True)
        self.assertEqual(60, facts["not_running_after_first_real_erpm"])
        self.assertEqual(60, facts["replies_not_running"])

    def test_early_reply_and_bad_crc_reply_are_counted(self):
        records = [record(0, 100, reply(erpm=1.0, delay=2)), record(10450, 100, reply(crc_ok=False)), record(20900, 100, None)]
        facts = session_summary.line_facts(records, True)
        self.assertEqual(1, facts["early_replies"])
        self.assertEqual(1, facts["replies_bad_crc"])
        self.assertEqual(2, facts["answered"])

    def test_unidirectional_line_has_no_reply_fields(self):
        facts = session_summary.line_facts([record(i * 10450, 0) for i in range(5)], False)
        self.assertNotIn("answered", facts)
        self.assertFalse(facts["commanded"])

    def test_groups_without_a_complete_frame_are_counted(self):
        records = [record(0, 60), {"start": 10450, "tx": None, "reply": None, "edges": 5, "tx_edges": None}]
        self.assertEqual(1, session_summary.line_facts(records, False)["unparsed_groups"])


class SlowGapTest(unittest.TestCase):
    def test_only_intervals_between_40_and_400_us_count(self):
        # edges in 0.1 us units: 5 us (in a frame), 23 us (reply turnaround), 100 us, 399 us, 1000 us (between frames)
        times = [0, 50, 280, 1280, 5270, 15270]
        edge_list = [(t, 0) for t in times]
        self.assertEqual(2, session_summary.slow_gaps(edge_list))

    def test_a_line_with_many_slow_gaps_is_flagged(self):
        line = {"line": 1, "mode": "bidirectional", "gpio": 7,
                "facts": {"commanded": False, "replies_real_erpm": 0, "slow_gaps": session_summary.SLOW_GAPS_FLAGGED},
                "checks": session_summary.check_summary([])}
        self.assertEqual(1, len(session_summary.line_reasons(line)))
        line["facts"]["slow_gaps"] = session_summary.SLOW_GAPS_FLAGGED - 1
        self.assertEqual([], session_summary.line_reasons(line))


class ReasonTest(unittest.TestCase):
    def line(self, mode="bidirectional", commanded=True, real=0, lost=None, checks=()):
        return {"line": 3, "mode": mode, "gpio": 9,
                "facts": {"commanded": commanded, "replies_real_erpm": real, "not_running_after_first_real_erpm": lost},
                "checks": session_summary.check_summary(list(checks))}

    def test_commanded_but_never_spun(self):
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=0))))

    def test_not_commanded_line_is_not_blamed(self):
        self.assertEqual([], session_summary.line_reasons(self.line(commanded=False)))

    def test_lost_spin_needs_the_frame_threshold(self):
        self.assertEqual([], session_summary.line_reasons(self.line(real=5, lost=session_summary.LOST_SPIN_FRAMES - 1)))
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=5, lost=session_summary.LOST_SPIN_FRAMES))))

    def test_a_failed_am32_check_is_a_reason_a_warning_is_not(self):
        fail = spy_check_am32.Result("crc", spy_check_am32.FAIL, "bad", "")
        warn = spy_check_am32.Result("late", spy_check_am32.WARN, "slow", "")
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=5, lost=0, checks=[fail, warn]))))

    def test_unidirectional_line_needs_no_eRPM(self):
        self.assertEqual([], session_summary.line_reasons(self.line(mode="unidirectional", real=0)))


class ExpectOnTheWireTest(unittest.TestCase):
    def line(self, **facts):
        base = {"commanded": True, "answered": 1000, "replies_bad_crc": 0, "replies_real_erpm": 900, "median_erpm": 20000}
        base.update(facts)
        return {"line": 3, "mode": "bidirectional", "facts": base}

    def test_a_slow_but_steady_motor_fails_the_median_threshold(self):
        reasons, notes = session_summary.judge_expect(self.line(median_erpm=5000), 99.0, 10000)
        self.assertEqual(1, len(reasons))
        self.assertIn("median eRPM on the wire 5000", reasons[0])
        self.assertEqual([], notes)

    def test_bad_crc_share_below_the_threshold_fails(self):
        reasons, _ = session_summary.judge_expect(self.line(replies_bad_crc=50), 99.0, None)
        self.assertIn("95.0%", reasons[0])

    def test_too_few_replies_is_a_note_not_a_failure(self):
        reasons, notes = session_summary.judge_expect(self.line(answered=10, replies_real_erpm=10), 99.0, 10000)
        self.assertEqual([], reasons)
        self.assertEqual(2, len(notes))

    def test_an_uncommanded_or_unidirectional_line_is_not_judged_at_all(self):
        self.assertEqual(([], []), session_summary.judge_expect(self.line(commanded=False), 99.0, 10000))
        unidirectional = self.line()
        unidirectional["mode"] = "unidirectional"
        self.assertEqual(([], []), session_summary.judge_expect(unidirectional, 99.0, 10000))

    def test_the_pico_saying_it_did_not_judge_becomes_a_note_and_the_session_still_passes(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-10-31_00-00-00"
        folder.mkdir()
        device = {"device": {"outcome": "completed", "verdict": "pass (not judged: motor 0: only 17 captures decoded, need 20 to judge)"}}
        (folder / "session.json").write_text(json.dumps(device), encoding="utf-8")
        summary = session_summary.summarize(folder)
        self.assertEqual("pass", summary["verdict"])
        self.assertIn("only 17 captures decoded", summary["notes"][0])
        self.assertIn("not judged, not a failure", session_summary.table(summary))


class SummarizeFolderTest(unittest.TestCase):
    def folder(self, files):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-10-30_09-00-00"
        folder.mkdir()
        for name, text in files.items():
            (folder / name).write_text(text, encoding="utf-8")
        return folder

    def test_completed_run_without_a_recording_passes(self):
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_COMPLETED, "run.log": "Armed after 2000ms.\n"}))
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual(2000, summary["device"]["armed_ms"])
        self.assertIsNone(summary["spy"])
        self.assertEqual(["session.json", "session.json:device", "run.log"], summary["sources"])

    def test_failed_run_fails_and_says_why(self):
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_FAILED, "run.log": "RuntimeError: arming did not complete\n"}))
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("arming did not complete", summary["reasons"][0])

    def test_a_completed_run_that_missed_its_own_expectations_fails(self):
        device = json.dumps({"device": {"outcome": "completed", "verdict": "fail: motor 0: only 17 captures decoded, need 20"}})
        summary = session_summary.summarize(self.folder({"session.json": device, "run.log": "Armed after 2000ms."}))
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("only 17 captures decoded", summary["reasons"][0])
        ok = json.dumps({"device": {"outcome": "completed", "verdict": "pass"}})
        self.assertEqual("pass", session_summary.summarize(self.folder({"session.json": ok}))["verdict"])

    def test_a_folder_with_nothing_to_judge_is_unknown(self):
        self.assertEqual("unknown", session_summary.summarize(self.folder({"session.json": "{}"}))["verdict"])

    def test_device_section_of_session_json_is_read_like_meta_txt(self):
        folder = self.folder({"session.json": json.dumps({"device": {"outcome": "completed"}}), "run.log": "Armed after 2000ms.\n"})
        summary = session_summary.summarize(folder)
        self.assertEqual("pass", summary["verdict"])
        self.assertIn("session.json:device", summary["sources"])

    def test_without_a_device_outcome_the_wire_decides(self):
        # stand-in wire summaries: one commanded line that kept answering, one run that never commanded anything
        def with_wire(commanded):
            real = session_summary.spy_summary
            session_summary.spy_summary = lambda *a, **k: {"valid": True, "lines": [
                {"line": 0, "mode": "bidirectional", "reasons": [], "facts": {"commanded": commanded}}]}
            self.addCleanup(setattr, session_summary, "spy_summary", real)
            return session_summary.summarize(self.folder({"session.json": "{}", "spy.bin": "x"}))
        self.assertEqual("pass", with_wire(True)["verdict"])
        self.assertEqual("fail", with_wire(False)["verdict"])
        self.assertIn("no line was commanded", with_wire(False)["reasons"][0])

    def test_a_line_the_scenario_does_not_use_and_nothing_was_sent_on_is_not_driven(self):
        import spy_edge_log
        patches = {"parse_stream": lambda b: ("pins=6,7 sync_pin=15", {0: b"a", 1: b"b"}, "end"),
                   "parse_end": lambda e: {"valid": True}, "reconcile": lambda c, i: [],
                   "edges": lambda c, invert=False: [(0, 0)],
                   "analyse": lambda e, u: [record(0, 100)] if e is not None and self.calls.pop() else []}
        self.calls = [False, True]            # line 0 gets a frame, line 1 none
        for name, fn in patches.items():
            self.addCleanup(setattr, spy_edge_log, name, getattr(spy_edge_log, name))
            setattr(spy_edge_log, name, fn)
        self.addCleanup(setattr, spy_check_am32, "run_checks", spy_check_am32.run_checks)
        spy_check_am32.run_checks = lambda records, unidirectional, from_first_frame: []
        scenario = {"motors": [{"pin": 6, "bidirectional": False}]}
        lines = session_summary.spy_summary(b"x", scenario)["lines"]
        self.assertEqual(["unidirectional", "not driven"], [line["mode"] for line in lines])
        self.assertEqual([], lines[1]["reasons"])
        self.assertEqual({}, lines[1]["checks"]["counts"])

    def test_write_summary_writes_json_that_reads_back(self):
        folder = self.folder({"session.json": DEVICE_COMPLETED})
        session_summary.write_summary(folder)
        self.assertEqual("pass", json.loads((folder / "summary.json").read_text())["verdict"])

    def test_table_prints_verdict_and_reasons(self):
        text = session_summary.table(session_summary.summarize(self.folder({"session.json": DEVICE_FAILED})))
        self.assertIn("FAIL", text)
        self.assertIn("the scenario did not complete", text)


class PropellersTest(unittest.TestCase):
    def test_the_propellers_come_from_the_scenarios_hardware_blocks_in_motor_order(self):
        scenario = {"motors": [{"hardware": {"propeller": "Gemfan 2218 3-Blade"}}, {"hardware": {"propeller": "none"}}, {}]}
        self.assertEqual(["Gemfan 2218 3-Blade", "none", None], session_summary.propellers(scenario))

    def test_all_the_same_is_said_once_and_differences_are_listed(self):
        self.assertEqual("Gemfan 2218 3-Blade on all 4 motors", session_summary.propellers_text(["Gemfan 2218 3-Blade"] * 4))
        self.assertEqual("motor 1: A, motor 2: none, motor 3: not recorded", session_summary.propellers_text(["A", "none", None]))


if __name__ == "__main__":
    unittest.main()
