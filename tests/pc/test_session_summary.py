import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
import session_files
import session_summary
import observer_check_am32


def record(start, throttle, reply=None, crc_ok=True):
    """An observer_edge_log.analyse() record with just the fields observer_stats.frame_table() reads."""
    bits = [1, 0] * 8
    return {"start": start, "edges": 32, "tx_edges": None, "reply": reply,
            "tx": {"throttle": throttle, "crc_ok": crc_ok, "bits": bits, "lows": [12] * 16, "periods": [17] * 16}}


def reply(erpm=None, not_running=False, crc_ok=True, delay=230):
    return {"crc_ok": crc_ok, "not_running": not_running, "erpm": erpm, "delay": delay}


DEVICE_COMPLETED = json.dumps({"device": {"outcome": "completed"}})
ARMING_REFUSED = "aborted: arming did not complete within 18000ms; per motor (replying_for_ms, last_reply_ms_ago), None = no reply: [None]"
DEVICE_ABORTED = json.dumps({"device": {"outcome": ARMING_REFUSED}})


ARMED_LOG = "Armed after 2000ms.\n"


def wire_line(line=0, gpio=6, mode="bidirectional", commanded=True, frames_reasons=(), spin_reasons=(), notes=(), expect=None,
              scenario_max_throttle=None, **facts):
    """An observer_summary() line as the proofs read it: a healthy commanded line unless a fact says otherwise."""
    base = {"commanded": commanded, "frames": 100, "answered_pct": 100.0, "replies_real_erpm": 90, "replies_not_running": 0,
            "not_running_after_first_real_erpm": 0, "early_replies": 0, "slow_gaps": 0, "last_throttle": 0, "stop_commanded": True}
    base.update(facts)
    reasons = list(frames_reasons) + list(spin_reasons)
    return {"line": line, "gpio": gpio, "mode": mode, "reasons": reasons, "frame_reasons": list(frames_reasons), "spin_reasons": list(spin_reasons),
            "notes": list(notes), "expect": expect or {}, "scenario_max_throttle": scenario_max_throttle, "checks": {"counts": {}}, "facts": base}


def stand_in_wire(test, valid, mode="bidirectional", reasons=(), lines=None, missing_pins=()):
    """Replace the signal_observation.bin analysis with one commanded line (or the given lines), so a folder needs only a placeholder signal_observation.bin."""
    real = session_summary.observer_summary
    session_summary.observer_summary = lambda *a, **k: {"valid": valid, "missing_pins": list(missing_pins),
                                                   "lines": lines if lines is not None else [wire_line(mode=mode, spin_reasons=reasons)]}
    test.addCleanup(setattr, session_summary, "observer_summary", real)


class ParseTest(unittest.TestCase):
    def test_meta_is_key_value_lines(self):
        self.assertEqual({"outcome": "completed", "total_records": "0"}, session_files.parse_meta("outcome=completed\ntotal_records=0\nnoise\n"))

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

    def test_frames_without_a_reply_after_a_real_erpm_are_counted_while_commanded(self):
        records = [record(i * 10450, 0, None) for i in range(5)]                                 # unanswered before any eRPM: not counted
        records += [record((5 + i) * 10450, 100, reply(erpm=8000.0)) for i in range(10)]
        records += [record((15 + i) * 10450, 100, None) for i in range(60)]                      # the ESC went silent while commanded
        records += [record((75 + i) * 10450, 0, None) for i in range(7)]                         # silent at throttle 0: not counted
        facts = session_summary.line_facts(records, True)
        self.assertEqual(60, facts["unanswered_after_first_real_erpm"])
        self.assertEqual(0, facts["not_running_after_first_real_erpm"])

    def test_a_line_that_never_spun_has_no_unanswered_count(self):
        facts = session_summary.line_facts([record(i * 10450, 100, None) for i in range(60)], True)
        self.assertIsNone(facts["unanswered_after_first_real_erpm"])

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
    def line(self, mode="bidirectional", commanded=True, real=0, lost=None, checks=(), unanswered=None):
        return {"line": 3, "mode": mode, "gpio": 9,
                "facts": {"commanded": commanded, "replies_real_erpm": real, "not_running_after_first_real_erpm": lost,
                          "unanswered_after_first_real_erpm": unanswered},
                "checks": session_summary.check_summary(list(checks))}

    def test_an_esc_that_stops_answering_needs_the_frame_threshold(self):
        below = self.line(real=5, unanswered=session_summary.LOST_REPLY_FRAMES - 1)
        at = self.line(real=5, unanswered=session_summary.LOST_REPLY_FRAMES)
        self.assertEqual([], session_summary.line_reasons(below))
        reasons = session_summary.line_reasons(at)
        self.assertEqual(1, len(reasons))
        self.assertIn("got no reply", reasons[0])

    def test_an_unanswered_unidirectional_line_is_not_blamed(self):
        line = self.line(mode="unidirectional", real=0, unanswered=session_summary.LOST_REPLY_FRAMES)
        self.assertEqual([], session_summary.line_reasons(line))

    def test_commanded_but_never_spun(self):
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=0))))

    def test_not_commanded_line_is_not_blamed(self):
        self.assertEqual([], session_summary.line_reasons(self.line(commanded=False)))

    def test_lost_spin_needs_the_frame_threshold(self):
        self.assertEqual([], session_summary.line_reasons(self.line(real=5, lost=session_summary.LOST_SPIN_FRAMES - 1)))
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=5, lost=session_summary.LOST_SPIN_FRAMES))))

    def test_a_failed_am32_check_is_a_reason_a_warning_is_not(self):
        fail = observer_check_am32.Result("crc", observer_check_am32.FAIL, "bad", "")
        warn = observer_check_am32.Result("late", observer_check_am32.WARN, "slow", "")
        self.assertEqual(1, len(session_summary.line_reasons(self.line(real=5, lost=0, checks=[fail, warn]))))

    def test_unidirectional_line_needs_no_eRPM(self):
        self.assertEqual([], session_summary.line_reasons(self.line(mode="unidirectional", real=0)))


class ExpectOnTheWireTest(unittest.TestCase):
    def line(self, commanded=True, **window):
        from_step = {"replies": 1000, "replies_bad_crc": 0, "early_replies": 0, "replies_real_erpm": 900, "median_erpm": 20000}
        from_step.update(window)
        return {"line": 3, "mode": "bidirectional", "facts": {"commanded": commanded, "from_step": from_step}}

    def test_a_slow_but_steady_motor_fails_the_median_threshold(self):
        reasons, notes = session_summary.judge_expect(self.line(median_erpm=5000), 99.0, 10000)
        self.assertEqual(1, len(reasons))
        self.assertIn("median eRPM on the wire from the throttle step on 5000", reasons[0])
        self.assertEqual([], notes)

    def test_bad_crc_share_below_the_threshold_fails(self):
        reasons, _ = session_summary.judge_expect(self.line(replies_bad_crc=50), 99.0, None)
        self.assertIn("95.0%", reasons[0])

    def test_too_few_replies_is_a_note_not_a_failure(self):
        reasons, notes = session_summary.judge_expect(self.line(replies=10, replies_real_erpm=10), 99.0, 10000)
        self.assertEqual([], reasons)
        self.assertEqual(2, len(notes))

    def test_an_uncommanded_or_unidirectional_line_is_not_judged_at_all(self):
        self.assertEqual(([], []), session_summary.judge_expect(self.line(commanded=False), 99.0, 10000))
        unidirectional = self.line()
        unidirectional["mode"] = "unidirectional"
        self.assertEqual(([], []), session_summary.judge_expect(unidirectional, 99.0, 10000))

    def test_a_line_with_no_throttle_step_is_not_judged_and_says_so(self):
        line = self.line()
        line["facts"]["from_step"] = None
        reasons, notes = session_summary.judge_expect(line, 99.0, 10000)
        self.assertEqual([], reasons)
        self.assertIn("no throttle step", notes[0])


class ArmingPhaseTest(unittest.TestCase):
    """The decode thresholds are judged from the throttle step on; replies before it (arming) are facts and a note (ADR-008)."""

    def records(self):
        # arming: 300 frames at throttle 0, 100 of them answered with a bad CRC that starts 0.5 us after our frame, the rest 'not running'
        records = [record(i * 10450, 0, reply(crc_ok=False, delay=5) if i < 100 else reply(not_running=True)) for i in range(300)]
        # the step: 1000 frames at 215, every reply a real eRPM
        records += [record((300 + i) * 10450, 215, reply(erpm=46000.0)) for i in range(1000)]
        records += [record((1300 + i) * 10450, 0, reply(erpm=30000.0)) for i in range(20)]
        return records

    def test_the_arming_phase_is_split_off_at_the_step(self):
        facts = session_summary.line_facts(self.records(), True)
        self.assertEqual({"replies": 300, "replies_bad_crc": 100, "early_replies": 100, "frames": 300}, facts["before_step"])
        self.assertEqual(0, facts["from_step"]["replies_bad_crc"])
        self.assertEqual(0, facts["from_step"]["early_replies"])
        self.assertEqual(1020, facts["from_step"]["replies"])
        self.assertEqual(round(300 * 10450 * 0.1 / 1000, 1), facts["step_ms"])
        self.assertEqual(100, facts["replies_bad_crc"])          # the whole-recording facts are kept as they were

    def test_bad_and_early_replies_before_the_step_do_not_fail_the_thresholds_and_are_printed_as_a_note(self):
        line = {"line": 0, "gpio": 6, "mode": "bidirectional", "facts": session_summary.line_facts(self.records(), True)}
        self.assertEqual(([], []), session_summary.judge_expect(line, 99.0, 10000))
        whole = 100.0 * (line["facts"]["answered"] - line["facts"]["replies_bad_crc"]) / line["facts"]["answered"]
        self.assertLess(whole, 99.0)                              # judged over the whole recording it would have failed
        stand_in_wire(self, True, lines=[wire_line(**line["facts"])])
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-11-11_09-00-00"
        folder.mkdir()
        (folder / "session.json").write_text(DEVICE_COMPLETED, encoding="utf-8")
        (folder / "run.log").write_text(ARMED_LOG, encoding="utf-8")
        (folder / "signal_observation.bin").write_bytes(b"x")
        summary = session_summary.summarize(folder)
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual(1, len(summary["notes"]))
        note = summary["notes"][0]
        for part in ("line 0 (channel 1)", "arming phase, not judged", "300 replies", "100 with a bad CRC", "100 starting less than 10 us"):
            self.assertIn(part, note)
        self.assertIn("note (not judged, not a failure): " + note, session_summary.table(summary))

    def test_end_to_end_the_scenarios_thresholds_reach_the_window_and_the_session_passes_with_the_note(self):
        # the real observer_summary() and summarize() on a folder with a scenario.json; only the signal_observation.bin parsing is stood in for
        import observer_edge_log
        records = self.records()
        patches = {"parse_stream": lambda b: ("pins=6 sync_pin=15", {0: b"a"}, "end"),
                   "parse_end": lambda e: {"valid": True}, "reconcile": lambda c, i: [],
                   "edges": lambda c, invert=False: [(0, 0)], "analyse": lambda e, u: records}
        for name, fn in patches.items():
            self.addCleanup(setattr, observer_edge_log, name, getattr(observer_edge_log, name))
            setattr(observer_edge_log, name, fn)
        self.addCleanup(setattr, observer_check_am32, "run_checks", observer_check_am32.run_checks)
        observer_check_am32.run_checks = lambda records, unidirectional, from_first_frame: []
        scenario = {"expect": {"min_crc_valid_pct": {"0": 99.0}, "min_median_erpm": {"0": 10000}},
                    "motors": [{"pin": 6, "bidirectional": True, "profile": [{"type": "hold", "throttle": 215, "duration_ms": 1000}]}]}
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-11-11_09-00-00"
        folder.mkdir()
        (folder / "session.json").write_text(json.dumps({"observer": True, "device": {"outcome": "completed"}}), encoding="utf-8")
        (folder / "scenario.json").write_text(json.dumps(scenario), encoding="utf-8")
        (folder / "run.log").write_text("Armed after 2636ms.\n", encoding="utf-8")
        (folder / "signal_observation.bin").write_bytes(b"x")
        summary = session_summary.summarize(folder)
        self.assertEqual("pass", summary["verdict"], summary["reasons"])
        line = summary["observer"]["lines"][0]
        self.assertEqual({"min_crc_valid_pct": 99.0, "min_median_erpm": 10000}, line["expect"])
        self.assertEqual(100, line["facts"]["before_step"]["replies_bad_crc"])
        self.assertIn("and met the scenario's 2 decode threshold(s)", [p for p in summary["proofs"] if p["name"] == "motors spun as specified"][0]["detail"])
        self.assertIn("100 with a bad CRC", session_summary.table(summary))
        # the same bad replies after the step fail the run
        records[400:500] = [record((400 + i) * 10450, 215, reply(crc_ok=False)) for i in range(100)]
        failed = session_summary.summarize(folder)
        self.assertEqual("fail", failed["verdict"])
        self.assertIn("from the throttle step on had a valid CRC, below min_crc_valid_pct=99.0", failed["reasons"][0])

    def test_a_clean_arming_phase_gives_no_note(self):
        clean = [record(i * 10450, 0, reply(not_running=True)) for i in range(300)] + self.records()[300:]
        line = {"line": 0, "gpio": 6, "mode": "bidirectional", "facts": session_summary.line_facts(clean, True)}
        self.assertIsNone(session_summary.arming_phase_note(line))

    def test_a_line_held_at_zero_has_no_arming_phase_block(self):
        facts = session_summary.line_facts([record(i * 10450, 0, reply(crc_ok=False, delay=5)) for i in range(100)], True)
        self.assertIsNone(facts["before_step"])
        self.assertIsNone(facts["from_step"])


class SummarizeFolderTest(unittest.TestCase):
    def folder(self, files):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-10-30_09-00-00"
        folder.mkdir()
        for name, text in files.items():
            (folder / name).write_text(text, encoding="utf-8")
        return folder

    def test_completed_run_without_a_recording_is_unknown_and_says_why(self):
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_COMPLETED, "run.log": "Armed after 2000ms.\n"}))
        self.assertEqual("unknown", summary["verdict"])
        self.assertEqual([], summary["reasons"])
        self.assertIn("no wire recording", summary["notes"][0])
        self.assertIn("completed on the Pico", summary["notes"][0])
        self.assertIn("UNKNOWN", session_summary.table(summary))
        self.assertEqual(2000, summary["device"]["armed_ms"])
        self.assertIsNone(summary["observer"])
        self.assertEqual(["session.json", "session.json:device", "run.log"], summary["sources"])

    def test_a_completed_run_with_a_valid_recording_passes_and_with_an_invalid_one_fails(self):
        stand_in_wire(self, True)
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_COMPLETED, "signal_observation.bin": "x", "run.log": ARMED_LOG}))
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual([], summary["notes"])
        stand_in_wire(self, False)
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_COMPLETED, "signal_observation.bin": "x", "run.log": ARMED_LOG}))
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("cannot be trusted", summary["reasons"][0])

    def test_unidirectional_lines_only_pass_on_a_valid_recording_without_replies(self):
        stand_in_wire(self, True, mode="unidirectional")
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_COMPLETED, "signal_observation.bin": "x", "run.log": ARMED_LOG}))
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual([], summary["reasons"])

    def test_a_failed_outcome_fails_with_or_without_a_recording(self):
        stand_in_wire(self, True)
        with_wire = session_summary.summarize(self.folder({"session.json": DEVICE_ABORTED, "signal_observation.bin": "x"}))
        without = session_summary.summarize(self.folder({"session.json": DEVICE_ABORTED}))
        self.assertEqual(["fail", "fail"], [with_wire["verdict"], without["verdict"]])
        self.assertIn("no wire recording", without["notes"][0])

    def test_a_run_that_never_armed_fails_with_the_picos_reason(self):
        summary = session_summary.summarize(self.folder({"session.json": DEVICE_ABORTED, "run.log": "RuntimeError: arming did not complete\n"}))
        self.assertEqual("fail", summary["verdict"])
        self.assertIn("the Pico aborted the run: arming did not complete within 18000ms", summary["reasons"][0])
        armed = [p for p in summary["proofs"] if p["name"] == "armed within the bound"][0]
        self.assertEqual(session_summary.FAILED, armed["status"])
        self.assertIn("arming did not complete", armed["detail"])

    def test_a_missing_recording_names_why_it_is_missing(self):
        # a device script run on the Pico alone has no recording by design; a recording that failed to arrive is told apart
        without = session_summary.summarize(self.folder({"session.json": json.dumps({"observer": False, "device": {"outcome": "completed"}})}))
        self.assertEqual("unknown", without["verdict"])
        self.assertIn("this device script was run on the Pico alone, without the wingman", without["notes"][0])
        lost = session_summary.summarize(self.folder({"session.json": json.dumps({"observer": True, "device": {"outcome": "completed"}})}))
        self.assertEqual("unknown", lost["verdict"])
        self.assertIn("the wingman was started for this session but no signal_observation.bin arrived", lost["notes"][0])
        detail = [p["detail"] for p in lost["proofs"] if p["name"] == "wire recording complete"][0]
        self.assertIn("no signal_observation.bin arrived", detail)

    def test_a_folder_with_nothing_to_judge_is_unknown(self):
        self.assertEqual("unknown", session_summary.summarize(self.folder({"session.json": "{}"}))["verdict"])

    def test_device_section_of_session_json_is_read_like_meta_txt(self):
        folder = self.folder({"session.json": json.dumps({"device": {"outcome": "completed"}}), "run.log": "Armed after 2000ms.\n"})
        summary = session_summary.summarize(folder)
        self.assertEqual("unknown", summary["verdict"])
        self.assertIn("session.json:device", summary["sources"])

    def test_without_a_device_outcome_the_wire_decides(self):
        # stand-in wire summaries: one commanded line that kept answering, one run that never commanded anything
        def with_wire(commanded):
            stand_in_wire(self, True, lines=[wire_line(commanded=commanded)])
            return session_summary.summarize(self.folder({"session.json": json.dumps({"command": ["test_x.py"]}), "signal_observation.bin": "x"}))
        self.assertEqual("pass", with_wire(True)["verdict"])
        self.assertEqual("fail", with_wire(False)["verdict"])
        self.assertIn("no line was commanded", with_wire(False)["reasons"][0])

    def test_a_line_the_scenario_does_not_use_and_nothing_was_sent_on_is_not_driven(self):
        import observer_edge_log
        patches = {"parse_stream": lambda b: ("pins=6,7 sync_pin=15", {0: b"a", 1: b"b"}, "end"),
                   "parse_end": lambda e: {"valid": True}, "reconcile": lambda c, i: [],
                   "edges": lambda c, invert=False: [(0, 0)],
                   "analyse": lambda e, u: [record(0, 100)] if e is not None and self.calls.pop() else []}
        self.calls = [False, True]            # line 0 gets a frame, line 1 none
        for name, fn in patches.items():
            self.addCleanup(setattr, observer_edge_log, name, getattr(observer_edge_log, name))
            setattr(observer_edge_log, name, fn)
        self.addCleanup(setattr, observer_check_am32, "run_checks", observer_check_am32.run_checks)
        observer_check_am32.run_checks = lambda records, unidirectional, from_first_frame: []
        scenario = {"motors": [{"pin": 6, "bidirectional": False}]}
        lines = session_summary.observer_summary(b"x", scenario)["lines"]
        self.assertEqual(["unidirectional", "not driven"], [line["mode"] for line in lines])
        self.assertEqual([], lines[1]["reasons"])
        self.assertEqual({}, lines[1]["checks"]["counts"])

    def test_write_summary_writes_json_that_reads_back(self):
        folder = self.folder({"session.json": DEVICE_COMPLETED})
        session_summary.write_summary(folder)
        self.assertEqual("unknown", json.loads((folder / "summary.json").read_text())["verdict"])

    def test_table_prints_verdict_and_reasons(self):
        text = session_summary.table(session_summary.summarize(self.folder({"session.json": DEVICE_ABORTED})))
        self.assertIn("FAIL", text)
        self.assertIn("the Pico aborted the run", text)


class StartFailureTest(unittest.TestCase):
    """A session that did not start (session.json "start_failure") is failed with the reason; its other proofs are not shown."""

    def folder(self, who, message, extra=None):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-11-12_09-00-00"
        folder.mkdir()
        files = {"session.json": json.dumps({"start_failure": {"who": who, "message": message}, "device": {"outcome": "completed"}}),
                 "run.log": "Armed after 2000ms.\n"}
        files.update(extra or {})
        for name, text in files.items():
            (folder / name).write_text(text, encoding="utf-8")
        return folder

    def test_a_wingman_that_did_not_start_fails_the_session_with_its_reason(self):
        summary = session_summary.summarize(self.folder("wingman", "the wingman Pico (ID 5d70) is not connected"))
        self.assertEqual("fail", summary["verdict"])
        self.assertEqual(["the wingman did not start: the wingman Pico (ID 5d70) is not connected"], summary["reasons"])
        self.assertEqual({"who": "wingman", "message": "the wingman Pico (ID 5d70) is not connected"}, summary["start_failure"])

    def test_a_main_pico_that_did_not_start_names_the_main_pico(self):
        summary = session_summary.summarize(self.folder("main_pico", "the upload to it did not complete"))
        self.assertEqual("fail", summary["verdict"])
        self.assertEqual(["the main Pico did not start: the upload to it did not complete"], summary["reasons"])

    def test_the_other_proofs_are_not_shown_and_nothing_cascades(self):
        summary = session_summary.summarize(self.folder("wingman", "x"))
        self.assertEqual(["failed"] + ["not shown"] * 5, [p["status"] for p in summary["proofs"]])
        self.assertEqual("run finished cleanly", summary["proofs"][0]["name"])
        self.assertEqual([], summary["notes"])
        self.assertIsNone(summary["observer"])
        self.assertIsNone(summary["device"]["armed_ms"])        # run.log is not read as the record of a run
        self.assertIsNone(summary["device"]["outcome"])
        self.assertIn("did not start", session_summary.table(summary))

    def test_a_recording_left_in_the_folder_is_not_read(self):
        summary = session_summary.summarize(self.folder("main_pico", "x", {"signal_observation.bin": "not a stream"}))
        self.assertIsNone(summary["observer"])
        self.assertEqual("fail", summary["verdict"])

    def test_re_running_on_the_folder_gives_the_same_verdict(self):
        folder = self.folder("wingman", "x")
        first = session_summary.write_summary(folder)
        again = session_summary.summarize(folder)
        self.assertEqual((first["verdict"], first["reasons"], first["proofs"]), (again["verdict"], again["reasons"], again["proofs"]))
        self.assertEqual("fail", json.loads((folder / "summary.json").read_text())["verdict"])


class ProofTest(unittest.TestCase):
    """The verdict is built from required proofs: pass needs every applicable one shown."""

    def summary(self, files=None, scenario=None, manifest=None, **wire):
        files = {"run.log": ARMED_LOG, "signal_observation.bin": "x"} if files is None else dict(files)
        manifest = {"device": {"outcome": "completed"}, **(manifest or {})}
        files["session.json"] = json.dumps(manifest)
        if scenario is not None:
            files["scenario.json"] = json.dumps(scenario)
        if "signal_observation.bin" in files:
            stand_in_wire(self, **{"valid": True, **wire})
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name) / "2026-11-11_09-00-00"
        folder.mkdir()
        for name, text in files.items():
            (folder / name).write_text(text, encoding="utf-8")
        return session_summary.summarize(folder)

    def status(self, summary, name):
        return {p["name"]: p["status"] for p in summary["proofs"]}[name]

    def test_a_healthy_run_shows_all_six_proofs(self):
        self.assertIsNone(self.summary()["start_failure"])
        summary = self.summary()
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual(6, len(summary["proofs"]))
        self.assertEqual({"shown"}, {p["status"] for p in summary["proofs"]})
        self.assertEqual([], summary["limitations"])
        self.assertIn("[shown]", session_summary.table(summary))

    def test_every_proof_can_fail_and_fails_the_run(self):
        late = "Armed after %dms.\n" % session_summary.CLEAN_ARM_THRESHOLD_MS
        cases = {
            "run finished cleanly": dict(manifest={"device": {"outcome": "aborted: Core 1 loop did not stop before disarm()"}}),
            "armed within the bound": dict(files={"run.log": late, "signal_observation.bin": "x"}),
            "wire recording complete": dict(valid=False),
            "our frames were correct": dict(lines=[wire_line(frames_reasons=["line 0 (channel 1): AM32 check failed: CRC"])]),
            "motors spun as specified": dict(lines=[wire_line(spin_reasons=["line 0 (channel 1): commanded to spin but never reported a real eRPM"])]),
            "motors stopped at the end": dict(lines=[wire_line(stop_commanded=False, last_throttle=215)]),
        }
        for name, kwargs in cases.items():
            with self.subTest(name):
                summary = self.summary(**kwargs)
                self.assertEqual("failed", self.status(summary, name))
                self.assertEqual("fail", summary["verdict"])
                self.assertTrue(summary["reasons"])

    def test_every_proof_can_be_not_shown_and_then_the_verdict_is_unknown(self):
        cases = {
            "run finished cleanly": dict(manifest={"device": None}),
            "armed within the bound": dict(files={"signal_observation.bin": "x"}),
            "wire recording complete": dict(files={"run.log": ARMED_LOG}),
            "our frames were correct": dict(files={"run.log": ARMED_LOG}),
            "motors spun as specified": dict(files={"run.log": ARMED_LOG}),
            "motors stopped at the end": dict(files={"run.log": ARMED_LOG}),
        }
        for name, kwargs in cases.items():
            with self.subTest(name):
                summary = self.summary(**kwargs)
                self.assertEqual("not shown", self.status(summary, name))
                self.assertEqual("unknown", summary["verdict"])
                self.assertEqual([], summary["reasons"])

    def test_a_missing_arming_time_is_unknown_and_a_late_one_is_a_failure(self):
        missing = self.summary(files={"signal_observation.bin": "x"})
        self.assertEqual("unknown", missing["verdict"])
        self.assertIn("no arming time", missing["notes"][0])
        late = self.summary(files={"run.log": "Armed after 4600ms.\n", "signal_observation.bin": "x"})
        self.assertEqual("fail", late["verdict"])
        self.assertIn("4600 ms", late["reasons"][0])
        on_the_bound = "Armed after %dms.\n" % session_summary.CLEAN_ARM_THRESHOLD_MS
        self.assertEqual("fail", self.summary(files={"run.log": on_the_bound, "signal_observation.bin": "x"})["verdict"])

    def test_the_bound_is_the_one_run_scenarios_uses(self):
        import arming_bound
        import run_scenarios
        self.assertEqual(arming_bound.CLEAN_ARM_THRESHOLD_MS, session_summary.CLEAN_ARM_THRESHOLD_MS)
        self.assertEqual(arming_bound.CLEAN_ARM_THRESHOLD_MS, run_scenarios.CLEAN_ARM_THRESHOLD_MS)

    def test_a_scenario_pin_missing_from_the_recording_or_without_a_frame_fails_the_recording_proof(self):
        scenario = {"motors": [{"pin": 6, "bidirectional": True}, {"pin": 7, "bidirectional": True}]}
        missing = self.summary(scenario=scenario, missing_pins=[7])
        self.assertEqual("failed", self.status(missing, "wire recording complete"))
        self.assertIn("GPIO 7", missing["reasons"][0])
        silent = self.summary(scenario=scenario, lines=[wire_line(line=0, gpio=6), wire_line(line=1, gpio=7, frames=0, stop_commanded=False)])
        self.assertEqual("failed", self.status(silent, "wire recording complete"))
        self.assertIn("GPIO 7 of the scenario has no complete frame", silent["reasons"][0])

    def test_a_unidirectional_only_scenario_passes_and_carries_the_limitation(self):
        summary = self.summary(lines=[wire_line(gpio=6, mode="unidirectional"), wire_line(line=1, gpio=7, mode="unidirectional")])
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual("not applicable", self.status(summary, "motors spun as specified"))
        self.assertEqual(2, len(summary["limitations"]))
        self.assertIn("motor on GPIO 6: spin not verified - unidirectional, no reply; visual confirmation by the operator", summary["limitations"][0])
        self.assertIn("optical RPM measurement", summary["limitations"][0])
        self.assertIn("limitation", session_summary.table(summary))

    def test_a_mixed_scenario_judges_the_bidirectional_motor_and_limits_the_other(self):
        lines = [wire_line(gpio=6), wire_line(line=1, gpio=7, mode="unidirectional")]
        summary = self.summary(lines=lines)
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual("shown", self.status(summary, "motors spun as specified"))
        self.assertEqual(1, len(summary["limitations"]))
        self.assertIn("GPIO 7", summary["limitations"][0])
        lines[0] = wire_line(gpio=6, spin_reasons=["line 0 (channel 1): commanded to spin but never reported a real eRPM"])
        self.assertEqual("fail", self.summary(lines=lines)["verdict"])

    def test_a_motor_held_at_zero_needs_no_spin_proof_and_no_limitation(self):
        idle = wire_line(gpio=7, mode="unidirectional", commanded=False)
        held = wire_line(line=1, gpio=8, commanded=False, replies_real_erpm=0)
        summary = self.summary(lines=[wire_line(gpio=6), idle, held])
        self.assertEqual("pass", summary["verdict"])
        self.assertEqual([], summary["limitations"])
        only_held = self.summary(lines=[held])
        self.assertEqual("not applicable", self.status(only_held, "motors spun as specified"))
        self.assertIn("held at throttle 0", [p for p in only_held["proofs"] if p["name"] == "motors spun as specified"][0]["detail"])

    def test_a_scenario_that_commands_a_spin_the_wire_never_shows_fails(self):
        line = wire_line(commanded=False, replies_real_erpm=0, scenario_max_throttle=215)
        line["spin_reasons"] = session_summary.spin_reasons(line)
        self.assertIn("never shows a throttle", line["spin_reasons"][0])
        self.assertEqual("fail", self.summary(lines=[line])["verdict"])

    def test_thresholds_set_but_too_few_replies_are_not_shown_not_shown_by_default(self):
        line = wire_line(notes=["line 0 (channel 1): only 10 replies on the wire, need 20 to judge min_crc_valid_pct"],
                         expect={"min_crc_valid_pct": 99.0})
        summary = self.summary(lines=[line])
        self.assertEqual("not shown", self.status(summary, "motors spun as specified"))
        self.assertEqual("unknown", summary["verdict"])
        self.assertIn("only 10 replies", summary["notes"][0])

    def test_the_stop_is_the_last_frames_carrying_throttle_zero(self):
        n = session_summary.STOP_TAIL_FRAMES
        down = [record(i * 10450, 100, reply(erpm=8000.0)) for i in range(20)] + [record((20 + i) * 10450, 0, reply(erpm=8000.0)) for i in range(n)]
        self.assertTrue(session_summary.line_facts(down, True)["stop_commanded"])
        still_running = [record(i * 10450, 0, None) for i in range(20)] + [record((20 + i) * 10450, 100, None) for i in range(n)]
        facts = session_summary.line_facts(still_running, False)
        self.assertFalse(facts["stop_commanded"])
        self.assertEqual(100, facts["last_throttle"])
        one_nonzero = ([record(i * 10450, 0, None) for i in range(20)] + [record(20 * 10450, 50, None)]
                       + [record((21 + i) * 10450, 0, None) for i in range(n - 1)])
        self.assertFalse(session_summary.line_facts(one_nonzero, False)["stop_commanded"])
        self.assertFalse(session_summary.line_facts([record(0, 0, None)], False)["stop_commanded"])

    def test_the_esc_mock_makes_arming_and_spin_not_shown_so_the_verdict_is_unknown_and_keeps_the_note(self):
        summary = self.summary(files={"signal_observation.bin": "x"}, manifest={"esc_mock": {"replays": "x"}})
        self.assertEqual("not shown", self.status(summary, "armed within the bound"))
        self.assertEqual("not shown", self.status(summary, "motors spun as specified"))
        self.assertEqual("unknown", summary["verdict"])
        self.assertIn("ESC mock", summary["notes"][0])

    def test_an_esc_mock_session_with_a_real_failure_still_fails(self):
        summary = self.summary(files={"signal_observation.bin": "x"}, manifest={"esc_mock": {"replays": "x"}, "device": {"outcome": ARMING_REFUSED}})
        self.assertEqual("fail", summary["verdict"])

    def test_a_device_script_is_judged_from_the_wire(self):
        manifest = {"command": ["test_x.py"], "device": None}
        summary = self.summary(files={"signal_observation.bin": "x"}, manifest=manifest, lines=[wire_line(mode="unknown")])
        self.assertEqual("not applicable", self.status(summary, "run finished cleanly"))
        self.assertEqual("not applicable", self.status(summary, "armed within the bound"))
        self.assertEqual("pass", summary["verdict"])
        self.assertIn("no scenario says", summary["limitations"][0])
        idle = self.summary(files={"signal_observation.bin": "x"}, manifest=manifest, lines=[wire_line(mode="unknown", commanded=False)])
        self.assertEqual("fail", idle["verdict"])
        self.assertIn("no line was commanded", idle["reasons"][0])

    def test_a_script_that_reports_an_arming_time_over_the_bound_fails(self):
        late = self.summary(files={"run.log": "Armed after 5000ms.\n", "signal_observation.bin": "x"}, manifest={"command": ["t.py"], "device": None})
        self.assertEqual("fail", late["verdict"])

    def test_an_aborted_run_is_a_fail_with_or_without_a_recording(self):
        for files in ({"signal_observation.bin": "x"}, {}):
            summary = self.summary(files=files, manifest={"device": {"outcome": "aborted: motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms"}})
            self.assertEqual("fail", summary["verdict"])
            self.assertIn("no real eRPM for 2004ms", summary["reasons"][0])

    def test_a_run_whose_record_never_got_its_end_fails(self):
        summary = self.summary(manifest={"device": {"outcome": "running"}})
        self.assertEqual("failed", self.status(summary, "run finished cleanly"))
        self.assertIn("does not say the run completed (outcome: running", summary["reasons"][0])

    def test_missing_pins_are_reported_by_observer_summary(self):
        import observer_edge_log
        patches = {"parse_stream": lambda b: ("pins=6 sync_pin=15", {0: b"a"}, "end"),
                   "parse_end": lambda e: {"valid": True}, "reconcile": lambda c, i: [],
                   "edges": lambda c, invert=False: [(0, 0)], "analyse": lambda e, u: [record(0, 100)]}
        for name, fn in patches.items():
            self.addCleanup(setattr, observer_edge_log, name, getattr(observer_edge_log, name))
            setattr(observer_edge_log, name, fn)
        self.addCleanup(setattr, observer_check_am32, "run_checks", observer_check_am32.run_checks)
        observer_check_am32.run_checks = lambda records, unidirectional, from_first_frame: []
        scenario = {"motors": [{"pin": 6, "bidirectional": False}, {"pin": 7, "bidirectional": False}]}
        self.assertEqual([7], session_summary.observer_summary(b"x", scenario)["missing_pins"])


class PropellersTest(unittest.TestCase):
    def test_the_propellers_come_from_the_scenarios_hardware_blocks_in_motor_order(self):
        scenario = {"motors": [{"hardware": {"propeller": "Gemfan 2218 3-Blade"}}, {"hardware": {"propeller": "none"}}, {}]}
        self.assertEqual(["Gemfan 2218 3-Blade", "none", None], session_summary.propellers(scenario))

    def test_all_the_same_is_said_once_and_differences_are_listed(self):
        self.assertEqual("Gemfan 2218 3-Blade on all 4 motors", session_summary.propellers_text(["Gemfan 2218 3-Blade"] * 4))
        self.assertEqual("motor 1: A, motor 2: none, motor 3: not recorded", session_summary.propellers_text(["A", "none", None]))


if __name__ == "__main__":
    unittest.main()
