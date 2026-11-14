"""
session_summary.py - one computed summary of a finished session, written as summary.json in the session folder.

  python scripts/session_summary.py captures/<session>          # compute, write summary.json, print the table
  python scripts/session_summary.py captures/<session> --no-write

scripts/run_session.py runs this at the end of every run. It can be re-run on any session folder: the summary is
computed from the files in it, never stored anywhere else, so when the analysis code improves, re-running refreshes it.

This is the only judge of a session (decision/ADR-008): the main Pico reports facts and may bail out of a run, it never judges one.

What goes in (each source is optional; what is missing is left out and named under "sources"):
  - copied from the Pico, not recomputed: how the run ended, "completed" or "aborted: <reason>" (the "outcome" of the "device" section of
    session.json), the arming time and the last error line (run.log);
  - computed from signal_observation.bin, per line: the line's mode (the scenario says which motors are bidirectional), frames,
    how many were answered, replies with a bad CRC, replies reporting a real eRPM, replies reporting "not running" (and how many
    of those came after the line had reported a real eRPM), replies that started within 10 us of our frame, the intervals of 40-400 us between edges ("slow gaps": not
    frames, not replies), and the verdicts of tests/bench/wingman/tools/observer_check_am32.py; per bidirectional line commanded to spin, the
    reply counts split at the throttle step (the first of three equal frames with a throttle of 48 or more, observer_start_summary.find_step):
    "before_step" (the arming phase: replies, bad CRC, early starts) and "from_step" (the window the decode thresholds are judged on).

The verdict is built from required proofs, not from the absence of failures. Each proof has a name, a status and a one-line detail
(summary.json "proofs", and the printed table):
  shown            the evidence in the folder shows it
  failed           the evidence shows it did not hold
  not shown        it could not be judged from the evidence in the folder (a missing file, too few replies for a threshold, ...)
  not applicable   it does not apply to this session (see below)
The verdict is "fail" when any proof failed, "pass" only when every applicable proof is shown, and "unknown" otherwise. The proofs:
  1. run finished cleanly         the Pico's outcome is "completed" (this includes the command loop having stopped before deactivate()); an
                                  outcome "aborted: <reason>" fails it with that reason; a session that did not start (session.json
                                  "start_failure": who and message, written by run_session.py) fails it with "the wingman / the main
                                  Pico did not start: <message>", and then every other proof is "not shown" ("the session did not
                                  start"): nothing ran, so the folder's record of a run is not read
  2. armed within the bound       "Armed after N ms" in run.log is known and below CLEAN_ARM_THRESHOLD_MS (scripts/arming_bound.py)
  3. wire recording complete      signal_observation.bin exists, lost no data, and every motor pin of the scenario is a recorded line with a complete frame of ours
  4. our frames were correct      no AM32 check failed on any line, and no stray activity (40-400 us intervals) on the wires
  5. motors spun as specified     every bidirectional motor the scenario commands to spin reached a real eRPM, kept reporting one
                                  (LOST_SPIN_FRAMES) and kept answering (LOST_REPLY_FRAMES), and met the scenario's min_crc_valid_pct and
                                  min_median_erpm where it sets them, judged on the replies from the throttle step onward (a threshold set but
                                  without enough replies in that window, or a line with no step on the wire, is "not shown"). Replies before the
                                  step (arming) are reported as facts and, when any had a bad CRC or started early, as a note; they are not judged
  6. motors stopped at the end    the last STOP_TAIL_FRAMES frames of every driven line carry throttle 0
Not required, by decision: a bidirectional motor held at throttle 0 for the whole run needs no spin proof (the Pico's arming already required
its replies); the ESC reporting "not running" after the stop is not required (the two sessions of 2026-11-10 show the ESC still reporting a
decaying real eRPM in the last frame, so the wire does not show it reliably).
A unidirectional motor sends no reply: that it spun cannot be shown from the wire and does not block a pass. Every such run carries it as a
"limitations" entry (summary.json and the table) until optical RPM measurement exists.
Special cases: a session played with the ESC mock is not evidence about an ESC, a motor or arming, so proofs 2 and 5 are "not shown" and
its verdict is never "pass" (it is "unknown" when nothing failed, "fail" when something did; a note says so). A device script run through run_session.py has no scenario
outcome: proof 1 is not applicable, proof 2 is judged only if run.log has an arming time, proof 5 needs a commanded line (the wire alone
decides; with no scenario the motors' modes are unknown, so replies are not judged). An aborted run is "fail" with or without a
recording; no recording at all is "unknown" with a note that names why there is none (a device script run on the Pico alone, or the wingman delivered no
signal_observation.bin). "reasons" lists why a run failed, "notes" what was not shown and other remarks.
Only the session format the current scenario runner writes is read; folders written by an older runner are not supported (ADR-008).
"""

import argparse
import json
import re
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
import session_files
import observer_check_am32
import observer_edge_log
import observer_start_summary
import start_kpi
import observer_stats
from arming_bound import ARMING_REFUSED_TEXT, CLEAN_ARM_THRESHOLD_MS

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "tests" / "bench"))      # throttle_profile.py: what a scenario's profile commands (pure Python, shared with the Pico)
try:
    from throttle_profile import ThrottleProfile
except ImportError:      # a tree without it: the scenario's own throttle is then not compared with the wire
    ThrottleProfile = None
COMMANDED_MIN = 48                  # throttle values from here up make a motor spin (1-47 are commands)
EARLY_REPLY_US = 10                 # a normal reply starts about 23 us after our frame
SLOW_GAP_US = (40, 400)             # no frame, reply or turnaround has an interval between edges in this range
SLOW_GAPS_FLAGGED = 100             # a line with this many such intervals is flagged; my choice, not measured
LOST_SPIN_FRAMES = 50               # 'not running' replies this many frames after a real eRPM (about 50 ms) flag a line; my choice, not measured
LOST_REPLY_FRAMES = 50              # frames without any reply, this many after a real eRPM while commanded, flag a line; my choice, not measured
STOP_TAIL_FRAMES = 5                # the stop is shown by the last this many frames of a line all carrying throttle 0 (the real runs end with about 19)
MIN_JUDGED_REPLIES = 20            # fewer replies than this cannot support a percentage or a median: reported as not judged, never failed
ABORTED = "aborted: "               # how the Pico's outcome starts when it bailed out of a run; the reason follows
RUN_LOG_ERROR = re.compile(r"^(?:\s*\|\s*)?(\w*Error: .*)$")
ARMED_AFTER = re.compile(r"Armed after (\d+)ms")
SHOWN, FAILED, NOT_SHOWN, NOT_APPLICABLE = "shown", "failed", "not shown", "not applicable"     # the status of a proof
UNIDIRECTIONAL_LIMITATION = ("motor on GPIO %s: spin not verified - unidirectional, no reply; visual confirmation by the operator is the only "
                             "check until optical RPM measurement exists")


def parse_run_log(text):
    """(arming time in ms or None, the last error line of the run or None) from the console log."""
    armed = ARMED_AFTER.search(text)
    errors = [m.group(1) for m in (RUN_LOG_ERROR.match(line) for line in text.splitlines()) if m]
    return (int(armed.group(1)) if armed else None), (errors[-1] if errors else None)


def motors_by_pin(scenario):
    """{GPIO pin: motor dict} of a scenario JSON."""
    return {motor["pin"]: motor for motor in scenario.get("motors", [])}


def pins_of_stream(start_text):
    """The observer's start text 'pins=6,7,8,9 ...' -> {line index: GPIO}."""
    for token in start_text.split():
        if token.startswith("pins="):
            return dict(enumerate(int(p) for p in token[5:].split(",")))
    return {}


def line_facts(records, bidirectional):
    """The numbers about one line, from observer_edge_log.analyse() records."""
    rows = observer_stats.frame_table(records)
    frames = len(rows)
    throttles = [row["throttle"] for row in rows]
    facts = {
        "frames": frames,
        "unparsed_groups": sum(1 for r in records if r["tx"] is None),
        "frames_bad_crc": sum(1 for row in rows if not row["crc_ok"]),
        "bursts_before_frame": sum(1 for r in records if r.get("stray_edges_before")),
        "burst_clearance_min_us": min((r["stray_clearance"] / 10.0 for r in records if r.get("stray_clearance") is not None), default=None),
        "max_throttle": max(throttles) if throttles else None,
    }
    facts["commanded"] = bool(throttles) and facts["max_throttle"] >= COMMANDED_MIN
    facts["last_throttle"] = throttles[-1] if throttles else None
    facts["stop_commanded"] = len(throttles) >= STOP_TAIL_FRAMES and all(t == 0 for t in throttles[-STOP_TAIL_FRAMES:])
    if not bidirectional:
        return facts
    answered = [row for row in rows if row["reply"] != "none"]
    run = [row for row in rows if row["reply"] == "run"]
    first_run = run[0]["t_ms"] if run else None
    erpms = [row["erpm"] for row in run if row["erpm"] is not None]
    facts.update({
        "answered": len(answered),
        "answered_pct": round(100.0 * len(answered) / frames, 1) if frames else None,
        "replies_bad_crc": sum(1 for row in answered if row["reply"] == "bad"),
        "replies_real_erpm": len(run),
        "replies_not_running": sum(1 for row in answered if row["reply"] == "idle"),
        "not_running_after_first_real_erpm": (sum(1 for row in rows if row["t_ms"] > first_run and row["reply"] == "idle"
                                                  and row["throttle"] >= COMMANDED_MIN) if run else None),
        "unanswered_after_first_real_erpm": (sum(1 for row in rows if row["t_ms"] > first_run and row["reply"] == "none"
                                                 and row["throttle"] >= COMMANDED_MIN) if run else None),
        "early_replies": sum(1 for row in answered if row["delay_us"] is not None and row["delay_us"] < EARLY_REPLY_US),
        "first_real_erpm_ms": None if first_run is None else round(first_run, 1),
        "median_erpm": round(statistics.median(erpms)) if erpms else None,
    })
    # spin-up: ms from the first nonzero throttle frame to 80% of the motor's own settled speed (reported, not judged: no limit is set yet)
    step = observer_start_summary.find_step(rows) if facts["commanded"] else None
    facts.update(split_at_step(rows, step))
    spin_up_ms, settled_erpm = observer_start_summary.spin_up(rows, step) if step is not None else (None, None)
    cut_after_ms = observer_start_summary.final_cut_after_ms(rows, step) if step is not None else None
    hold_outside = observer_start_summary.hold_outside_pct(rows, step, spin_up_ms, settled_erpm) if step is not None else None
    facts.update({"spin_up_ms": None if spin_up_ms is None else round(spin_up_ms, 1), "settled_erpm": settled_erpm,
                  "cut_after_ms": None if cut_after_ms is None else round(cut_after_ms, 1),
                  "step_throttle": rows[step]["throttle"] if step is not None else None,
                  "hold_outside_pct": None if hold_outside is None else round(hold_outside, 2)})
    return facts


def reply_counts(rows):
    """Replies, replies with a bad CRC and replies that started earlier than a normal reply, over frame-table rows."""
    answered = [row for row in rows if row["reply"] != "none"]
    return {"replies": len(answered), "replies_bad_crc": sum(1 for row in answered if row["reply"] == "bad"),
            "early_replies": sum(1 for row in answered if row["delay_us"] is not None and row["delay_us"] < EARLY_REPLY_US)}


def split_at_step(rows, step):
    """The reply facts of one bidirectional line split at the throttle step (rows[step], the first of three equal frames of 48 or more):
    "before_step" is the arming phase, reported only; "from_step" is the window the scenario's decode thresholds are judged on (ADR-008).
    All None for a line with no step on the wire."""
    if step is None:
        return {"step_ms": None, "before_step": None, "from_step": None}
    after = rows[step:]
    erpms = [row["erpm"] for row in after if row["reply"] == "run" and row["erpm"] is not None]
    from_step = reply_counts(after)
    from_step.update({"replies_real_erpm": sum(1 for row in after if row["reply"] == "run"),
                      "median_erpm": round(statistics.median(erpms)) if erpms else None})
    before = reply_counts(rows[:step])
    before["frames"] = step
    return {"step_ms": round(rows[step]["t_ms"], 1), "before_step": before, "from_step": from_step}


def arming_phase_note(line):
    """A note on a line whose replies before the throttle step include a bad CRC or an early start, else None: facts, not a failure."""
    before = line["facts"].get("before_step")
    if not before or not (before["replies_bad_crc"] or before["early_replies"]):
        return None
    return ("%s: before the throttle step at %.1f ms (arming phase, not judged): %d replies, %d with a bad CRC, %d starting less than %d us "
            "after our frame" % (where_of(line), line["facts"]["step_ms"], before["replies"], before["replies_bad_crc"],
                                 before["early_replies"], EARLY_REPLY_US))


def slow_gaps(edge_list):
    """How many intervals between consecutive edges are 40-400 us: not inside a frame (bits are under 4 us), not a reply
    turnaround (about 23 us), not between frames (about 1 ms). Edges are (time in 0.1 us units, level)."""
    low, high = SLOW_GAP_US
    return sum(1 for (a, _), (b, _) in zip(edge_list, edge_list[1:]) if low <= (b - a) / 10.0 < high)


def check_summary(results, esc_mock=False):
    """AM32 check results -> status counts and the details of every FAIL and WARN. esc_mock: the replies came from the
    ESC mock, so the checks about the real ESC's behaviour count as not applicable."""
    counts = {}
    notable = []
    for result in results:
        if esc_mock and result.esc_only:        # about the real ESC's own behaviour; the ESC mock answers from the first frame, with no arming silence
            counts[observer_check_am32.NA] = counts.get(observer_check_am32.NA, 0) + 1
            continue
        counts[result.status] = counts.get(result.status, 0) + 1
        if result.status in (observer_check_am32.FAIL, observer_check_am32.WARN):
            notable.append({"status": result.status, "check": result.name, "detail": result.detail})
    return {"counts": counts, "notable": notable}


def where_of(line):
    return "line %d (channel %d)" % (line["line"], line["line"] + 1)


def frame_reasons(line):
    """What makes proof 4 (our frames were correct) fail on one line: a failed AM32 check, or stray activity on the wire."""
    reasons = []
    for item in line["checks"]["notable"]:
        if item["status"] == observer_check_am32.FAIL:
            reasons.append("%s: AM32 check failed: %s" % (where_of(line), item["check"]))
    slow = line["facts"].get("slow_gaps", 0)
    if slow >= SLOW_GAPS_FLAGGED:
        reasons.append("%s: %d intervals of 40-400 us between edges - activity that is neither our frames nor replies" % (where_of(line), slow))
    return reasons


def spin_reasons(line):
    """What makes proof 5 (motors spun as specified) fail on one bidirectional line, from the wire alone."""
    reasons = []
    if line["mode"] != "bidirectional":
        return reasons
    facts = line["facts"]
    if (line.get("scenario_max_throttle") or 0) >= COMMANDED_MIN and not facts["commanded"]:
        reasons.append("%s: the scenario commands it to spin (throttle %s) but the wire never shows a throttle of %d or more" % (
            where_of(line), line["scenario_max_throttle"], COMMANDED_MIN))
    if facts["commanded"] and not facts.get("replies_real_erpm"):
        reasons.append("%s: commanded to spin but never reported a real eRPM" % where_of(line))
    lost = facts.get("not_running_after_first_real_erpm")
    if lost is not None and lost >= LOST_SPIN_FRAMES:
        reasons.append("%s: reported 'not running' for %d frames after it had reported a real eRPM, while commanded to spin" % (where_of(line), lost))
    unanswered = facts.get("unanswered_after_first_real_erpm")
    if unanswered is not None and unanswered >= LOST_REPLY_FRAMES:
        reasons.append("%s: %d frames got no reply after it had reported a real eRPM, while commanded to spin" % (where_of(line), unanswered))
    return reasons


def line_reasons(line):
    return frame_reasons(line) + spin_reasons(line)


def scenario_max_throttle(motor):
    """The highest throttle the scenario's profile for that motor ever commands, or None when it cannot be told."""
    if ThrottleProfile is None or not motor or not motor.get("profile"):
        return None
    try:
        return ThrottleProfile(motor["profile"]).max_throttle
    except (ValueError, KeyError, TypeError):
        return None


def expect_for(scenario, key, index):
    """The scenario's `expect` threshold of that name for the motor at that list position, or None."""
    return ((scenario or {}).get("expect") or {}).get(key, {}).get(str(index))


def judge_expect(line, min_crc_valid_pct, min_median_erpm):
    """(reasons, notes): the scenario's decode thresholds judged on the replies the observer saw from the throttle step onward (facts
    "from_step"), for a commanded bidirectional line; the arming phase before it is not judged (ADR-008). Too few replies to judge a
    threshold, or no step on the wire, is a note, never a failure."""
    reasons, notes = [], []
    facts = line["facts"]
    if line["mode"] != "bidirectional" or not facts["commanded"] or (min_crc_valid_pct is None and min_median_erpm is None):
        return reasons, notes
    where = where_of(line)
    window = facts.get("from_step")
    if window is None:
        notes.append("%s: no throttle step (three equal frames of %d or more) on the wire, so the decode thresholds were not judged" % (
            where, COMMANDED_MIN))
        return reasons, notes
    if min_crc_valid_pct is not None:
        answered = window["replies"]
        if answered < MIN_JUDGED_REPLIES:
            notes.append("%s: only %d replies on the wire from the throttle step on, need %d to judge min_crc_valid_pct" % (
                where, answered, MIN_JUDGED_REPLIES))
        else:
            valid = 100.0 * (answered - window["replies_bad_crc"]) / answered
            if valid < min_crc_valid_pct:
                reasons.append("%s: %.1f%% of replies on the wire from the throttle step on had a valid CRC, below min_crc_valid_pct=%s" % (
                    where, valid, min_crc_valid_pct))
    if min_median_erpm is not None:
        real = window["replies_real_erpm"]
        if real < MIN_JUDGED_REPLIES:
            if real:       # none at all is already a reason of its own (never spun)
                notes.append("%s: only %d real-eRPM replies on the wire from the throttle step on, need %d to judge min_median_erpm" % (
                    where, real, MIN_JUDGED_REPLIES))
        elif window["median_erpm"] < min_median_erpm:
            reasons.append("%s: median eRPM on the wire from the throttle step on %s, below min_median_erpm=%s" % (
                where, window["median_erpm"], min_median_erpm))
    return reasons, notes


def observer_summary(observer_bytes, scenario, from_first_frame=True, esc_mock=False):
    """The wire part of the summary from the bytes of signal_observation.bin and the scenario dict (None when unknown)."""
    start, channels, end = observer_edge_log.parse_stream(observer_bytes)
    info = observer_edge_log.parse_end(end)
    problems = observer_edge_log.reconcile(channels, info)
    pins = pins_of_stream(start)
    by_pin = motors_by_pin(scenario) if scenario else {}
    index_of_pin = {motor["pin"]: i for i, motor in enumerate((scenario or {}).get("motors", []))}
    lines = []
    for index in sorted(channels):
        pin = pins.get(index)
        motor = by_pin.get(pin)
        if motor is None:
            mode = "unknown"
        else:
            mode = "bidirectional" if motor.get("bidirectional") else "unidirectional"
        unidirectional = mode == "unidirectional"
        edge_list = observer_edge_log.edges(channels[index], invert=unidirectional)
        records = observer_edge_log.analyse(edge_list, unidirectional)
        facts = line_facts(records, mode == "bidirectional")
        facts["slow_gaps"] = slow_gaps(edge_list)
        if mode == "unknown" and not facts["frames"]:
            mode = "not driven"         # a line the scenario does not use and nothing was sent on: nothing to judge
            checks = check_summary([])
        else:
            checks = check_summary(observer_check_am32.run_checks(records, unidirectional, from_first_frame), esc_mock)
        line = {"line": index, "gpio": pin, "mode": mode, "facts": facts, "checks": checks,
                "scenario_max_throttle": scenario_max_throttle(motor)}
        line["frame_reasons"] = frame_reasons(line)
        line["spin_reasons"] = spin_reasons(line)
        line["notes"] = []
        line["expect"] = {}
        if pin in index_of_pin:
            thresholds = {key: expect_for(scenario, key, index_of_pin[pin]) for key in ("min_crc_valid_pct", "min_median_erpm")}
            line["expect"] = {key: value for key, value in thresholds.items() if value is not None}
            more, line["notes"] = judge_expect(line, thresholds["min_crc_valid_pct"], thresholds["min_median_erpm"])
            line["spin_reasons"] += more
        line["reasons"] = line["frame_reasons"] + line["spin_reasons"]
        lines.append(line)
    recorded = {line["gpio"] for line in lines}
    return {"valid": bool(info["valid"]) and not problems, "stream_problems": problems, "recording": start,
            "observer_end": end, "lines": lines, "missing_pins": sorted(pin for pin in by_pin if pin not in recorded)}


def proof(name, status, detail, reasons=(), notes=()):
    """One required proof of the verdict. reasons: why it failed; notes: why it is not shown (both are kept for summary.json's
    "reasons" and "notes" and are not part of the proof's own dict)."""
    return {"name": name, "status": status, "detail": detail, "reasons": list(reasons), "notes": list(notes)}


def failed_detail(reasons):
    return reasons[0] if len(reasons) == 1 else "%s (and %d more)" % (reasons[0], len(reasons) - 1)


def judge(name, reasons, notes, shown_detail):
    """failed when there are reasons, not shown when only notes, else shown."""
    if reasons:
        return proof(name, FAILED, failed_detail(reasons), reasons=reasons)
    if notes:
        return proof(name, NOT_SHOWN, notes[0] if len(notes) == 1 else "%s (and %d more)" % (notes[0], len(notes) - 1), notes=notes)
    return proof(name, SHOWN, shown_detail)


def proof_finished(device, is_script):
    name = "run finished cleanly"
    outcome = device["outcome"]
    if outcome is None:
        if is_script:
            return proof(name, NOT_APPLICABLE, "a device script reports no scenario outcome; the wire alone decides")
        return proof(name, NOT_SHOWN, "the Pico's record of the run (session.json device section) is missing", notes=[
            "not shown: the Pico's record of the run is missing, so whether it finished cleanly is unknown"])
    if outcome.startswith(ABORTED):
        return judge(name, ["the Pico aborted the run: %s" % outcome[len(ABORTED):]], [], "")
    if outcome != "completed":
        return judge(name, ["the Pico's record does not say the run completed (outcome: %s%s)" % (
            outcome, "; " + device["error"] if device["error"] else "")], [], "")
    return proof(name, SHOWN, "the Pico reports outcome completed (the command loop stopped before deactivate())")


def proof_armed(device, is_script, esc_mock, bound_ms=CLEAN_ARM_THRESHOLD_MS):
    name = "armed within the bound"
    if esc_mock:
        return proof(name, NOT_SHOWN, "the replies came from the ESC mock, not an ESC: they are not evidence that it armed")
    armed = device["armed_ms"]
    if armed is None:
        if ARMING_REFUSED_TEXT in (device["outcome"] or ""):
            return judge(name, ["the Pico reports that arming did not complete"], [], "")
        if is_script:
            return proof(name, NOT_APPLICABLE, "a device script with no arming time in run.log")
        return proof(name, NOT_SHOWN, "no arming time is known (run.log has no 'Armed after N ms' line)", notes=[
            "not shown: no arming time is known (run.log has no 'Armed after N ms' line)"])
    if armed >= bound_ms:
        return judge(name, ["armed after %d ms, at or over the %d ms bound" % (armed, bound_ms)], [], "")
    return proof(name, SHOWN, "armed after %d ms, under the %d ms bound" % (armed, bound_ms))


def driven_lines(observer):
    """The lines that carry a motor: not the ones the scenario does not use and nothing was sent on."""
    return [line for line in observer["lines"] if line["mode"] != "not driven"]


def missing_recording(manifest):
    """Why a session has no signal_observation.bin, from what session.json says about the wingman. A scenario session is always recorded, so
    only a device script (Pico alone, recorded on request) can have been run without the wingman."""
    if manifest.get("observer") is False:
        return "there is no wire recording: this device script was run on the Pico alone, without the wingman"
    if manifest.get("observer"):
        return "there is no wire recording: the wingman was started for this session but no signal_observation.bin arrived"
    return "there is no wire recording (signal_observation.bin)"


def proof_recording(observer, scenario, manifest=None):
    name = "wire recording complete"
    if observer is None:
        text = missing_recording(manifest or {})
        return proof(name, NOT_SHOWN, text, notes=[text])
    if not observer["valid"]:
        return judge(name, ["the observer lost data or the byte counts disagree: the wire record cannot be trusted"], [], "")
    reasons = ["GPIO %s of the scenario is not a recorded line" % pin for pin in observer.get("missing_pins", [])]
    lines = driven_lines(observer)
    if scenario is not None:
        pins = set(motors_by_pin(scenario))
        reasons += ["GPIO %s of the scenario has no complete frame of ours on the wire" % line["gpio"]
                    for line in lines if line["gpio"] in pins and not line["facts"]["frames"]]
    elif not any(line["facts"]["frames"] for line in lines):
        reasons.append("no recorded line has a complete frame of ours")
    return judge(name, reasons, [], "the recording is valid and %d line(s) carry complete frames of ours" % sum(
        1 for line in lines if line["facts"]["frames"]))


def wire_proof(name, observer, reasons, notes, shown_detail):
    """A proof read from the lines of the recording: failed on any reason, not shown without a valid recording."""
    if reasons:
        return judge(name, reasons, [], "")
    if observer is None:
        return proof(name, NOT_SHOWN, "there is no wire recording")
    if not observer["valid"]:
        return proof(name, NOT_SHOWN, "the wire recording is not valid")
    return judge(name, [], notes, shown_detail)


def proof_frames(observer):
    reasons = [] if observer is None else [reason for line in observer["lines"] for reason in line.get("frame_reasons", [])]
    count = 0 if observer is None else len(driven_lines(observer))
    return wire_proof("our frames were correct", observer, reasons, [], "no AM32 check failed and no stray activity on %d line(s)" % count)


def proof_spin(observer, device, esc_mock):
    name = "motors spun as specified"
    if esc_mock:
        return proof(name, NOT_SHOWN, "the replies came from the ESC mock, not an ESC: they are not evidence that a motor spun")
    if observer is None:
        return wire_proof(name, observer, [], [], "")
    lines = driven_lines(observer)
    spinning = [line for line in lines if line["mode"] == "bidirectional"
                and (line["facts"]["commanded"] or (line.get("scenario_max_throttle") or 0) >= COMMANDED_MIN)]
    reasons = [reason for line in spinning for reason in line.get("spin_reasons", [])]
    if device["outcome"] is None and not any(line["facts"]["commanded"] for line in lines):
        reasons.append("no line was commanded to spin (the run stopped before the throttle step)")
    if not spinning:
        if reasons:
            return judge(name, reasons, [], "")
        idle = [str(line["gpio"]) for line in lines if line["mode"] == "bidirectional"]
        return proof(name, NOT_APPLICABLE, "no bidirectional motor is commanded to spin" + (
            " (held at throttle 0: GPIO %s)" % ", ".join(idle) if idle else "; the wire alone decided, no scenario names the motors' modes"
            if any(line["mode"] == "unknown" for line in lines) else ""))
    notes = [note for line in spinning for note in line.get("notes", [])]
    thresholds = sum(len(line.get("expect", {})) for line in spinning)
    return wire_proof(name, observer, reasons, notes, "%d bidirectional motor(s) reached a real eRPM, kept reporting it and answering%s" % (
        len(spinning), ", and met the scenario's %d decode threshold(s)" % thresholds if thresholds else ""))


def proof_stopped(observer):
    name = "motors stopped at the end"
    lines = [] if observer is None else [line for line in driven_lines(observer) if line["facts"]["frames"]]
    reasons = ["%s: the last %d frames do not all carry throttle 0 (the last one carries %s): no stop was commanded" % (
        where_of(line), STOP_TAIL_FRAMES, line["facts"].get("last_throttle")) for line in lines if not line["facts"].get("stop_commanded")]
    if observer is not None and observer["valid"] and not reasons and not lines:
        return proof(name, NOT_SHOWN, "no line has frames", notes=["not shown: no line has frames, so a stop could not be seen"])
    return wire_proof(name, observer, reasons, [], "the last %d frames of all %d line(s) carry throttle 0" % (STOP_TAIL_FRAMES, len(lines)))


def limitations(observer, scenario, esc_mock):
    """What a pass does not show, per motor: a unidirectional motor sends no reply, so the wire cannot show that it spun."""
    if observer is not None:
        gpios = [line["gpio"] for line in observer["lines"] if line["mode"] == "unidirectional"
                 and (line["facts"]["commanded"] or (line.get("scenario_max_throttle") or 0) >= COMMANDED_MIN)]
        unknown = [line["gpio"] for line in observer["lines"] if line["mode"] == "unknown" and line["facts"]["commanded"]]
    else:
        gpios = [motor["pin"] for motor in (scenario or {}).get("motors", []) if not motor.get("bidirectional")]
        unknown = []
    out = [UNIDIRECTIONAL_LIMITATION % gpio for gpio in gpios]
    out += ["line on GPIO %s: spin not verified - no scenario says whether it replies" % gpio for gpio in unknown]
    return out


def start_failure_text(failure):
    """The reason of a session that did not start, from session.json's "start_failure" ({"who": "wingman" | "main_pico", "message": ...})."""
    who = {"wingman": "the wingman", "main_pico": "the main Pico"}.get(failure.get("who"), "a board")
    return "%s did not start: %s" % (who, failure.get("message"))


def build_proofs(device, observer, scenario, esc_mock=False, is_script=False, manifest=None):
    """The six required proofs, in order. See the module docstring for what each one needs. A session that did not start fails the first
    and shows none of the others."""
    failure = (manifest or {}).get("start_failure")
    if failure:
        return [judge("run finished cleanly", [start_failure_text(failure)], [], "")] + [
            proof(name, NOT_SHOWN, "the session did not start") for name in (
                "armed within the bound", "wire recording complete", "our frames were correct", "motors spun as specified", "motors stopped at the end")]
    return [proof_finished(device, is_script), proof_armed(device, is_script, esc_mock), proof_recording(observer, scenario, manifest),
            proof_frames(observer), proof_spin(observer, device, esc_mock), proof_stopped(observer)]


def verdict_of(proofs):
    """fail when any proof failed, pass only when every applicable proof is shown, else unknown."""
    statuses = [p["status"] for p in proofs]
    if FAILED in statuses:
        return "fail"
    return "pass" if all(status in (SHOWN, NOT_APPLICABLE) for status in statuses) else "unknown"


def git_commit(root=ROOT):
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def load_scenario(folder, manifest):
    """The scenario dict: the copy the Pico stored in the session, else the file the manifest names."""
    for path in (folder / "scenario.json", ROOT / manifest.get("scenario", "") if manifest.get("scenario") else None):
        if path is not None and path.is_file():
            try:
                return json.loads(path.read_text(encoding="utf-8")), path.name
            except ValueError:
                continue
    return None, None


def summarize(folder):
    folder = Path(folder)
    sources = []
    manifest = {}
    if (folder / "session.json").is_file():
        manifest = json.loads((folder / "session.json").read_text(encoding="utf-8"))
        sources.append("session.json")
    scenario, scenario_source = load_scenario(folder, manifest)
    if scenario_source:
        sources.append(scenario_source)
    start_failure = manifest.get("start_failure") or None       # the session did not start: nothing ran, so no record of a run is read
    device = {"outcome": None, "armed_ms": None, "error": None}
    meta = {} if start_failure else session_files.load_device_meta(folder)
    if meta:
        device["outcome"] = meta.get("outcome")
        sources.append("session.json:device")
    if (folder / "run.log").is_file():
        if not start_failure:
            device["armed_ms"], device["error"] = parse_run_log((folder / "run.log").read_text(encoding="utf-8", errors="replace"))
        sources.append("run.log")
    observer = None
    if not start_failure and (folder / observer_edge_log.OBSERVATION_FILE).is_file():
        observer = observer_summary((folder / observer_edge_log.OBSERVATION_FILE).read_bytes(), scenario, esc_mock=bool(manifest.get("esc_mock")))
        sources.append(observer_edge_log.OBSERVATION_FILE)
    esc_mock = bool(manifest.get("esc_mock"))
    command = manifest.get("command") or []
    is_script = bool(command) and str(command[0]).lower().endswith(".py")      # a device script: the Pico reports no scenario outcome
    proofs = build_proofs(device, observer, scenario, esc_mock, is_script, manifest)
    reasons = [reason for p in proofs for reason in p["reasons"]]
    notes = []
    if esc_mock:
        notes.append("the replies were replayed by the ESC mock, not sent by an ESC: not evidence about an ESC, a motor or arming; "
                     "the checks about the real ESC's own behaviour (first reply after the latch, arming silence) are not applicable")
    if observer is None and not start_failure:
        notes.append("not judged: %s%s, so whether the motors behaved as the scenario specifies was not shown" % (
            missing_recording(manifest), "; the run completed on the Pico" if device["outcome"] == "completed" else ""))
    notes += [note for p in proofs if p["name"] != "wire recording complete" for note in p["notes"]]
    if observer is not None and not esc_mock:
        notes += [note for note in (arming_phase_note(line) for line in observer["lines"] if line["mode"] == "bidirectional") if note]
    return {
        "session": folder.name,
        "computed": {"at": datetime.now().isoformat(timespec="seconds"), "git_commit": git_commit(), "by": "scripts/session_summary.py"},
        "sources": sources,
        "scenario": None if scenario is None else {"dshot_speed": scenario.get("dshot_speed"), "duration_ms": scenario.get("duration_ms"),
                                                    "propellers": propellers(scenario)},
        "device": device,
        "start_failure": start_failure,
        "observer": observer,
        "start": None if observer is None else start_kpi.start_block(observer["lines"], propellers(scenario) if scenario else [], bool(manifest.get("esc_mock"))),
        "verdict": verdict_of(proofs),
        "proofs": [{key: p[key] for key in ("name", "status", "detail")} for p in proofs],
        "reasons": reasons,
        "notes": notes,
        "limitations": limitations(observer, scenario, esc_mock),
    }


def propellers(scenario):
    """What each motor carried, in the scenario's motor order: its propeller entry, 'none' for a bare shaft, or None for a motor with no
    hardware entry or a session recorded before propellers were part of the scenario."""
    return [(motor.get("hardware") or {}).get("propeller") for motor in scenario.get("motors", [])]


def propellers_text(names):
    """'Gemfan 2218 3-Blade on all 4 motors', or the motors listed when they differ."""
    shown = [name if name is not None else "not recorded" for name in names]
    if shown and all(name == shown[0] for name in shown):
        return "%s on all %d motors" % (shown[0], len(shown))
    return ", ".join("motor %d: %s" % (i + 1, name) for i, name in enumerate(shown))


def table(summary):
    out = ["session %s   verdict: %s" % (summary["session"], summary["verdict"].upper())]
    if summary.get("scenario") and summary["scenario"].get("propellers"):
        out.append("propellers: " + propellers_text(summary["scenario"]["propellers"]))
    device = summary["device"]
    out.append("scenario outcome: %s   armed after: %s   sources: %s" % (
        device["outcome"], "%d ms" % device["armed_ms"] if device["armed_ms"] is not None else "-", ", ".join(summary["sources"])))
    observer = summary["observer"]
    if observer is not None:
        out.append("observer: %s" % ("valid" if observer["valid"] else "NOT valid"))
        if summary.get("start"):
            out.append(start_kpi.start_text(summary["start"]))
        out.append("%-10s %-14s %8s %9s %10s %12s %14s %6s %9s %9s  %s" % (
            "line", "mode", "frames", "answered", "real eRPM", "not running", "idle after run", "early", "slow gaps", "spin up", "AM32 checks"))
        for line in observer["lines"]:
            f = line["facts"]
            c = line["checks"]["counts"]
            if line["mode"] == "bidirectional":
                answered = "%.0f%%" % f["answered_pct"] if f["answered_pct"] is not None else "-"
                tail = (answered, f["replies_real_erpm"], f["replies_not_running"], f["not_running_after_first_real_erpm"], f["early_replies"])
            else:
                tail = ("-", "-", "-", "-", "-")
            spin_up = f.get("spin_up_ms")
            spin_up = "-" if line["mode"] != "bidirectional" or not f["commanded"] else ("none" if spin_up is None else "%.0f ms" % spin_up)
            out.append("%-10s %-14s %8d %9s %10s %12s %14s %6s %9d %9s  %s" % (
                "ch%d/GPIO%s" % (line["line"] + 1, line["gpio"]), line["mode"], f["frames"], *[str(x) for x in tail], f["slow_gaps"], spin_up,
                " ".join("%d %s" % (n, s) for s, n in sorted(c.items()) if s != observer_check_am32.NA)))
    if summary.get("proofs"):
        out.append("proofs required for a pass (pass = every applicable one shown):")
        for p in summary["proofs"]:
            out.append("  %-15s %-33s %s" % ("[" + p["status"] + "]", p["name"], p["detail"]))
    for reason in summary["reasons"]:
        out.append("  - " + reason)
    for note in summary.get("notes", []):
        out.append("  note (not judged, not a failure): " + note)
    for item in summary.get("limitations", []):
        out.append("  limitation (a pass does not show this): " + item)
    return "\n".join(out)


def write_summary(folder):
    summary = summarize(folder)
    (Path(folder) / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", help="captures/<session>")
    parser.add_argument("--no-write", action="store_true", help="print only, do not write summary.json")
    args = parser.parse_args()
    summary = summarize(args.folder) if args.no_write else write_summary(args.folder)
    print(table(summary))


if __name__ == "__main__":
    main()
