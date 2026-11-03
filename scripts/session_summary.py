"""
session_summary.py - one computed summary of a finished session, written as summary.json in the session folder.

  python scripts/session_summary.py captures/<session>          # compute, write summary.json, print the table
  python scripts/session_summary.py captures/<session> --no-write

scripts/run_session.py runs this at the end of every run. It can be re-run on any session folder: the summary is
computed from the files in it, never stored anywhere else, so when the analysis code improves, re-running refreshes it.

What goes in (each source is optional; what is missing is left out and named under "sources"):
  - copied from the Pico, not recomputed: the scenario's own outcome (the "device" section of session.json), the arming time and the error text
    of a failed run (run.log);
  - computed from spy.bin, per line: the line's mode (the scenario says which motors are bidirectional), frames,
    how many were answered, replies with a bad CRC, replies reporting a real eRPM, replies reporting "not running" (and how many
    of those came after the line had reported a real eRPM), replies that started within 10 us of our frame, the intervals of 40-400 us between edges ("slow gaps": not
    frames, not replies), and the verdicts of tests/bench/wingman/tools/spy_check_am32.py.
A run without a scenario outcome from the Pico (a device script run through run_session.py) is judged from the wire alone: it needs at
least one commanded line and the same conditions below. The verdict is "pass" only when the scenario completed (or,
without an outcome, when the wire record is valid), the spy lost no data, no AM32 check failed on any line
and every commanded bidirectional line reached a real eRPM and kept reporting one (LOST_SPIN_FRAMES); "reasons" lists what made it fail.
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
import spy_check_am32
import spy_edge_log
import spy_start_summary
import start_kpi
import spy_stats

ROOT = Path(__file__).resolve().parents[1]
COMMANDED_MIN = 48                  # throttle values from here up make a motor spin (1-47 are commands)
EARLY_REPLY_US = 10                 # a normal reply starts about 23 us after our frame
SLOW_GAP_US = (40, 400)             # no frame, reply or turnaround has an interval between edges in this range
SLOW_GAPS_FLAGGED = 100             # a line with this many such intervals is flagged; my choice, not measured
LOST_SPIN_FRAMES = 50               # 'not running' replies this many frames after a real eRPM (about 50 ms) flag a line; my choice, not measured
MIN_JUDGED_REPLIES = 20             # fewer replies than this cannot support a percentage or a median: reported as not judged, never failed
NOT_JUDGED = "(not judged: "        # how the Pico words a verdict that skipped a threshold for want of samples
RUN_LOG_ERROR = re.compile(r"^(?:\s*\|\s*)?(\w*Error: .*)$")
ARMED_AFTER = re.compile(r"Armed after (\d+)ms")


def parse_run_log(text):
    """(arming time in ms or None, the last error line of the run or None) from the console log."""
    armed = ARMED_AFTER.search(text)
    errors = [m.group(1) for m in (RUN_LOG_ERROR.match(line) for line in text.splitlines()) if m]
    return (int(armed.group(1)) if armed else None), (errors[-1] if errors else None)


def motors_by_pin(scenario):
    """{GPIO pin: motor dict} of a scenario JSON."""
    return {motor["pin"]: motor for motor in scenario.get("motors", [])}


def pins_of_stream(start_text):
    """The spy's start text 'pins=6,7,8,9 ...' -> {line index: GPIO}."""
    for token in start_text.split():
        if token.startswith("pins="):
            return dict(enumerate(int(p) for p in token[5:].split(",")))
    return {}


def line_facts(records, bidirectional):
    """The numbers about one line, from spy_edge_log.analyse() records."""
    rows = spy_stats.frame_table(records)
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
        "early_replies": sum(1 for row in answered if row["delay_us"] is not None and row["delay_us"] < EARLY_REPLY_US),
        "first_real_erpm_ms": None if first_run is None else round(first_run, 1),
        "median_erpm": round(statistics.median(erpms)) if erpms else None,
    })
    # spin-up: ms from the first nonzero throttle frame to 80% of the motor's own settled speed (reported, not judged: no limit is set yet)
    step = spy_start_summary.find_step(rows) if facts["commanded"] else None
    spin_up_ms, settled_erpm = spy_start_summary.spin_up(rows, step) if step is not None else (None, None)
    cut_after_ms = spy_start_summary.final_cut_after_ms(rows, step) if step is not None else None
    hold_outside = spy_start_summary.hold_outside_pct(rows, step, spin_up_ms, settled_erpm) if step is not None else None
    facts.update({"spin_up_ms": None if spin_up_ms is None else round(spin_up_ms, 1), "settled_erpm": settled_erpm,
                  "cut_after_ms": None if cut_after_ms is None else round(cut_after_ms, 1),
                  "step_throttle": rows[step]["throttle"] if step is not None else None,
                  "hold_outside_pct": None if hold_outside is None else round(hold_outside, 2)})
    return facts


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
            counts[spy_check_am32.NA] = counts.get(spy_check_am32.NA, 0) + 1
            continue
        counts[result.status] = counts.get(result.status, 0) + 1
        if result.status in (spy_check_am32.FAIL, spy_check_am32.WARN):
            notable.append({"status": result.status, "check": result.name, "detail": result.detail})
    return {"counts": counts, "notable": notable}


def line_reasons(line):
    reasons = []
    for item in line["checks"]["notable"]:
        if item["status"] == spy_check_am32.FAIL:
            reasons.append("line %d (channel %d): AM32 check failed: %s" % (line["line"], line["line"] + 1, item["check"]))
    facts = line["facts"]
    if line["mode"] == "bidirectional" and facts["commanded"] and not facts.get("replies_real_erpm"):
        reasons.append("line %d (channel %d): commanded to spin but never reported a real eRPM" % (line["line"], line["line"] + 1))
    if facts.get("slow_gaps", 0) >= SLOW_GAPS_FLAGGED:
        reasons.append("line %d (channel %d): %d intervals of 40-400 us between edges - activity that is neither our frames nor replies" % (
            line["line"], line["line"] + 1, facts["slow_gaps"]))
    lost = facts.get("not_running_after_first_real_erpm")
    if line["mode"] == "bidirectional" and lost is not None and lost >= LOST_SPIN_FRAMES:
        reasons.append("line %d (channel %d): reported 'not running' for %d frames after it had reported a real eRPM, while commanded to spin" % (
            line["line"], line["line"] + 1, lost))
    return reasons


def expect_for(scenario, key, index):
    """The scenario's `expect` threshold of that name for the motor at that list position, or None."""
    return ((scenario or {}).get("expect") or {}).get(key, {}).get(str(index))


def judge_expect(line, min_crc_valid_pct, min_median_erpm):
    """(reasons, notes): the scenario's decode thresholds judged on every reply the spy saw, for a commanded
    bidirectional line. Too few replies to judge a threshold is a note, never a failure."""
    reasons, notes = [], []
    facts = line["facts"]
    if line["mode"] != "bidirectional" or not facts["commanded"]:
        return reasons, notes
    where = "line %d (channel %d)" % (line["line"], line["line"] + 1)
    if min_crc_valid_pct is not None:
        answered = facts["answered"]
        if answered < MIN_JUDGED_REPLIES:
            notes.append("%s: only %d replies on the wire, need %d to judge min_crc_valid_pct" % (where, answered, MIN_JUDGED_REPLIES))
        else:
            valid = 100.0 * (answered - facts["replies_bad_crc"]) / answered
            if valid < min_crc_valid_pct:
                reasons.append("%s: %.1f%% of replies on the wire had a valid CRC, below min_crc_valid_pct=%s" % (where, valid, min_crc_valid_pct))
    if min_median_erpm is not None:
        real = facts["replies_real_erpm"]
        if real < MIN_JUDGED_REPLIES:
            if real:       # none at all is already a reason of its own (never spun)
                notes.append("%s: only %d real-eRPM replies on the wire, need %d to judge min_median_erpm" % (where, real, MIN_JUDGED_REPLIES))
        elif facts["median_erpm"] < min_median_erpm:
            reasons.append("%s: median eRPM on the wire %s, below min_median_erpm=%s" % (where, facts["median_erpm"], min_median_erpm))
    return reasons, notes


def spy_summary(spy_bytes, scenario, from_first_frame=True, esc_mock=False):
    """The wire part of the summary from the bytes of spy.bin and the scenario dict (None when unknown)."""
    start, channels, end = spy_edge_log.parse_stream(spy_bytes)
    info = spy_edge_log.parse_end(end)
    problems = spy_edge_log.reconcile(channels, info)
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
        edge_list = spy_edge_log.edges(channels[index], invert=unidirectional)
        records = spy_edge_log.analyse(edge_list, unidirectional)
        facts = line_facts(records, mode == "bidirectional")
        facts["slow_gaps"] = slow_gaps(edge_list)
        if mode == "unknown" and not facts["frames"]:
            mode = "not driven"         # a line the scenario does not use and nothing was sent on: nothing to judge
            checks = check_summary([])
        else:
            checks = check_summary(spy_check_am32.run_checks(records, unidirectional, from_first_frame), esc_mock)
        line = {"line": index, "gpio": pin, "mode": mode, "facts": facts, "checks": checks}
        line["reasons"] = line_reasons(line)
        line["notes"] = []
        if pin in index_of_pin:
            more, line["notes"] = judge_expect(line, expect_for(scenario, "min_crc_valid_pct", index_of_pin[pin]),
                                               expect_for(scenario, "min_median_erpm", index_of_pin[pin]))
            line["reasons"] += more
        lines.append(line)
    return {"valid": bool(info["valid"]) and not problems, "stream_problems": problems, "recording": start,
            "spy_end": end, "lines": lines}


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
    device = {"outcome": None, "armed_ms": None, "error": None, "verdict": None}
    meta = session_files.load_device_meta(folder)
    if meta:
        device["outcome"] = meta.get("outcome")
        device["verdict"] = meta.get("verdict")
        sources.append("session.json:device")
    if (folder / "run.log").is_file():
        device["armed_ms"], device["error"] = parse_run_log((folder / "run.log").read_text(encoding="utf-8", errors="replace"))
        sources.append("run.log")
    spy = None
    if (folder / "spy.bin").is_file():
        spy = spy_summary((folder / "spy.bin").read_bytes(), scenario, esc_mock=bool(manifest.get("esc_mock")))
        sources.append("spy.bin")
    reasons = []
    notes = []
    if manifest.get("esc_mock"):
        notes.append("the replies were replayed by the ESC mock, not sent by an ESC: not evidence about an ESC, a motor or arming; "
                     "the checks about the real ESC's own behaviour (first reply after the latch, arming silence) are not applicable")
    if device["outcome"] == "completed" and NOT_JUDGED in (device["verdict"] or ""):
        notes.append("the Pico did not judge the decode thresholds: " + device["verdict"].split(NOT_JUDGED, 1)[1].rstrip(")"))
    if device["outcome"] is not None and device["outcome"] != "completed":
        reasons.append("the scenario did not complete (outcome: %s%s)" % (
            device["outcome"], "; " + device["error"] if device["error"] else ""))
    if device["outcome"] == "completed" and (device["verdict"] or "").startswith("fail"):
        reasons.append("the run completed but missed the scenario's own expectations on the Pico (%s)" % device["verdict"])
    if spy is not None:
        if not spy["valid"]:
            reasons.append("the spy lost data or the byte counts disagree: the wire record cannot be trusted")
        for line in spy["lines"]:
            reasons += line["reasons"]
            notes += line.get("notes", [])
        if device["outcome"] is None and not any(line["facts"]["commanded"] for line in spy["lines"]):
            reasons.append("no line was commanded to spin (the run stopped before the throttle step)")
    return {
        "session": folder.name,
        "computed": {"at": datetime.now().isoformat(timespec="seconds"), "git_commit": git_commit(), "by": "scripts/session_summary.py"},
        "sources": sources,
        "scenario": None if scenario is None else {"dshot_speed": scenario.get("dshot_speed"), "duration_ms": scenario.get("duration_ms"),
                                                    "propellers": propellers(scenario)},
        "device": device,
        "spy": spy,
        "start": None if spy is None else start_kpi.start_block(spy["lines"], propellers(scenario) if scenario else [], bool(manifest.get("esc_mock"))),
        "verdict": "fail" if reasons else ("pass" if device["outcome"] == "completed" or (
            device["outcome"] is None and spy is not None and spy["valid"]) else "unknown"),
        "reasons": reasons,
        "notes": notes,
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
    spy = summary["spy"]
    if spy is not None:
        out.append("spy: %s" % ("valid" if spy["valid"] else "NOT valid"))
        if summary.get("start"):
            out.append(start_kpi.start_text(summary["start"]))
        out.append("%-10s %-14s %8s %9s %10s %12s %14s %6s %9s %9s  %s" % (
            "line", "mode", "frames", "answered", "real eRPM", "not running", "idle after run", "early", "slow gaps", "spin up", "AM32 checks"))
        for line in spy["lines"]:
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
                " ".join("%d %s" % (n, s) for s, n in sorted(c.items()) if s != spy_check_am32.NA)))
    for reason in summary["reasons"]:
        out.append("  - " + reason)
    for note in summary.get("notes", []):
        out.append("  note (not judged, not a failure): " + note)
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
