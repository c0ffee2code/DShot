"""
arming_stats.py - how often each bidirectional ESC accepted our frames while
arming, per scenario configuration, across a batch of pulled sessions. BUG-002's
bisection runs are judged by these numbers.

Run from the project root:
  python scripts/arming_stats.py captures/2026-09-28_*          # these sessions
  python scripts/arming_stats.py --since 2026-09-28_10-00-00    # every session from then on
  add -v to list each session's listening periods as well

A listening period starts when an ESC can first hear us: at arm() if it was
already running, or where a startup tune (the line held low for ~600 ms) ends.
It ends at the ESC's first reply - accepted, 60-100 ms in, its bidirectional
latch - or at its next reset - rejected, ~1.86 s in, after it armed on frames it
never validated and timed out (BUG-002's "What actually happens"). A period the
log ends inside counts as undecided. Periods are counted by how they started:

  first contact  the ESC was already running at arm() and met our first frame
  after tune     the ESC was mid-startup-tune at arm(), so it booted into our frames
  after reset    the ESC reset while arming and booted into our frames again

Sessions are grouped by configuration (DShot speed, the bidirectional motors'
pins and state machines, arming_frame_gap_us, core1_interval_us,
arming_class_bin_width_us), read from each session's own scenario.json. A session where a bidirectional ESC never replied
and never held the line low is reported and left out: its ESC was not powered.
"""

import argparse
import json
import sys
from collections import OrderedDict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import classify_reply_timeline as timeline_tools  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# A first reply later than this after a period starts is not the ESC's
# bidirectional latch (60-100 ms measured), so the period is not "accepted"
LATCH_MAX_MS = 400

KINDS = ("first contact", "after tune", "after reset")


def config_label(session_dir, meta):
    scenario = json.loads((session_dir / "scenario.json").read_text())
    speed = scenario.get("dshot_speed", "DSHOT%d" % (int(meta["dshot_speed"]) // 8000))
    bidir = [m for m in scenario["motors"] if m.get("bidirectional")]
    label = "%s, bidirectional pins %s on SM %s" % (
        speed, "/".join(str(m["pin"]) for m in bidir), "/".join(str(m["sm_id"]) for m in bidir))
    label += ", frame gap %d us" % scenario.get("arming_frame_gap_us", 0)
    if scenario.get("core1_interval_us") is not None:
        label += ", core1_interval_us %d" % scenario["core1_interval_us"]
    label += ", class bins %d us" % scenario.get("arming_class_bin_width_us", 100000)
    return label


def listening_periods(events):
    """(kind, outcome, start_ms, detail) for each listening period in one motor's arming log."""
    segs = timeline_tools.segments(events, 100.0)
    if not segs:
        return []
    boots = [e for e in timeline_tools.am32_events(segs, events) if e[0] == "boot"]
    if segs[0][2] == "low":
        starts = [(timeline_tools.span(segs[0], events)[1], "after tune")]
    else:
        starts = [(0.0, "first contact")]
    starts += [(boot[2], "after reset") for boot in boots if boot[1] > 50]

    periods = []
    for start, kind in starts:
        next_boot = next((b[1] for b in boots if b[1] > start + 50), None)
        reply = timeline_tools.first_reply_after(start, events)
        if (reply is not None and reply - start < LATCH_MAX_MS
                and (next_boot is None or reply < next_boot)):
            periods.append((kind, "accepted", start, reply - start))
        elif next_boot is not None and (reply is None or reply > next_boot):
            periods.append((kind, "rejected", start, next_boot - start))
        else:
            periods.append((kind, "undecided", start, None))
    return periods


def session_result(session_dir):
    """None if the session has nothing to count, else a dict describing it."""
    if not (session_dir / "arming.bin").exists() or not (session_dir / "scenario.json").exists():
        return None
    meta = timeline_tools.load_meta(session_dir)
    bidir = [int(s) for s in meta.get("bidir_motor_indices", "").split(",") if s]
    arming = timeline_tools.load_records(session_dir, "arming.bin")
    if not bidir or not arming:
        return None
    t0_us = int(meta["arm_ticks_us"]) if "arm_ticks_us" in meta else arming[0][0]
    echo_model = timeline_tools.EchoModel(int(meta["dshot_speed"]))

    armed_ms = None
    if "arm_ticks_us" in meta and "armed_ticks_us" in meta:
        armed_ms = ((int(meta["armed_ticks_us"]) - int(meta["arm_ticks_us"])) & 0x3FFFFFFF) / 1000

    motors = {}
    unpowered = []
    for index in bidir:
        events = timeline_tools.timeline(arming, index, echo_model, t0_us)
        if not any(label in ("stop", "spin", "low") for _, label, _ in events):
            unpowered.append(index)
        motors[index] = listening_periods(events)
    return {
        "name": session_dir.name,
        "config": config_label(session_dir, meta),
        "armed_ms": armed_ms,
        "refused": "armed_ticks_us" not in meta,
        "motors": motors,
        "unpowered": unpowered,
    }


def describe_period(period):
    kind, outcome, start, detail = period
    text = "%.2f s %s: %s" % (start / 1000, kind, outcome)
    if outcome == "accepted":
        text += " (reply +%d ms)" % detail
    elif outcome == "rejected":
        text += " (reset +%.3f s)" % (detail / 1000)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("sessions", nargs="*", help="captures/<session> directories")
    parser.add_argument("--since", help="every session in captures/ named this or later, "
                                        "e.g. 2026-09-28_10-00-00")
    parser.add_argument("-v", "--verbose", action="store_true", help="list each session's periods")
    args = parser.parse_args()

    dirs = [Path(s) for s in args.sessions]
    if args.since:
        dirs += [d for d in sorted((ROOT / "captures").iterdir())
                 if d.is_dir() and d.name >= args.since]
    if not dirs:
        parser.error("name sessions, or pass --since")

    groups = OrderedDict()
    for session_dir in sorted(set(dirs), key=lambda d: d.name):
        result = session_result(session_dir)
        if result is None:
            continue
        if result["unpowered"]:
            print("%s: left out - motor(s) %s never replied nor held the line low (ESC unpowered?)"
                  % (result["name"], result["unpowered"]))
            continue
        groups.setdefault(result["config"], []).append(result)
        if args.verbose:
            armed = ("ARMED %.2f s" % (result["armed_ms"] / 1000) if result["armed_ms"] is not None
                     else "refused" if result["refused"] else "ARMED time unknown")
            print("%s  %s  [%s]" % (result["name"], armed, result["config"]))
            for index, periods in result["motors"].items():
                print("    motor %d: %s" % (index, " | ".join(describe_period(p) for p in periods)))
    if args.verbose and groups:
        print()

    for config, results in groups.items():
        armed_times = sorted(r["armed_ms"] / 1000 for r in results if r["armed_ms"] is not None)
        refused = sum(1 for r in results if r["refused"])
        print("%s: %d runs" % (config, len(results)))
        line = "  armed %d, refused %d" % (len(results) - refused, refused)
        if armed_times:
            line += ", ARMED after arm(): median %.2f s (%.2f-%.2f)" % (
                armed_times[len(armed_times) // 2], armed_times[0], armed_times[-1])
        print(line)
        for kind in KINDS:
            outcomes = [p[1] for r in results for periods in r["motors"].values()
                        for p in periods if p[0] == kind]
            accepted = outcomes.count("accepted")
            decided = accepted + outcomes.count("rejected")
            undecided = outcomes.count("undecided")
            if not outcomes:
                continue
            share = " (%3.0f%%)" % (100.0 * accepted / decided) if decided else ""
            extra = ", %d undecided" % undecided if undecided else ""
            print("  %-14s accepted %2d of %2d%s%s" % (kind, accepted, decided, share, extra))
        print()


if __name__ == "__main__":
    main()
