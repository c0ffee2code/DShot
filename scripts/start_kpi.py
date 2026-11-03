"""
start_kpi.py - the start KPI: its definition in one place, the per-run block that session_summary.py writes into summary.json, and the totals over
many sessions (printed at the end of a batch by run_scenarios.py, or on any folder of sessions from the command line).

The KPI (owner, 2026-11-02; BUG-009 "Current KPI"): all four motors spin up with Gemfan 2218 3-Blade props. RPM disks and heavier or draggier props are
paused.
  start success   the headline (start_ok): a run succeeds when ALL FOUR motors spin up and hold, and the slowest spin-up is within SPIN_UP_LIMIT_MS. A drone with
                  fewer than four motors turning does not fly. The unit is the run (the motors start together, so four motor-starts are one trial). Target 99% of
                  runs. all_spun_up is the same without the time limit, kept beside it to show which of the two a failure is.
  a motor spins up and holds when
                  - it reaches 80% of its own settled speed (the median eRPM 1.5-4 s after the step) and the ESC never cuts it (not-running that stays), and
                  - its settled speed is within PEER_TOLERANCE of the median of the four motors' settled speeds in the same run (same motors, props and throttle), so a
                    motor stuck at a low reading, or on a wrong plateau, is not "spun up" just because it settled there, and
                  - after spin-up at most HOLD_OUTSIDE_MAX_PCT of its replies are outside +-35% of its own settled speed, so a motor that stalls or derails
                    without being cut does not hold.
  spin-up         per motor, ms from the first nonzero throttle frame to 80% of its settled speed; the limit is SPIN_UP_LIMIT_MS.

What counts as a KPI run: a real ESC, four bidirectional motors, the scenario's propellers all KPI_PROPELLER, and a throttle step from 0 to KPI_THROTTLE on all four
lines (so an experiment at another throttle, or a gentle ramp, is outside). A run that never reached the throttle step (it did not arm, or stopped first) is not
silently dropped: it is counted as not_started, apart from the runs, and reported both ways (the owner decides whether it is a failure). A session whose spy
recording lost data is an instrument problem, counted apart and not judged.

Statistics: the lower bound is the exact one-sided 95% bound, the convention of the arming evidence (100 of 100 runs -> 97%, 104 of 104 -> 97%, 300 of 300 -> 99%).
At 26 of 28 runs it can never reach 99%; only failures that stop happening can.

Limits of the measurement (say it wherever the number is quoted): the only measurement is the ESC's own eRPM (no shaft check while the RPM disks are paused); the trials
are back-to-back warm restarts of an already powered ESC, one evening's conditions - cold power-on, idle gaps and several days are not covered (the first run after a
pause armed late and had damaged replies).

  python scripts/start_kpi.py                         # every session in captures/ that has a start block
  python scripts/start_kpi.py --from 2026-11-01_21-29 --to 2026-11-01_21-52   # only sessions from that folder name on, up to that one
  python scripts/start_kpi.py captures/<session> ...    # named sessions

A summary.json written before the start block existed is skipped with a note: re-run scripts/session_summary.py on that folder.
"""

import argparse
import json
import statistics
import sys
from math import comb
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

KPI_PROPELLER = "Gemfan 2218 3-Blade"   # one line to change if the KPI's props change
KPI_MOTORS = 4
KPI_THROTTLE = 215                      # the KPI's step: 0 -> 215 on all four motors (owner, 2026-11-01)
SPIN_UP_LIMIT_MS = 500                  # owner, 2026-11-02, from the 28-run baseline (ordinary slowest 326 ms, two outliers at 895 and 1,052 ms); provisional until
                                        # more runs exist
PEER_TOLERANCE = 0.15                   # a motor's settled speed against the median of the four in its run
HOLD_OUTSIDE_MAX_PCT = 1.0              # replies outside the motor's own +-35% band after spin-up (baseline brief dips are about 0.1%)
CLEAN_RUNS_FOR_99 = 300                 # the evidence rule the arming promise uses (CLAUDE.md, Operational Envelope)


def start_block(lines, propeller_names, esc_mock):
    """The start KPI facts of one run, from the spy's per-line facts. status is in_scope (a KPI run), not_started (the intended KPI run never reached the throttle
    step) or out_of_scope (why: outside_the_scope_because)."""
    commanded = [line for line in lines if line.get("mode") == "bidirectional" and line["facts"].get("commanded")]
    names = [name if name is not None else "not recorded" for name in propeller_names]
    props_ok = len(names) >= KPI_MOTORS and names.count(KPI_PROPELLER) == len(names)
    outside = []
    if esc_mock:
        outside.append("the replies were played by the ESC mock")
    if not props_ok:
        outside.append("the propellers are not all %s (%s)" % (KPI_PROPELLER, ", ".join(sorted(set(names))) or "none recorded"))
    not_started = not outside and len(names) == KPI_MOTORS and len(commanded) < KPI_MOTORS
    if not not_started:
        if len(commanded) != KPI_MOTORS:
            outside.append("%d bidirectional motors were commanded, the KPI is about %d" % (len(commanded), KPI_MOTORS))
        wrong = sorted({(line["facts"].get("step_throttle"), line["facts"].get("max_throttle")) for line in commanded
                        if line["facts"].get("step_throttle") != KPI_THROTTLE or line["facts"].get("max_throttle") != KPI_THROTTLE}, key=str)
        if wrong:
            outside.append("the throttle step is not 0 -> %d on every motor (step, max seen: %s)" % (KPI_THROTTLE, wrong))
    settled = [(line.get("gpio"), line["facts"].get("settled_erpm")) for line in commanded]
    values = [v for _, v in settled if v]
    reference = statistics.median(values) if values else None
    peer_outliers = [g for g, v in settled if v and reference and abs(v - reference) > PEER_TOLERANCE * reference]
    derailed = [line.get("gpio") for line in commanded
                if line["facts"].get("hold_outside_pct") is not None and line["facts"]["hold_outside_pct"] > HOLD_OUTSIDE_MAX_PCT]
    failed = []
    for line in commanded:
        gpio = line.get("gpio")
        if (line["facts"].get("spin_up_ms") is None or line["facts"].get("cut_after_ms") is not None or gpio in peer_outliers or gpio in derailed) and gpio not in failed:
            failed.append(gpio)
    times = [(line["facts"]["spin_up_ms"], line.get("gpio")) for line in commanded if line["facts"].get("spin_up_ms") is not None]
    slowest = max(times) if times else (None, None)
    all_spun_up = len(commanded) == KPI_MOTORS and not failed
    within = None if not all_spun_up else slowest[0] <= SPIN_UP_LIMIT_MS
    status = "not_started" if not_started else ("in_scope" if not outside else "out_of_scope")
    return {"status": status, "in_kpi_scope": status == "in_scope", "outside_the_scope_because": outside, "motors_commanded": len(commanded),
            "all_spun_up": all_spun_up, "not_spun_up_or_cut": failed, "peer_outliers": peer_outliers, "derailed": derailed,
            "slowest_spin_up_ms": slowest[0], "slowest_gpio": slowest[1],
            "spin_up_limit_ms": SPIN_UP_LIMIT_MS, "spin_up_within_limit": within, "start_ok": bool(all_spun_up and within)}


def start_text(start):
    """One line for the printed table."""
    if start["status"] == "not_started":
        return "start: the run never reached the throttle step (%d of %d motors commanded)   [not started: counted apart from the runs]" % (
            start["motors_commanded"], KPI_MOTORS)
    if not start["motors_commanded"]:
        return "start: no motor was commanded to spin   [outside the start KPI: %s]" % "; ".join(start["outside_the_scope_because"])
    if start["all_spun_up"]:
        text = "start: all %d motors spun up and held, slowest spin-up %.0f ms (GPIO %s), %s the %d ms limit" % (
            start["motors_commanded"], start["slowest_spin_up_ms"], start["slowest_gpio"],
            "within" if start["spin_up_within_limit"] else "OVER", start["spin_up_limit_ms"])
    else:
        why = []
        if start["peer_outliers"]:
            why.append("settled speed far from the other motors: GPIO %s" % ", ".join(str(g) for g in start["peer_outliers"]))
        if start["derailed"]:
            why.append("did not hold: GPIO %s" % ", ".join(str(g) for g in start["derailed"]))
        text = "start: NOT all spun up and held - GPIO %s%s" % (", ".join(str(g) for g in start["not_spun_up_or_cut"]), " (%s)" % "; ".join(why) if why else "")
    return text + ("   [counts toward the start KPI]" if start["in_kpi_scope"] else "   [outside the start KPI: %s]" % "; ".join(start["outside_the_scope_because"]))


def run_field(summary):
    """For a batch RUN line: 'ok' (all spun up, within the limit), 'slow' (all spun up, over the limit), 'fail', 'not_started', or 'out_of_scope'; and the slowest spin-up."""
    start = (summary or {}).get("start")
    if not start:
        return "unknown", None
    if start["status"] == "not_started":
        return "not_started", None
    if not start["in_kpi_scope"]:
        return "out_of_scope", start["slowest_spin_up_ms"]
    if not start["all_spun_up"]:
        return "fail", start["slowest_spin_up_ms"]
    return ("ok" if start["spin_up_within_limit"] else "slow"), start["slowest_spin_up_ms"]


def lower_bound(successes, trials, alpha=0.05):
    """The exact one-sided (1 - alpha) lower confidence bound on the success rate (Clopper-Pearson): the p for which seeing this many successes or more has
    probability alpha. For no failures it is alpha ** (1 / trials). None for no trials."""
    if not trials:
        return None
    if successes == 0:
        return 0.0
    low, high = 0.0, 1.0
    for _ in range(60):
        mid = (low + high) / 2
        tail = sum(comb(trials, i) * mid ** i * (1 - mid) ** (trials - i) for i in range(successes, trials + 1))
        if tail >= alpha:
            high = mid
        else:
            low = mid
    return high


def percentile(sorted_values, fraction):
    import math
    return sorted_values[min(len(sorted_values) - 1, int(math.ceil(fraction * len(sorted_values))) - 1)]


def aggregate(summaries):
    """The KPI totals over session summaries (dicts as in summary.json). Only valid, in-scope sessions are runs; the others are counted apart: out_of_scope,
    not_started (the intended KPI run never reached the throttle step), instrument_problem (the spy record lost data) and skipped_no_start_block."""
    skipped_old = [s for s in summaries if not s.get("start")]
    with_start = [s for s in summaries if s.get("start")]
    instrument = [s for s in with_start if s.get("spy") and not s["spy"].get("valid", True)]
    usable = [s for s in with_start if s not in instrument]
    runs = [s for s in usable if s["start"]["status"] == "in_scope"]
    not_started = sum(1 for s in usable if s["start"]["status"] == "not_started")
    out_of_scope = sum(1 for s in usable if s["start"]["status"] == "out_of_scope")
    spun_up = sum(1 for s in runs if s["start"]["all_spun_up"])
    start_ok = sum(1 for s in runs if s["start"]["start_ok"])
    per_gpio = {}
    for s in runs:
        for line in s["spy"]["lines"]:
            if line.get("mode") != "bidirectional" or not line["facts"].get("commanded"):
                continue
            cell = per_gpio.setdefault(line["gpio"], {"starts": 0, "not_spun_up": 0, "times": []})
            cell["starts"] += 1
            if line["gpio"] in s["start"]["not_spun_up_or_cut"]:
                cell["not_spun_up"] += 1
            if line["facts"].get("spin_up_ms") is not None:
                cell["times"].append(line["facts"]["spin_up_ms"])
    gpio = {}
    for g, cell in sorted(per_gpio.items()):
        times = sorted(cell["times"])
        gpio[g] = {"starts": cell["starts"], "not_spun_up": cell["not_spun_up"],
                   "median_ms": statistics.median(times) if times else None, "p90_ms": percentile(times, 0.9) if times else None,
                   "max_ms": times[-1] if times else None, "over_limit": sum(1 for t in times if t > SPIN_UP_LIMIT_MS)}

    def pct(k, n):
        return None if not n else round(100.0 * k / n, 1)

    def low(k, n):
        return None if not n else round(100.0 * lower_bound(k, n), 1)
    n = len(runs)
    return {"sessions": len(summaries), "runs": n, "out_of_scope": out_of_scope, "not_started": not_started, "instrument_problem": len(instrument),
            "skipped_no_start_block": len(skipped_old),
            "all_spun_up": spun_up, "all_spun_up_pct": pct(spun_up, n), "all_spun_up_lower95_pct": low(spun_up, n),
            "start_ok": start_ok, "start_ok_pct": pct(start_ok, n), "start_ok_lower95_pct": low(start_ok, n),
            "start_ok_if_not_started_fail_pct": pct(start_ok, n + not_started), "start_ok_if_not_started_fail_lower95_pct": low(start_ok, n + not_started),
            "runs_with_a_start_over_the_limit": sum(1 for s in runs if s["start"]["all_spun_up"] and not s["start"]["spin_up_within_limit"]),
            "spin_up_limit_ms": SPIN_UP_LIMIT_MS, "clean_runs_for_99": CLEAN_RUNS_FOR_99, "per_gpio": gpio}


def format_lines(agg):
    """key=value lines (the style of run_scenarios.py's RESULT lines) for the totals."""
    def val(v):
        return "-" if v is None else ("%g" % v if isinstance(v, float) else str(v))
    lines = ["START_KPI runs=%d start_ok=%d start_ok_pct=%s start_ok_lower95_pct=%s all_spun_up=%d all_spun_up_pct=%s all_spun_up_lower95_pct=%s "
             "runs_over_spin_up_limit=%d spin_up_limit_ms=%d not_started=%d start_ok_if_not_started_fail_pct=%s start_ok_if_not_started_fail_lower95_pct=%s "
             "out_of_scope=%d instrument_problem=%d skipped_no_start_block=%d clean_runs_for_99=%d" % (
                 agg["runs"], agg["start_ok"], val(agg["start_ok_pct"]), val(agg["start_ok_lower95_pct"]), agg["all_spun_up"], val(agg["all_spun_up_pct"]),
                 val(agg["all_spun_up_lower95_pct"]), agg["runs_with_a_start_over_the_limit"], agg["spin_up_limit_ms"], agg["not_started"],
                 val(agg["start_ok_if_not_started_fail_pct"]), val(agg["start_ok_if_not_started_fail_lower95_pct"]), agg["out_of_scope"],
                 agg["instrument_problem"], agg["skipped_no_start_block"], agg["clean_runs_for_99"])]
    for g, c in agg["per_gpio"].items():
        lines.append("SPIN_UP gpio=%s starts=%d not_spun_up=%d median_ms=%s p90_ms=%s max_ms=%s over_limit=%d" % (
            g, c["starts"], c["not_spun_up"], val(None if c["median_ms"] is None else round(c["median_ms"])),
            val(None if c["p90_ms"] is None else round(c["p90_ms"])), val(None if c["max_ms"] is None else round(c["max_ms"])), c["over_limit"]))
    return lines


def load_summaries(folders):
    out = []
    for folder in folders:
        path = Path(folder) / "summary.json"
        if path.is_file():
            out.append(json.loads(path.read_text(encoding="utf-8")))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="*", help="session folders (default: every folder in captures/)")
    parser.add_argument("--from", dest="start_from", default=None, help="only sessions whose folder name is this or later (e.g. 2026-11-01_21-29)")
    parser.add_argument("--to", dest="start_to", default=None, help="only sessions whose folder name is this or earlier (a prefix works: 2026-11-01_21-52)")
    args = parser.parse_args()
    folders = [Path(s) for s in args.sessions] if args.sessions else sorted(p for p in (ROOT / "captures").glob("*") if p.is_dir())
    if args.start_from:
        folders = [f for f in folders if f.name >= args.start_from]
    if args.start_to:
        folders = [f for f in folders if f.name[:len(args.start_to)] <= args.start_to]
    for line in format_lines(aggregate(load_summaries(folders))):
        print(line)


if __name__ == "__main__":
    main()
