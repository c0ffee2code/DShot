"""
run_scenarios.py - run one or more scenarios repeatedly on the bench,
tallying outcomes.

Run from project root:
  python scripts/run_scenarios.py --scenario tests/bench/dshot_protocol/scenarios/four_channel_bidirectional_600.json --rounds 10
  python scripts/run_scenarios.py --scenario A.json --scenario B.json --scenario C.json --rounds 8
      # interleaves A, B, C, in that order, for 8 rounds each (24 runs total),
      # so anything that drifts over the session lands on every configuration
      # alike rather than piling onto whichever runs first.
  --scenario also accepts a bare device script (under tests/bench/, a .py
      path), run directly by name via run_session.py instead of as --scenario
      JSON - for interleaving a diagnostic tool against a real scenario under
      the exact same deploy/reset path (see run_once()'s own docstring)
  every attempt is a full session (scripts/run_session.py): its own captures/<session>/ folder with the
      spy's recording and a summary.json, pulled from the SD card as the attempt ends; the RUN line
      names it (session=) and carries its verdict (session_verdict=). Use scripts/session_summary.py on any of
      them to see its table.
  add --rotate to cycle the --scenario order by one position each round
      (round 1: A,B,C; round 2: B,C,A; round 3: C,A,B; ...) so no scenario is
      always first or always last - without it, whichever comes last in a
      fixed order always runs right after the same neighbor and after the
      same elapsed time since the batch started, which is itself a variable
      (bug-reports/BUG-002-...md's hypothesis 4, ESC-state carryover)

Classification, per attempt, is read from run_test.py's own printed output (the run_session.py returns it) -
not its exit code. An on-device refusal (arm_group() raises inside
run_scenario.py) and a genuine deploy failure both make run_test.py exit
non-zero, so exit code alone can't tell them apart, and only one of those is
safe to retry:

  deploy_failed  the "Uploaded" line shows any failed upload, or is missing
                 entirely - retried once automatically (clears a transient
                 Pico USB re-enumeration race). A second failure in a row is
                 reported, not retried again or silently folded into any
                 other count.
  clean          "Outcome: completed" was printed, and "Armed after Nms."
                 shows N below CLEAN_ARM_THRESHOLD_MS. This is BUG-002's
                 stable-arming KPI (target: in >=99% of attempts, every
                 configured motor reaches ARMED within CLEAN_ARM_THRESHOLD_MS
                 of arm()) - the duration bound, not "did an ESC reboot cycle
                 happen," is the actual requirement: a caller doesn't know or
                 care about AM32's internal reboot mechanism, only how long
                 they waited
                 - a design principle for the driver's own public contract,
                 not just a bench convenience: arm() tries to arm within a
                 bounded time, and a caller that doesn't see it succeed
                 promptly should treat that as a failure to handle, not keep
                 waiting indefinitely for an eventual, possibly much later,
                 success.
  reboot         "Outcome: completed" was printed, but N was at or above
                 CLEAN_ARM_THRESHOLD_MS - it took at least one ESC reboot
                 cycle to arm, which is also why it missed the KPI's duration
                 bound. Still "armed" in the old binary sense, but a KPI
                 failure.
  refused        "arming did not complete within" was printed (the arming
                 timeout's own message) - never armed at all within the
                 scenario's own (generous, research-mode) timeout.
  error          deploy succeeded but none of the above appeared - an
                 unexpected crash. Always reported with its full output,
                 never silently dropped into another bucket.

Output is line-oriented key=value, one fact per field, so a long batch can be
filtered with grep or loaded into a table without parsing prose:
  RUN n= of= scenario= verdict=pass|fail|excluded outcome= armed_ms= margin_ms=
      session= pass_so_far= fail_so_far= schedule_behind= session_verdict=pass|fail|unknown
      start=ok|slow|fail|out_of_scope|unknown spin_up_max_ms=      (the start KPI of that run, scripts/start_kpi.py)
  RESULT scenario= attempts= pass= fail= pass_pct= fail_late= fail_refused=
      error= deploy_failed=
  ARMING_TIME scenario= successful= min_ms= median_ms= max_ms= limit_ms=
      marginal_band_ms= marginal=        (arms that passed only)
  SCHEDULE scenario= attempts= behind_total= behind_max=
  START_KPI runs= all_spun_up= all_spun_up_pct= all_spun_up_lower95_pct= start_ok= start_ok_pct= start_ok_lower95_pct= runs_over_spin_up_limit=
      spin_up_limit_ms= out_of_scope= skipped_no_start_block= clean_runs_for_99=      (over the sessions of this batch; the run is the unit)
  SPIN_UP gpio= starts= not_spun_up= median_ms= p90_ms= max_ms= over_limit=      (one line per motor)
pass = armed inside the bound; fail = armed late (an ESC reboot) or never armed;
error and deploy_failed are excluded from the verdict. A field whose value is
not known is left out of its line.

The series summary also reports how often Core1Runner's deadline schedule
fell behind (update() cost exceeded UPDATE_INTERVAL_US for at least one tick),
parsed from run_scenario.py's own WARNING line - total, average per attempt,
and the worst single attempt. Every run is paced (an unpaced loop was
removed 2026-10-19), so the count is always meaningful. Excludes
deploy-failed attempts, which never reached the device.
"""

import argparse
import json
import re
import sys
from pathlib import Path

# When stdout isn't a real terminal (piped, redirected), Python fully
# buffers it - our own prints would otherwise appear out of order against
# child processes' directly-inherited output -
# see deploy.py's own copy of this line for the same reason.
sys.stdout.reconfigure(line_buffering=True)

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_session  # noqa: E402
import start_kpi  # noqa: E402

# A run that never got past deploy has no "failed 0" - matched separately
# from a bare "Uploaded" line so a partial deploy (some files failed) is
# still caught, not mistaken for success because the word appeared.
DEPLOY_OK_RE = re.compile(r"Uploaded \d+, failed 0\.")
REFUSED_RE = re.compile(r"arming did not complete within")
ARMED_AFTER_RE = re.compile(r"Armed after (\d+)ms\.")
# run_scenario.py's own WARNING line (printed from its `finally`, so it's
# there on clean/reboot/refused alike - anything that got far enough to
# start Core1Runner) - see Core1Runner.schedule_behind_count's own docstring
# for what "fell behind" means: update() cost exceeded UPDATE_INTERVAL_US for
# at least that many ticks. Absent entirely means 0, not unknown - the line
# only prints when the count is nonzero.
SCHEDULE_BEHIND_RE = re.compile(r"fell behind (\d+) time\(s\)")

# This IS the KPI's duration bound (BUG-002's stable-arming KPI: in >=99% of
# attempts, every configured motor reaches ARMED within this many ms of
# arm() - not a classification convenience). It also happens to sit in the
# gap between the "no reboot" duration cluster (~2.1-2.6s) and the "one
# reboot" cluster (~4.6s, one ~2.46s reboot cycle added) - reconfirmed
# archive-wide 2026-10-04 (698 sessions): clean arms 2078-2593ms, reboot arms
# >=4057ms, a clean ~1.4s gap with zero occupancy either side. Recheck this
# constant if a session is ever seen landing between the clusters, or if
# MotorGroup's own arm_duration_ms/READY_SPAN_MS defaults change (this number
# is derived from those, not independent of them). Set to 3000 on 2026-10-13 when
# the 3.0 s promise was published in README/CLAUDE.md (it was 3300 before);
# 3000 still sits inside the clean/reboot gap above, so no archived session
# changes category.
CLEAN_ARM_THRESHOLD_MS = 3000

# A clean arm is "marginal" when it landed within this many ms of the bound:
# still a pass, but it says the promise holds with little room. Reported next
# to the clean count so a batch of 300 arms at ~2.0 s is not read the same way
# as 300 arms at ~2.9 s.
MARGINAL_BAND_MS = 500


def arming_time_summary(armed_ms, bound_ms=None, band_ms=None):
    """
    How much room the arms in a batch had, from each armed run's "Armed
    after N ms" figure (clean and late alike).

    Returns None for an empty list, else a dict: n, min, median, max (ms), and
    marginal - how many arms finished under the bound but within band_ms of
    it. Meant for the arms that passed; a late arm (at or over the bound) is a
    failure and is counted by the pass/fail tally instead.
    """
    if bound_ms is None:
        bound_ms = CLEAN_ARM_THRESHOLD_MS
    if band_ms is None:
        band_ms = MARGINAL_BAND_MS
    if not armed_ms:
        return None
    ordered = sorted(armed_ms)
    mid = len(ordered) // 2
    median = ordered[mid] if len(ordered) % 2 else (ordered[mid - 1] + ordered[mid]) / 2
    marginal = sum(1 for t in ordered if bound_ms - band_ms <= t < bound_ms)
    return {"n": len(ordered), "min": ordered[0], "median": median,
            "max": ordered[-1], "marginal": marginal}


# One retry has cleared every deploy failure seen so far (a transient USB
# re-enumeration race after the previous run's reset) - not a tunable meant
# to paper over a real, repeated hardware problem.
DEPLOY_RETRIES = 1

# A batch that hits this many errors or deploy failures in a row stops: the bench is not in a state worth burning
# more attempts on, and the output of the last attempts is printed above.
MAX_CONSECUTIVE_PROBLEMS = 3


class StopBatch(Exception):
    pass


def fields(label, **pairs):
    """
    One machine-readable line: LABEL key=value key=value ... Keys keep the
    order given, values never contain spaces, and a value that is not known
    (None) is left out rather than printed as a placeholder, so a line can be
    split on whitespace and then on "=" with no further parsing, and a batch
    of hundreds of runs can be filtered with grep or loaded into a table.
    """
    parts = [label]
    for key, value in pairs.items():
        if value is not None:
            parts.append("%s=%s" % (key, value))
    return " ".join(parts)


def run_record(index, total, scenario_name, verdict, outcome, armed_ms, session,
               passes_so_far, fails_so_far, behind, session_verdict=None, start=None, spin_up_max_ms=None):
    """
    One line per attempt. verdict is pass / fail / excluded (see main());
    outcome is the finer classification (clean, reboot, refused, error,
    deploy_failed). margin_ms is the room left under the bound, for a pass.
    """
    margin_ms = None
    if verdict == "pass" and armed_ms is not None:
        margin_ms = CLEAN_ARM_THRESHOLD_MS - armed_ms
    return fields("RUN", n=index, of=total, scenario=scenario_name, verdict=verdict,
                  outcome=outcome, armed_ms=armed_ms, margin_ms=margin_ms,
                  session=session, pass_so_far=passes_so_far, fail_so_far=fails_so_far,
                  schedule_behind=behind, session_verdict=session_verdict, start=start,
                  spin_up_max_ms=None if spin_up_max_ms is None else round(spin_up_max_ms))


def run_once(scenario_path, load=None):
    """
    Run one scenario once as a full session (scripts/run_session.py: session folder, spy, summary.json),
    no retry (see run_attempt()).
    Returns (outcome, session_name, output, session_verdict); outcome is one of
    "deploy_failed", "clean", "reboot", "refused", "error"; session_verdict is the verdict of the session's own
    summary.json (pass / fail / unknown), a wider judgement than the arming outcome.

    scenario_path is either a scenario JSON or a bare device script (under tests/bench/ - see run_test.py's own
    SCRIPT_DIRS), run directly by name so it can be interleaved with scenario JSON files in the same batch
    (BUG-002, 2026-10-10: isolating the Pico-reset-per-attempt variable by running a diagnostic tool and a scenario
    through the exact same reset path, instead of comparing a tool run that never reset against one that always does).
    """
    try:
        session_name, output, _, summary = run_session.run_session(scenario_path, echo=False, argv=[str(scenario_path)], load=load)
    except SystemExit as stop:          # run_session gives up with a message (the spy never started, ...)
        return "error", None, "run_session stopped: %s" % stop.code, None
    session_verdict = summary["verdict"]

    if not DEPLOY_OK_RE.search(output):
        return "deploy_failed", session_name, output, session_verdict
    if "Outcome: completed" in output:
        armed_after_match = ARMED_AFTER_RE.search(output)
        if armed_after_match is None:
            # "Armed after Nms." always precedes "Outcome: completed" on a
            # real run - its absence here means the output doesn't look like
            # a normal completed run, so this is unclassifiable, not clean
            return "error", session_name, output, session_verdict
        if int(armed_after_match.group(1)) < CLEAN_ARM_THRESHOLD_MS:
            return "clean", session_name, output, session_verdict
        return "reboot", session_name, output, session_verdict
    if REFUSED_RE.search(output):
        return "refused", session_name, output, session_verdict
    return "error", session_name, output, session_verdict


def run_attempt(scenario_path, load=None):
    """run_once(), retrying a deploy failure up to DEPLOY_RETRIES times."""
    outcome, session_name, output, session_verdict = run_once(scenario_path, load)
    retries = 0
    while outcome == "deploy_failed" and retries < DEPLOY_RETRIES:
        print("  deploy failed, retrying...")
        retries += 1
        outcome, session_name, output, session_verdict = run_once(scenario_path, load)
    return outcome, session_name, output, session_verdict


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--scenario", action="append", required=True,
                         help="a scenario JSON path; repeat to interleave several")
    parser.add_argument("--load", required=True,
                         help='what is on the motor shafts, recorded in every session.json: "none", or e.g. '
                              '"Gemfan D2.2 3-blade, 1-1.25 g each"')
    parser.add_argument("--rounds", type=int, default=1,
                         help="how many times to run through all --scenario values (default 1)")
    parser.add_argument("--rotate", action="store_true",
                         help="cycle the --scenario order by one position each round, "
                              "instead of a fixed order every round")
    args = parser.parse_args()

    scenarios = [Path(s) for s in args.scenario]
    for s in scenarios:
        if not s.exists():
            parser.error("scenario not found: %s" % s)
    if args.rounds < 1:
        parser.error("--rounds must be at least 1")

    tally = {str(s): {"clean": 0, "reboot": 0, "refused": 0, "error": 0, "deploy_failed": 0,
                       "schedule_behind_total": 0, "schedule_behind_max": 0,
                       "schedule_behind_attempts": 0, "armed_ms": []}
             for s in scenarios}
    sessions = []
    summaries = []
    in_a_row = 0
    total = len(scenarios) * args.rounds
    n = 0

    try:
        for round_index in range(1, args.rounds + 1):
            order = scenarios
            if args.rotate:
                shift = (round_index - 1) % len(scenarios)
                order = scenarios[shift:] + scenarios[:shift]
            for scenario in order:
                n += 1
                print("=== Round %d/%d: %s (%d/%d) ===" % (round_index, args.rounds, scenario.name, n, total))
                outcome, session_name, output, session_verdict = run_attempt(scenario, args.load)
                counts = tally[str(scenario)]
                counts[outcome] += 1
                if session_name:
                    sessions.append(session_name)
                # A deploy failure never reached the device, so there is no
                # Core1Runner to have fallen behind - excluded from both the
                # count and its own denominator, same as from the KPI.
                armed_ms = None
                armed_match = ARMED_AFTER_RE.search(output)
                if armed_match and outcome in ("clean", "reboot"):
                    armed_ms = int(armed_match.group(1))
                    if outcome == "clean":
                        counts["armed_ms"].append(armed_ms)
                behind_count = None
                if outcome != "deploy_failed":
                    behind_match = SCHEDULE_BEHIND_RE.search(output)
                    behind_count = int(behind_match.group(1)) if behind_match else 0
                    counts["schedule_behind_total"] += behind_count
                    counts["schedule_behind_max"] = max(counts["schedule_behind_max"], behind_count)
                    counts["schedule_behind_attempts"] += 1
                # pass = armed inside the bound (outcome "clean"); fail = armed
                # late after an ESC reboot, or never armed (a refusal counts as
                # a failure, BUG-002's own KPI definition). error and
                # deploy_failed are neither: a crash or a USB hiccup is no
                # verdict on arming itself.
                verdict = ("pass" if outcome == "clean"
                           else "fail" if outcome in ("reboot", "refused")
                           else "excluded")
                passes = counts["clean"]
                fails = counts["reboot"] + counts["refused"]
                summary_path = run_session.CAPTURES / session_name / "summary.json" if session_name else None
                run_summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path is not None and summary_path.is_file() else None
                if run_summary is not None:
                    summaries.append(run_summary)
                start, spin_up_max_ms = start_kpi.run_field(run_summary)
                print(run_record(n, total, scenario.name, verdict, outcome, armed_ms,
                                 session_name, passes, fails, behind_count, session_verdict, start, spin_up_max_ms))
                if outcome in ("error", "deploy_failed"):
                    print("  --- full output, for diagnosis ---")
                    print(output)
                    print("  --- end output ---")
                in_a_row = in_a_row + 1 if outcome in ("error", "deploy_failed") else 0
                if in_a_row >= MAX_CONSECUTIVE_PROBLEMS:
                    print("STOPPED: %d errors or deploy failures in a row; the batch ends here." % in_a_row)
                    raise StopBatch
    except StopBatch:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted - printing the tally gathered so far.")

    print()
    print("=== Series summary ===")
    for scenario in scenarios:
        counts = tally[str(scenario)]
        passes = counts["clean"]
        fails = counts["reboot"] + counts["refused"]
        judged = passes + fails
        # "pass" is a Python keyword-argument name only through ** unpacking
        result = {"scenario": scenario.name,
                  "attempts": judged + counts["error"] + counts["deploy_failed"],
                  "pass": passes, "fail": fails,
                  "pass_pct": "%.1f" % (100.0 * passes / judged) if judged else None,
                  "fail_late": counts["reboot"], "fail_refused": counts["refused"],
                  "error": counts["error"], "deploy_failed": counts["deploy_failed"]}
        print(fields("RESULT", **result))
        # Arming time of the arms that passed only: a late arm already shows
        # in fail_late and in its own RUN line.
        room = arming_time_summary(counts["armed_ms"])
        if room:
            print(fields("ARMING_TIME", scenario=scenario.name, successful=room["n"],
                         min_ms=room["min"], median_ms=room["median"], max_ms=room["max"],
                         limit_ms=CLEAN_ARM_THRESHOLD_MS, marginal_band_ms=MARGINAL_BAND_MS,
                         marginal=room["marginal"]))
        behind_attempts = counts["schedule_behind_attempts"]
        if behind_attempts:
            print(fields("SCHEDULE", scenario=scenario.name, attempts=behind_attempts,
                         behind_total=counts["schedule_behind_total"],
                         behind_max=counts["schedule_behind_max"]))
    if any(s.get("start") for s in summaries):
        for line in start_kpi.format_lines(start_kpi.aggregate(summaries)):
            print(line)


if __name__ == "__main__":
    main()
