"""
run_scenarios.py - run one or more scenarios repeatedly on the bench,
tallying outcomes.

Run from project root:
  python scripts/run_scenarios.py --scenario tests/harness/scenarios/two_channel_arming_check_600.json --rounds 10
  python scripts/run_scenarios.py --scenario A.json --scenario B.json --scenario C.json --rounds 8
      # interleaves A, B, C, in that order, for 8 rounds each (24 runs total),
      # so anything that drifts over the session lands on every configuration
      # alike rather than piling onto whichever runs first.
  --scenario also accepts a bare device script (tools/ or tests/device/, a .py
      path), run directly by name via run_test.py instead of as --scenario
      JSON - for interleaving a diagnostic tool against a real scenario under
      the exact same deploy/reset path (see run_once()'s own docstring)
  add --pull to pull every session's captures at the end (scripts/pull_captures.py)
  add --rotate to cycle the --scenario order by one position each round
      (round 1: A,B,C; round 2: B,C,A; round 3: C,A,B; ...) so no scenario is
      always first or always last - without it, whichever comes last in a
      fixed order always runs right after the same neighbor and after the
      same elapsed time since the batch started, which is itself a variable
      (bug-reports/BUG-002-...md's hypothesis 4, ESC-state carryover)

Classification, per attempt, is read from run_test.py's own printed output -
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
                 of arm() - see BUG-002.md's "Goal: stable-arming KPI" for why
                 the duration bound, not "did an ESC reboot cycle happen," is
                 the actual requirement: a caller doesn't know or care about
                 AM32's internal reboot mechanism, only how long they waited)
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

The series summary also reports how often Core1Runner's deadline schedule
fell behind (update() cost exceeded core1_interval_us for at least one tick),
parsed from run_scenario.py's own WARNING line - total, average per attempt,
and the worst single attempt. Zero for every attempt in a run with
core1_interval_us<=0 (no fixed pacing: nothing to fall behind). Excludes
deploy-failed attempts, which never reached the device.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

# When stdout isn't a real terminal (piped, redirected), Python fully
# buffers it - our own prints would otherwise appear out of order against
# child processes' directly-inherited output (pull_captures.py's, here) -
# see deploy.py's own copy of this line for the same reason.
sys.stdout.reconfigure(line_buffering=True)

ROOT = Path(__file__).resolve().parent.parent
PYTHON = sys.executable
RUN_TEST = ROOT / "scripts" / "run_test.py"

# A run that never got past deploy has no "failed 0" - matched separately
# from a bare "Uploaded" line so a partial deploy (some files failed) is
# still caught, not mistaken for success because the word appeared.
DEPLOY_OK_RE = re.compile(r"Uploaded \d+, failed 0\.")
REFUSED_RE = re.compile(r"arming did not complete within")
SESSION_RE = re.compile(r"Session: */sd/dshot_captures/(\S+)")
ARMED_AFTER_RE = re.compile(r"Armed after (\d+)ms\.")
# run_scenario.py's own WARNING line (printed from its `finally`, so it's
# there on clean/reboot/refused alike - anything that got far enough to
# start Core1Runner) - see Core1Runner.schedule_behind_count's own docstring
# for what "fell behind" means: update() cost exceeded core1_interval_us for
# at least that many ticks. Absent entirely means 0, not unknown - the line
# only prints when the count is nonzero.
SCHEDULE_BEHIND_RE = re.compile(r"fell behind (\d+) time\(s\)")

# This IS the KPI's duration bound (BUG-002.md's "Goal: stable-arming KPI": in
# >=99% of attempts, every configured motor reaches ARMED within this many ms
# of arm() - not a classification convenience). It also happens to sit in the
# gap between the "no reboot" duration cluster (~2.1-2.6s) and the "one
# reboot" cluster (~4.6s, one ~2.46s reboot cycle added) - reconfirmed
# archive-wide 2026-10-04 (698 sessions): clean arms 2078-2593ms, reboot arms
# >=4057ms, a clean ~1.4s gap with zero occupancy either side. Recheck this
# constant if a session is ever seen landing between the clusters, or if
# MotorGroup's own arm_duration_ms/READY_SPAN_MS defaults change (this number
# is derived from those, not independent of them).
CLEAN_ARM_THRESHOLD_MS = 3300

# One retry has cleared every deploy failure seen so far (a transient USB
# re-enumeration race after the previous run's reset) - not a tunable meant
# to paper over a real, repeated hardware problem.
DEPLOY_RETRIES = 1

# Comfortably past run_test.py's own 600s internal watchdog.
ATTEMPT_TIMEOUT_S = 650


def run_once(scenario_path):
    """
    Run one scenario once via run_test.py, no retry (see run_attempt()).
    Returns (outcome, session_name, output); outcome is one of
    "deploy_failed", "clean", "reboot", "refused", "error".

    scenario_path is either a scenario JSON (passed to run_test.py via
    --scenario) or a bare device script (tests/harness, tests/device or
    tools/ - see run_test.py's own SCRIPT_DIRS), run directly by name so it
    can be interleaved with scenario JSON files in the same batch (BUG-002,
    2026-10-10: isolating the Pico-reset-per-attempt variable by running a
    diagnostic tool and a scenario through the exact same reset path,
    instead of comparing a tool run that never reset against one that
    always does).
    """
    if scenario_path.suffix == ".py":
        run_args = [str(scenario_path.name)]
    else:
        run_args = ["--scenario", str(scenario_path)]
    try:
        # encoding/errors explicit, not text=True: text=True decodes with
        # the platform's default codepage (cp1252 on Windows), which raises
        # on bytes the Pico's own output can legitimately contain - the same
        # reason run_test.py decodes its own subprocess output as UTF-8 with
        # errors="replace" rather than relying on the default.
        result = subprocess.run(
            [PYTHON, str(RUN_TEST)] + run_args,
            capture_output=True, encoding="utf-8", errors="replace",
            timeout=ATTEMPT_TIMEOUT_S,
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired as e:
        output = (e.stdout or "") + (e.stderr or "") + "\n[run_scenarios.py: timed out after %ds]" % ATTEMPT_TIMEOUT_S
        return "error", None, output

    session_match = SESSION_RE.search(output)
    session_name = session_match.group(1) if session_match else None

    if not DEPLOY_OK_RE.search(output):
        return "deploy_failed", session_name, output
    if "Outcome: completed" in output:
        armed_after_match = ARMED_AFTER_RE.search(output)
        if armed_after_match is None:
            # "Armed after Nms." always precedes "Outcome: completed" on a
            # real run - its absence here means the output doesn't look like
            # a normal completed run, so this is unclassifiable, not clean
            return "error", session_name, output
        if int(armed_after_match.group(1)) < CLEAN_ARM_THRESHOLD_MS:
            return "clean", session_name, output
        return "reboot", session_name, output
    if REFUSED_RE.search(output):
        return "refused", session_name, output
    return "error", session_name, output


def run_attempt(scenario_path):
    """run_once(), retrying a deploy failure up to DEPLOY_RETRIES times."""
    outcome, session_name, output = run_once(scenario_path)
    retries = 0
    while outcome == "deploy_failed" and retries < DEPLOY_RETRIES:
        print("  deploy failed, retrying...")
        retries += 1
        outcome, session_name, output = run_once(scenario_path)
    return outcome, session_name, output


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--scenario", action="append", required=True,
                         help="a scenario JSON path; repeat to interleave several")
    parser.add_argument("--rounds", type=int, default=1,
                         help="how many times to run through all --scenario values (default 1)")
    parser.add_argument("--pull", action="store_true",
                         help="pull every session's captures at the end (scripts/pull_captures.py)")
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
                       "schedule_behind_attempts": 0}
             for s in scenarios}
    sessions = []
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
                outcome, session_name, output = run_attempt(scenario)
                counts = tally[str(scenario)]
                counts[outcome] += 1
                if session_name:
                    sessions.append(session_name)
                # A deploy failure never reached the device, so there is no
                # Core1Runner to have fallen behind - excluded from both the
                # count and its own denominator, same as from the KPI.
                behind_count = None
                if outcome != "deploy_failed":
                    behind_match = SCHEDULE_BEHIND_RE.search(output)
                    behind_count = int(behind_match.group(1)) if behind_match else 0
                    counts["schedule_behind_total"] += behind_count
                    counts["schedule_behind_max"] = max(counts["schedule_behind_max"], behind_count)
                    counts["schedule_behind_attempts"] += 1
                # Live running KPI: clean/n against clean+reboot+refused - a
                # refusal counts as not-clean (BUG-002's own KPI definition),
                # not excluded. error/deploy_failed are excluded: neither one
                # is a verdict on arming itself (a crash or a USB hiccup).
                kpi_n_so_far = counts["clean"] + counts["reboot"] + counts["refused"]
                kpi = ("clean %d/%d (%.1f%%)" % (counts["clean"], kpi_n_so_far,
                                                  100.0 * counts["clean"] / kpi_n_so_far)
                       if kpi_n_so_far else "clean -/0")
                behind_note = (", behind=%d" % behind_count) if behind_count else ""
                print("  %s%s  [%s%s]" % (outcome, ("  " + session_name) if session_name else "",
                                           kpi, behind_note))
                if outcome == "error":
                    print("  --- full output, for diagnosis ---")
                    print(output)
                    print("  --- end output ---")
    except KeyboardInterrupt:
        print("\nInterrupted - printing the tally gathered so far.")

    print()
    print("=== Series summary ===")
    for scenario in scenarios:
        counts = tally[str(scenario)]
        armed = counts["clean"] + counts["reboot"]
        kpi_n = armed + counts["refused"]
        completed = armed + counts["refused"] + counts["error"]
        clean_pct = (100.0 * counts["clean"] / kpi_n) if kpi_n else 0.0
        print("%s: clean %d, reboot %d (KPI: %.1f%% of %d - clean+reboot+refused), refused %d, "
              "error %d (of %d completed attempts); %d deploy failure(s)" % (
                  scenario.name, counts["clean"], counts["reboot"], clean_pct, kpi_n,
                  counts["refused"], counts["error"], completed, counts["deploy_failed"]))
        behind_attempts = counts["schedule_behind_attempts"]
        if behind_attempts:
            avg = counts["schedule_behind_total"] / behind_attempts
            print("  schedule fell behind: %d total across %d attempt(s) (avg %.1f/attempt, "
                  "max %d) - see Core1Runner.schedule_behind_count" % (
                      counts["schedule_behind_total"], behind_attempts, avg,
                      counts["schedule_behind_max"]))

    if args.pull and sessions:
        print()
        print("Pulling %d session(s)..." % len(sessions))
        subprocess.run([PYTHON, str(ROOT / "scripts" / "pull_captures.py")])


if __name__ == "__main__":
    main()
