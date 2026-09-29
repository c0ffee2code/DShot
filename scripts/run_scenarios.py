"""
run_scenarios.py - run one or more scenarios repeatedly on the bench,
tallying outcomes.

Run from project root:
  python scripts/run_scenarios.py --scenario tests/harness/scenarios/two_channel_arming_check_600.json --rounds 10
  python scripts/run_scenarios.py --scenario A.json --scenario B.json --scenario C.json --rounds 8
      # interleaves A, B, C, in that order, for 8 rounds each (24 runs total),
      # so anything that drifts over the session lands on every configuration
      # alike rather than piling onto whichever runs first.
  add --pull to pull every session's captures at the end (scripts/pull_captures.py)

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
  armed          "Outcome: completed" was printed.
  refused        "arming did not complete within" was printed (the arming
                 timeout's own message).
  error          deploy succeeded but neither of the above appeared - an
                 unexpected crash. Always reported with its full output,
                 never silently dropped into another bucket.
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
    "deploy_failed", "armed", "refused", "error".
    """
    try:
        result = subprocess.run(
            [PYTHON, str(RUN_TEST), "--scenario", str(scenario_path)],
            capture_output=True, text=True, timeout=ATTEMPT_TIMEOUT_S,
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
        return "armed", session_name, output
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
    args = parser.parse_args()

    scenarios = [Path(s) for s in args.scenario]
    for s in scenarios:
        if not s.exists():
            parser.error("scenario not found: %s" % s)
    if args.rounds < 1:
        parser.error("--rounds must be at least 1")

    tally = {str(s): {"armed": 0, "refused": 0, "error": 0, "deploy_failed": 0} for s in scenarios}
    sessions = []
    total = len(scenarios) * args.rounds
    n = 0

    try:
        for round_index in range(1, args.rounds + 1):
            for scenario in scenarios:
                n += 1
                print("=== Round %d/%d: %s (%d/%d) ===" % (round_index, args.rounds, scenario.name, n, total))
                outcome, session_name, output = run_attempt(scenario)
                tally[str(scenario)][outcome] += 1
                if session_name:
                    sessions.append(session_name)
                print("  %s%s" % (outcome, ("  " + session_name) if session_name else ""))
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
        completed = counts["armed"] + counts["refused"] + counts["error"]
        print("%s: armed %d, refused %d, error %d (of %d completed attempts); %d deploy failure(s)" % (
            scenario.name, counts["armed"], counts["refused"], counts["error"], completed,
            counts["deploy_failed"]))

    if args.pull and sessions:
        print()
        print("Pulling %d session(s)..." % len(sessions))
        subprocess.run([PYTHON, str(ROOT / "scripts" / "pull_captures.py")])


if __name__ == "__main__":
    main()
