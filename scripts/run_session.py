"""
run_session.py - run one scenario as one session: the PC names the session, both Picos use that name, and
everything the run produces lands in captures/<session>/.

  python scripts/run_session.py tests/bench/dshot_protocol/scenarios/four_channel_bidirectional_600.json
  python scripts/run_session.py <scenario> --esc-mock       # the wingman plays the ESC (ESC disconnected; not evidence)
  python scripts/run_session.py test_core0_io_options.py    # a device test script instead of a scenario: Pico only, no ESC, so it
                                                            # is recorded only with --observe (most do not raise the start line)

Two cases (decision/ADR-008): a scenario runs with an ESC, so it is always recorded by the wingman; there is no way to run one
without it. When the wingman is not connected, or does not start recording, nothing is uploaded to or run on the main Pico.
A session that did not start is a failed session, never an empty folder (ADR-008, amendment 2026-11-12 (2)): when the wingman does not start
(not connected, never reports that it is recording, cannot serve the scenario) or the main Pico does not start (the upload to it failed, or
a scenario's console never shows its first line, "=== Scenario Capture ==="), the fact is written to session.json as "start_failure"
({"who": "wingman" | "main_pico", "message": ...}), run.log holds what explains it, and summary.json says verdict "fail" with that reason
(session_summary.py reads the field; its other proofs are "not shown"). The command line then exits 1.
A device script runs on the Pico alone (unused pins, no ESC), has nothing for the wingman to record, and is recorded only when
asked with --observe. A session whose recording does not arrive (wingman started, no signal_observation.bin) has no verdict ("unknown", with a note naming why).

What it does, in order: makes captures/<session>/ (name from this PC's clock) and writes session.json there (what was
run: scenario, git commit and whether the tree had uncommitted changes, boards, arguments); starts the observer
(tests/bench/wingman/tools/wingman_run.py; always for a scenario, for a device script with --observe) filing signal_observation.bin into the session; runs the scenario through scripts/run_test.py
with the session name, which the main Pico uses as its SD folder name; then pulls the SD session into the same folder
(what scripts/pull_captures.py does: the Pico's record of the run goes into session.json's "device" section). The folder
ends up with session.json, run.log, scenario.json (copied by this script from the scenario it ran), capture.bin (the Pico's sampled record) and, if the observer ran, signal_observation.bin, and then summary.json,
the computed verdict (scripts/session_summary.py, which can be re-run on any session folder). This spins the motors the
scenario names - the caller owns that decision.

Exit code of the command line (the function run_session() returns run_test.py's code unchanged, for the batch runner): 0 only when the
session verdict is "pass"; 1 when it is "fail" (including a session that did not start) or run_test.py returned non-zero; 2 when it is "unknown" (nothing failed, but the
required proofs were not all shown, e.g. no wire recording, or an ESC-mock session).

scripts/run_scenarios.py (a batch) is a repetition of this: it calls run_session() once per attempt.
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
import observer_edge_log
import pull_captures
import session_files
import session_summary
from run_test import new_session_name
import wingman_board
import wingman_plan

ROOT = Path(__file__).resolve().parents[1]
CAPTURES = ROOT / "captures"
SCENARIO_LINES = ("Armed after", "Outcome:", "Motor ", "Largest gap", "WARNING", "=== Run ", "Error", "Traceback")
DEPLOY_OK = re.compile(r"Uploaded \d+, failed 0\.")
RUN_TIMEOUT_S = 650                 # past run_test.py's own 600 s watchdog
SCENARIO_BANNER = "=== Scenario Capture ==="       # the first line run_scenario.py prints: seen means the scenario began on the main Pico
DEPLOY_FAIL_LINES = ("  FAIL", "  MISSING", "Two library files")     # what deploy.py prints for an upload that did not work


class WingmanStartError(Exception):
    """The wingman did not start recording. reason: one line saying why; output: everything its launcher printed."""
    def __init__(self, reason, output):
        super().__init__(reason)
        self.reason = reason
        self.output = output


def git_state(root=ROOT):
    """(commit hash, True when the working tree has uncommitted changes); (None, None) outside a git checkout."""
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return None, None
    return commit, dirty


def build_manifest(session, scenario, observer, argv, commit, dirty, started, load=None, esc_mock=None):
    return {
        "session": session,
        "started": started.isoformat(timespec="seconds"),
        "command": argv,
        "scenario": scenario,
        "observer": observer,
        "main_pico_id": wingman_board.MAIN_ID,
        "wingman_pico_id": wingman_board.WINGMAN_ID if observer else None,
        "git_commit": commit,
        "git_tree_dirty": dirty,
        "load": load,
        "esc_mock": esc_mock or None,        # the replies the wingman played back: {"fixture": file, "source": where they were recorded} (WingmanPlan)
        "start_failure": None,               # set by record_start_failure(): {"who": "wingman" | "main_pico", "message": why it did not start}
    }


def write_manifest(folder, manifest):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "session.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def record_start_failure(folder, who, message):
    """Write the fact that the session did not start into session.json: who ("wingman" or "main_pico") and why. session_summary.py reads it."""
    manifest = session_files.read_manifest(folder) or {}
    manifest["start_failure"] = {"who": who, "message": message}
    write_manifest(folder, manifest)


def last_line(text):
    """The last non-empty line of text, or "" when there is none."""
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ""


def main_pico_start_failure(output, is_script):
    """Why the main Pico did not start, from the console output of run_test.py, or None when it did. Not started: deploy.py did not report a
    complete upload (the first failed upload line is the cause; failing that, the last line printed), or, for a scenario, the console has
    no first line of run_scenario.py (the board never ran it). A device script has no such line and is judged by the upload alone."""
    if not DEPLOY_OK.search(output):
        failed = next((line.strip() for line in output.splitlines() if line.startswith(DEPLOY_FAIL_LINES)), "")
        return "the upload to it did not complete (%s)" % (failed or last_line(output) or "no output from run_test.py")
    if not is_script and SCENARIO_BANNER not in output:
        return "the scenario never began on it (no '%s' line in the console; last line: %s)" % (SCENARIO_BANNER, last_line(output) or "none")
    return None


def start_observer(session, scenario_path, esc_mock=False):
    """Start wingman_run.py for this session and return (process, its first lines) once it is recording; raises WingmanStartError when it
    ends or goes silent before that (not connected, a failed upload, ...). The wingman reads the
    wiring and settings from scenario_path (the scenario or config the main Pico runs). esc_mock: it also answers the main Pico's
    frames with recorded ESC replies (tests/bench/wingman/)."""
    process = subprocess.Popen([sys.executable, str(ROOT / "tests" / "bench" / "wingman" / "tools" / "wingman_run.py"), "--session", session,
                                "--scenario", str(scenario_path)]
                               + (["--esc-mock"] if esc_mock else []),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
    seen = []
    deadline = time.time() + 90
    while time.time() < deadline:
        line = process.stdout.readline()
        if not line:
            break
        seen.append(line.rstrip())
        if "Recording" in line:
            return process, seen
    process.kill()
    raise WingmanStartError("the observer never reported that it is recording (%s)" % (last_line("\n".join(seen)) or "it printed nothing"),
                            "\n".join(seen) + "\n")


def release_wingman(process):
    """Make sure the wingman is not left playing the ESC: stop its launcher if that is still running (a killed launcher
    runs no cleanup of its own), then reset the board. Safe to call when everything already ended normally."""
    if process is not None and process.poll() is None:
        process.kill()
        process.communicate()
    port = wingman_board.find_port()
    if port is not None:
        wingman_board.reset(port)


def collect_sd_session(session, is_script=False):
    """Move the Pico's SD session into captures/<session>/ and delete it from the card once it arrived whole."""
    remote = pull_captures.list_remote()
    if session not in remote:
        if not is_script:
            print("WARNING: no session %s on the SD card (the run may have stopped before it was created)" % session)
        return False
    ok, failed = pull_captures.fetch([session])
    if ok:
        pull_captures.delete_remote(ok)
    return bool(ok)


def run_session(scenario, observer=None, echo=True, argv=None, config=None, load=None, esc_mock=False):
    """One session start to finish. scenario: a scenario JSON path or the bare name of a device test script.
    observer: a device script only: True records it with the wingman, None or False does not; a scenario is always recorded, and
    observer=False for a scenario stops with a message. Returns (session name, the console output of
    run_test.py, its exit code, the summary dict that was written to summary.json); when the wingman did not start, nothing ran on the
    main Pico: the output is the wingman's own text and the exit code 1. echo=False prints nothing, for a
    batch that prints its own line per attempt. config: for a device script that takes a config JSON (start quality,
    RPM modes), its path; it is passed to the script as --scenario and copied into the session as scenario.json.
    load: what is on the motor shafts for this run, in the owner's words ("none", "Gemfan D2.2 3-blade, 1-1.25 g each");
    written to session.json as `load`, since nothing else in a session says it. esc_mock: the wingman plays the ESC
    (recorded replies, tests/bench/wingman/); needs the wingman, and the ESC must be disconnected from the signal lines."""
    scenario = Path(scenario)
    is_script = scenario.suffix == ".py"
    if esc_mock and is_script and (config is None or not observer):
        sys.exit("--esc-mock plays the ESC on the wingman: it needs a scenario, or a device script with its config and --observe")
    if not is_script and not scenario.exists():
        sys.exit("no such scenario: %s" % scenario)
    if is_script:
        observer = bool(observer)
    elif observer is False:
        sys.exit("a scenario is always recorded by the wingman: it cannot be run without it")
    else:
        observer = True
    if config is not None and not Path(config).exists():
        sys.exit("no such config: %s" % config)
    plan = None
    plan_error = None
    if observer:
        wiring = config if config is not None else (None if is_script else scenario)
        if wiring is None:
            sys.exit("the observer needs a scenario or a config to know the wiring: pass one, or leave --observe off")
        try:
            plan = wingman_plan.plan_from_scenario(json.loads(Path(wiring).read_text(encoding="utf-8")), bool(esc_mock))
        except (OSError, ValueError) as error:
            plan_error = "the wingman cannot serve %s: %s" % (wiring, error)     # reported once the session folder exists
    run_args = [scenario.name] if is_script else ["--scenario", str(scenario)]
    if config is not None:
        run_args += ["--scenario", str(config)]
    started = datetime.now()
    session = new_session_name(started)
    folder = CAPTURES / session
    commit, dirty = git_state()
    write_manifest(folder, build_manifest(session, str(config if config is not None else scenario), observer,
                                          argv if argv is not None else [str(scenario)], commit, dirty, started, load, plan.mock.manifest_entry() if plan is not None and plan.mock else None))
    if config is not None:
        shutil.copyfile(config, folder / "scenario.json")
    elif not is_script:
        shutil.copyfile(scenario, folder / "scenario.json")     # the Pico does not keep a copy on its SD card
    if echo:
        print("Session %s -> %s" % (session, folder))

    observer_process = None
    start_failure = None            # (who, message) once either board is known not to have started
    output, code = "", 1
    try:
        if plan_error is not None:
            start_failure, output = ("wingman", plan_error), plan_error + "\n"
        elif observer:
            try:
                observer_process, _ = start_observer(session, wiring, esc_mock)
                time.sleep(2)
            except WingmanStartError as error:
                start_failure, output = ("wingman", error.reason), error.output
        if start_failure is None:
            try:
                test = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_test.py")] + run_args + ["--session", session],
                                      capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=ROOT, timeout=RUN_TIMEOUT_S)
                output, code = test.stdout + test.stderr, test.returncode
            except subprocess.TimeoutExpired as error:
                output = (error.stdout or "") + (error.stderr or "") + "\n[run_session.py: timed out after %d s]" % RUN_TIMEOUT_S
                code = -1
            (folder / "run.log").write_text(output, encoding="utf-8", errors="replace")
            why = main_pico_start_failure(output, is_script)
            if why is not None:
                start_failure = ("main_pico", why)
            if echo:
                shown = output.splitlines()[-8:] if is_script else [
                    line for line in output.splitlines() if line.startswith(SCENARIO_LINES) or "decoded on the device" in line]
                for line in shown:
                    print("  " + line)
                if code != 0:
                    print("  run_test exit code %d" % code)
                    print("\n".join("  | " + l for l in output.splitlines()[-12:]))
            if observer_process is not None:
                if start_failure is not None:
                    observer_process.kill()         # the run never started, so the start line never rises: do not wait for it
                try:
                    rest, _ = observer_process.communicate(timeout=plan.run_seconds + 30)
                except subprocess.TimeoutExpired:
                    observer_process.kill()             # an observer that outlives its wait must not stop the SD pull and the summary
                    rest, _ = observer_process.communicate()
                    print("WARNING: the observer did not exit in time and was killed")
                if echo:
                    end = next((l for l in rest.splitlines() if l.startswith("end:")), "")
                    print("  observer: %s" % end.replace("end: ", ""))
                    if start_failure is None and not (folder / observer_edge_log.OBSERVATION_FILE).exists():
                        print("WARNING: the observer produced no %s" % observer_edge_log.OBSERVATION_FILE)
    finally:
        if esc_mock:
            release_wingman(observer_process)       # the mock must not outlive the run, whatever happened above
    if start_failure is not None:
        record_start_failure(folder, *start_failure)
        if start_failure[0] == "wingman":
            (folder / "run.log").write_text(output, encoding="utf-8", errors="replace")      # nothing ran on the main Pico: the wingman's own text is the log
        if echo:
            print("The session did not start: %s" % session_summary.start_failure_text({"who": start_failure[0], "message": start_failure[1]}))
    else:
        collect_sd_session(session, is_script)      # not after a start failure: nothing ran, and the card may not even be reachable
    summary = session_summary.write_summary(folder)
    if echo:
        print()
        print(session_summary.table(summary))
        print("Files in %s: %s" % (folder, " ".join(sorted(p.name for p in folder.iterdir()))))
    return session, output, code, summary


def exit_code(test_code, verdict):
    """The command line's exit code: 0 only for a "pass" whose run_test.py returned 0; 1 for a fail or a non-zero run_test.py; 2 for unknown."""
    if test_code != 0 or verdict == "fail":
        return 1
    return 0 if verdict == "pass" else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", help="a scenario JSON, or the bare name of a device test script (*.py under tests/bench/)")
    parser.add_argument("--load", required=True, help='what is on the motor shafts, recorded in session.json: "none", or e.g. "Gemfan D2.2 3-blade, 1-1.25 g each"')
    parser.add_argument("--observe", action="store_true", help="device test scripts only (Pico alone, no ESC): record the run with the wingman; off by default. A scenario is always recorded")
    parser.add_argument("--esc-mock", action="store_true", help="the wingman also plays the ESC (recorded replies, tests/bench/wingman/): for runs with the ESC disconnected from the signal lines")
    args = parser.parse_args()
    is_script = Path(args.scenario).suffix == ".py"
    if args.observe and not is_script:
        parser.error("--observe is for device test scripts: a scenario is always recorded by the wingman")
    observer = args.observe if is_script else None
    _, _, code, summary = run_session(args.scenario, observer, argv=sys.argv[1:], load=args.load, esc_mock=args.esc_mock)
    sys.exit(exit_code(code, summary["verdict"]))


if __name__ == "__main__":
    main()
