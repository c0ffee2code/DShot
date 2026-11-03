"""
run_session.py - run one scenario as one session: the PC names the session, both Picos use that name, and
everything the run produces lands in captures/<session>/.

  python scripts/run_session.py tests/bench/dshot_protocol/scenarios/four_channel_bidirectional_600.json
  python scripts/run_session.py <scenario> --no-spy      # without the spy (no spy.bin)
  python scripts/run_session.py <scenario> --esc-mock       # the wingman plays the ESC (ESC disconnected; not evidence)
  python scripts/run_session.py test_core0_io_options.py    # a device test script instead of a scenario
                                                            # (spy off unless --spy: most do not raise the start line)

What it does, in order: makes captures/<session>/ (name from this PC's clock) and writes session.json there (what was
run: scenario, git commit and whether the tree had uncommitted changes, boards, arguments); starts the spy
(tests/bench/wingman/tools/wingman_run.py, on by default) filing spy.bin into the session; runs the scenario through scripts/run_test.py
with the session name, which the main Pico uses as its SD folder name; then pulls the SD session into the same folder
(what scripts/pull_captures.py does: the Pico's record of the run goes into session.json's "device" section). The folder
ends up with session.json, run.log, scenario.json (copied by this script from the scenario it ran), capture.bin (the Pico's sampled record) and, if the spy ran, spy.bin, and then summary.json,
the computed verdict (scripts/session_summary.py, which can be re-run on any session folder). This spins the motors the
scenario names - the caller owns that decision.

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
import pull_captures
import session_summary
from run_test import new_session_name
import wingman_board
import wingman_plan

ROOT = Path(__file__).resolve().parents[1]
CAPTURES = ROOT / "captures"
SCENARIO_LINES = ("Armed after", "Outcome:", "Motor ", "Largest gap", "WARNING", "FAILED", "Test ", "Error", "Traceback")
DEPLOY_OK = re.compile(r"Uploaded \d+, failed 0\.")
RUN_TIMEOUT_S = 650                 # past run_test.py's own 600 s watchdog


def git_state(root=ROOT):
    """(commit hash, True when the working tree has uncommitted changes); (None, None) outside a git checkout."""
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return None, None
    return commit, dirty


def build_manifest(session, scenario, spy, argv, commit, dirty, started, load=None, esc_mock=None):
    return {
        "session": session,
        "started": started.isoformat(timespec="seconds"),
        "command": argv,
        "scenario": scenario,
        "spy": spy,
        "main_pico_id": wingman_board.MAIN_ID,
        "wingman_pico_id": wingman_board.WINGMAN_ID if spy else None,
        "git_commit": commit,
        "git_tree_dirty": dirty,
        "load": load,
        "esc_mock": esc_mock or None,        # the replies the wingman played back: {"fixture": file, "source": where they were recorded} (WingmanPlan)
    }


def write_manifest(folder, manifest):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "session.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def start_spy(session, scenario_path, esc_mock=False):
    """Start wingman_run.py for this session and return (process, its first lines) once it is recording. The wingman reads the
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
    sys.exit("the spy never reported that it is recording:\n" + "\n".join(seen))


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


def run_session(scenario, spy=None, echo=True, argv=None, config=None, load=None, esc_mock=False):
    """One session start to finish. scenario: a scenario JSON path or the bare name of a device test script.
    spy: None = the default (on for a scenario, off for a script). Returns (session name, the console output of
    run_test.py, its exit code, the summary dict that was written to summary.json). echo=False prints nothing, for a
    batch that prints its own line per attempt. config: for a device script that takes a config JSON (start quality,
    RPM modes), its path; it is passed to the script as --scenario and copied into the session as scenario.json.
    load: what is on the motor shafts for this run, in the owner's words ("none", "Gemfan D2.2 3-blade, 1-1.25 g each");
    written to session.json as `load`, since nothing else in a session says it. esc_mock: the wingman plays the ESC
    (recorded replies, tests/bench/wingman/); needs the spy, and the ESC must be disconnected from the signal lines."""
    scenario = Path(scenario)
    is_script = scenario.suffix == ".py"
    if esc_mock and ((is_script and config is None) or spy is False):
        sys.exit("--esc-mock plays the ESC on the wingman: it needs a scenario (or a device script with its config) and the spy (no --no-spy)")
    if not is_script and not scenario.exists():
        sys.exit("no such scenario: %s" % scenario)
    if spy is None:
        spy = not is_script
    plan = None
    if spy:
        wiring = config if config is not None else (None if is_script else scenario)
        if wiring is None:
            sys.exit("the spy needs a scenario or a config to know the wiring: pass one, or leave the spy off")
        try:
            plan = wingman_plan.plan_from_scenario(json.loads(Path(wiring).read_text(encoding="utf-8")), bool(esc_mock))
        except (OSError, ValueError) as error:
            sys.exit("the wingman cannot serve %s: %s" % (wiring, error))
    run_args = [scenario.name] if is_script else ["--scenario", str(scenario)]
    if config is not None:
        run_args += ["--scenario", str(config)]
    started = datetime.now()
    session = new_session_name(started)
    folder = CAPTURES / session
    commit, dirty = git_state()
    write_manifest(folder, build_manifest(session, str(config if config is not None else scenario), spy,
                                          argv if argv is not None else [str(scenario)], commit, dirty, started, load, plan.mock.manifest_entry() if plan is not None and plan.mock else None))
    if config is not None:
        shutil.copyfile(config, folder / "scenario.json")
    elif not is_script:
        shutil.copyfile(scenario, folder / "scenario.json")     # the Pico does not keep a copy on its SD card
    if echo:
        print("Session %s -> %s" % (session, folder))

    spy_process = None
    try:
        if spy:
            spy_process, _ = start_spy(session, wiring, esc_mock)
            time.sleep(2)
        try:
            test = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_test.py")] + run_args + ["--session", session],
                                  capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=ROOT, timeout=RUN_TIMEOUT_S)
            output, code = test.stdout + test.stderr, test.returncode
        except subprocess.TimeoutExpired as error:
            output = (error.stdout or "") + (error.stderr or "") + "\n[run_session.py: timed out after %d s]" % RUN_TIMEOUT_S
            code = -1
        (folder / "run.log").write_text(output, encoding="utf-8", errors="replace")
        if echo:
            shown = output.splitlines()[-8:] if is_script else [
                line for line in output.splitlines() if line.startswith(SCENARIO_LINES) or "decoded on the device" in line]
            for line in shown:
                print("  " + line)
            if code != 0:
                print("  run_test exit code %d" % code)
                print("\n".join("  | " + l for l in output.splitlines()[-12:]))
        if spy_process is not None:
            if not DEPLOY_OK.search(output):
                spy_process.kill()             # the run never started, so the start line never rises: do not wait for it
            try:
                rest, _ = spy_process.communicate(timeout=plan.run_seconds + 30)
            except subprocess.TimeoutExpired:
                spy_process.kill()             # a spy that outlives its wait must not stop the SD pull and the summary
                rest, _ = spy_process.communicate()
                print("WARNING: the spy did not exit in time and was killed")
            if echo:
                end = next((l for l in rest.splitlines() if l.startswith("end:")), "")
                print("  spy: %s" % end.replace("end: ", ""))
                if not (folder / "spy.bin").exists():
                    print("WARNING: the spy produced no spy.bin")
    finally:
        if esc_mock:
            release_wingman(spy_process)       # the mock must not outlive the run, whatever happened above
    collect_sd_session(session, is_script)
    summary = session_summary.write_summary(folder)
    if echo:
        print()
        print(session_summary.table(summary))
        print("Files in %s: %s" % (folder, " ".join(sorted(p.name for p in folder.iterdir()))))
    return session, output, code, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", help="a scenario JSON, or the bare name of a device test script (*.py under tests/bench/)")
    parser.add_argument("--load", required=True, help='what is on the motor shafts, recorded in session.json: "none", or e.g. "Gemfan D2.2 3-blade, 1-1.25 g each"')
    parser.add_argument("--no-spy", action="store_true", help="do not start the spy (no spy.bin)")
    parser.add_argument("--spy", action="store_true", help="for a device test script: start the spy (default off)")
    parser.add_argument("--esc-mock", action="store_true", help="the wingman also plays the ESC (recorded replies, tests/bench/wingman/): for runs with the ESC disconnected from the signal lines")
    args = parser.parse_args()
    is_script = Path(args.scenario).suffix == ".py"
    spy = args.spy if is_script else not args.no_spy
    _, _, code, _ = run_session(args.scenario, spy, argv=sys.argv[1:], load=args.load, esc_mock=args.esc_mock)
    sys.exit(code)


if __name__ == "__main__":
    main()
