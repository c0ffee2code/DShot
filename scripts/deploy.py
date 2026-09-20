"""
deploy.py - upload the DShot driver + test harness to the Pico, then run a
test script live (streams output for the duration of the test).

Run from project root:
  python scripts/deploy.py                       # runs the default test script
  python scripts/deploy.py test_capture_slot_stress.py   # runs another script from tests/harness or tests/device
  python scripts/deploy.py run_scenario.py --scenario tests/harness/scenarios/two_channel_divergent_300.json
                                                   # also uploads the scenario file as scenario.json

Pico must be connected on COM10. mpremote interrupts any running script on connect.

`mpremote run` does NOT reset the board - it execs the script in the same
live MicroPython VM the previous invocation left behind. Confirmed on
hardware 2026-09-12: a bidirectional test run immediately following another
one (same or different channel, same or different exit path - clean
completion or an uncaught exception, it didn't matter) reliably corrupted
that next run's RX capture - the ESC still armed and TX still went out, but
every captured word came back zero, exactly mimicking a dead ESC. A hard
reset before the run made it succeed every time; skipping the reset and
simply re-running failed every time. This is why `main()` below resets the
board before every run rather than relying on the test script's own
cleanup - whatever state doesn't get cleanly torn down between runs
(a leftover Core 1 thread and/or PIO state is the leading suspect, not yet
root-caused further) survives a clean Python-level exit, so only an actual
hardware reset is a reliable fix. This also means every hardware capture
session from before this fix that immediately followed another `mpremote
run` invocation (not a fresh reset) is suspect - see decision/
ADR-002-bidirectional-dshot.md's fixed-ratio RX sampling section for which
sessions that implicates.
"""

import subprocess
import sys
import time
from pathlib import Path

# When stdout isn't a real terminal (piped, redirected), Python fully
# buffers it - our own prints would otherwise sit behind the mpremote
# child's directly-inherited output and only appear at exit.
sys.stdout.reconfigure(line_buffering=True)

PYTHON = sys.executable
COM_PORT = "COM10"
ROOT = Path(__file__).resolve().parents[1]

LIBRARY_FILES = [
    ("driver/dshot_pio.py", "dshot_pio.py"),
    ("driver/capture_mailbox.py", "capture_mailbox.py"),
    ("driver/dshot_profiles.py", "dshot_profiles.py"),
    ("driver/gcr_decode.py", "gcr_decode.py"),
    ("driver/motor_group.py", "motor_group.py"),
    ("tests/harness/core1_runner.py", "core1_runner.py"),
    ("tests/harness/throttle_profile.py", "throttle_profile.py"),
    ("tests/harness/scenario.py", "scenario.py"),
    ("tests/harness/sdcard.py", "sdcard.py"),
    ("tests/harness/pcf8523.py", "pcf8523.py"),
    ("tests/harness/capture_sink.py", "capture_sink.py"),
    ("tests/harness/bidir_capture_sink.py", "bidir_capture_sink.py"),
    ("tests/harness/decode_tally.py", "decode_tally.py"),
]

DEFAULT_TEST_SCRIPT = "test_slow_spin.py"

# Where a script named on the command line is looked for, in order. tests/unit is
# not here: those tests run on a PC, not on the Pico.
SCRIPT_DIRS = ["tests/harness", "tests/device", "tests"]

# Fixed device-side name run_scenario.py opens - mpremote's `run`
# has no mechanism to pass an extra file/argument into the running script,
# so a chosen scenario file has to land at this fixed name instead.
SCENARIO_REMOTE_NAME = "scenario.json"


def _upload(local_rel, remote_name):
    local = ROOT / local_rel
    if not local.exists():
        print(f"  MISSING  {local_rel}")
        return False
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "cp", str(local), f":{remote_name}"],
        capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        print(f"  FAIL     {local_rel}: {result.stderr.strip()}")
        return False
    print(f"  OK       {local_rel} -> :{remote_name}")
    return True


def main():
    args = sys.argv[1:]

    scenario_path = None
    if "--scenario" in args:
        idx = args.index("--scenario")
        if idx + 1 >= len(args):
            print("--scenario requires a path argument")
            sys.exit(1)
        scenario_path = args[idx + 1]
        del args[idx:idx + 2]

    name = args[0] if args else DEFAULT_TEST_SCRIPT
    test_script = next((ROOT / d / name for d in SCRIPT_DIRS if (ROOT / d / name).exists()), None)
    if test_script is None:
        print(f"MISSING test script {name}: looked in {', '.join(SCRIPT_DIRS)}")
        sys.exit(1)

    if test_script.name == "run_scenario.py" and scenario_path is None:
        print("run_scenario.py needs --scenario <path to a scenario JSON>")
        sys.exit(1)

    print(f"Deploying to Pico on {COM_PORT}...")
    ok = sum(_upload(loc, rem) for loc, rem in LIBRARY_FILES)
    failed = len(LIBRARY_FILES) - ok

    if scenario_path is not None:
        if _upload(scenario_path, SCENARIO_REMOTE_NAME):
            ok += 1
        else:
            failed += 1

    print(f"\nUploaded {ok}, failed {failed}.")
    if failed:
        sys.exit(1)

    # See this module's docstring: without this, a run immediately following
    # another mpremote run invocation reliably corrupts RX capture on
    # whichever bidirectional channel runs next - confirmed on hardware
    # 2026-09-12. A hard reset first, every time, is the only fix found so
    # far; it costs about a second.
    print(f"\nResetting {COM_PORT}...")
    subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "reset"],
        timeout=30,
    )
    # The board reboots and the USB CDC serial port briefly disappears and
    # re-enumerates - connecting too soon fails with "failed to access
    # COM10 (it may be in use by another program)" (confirmed on hardware
    # 2026-09-12). This is comfortably longer than the re-enumeration takes.
    time.sleep(3)

    print(f"\nRunning {test_script.relative_to(ROOT)} on {COM_PORT} (live output)...\n")
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "run", str(test_script)],
        timeout=300,
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
