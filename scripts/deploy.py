"""
deploy.py - upload the DShot driver + test harness to the Pico, then run a
test script live (streams output for the duration of the test).

Run from project root:
  python scripts/deploy.py                       # runs tests/test_slow_spin.py
  python scripts/deploy.py test_bidir_tx_arm.py   # runs a different test under tests/
  python scripts/deploy.py test_scenario_capture.py --scenario tests/harness/scenarios/dual_motor_divergent.json
                                                   # also uploads the scenario file as scenario.json

Pico must be connected on COM10. mpremote interrupts any running script on connect.
"""

import subprocess
import sys
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
    ("driver/motor_throttle_group.py", "motor_throttle_group.py"),
    ("tests/harness/core1_runner.py", "core1_runner.py"),
    ("tests/harness/throttle_profile.py", "throttle_profile.py"),
    ("tests/harness/scenario.py", "scenario.py"),
    ("tests/harness/scenario_runner.py", "scenario_runner.py"),
    ("tests/harness/sdcard.py", "sdcard.py"),
    ("tests/harness/pcf8523.py", "pcf8523.py"),
    ("tests/harness/bidir_capture_sink.py", "bidir_capture_sink.py"),
    ("tests/harness/stress_capture_sink.py", "stress_capture_sink.py"),
]

DEFAULT_TEST_SCRIPT = "test_slow_spin.py"

# Fixed device-side name test_scenario_capture.py opens - mpremote's `run`
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

    test_script = ROOT / "tests" / (args[0] if args else DEFAULT_TEST_SCRIPT)
    if not test_script.exists():
        print(f"MISSING test script: {test_script}")
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

    print(f"\nRunning {test_script.relative_to(ROOT)} on {COM_PORT} (live output)...\n")
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "run", str(test_script)],
        timeout=300,
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
