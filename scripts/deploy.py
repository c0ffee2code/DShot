"""
deploy.py - upload the DShot driver + test harness to the Pico, then reset it.
Does not run anything on the Pico - see run_test.py for that.

Run from project root:
  python scripts/deploy.py                          # library files only
  python scripts/deploy.py --scenario tests/harness/scenarios/two_channel_divergent_300.json
                                                      # also uploads the scenario as scenario.json

Pico must be connected on COM10.

The reset is required, not just tidy: `mpremote run` execs a script in the
same live MicroPython VM the previous invocation left behind, not a fresh
one, and skipping the reset reliably corrupts the next run's RX capture -
every captured word comes back zero, mimicking a dead ESC. A hard reset
before the run is the only fix found for it.
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

# Fixed device-side name run_scenario.py opens - mpremote's `run`
# has no mechanism to pass an extra file/argument into the running script,
# so a chosen scenario file has to land at this fixed name instead.
SCENARIO_REMOTE_NAME = "scenario.json"


def upload(local_rel, remote_name):
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


def deploy(scenario_path=None):
    """
    Upload the library files (and, if given, a scenario JSON as scenario.json),
    then reset the board. Returns True if every upload succeeded, False otherwise
    - the board is only reset on full success, since resetting after a partial
    upload would leave a half-updated, freshly-reset device.
    """
    print(f"Deploying to Pico on {COM_PORT}...")
    ok = sum(upload(loc, rem) for loc, rem in LIBRARY_FILES)
    failed = len(LIBRARY_FILES) - ok

    if scenario_path is not None:
        if upload(scenario_path, SCENARIO_REMOTE_NAME):
            ok += 1
        else:
            failed += 1

    print(f"\nUploaded {ok}, failed {failed}.")
    if failed:
        return False

    # See this module's docstring: skipping this corrupts the next run's RX
    # capture. Costs about a second.
    print(f"\nResetting {COM_PORT}...")
    subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "reset"],
        timeout=30,
    )
    # The board reboots and the USB CDC serial port briefly disappears and
    # re-enumerates - connecting too soon fails with "failed to access
    # COM10 (it may be in use by another program)". This is comfortably
    # longer than the re-enumeration takes.
    time.sleep(3)
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

    if args:
        print("deploy.py only uploads files and resets the board - it takes no other "
              "arguments and runs nothing on the Pico. Use scripts/run_test.py to run a "
              "test script.")
        sys.exit(1)

    sys.exit(0 if deploy(scenario_path) else 1)


if __name__ == "__main__":
    main()
