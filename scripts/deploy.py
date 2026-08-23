"""
deploy.py - upload the DShot driver + Core1Runner to the Pico, then run
tests/test_slow_spin.py live (streams output for the full ~2-minute test).

Run from project root:
  python scripts/deploy.py

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
    ("tests/core1_runner.py", "core1_runner.py"),
]

TEST_SCRIPT = ROOT / "tests" / "test_slow_spin.py"


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
    print(f"Deploying to Pico on {COM_PORT}...")
    ok = sum(_upload(loc, rem) for loc, rem in LIBRARY_FILES)
    failed = len(LIBRARY_FILES) - ok
    print(f"\nUploaded {ok}, failed {failed}.")
    if failed:
        sys.exit(1)

    print(f"\nRunning {TEST_SCRIPT.relative_to(ROOT)} on {COM_PORT} (live output)...\n")
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "run", str(TEST_SCRIPT)],
        timeout=150,
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
