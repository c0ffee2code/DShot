"""
check_scenario.py — load-only validation of a scenario JSON file, no
hardware involved. Prints each motor's compiled throttle waypoints so a
ramp/repeat expansion can be eyeballed before ever touching hardware.

Run from project root:
  python scripts/check_scenario.py tests/harness/scenarios/two_channel_divergent_300.json

Reuses tests/harness/scenario.py and throttle_profile.py unmodified - they
avoid MicroPython-only APIs, so the exact same validation this script runs
also runs, verbatim, on-device before a scenario is ever armed.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "harness"))

from scenario import load_scenario


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: python scripts/check_scenario.py <scenario.json>")

    path = Path(sys.argv[1])
    try:
        scenario = load_scenario(path)
    except (ValueError, OSError) as e:
        sys.exit(f"INVALID: {e}")

    print(f"OK: {path}")
    print(f"dshot_speed={scenario.dshot_speed} duration_ms={scenario.duration_ms} "
          f"bidir_motors={scenario.bidir_indices}")
    print(f"arm_duration_ms={scenario.arm_duration_ms} "
          f"poll_ms={scenario.poll_ms}")
    if scenario.expect:
        print(f"expect={scenario.expect}")
    print()

    for index, spec in enumerate(scenario.motors):
        kind = "bidir" if spec.bidirectional else "tx-only"
        rx = f" rx_sm_id={spec.rx_sm_id}" if spec.bidirectional else ""
        print(f"Motor {index}: pin={spec.pin} sm_id={spec.sm_id}{rx} ({kind})")
        for start_ms, throttle in spec.profile.waypoints:
            print(f"  t={start_ms:>7}ms  throttle={throttle}")
        print(f"  total_duration_ms={spec.profile.total_duration_ms}")
        print()


if __name__ == "__main__":
    main()
