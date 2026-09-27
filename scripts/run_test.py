"""
run_test.py - deploy the DShot driver + test harness to the Pico (via deploy.py),
then run a test script on it live, streaming output for the duration of the run.
This is the script that actually spins motors - see deploy.py if you only want
to upload files without running anything.

Run from project root:
  python scripts/run_test.py --scenario tests/harness/scenarios/two_channel_divergent_300.json
                                                   # runs the scenario runner (the default script)
  python scripts/run_test.py test_capture_slot_stress.py   # runs another script from tests/harness,
                                                            # tests/device or tools
                                                   # --scenario uploads the file as scenario.json

Pico must be connected on COM10. mpremote interrupts any running script on connect;
deploy() resets the board before every run regardless (see deploy.py's module
docstring for why that reset is required, not merely tidy).
"""

import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy

ROOT = deploy.ROOT
PYTHON = deploy.PYTHON
COM_PORT = deploy.COM_PORT

DEFAULT_TEST_SCRIPT = "run_scenario.py"

# Where a script named on the command line is looked for, in order. tests/unit is
# not here: those tests run on a PC, not on the Pico.
SCRIPT_DIRS = ["tests/harness", "tests/device", "tools"]


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

    if not deploy.deploy(scenario_path):
        sys.exit(1)

    print(f"\nRunning {test_script.relative_to(ROOT)} on {COM_PORT} (live output)...\n")
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "run", str(test_script)],
        timeout=300,
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
