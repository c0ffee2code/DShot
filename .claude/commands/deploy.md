Run `python scripts/run_test.py [test_script.py]` and report the result.

Uploads the DShot driver, MotorGroup facade and the harness modules to the
Pico (flat filesystem, matching their `from dshot_pio import ...` style
imports) via `scripts/deploy.py`, resets the board, then runs the given
script (looked up in `tests/harness/`, then `tests/device/`, then `tools/`)
via `mpremote run` and streams its output live. `scripts/deploy.py` on its
own only uploads files and resets the board - it never runs anything, so use
it directly only when you want files on the Pico without spinning anything
(e.g. before a manual REPL session).

The normal bench run is a scenario:
`python scripts/run_test.py --scenario tests/harness/scenarios/<file>.json`
(the scenario is uploaded as `scenario.json`; the run spins the motors and
prints its own verdict). With no script name, `run_scenario.py` is assumed and
`--scenario` is required. Pull the session afterwards with
`python scripts/pull_captures.py` (it also deletes each pulled session from the SD
card) and analyse it with `python scripts/analyze_bidir_capture_log.py`.

The PC unit tests need no board: `python -m unittest discover -s tests/unit`.
