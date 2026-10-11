Run `python scripts/deploy.py [--scenario <path>]` and report the result.

Uploads the DShot driver and harness files to the Pico (flat filesystem,
matching their `from dshot_pio import ...` style imports), then resets the
board. With `--scenario <path>`, also uploads that file as `scenario.json`.

Runs nothing on the Pico - it only puts files in place and leaves the board
freshly reset. Use it on its own when you want files uploaded without
spinning anything (e.g. before a manual REPL session); for a run, use
`/run-scenarios`.
