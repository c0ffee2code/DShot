Run `python scripts/run_scenarios.py --scenario <path> [--scenario <path> ...] --rounds <N> [--pull]` and report the tally.

Runs one or more scenarios repeatedly on the bench, tallying each attempt as
armed, refused (arming timed out), or error (an unexpected crash) - reports
the tally, not just the last run's result.

Multiple `--scenario` values are interleaved round by round (all scenarios
once, then all again, ...), not run in blocks.

A deploy failure is retried once automatically and never counted as either
outcome - see `scripts/run_scenarios.py`'s own docstring for exactly how
each outcome is classified.

Add `--pull` to pull every session's captures at the end
(`scripts/pull_captures.py`) instead of running it separately after.

Example, interleaving two configurations for 10 rounds:
```
python scripts/run_scenarios.py \
  --scenario tests/harness/scenarios/two_channel_arming_check_600.json \
  --scenario tests/harness/scenarios/two_channel_arming_check_300.json \
  --rounds 10 --pull
```
