Run `python scripts/run_scenarios.py --scenario <path> [--scenario <path> ...] --rounds <N> [--pull] [--rotate]` and report the tally.

Runs one or more scenarios repeatedly on the bench, tallying each attempt as
clean (every configured motor reached ARMED within CLEAN_ARM_THRESHOLD_MS of
arm() - the stable-arming KPI's duration bound, target >=99%), reboot (armed,
but only after one or more ESC reboots pushed it past that duration), refused
(arming timed out), or error (an unexpected crash) - reports the tally, not
just the last run's result, including the running clean-rate KPI percentage.

Multiple `--scenario` values are interleaved round by round (all scenarios
once, then all again, ...), not run in blocks.

A deploy failure is retried once automatically and never counted as either
outcome - see `scripts/run_scenarios.py`'s own docstring for exactly how
each outcome is classified.

Add `--pull` to pull every session's captures at the end
(`scripts/pull_captures.py`) instead of running it separately after.

Add `--rotate` to cycle the `--scenario` order by one position each round,
so no scenario is always first or always last in a round - without it, a
fixed order confounds "which scenario" with "which position" and "how long
since the batch started."

Example, interleaving two configurations for 10 rounds:
```
python scripts/run_scenarios.py \
  --scenario tests/harness/scenarios/two_channel_divergent_600.json \
  --scenario tests/harness/scenarios/two_channel_divergent_300.json \
  --rounds 10 --pull
```
