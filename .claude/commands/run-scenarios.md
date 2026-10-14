Run `python scripts/run_scenarios.py --scenario <path> [--scenario <path> ...] --rounds <N> [--pull] [--rotate]` and report the tally.

Runs one or more scenarios repeatedly on the bench. Each attempt prints one `RUN` line of
`key=value` fields: `verdict=pass|fail|excluded` (pass = armed within 3.0 s of arm(), the
stable-arming KPI's duration bound, target >=99%; fail = armed late after an ESC reboot, or never
armed; error and deploy failures are excluded), `outcome=` (clean, reboot, refused, error,
deploy_failed), `armed_ms=` and `margin_ms=` (room left under the bound, for a pass). The series
summary prints `RESULT` (pass/fail counts and percentage), `ARMING_TIME` (min, median, max over the
arms that passed, plus how many were within 500 ms of the limit) and `SCHEDULE` lines. Report the
lines, not a paraphrase; they are meant to be grepped.

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

For arming reliability alone, use `tests/harness/scenarios/two_channel_arming_600.json`: same two
bidirectional motors, throttle 0 throughout, nothing spins, about 15 s per run.

A batch run without `--pull` leaves every session on the board's SD card. When the batch is
evidence (an arming-reliability run, not a smoke test), add `--pull` so the sessions are pulled
and can be analysed offline with `scripts/analyze_bidir_capture_log.py`. Note that a session's
capture begins at ARMED; the arming phase itself is not recorded.
