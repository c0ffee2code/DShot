Run `python scripts/run_scenarios.py --scenario <path> [--scenario <path> ...] --rounds <N> --load "<what is on the shafts>" [--rotate]` and report the tally.

Runs one or more scenarios repeatedly on the bench. Each attempt prints one `RUN` line of
`key=value` fields: `verdict=pass|fail|excluded` (pass = armed within 3.0 s of arm(), the
stable-arming KPI's duration bound, target >=99%; fail = armed late after an ESC reboot, or never
armed; error and deploy failures are excluded), `outcome=` (clean, reboot, refused, error,
deploy_failed), `armed_ms=` and `margin_ms=` (room left under the bound, for a pass), `session=` (the attempt's own folder
captures/<session>/) and `session_verdict=` (that session's summary.json verdict, a wider judgement than arming). The series
summary prints `RESULT` (pass/fail counts and percentage), `ARMING_TIME` (min, median, max over the
arms that passed, plus how many were within 500 ms of the limit) and `SCHEDULE` lines. Report the
lines, not a paraphrase; they are meant to be grepped.

Multiple `--scenario` values are interleaved round by round (all scenarios
once, then all again, ...), not run in blocks.

A deploy failure is retried once automatically and never counted as either
outcome - see `scripts/run_scenarios.py`'s own docstring for exactly how
each outcome is classified.

Every attempt is a full session (`scripts/run_session.py`): its own folder with the spy's recording
and a `summary.json`, pulled from the SD card when the attempt ends. A batch of 20 leaves 20 session folders.

Add `--rotate` to cycle the `--scenario` order by one position each round,
so no scenario is always first or always last in a round - without it, a
fixed order confounds "which scenario" with "which position" and "how long
since the batch started."

Example, interleaving the two four-motor scenarios for 10 rounds:
```
python scripts/run_scenarios.py \
  --scenario tests/bench/dshot_protocol/scenarios/four_channel_bidirectional_600.json \
  --scenario tests/bench/dshot_protocol/scenarios/four_channel_unidirectional_600.json \
  --rounds 10 \
  --load "Gemfan D2.2 3-blade, 1-1.25 g each"
```

`--load` is required (`"none"` for bare shafts): it is written into every session's `session.json`.

There is no arming-only scenario any more (`two_channel_arming_600.json`, removed 2026-10-17; it is in git
history). Every scenario run prints `Armed after Nms` and the batch runner judges it against the 3.0 s bound,
so arming reliability is read from any batch; for bidirectional motors use `four_channel_bidirectional_600.json` (only bidirectional runs test the reply-gated arming the 99% promise is about),
which spins them.

Every attempt's session can be read offline with `scripts/analyze_bidir_capture_log.py` or `scripts/session_summary.py`.
Note that a session's capture begins at ARMED; the arming phase itself is not recorded.
