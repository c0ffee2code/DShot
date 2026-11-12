# ADR-008: On the Bench, the Main Pico Reports Facts and the PC Judges

**Status:** Accepted (run on the Pico and the bench on 2026-11-11; "Verification" lists what was and was not exercised)
**Date:** 2026-11-11
**Builds on:** [ADR-004](ADR-004-client-owned-command-loop.md) ("The library reports facts, not health")

## Context

A bench run involves three machines. The **main Pico** runs a scenario (`tests/bench/dshot_protocol/run_scenario.py`): it
arms the ESCs through `MotorGroup`, follows each motor's throttle profile and polls the telemetry. The **wingman**, a second
Pico 2, records every edge of the signal lines and the start line (`tests/bench/wingman/`), so every frame sent and every
reply received is on its recording (`signal_observation.bin`). The **PC** names the session, collects both records and computes
`summary.json` (`scripts/session_summary.py`), whose verdict is built from required proofs (BUG-008, amendment (2) of
2026-11-11).

Until this decision two judges graded the same run:

1. **The main Pico.** At the end of a completed run `check_decode_expect()` compared the scenario's `expect.min_crc_valid_pct`
   and `expect.min_median_erpm` with a sample: every `decode_every`-th capture per motor (50 in the permanent scenarios),
   decoded on the device, from `ARMED` onward. It wrote a `verdict` (`pass`, `fail: ...` or `pass (not judged: ...)`) into its
   record, printed `=== Test FAILED ===` and raised on a miss. A sample could be too small to support a percentage, so
   `DecodeTally.not_judged()` existed to say so.
2. **The PC.** `session_summary.py` judged the same two thresholds on every reply the wingman recorded, over the whole
   recording, which starts before arming.

The two judges had different inputs (a 1-in-50 sample against every reply) and different scopes (from `ARMED` against from
the start of the recording). On 2026-11-11 two runs (four bidirectional motors, bare shafts, AM32 2.22, a temporary scenario)
armed in 2636 ms and 2630 ms and spun up normally. The main Pico judged its thresholds on its sample from `ARMED` onward
(in the second run 34 decodes per motor, all CRC-valid) and passed both. The PC failed both at 97.7 % to 98.1 % CRC-valid replies against a threshold of 99 %. The
failing replies were confined to the first 0.44 s to 0.53 s of the recording: in the second run 407 to 495 bad-CRC replies per
line, each starting less than 10 us after our frame, and none after the throttle step at 2644 ms. A third run of the same day
showed no such activity and passed the PC's verdict. The session folders of these runs were not retained; the figures above are the
observations recorded on that date. The cause of the early activity is not known and is not part of this decision.

The disagreement was not a defect of either judge. It followed from having two of them.

## Decision

Decided 2026-11-11.

1. **The main Pico reports facts; the PC judges.**
   - The main Pico reports facts and insights from the run: the arming time, the per-motor decode tallies of its sampled
     captures, the poll counters, the scheduling counters and how the run ended. It judges no session.
   - The main Pico may host debug and safety assertions that **bail out** of a run early: arming not complete within the
     scenario's window plus a margin, a Core 1 error, the scenario's `max_gap_ms` or `min_record_rate_hz` missed, a motor
     commanded to spin that decodes no real eRPM for 2 s, the over-speed guard, an interrupt, or a command loop that did not
     stop before `disarm()`. A bail-out stops the motors as every run does (`fast_stop`) and is reported as a fact: the
     run's outcome is `aborted: <reason>`. It is not a verdict.
   - A run that reached its end has the outcome `completed`. The main Pico's console says `=== Run completed ===` or
     `=== Run aborted ===`, never pass or fail. An aborted run ends the process with a non-zero exit code, a completed run
     with zero, so a caller still sees a run that stopped early.
   - The verdict of a session (`pass`, `fail`, `unknown`) is made on the PC only, by `scripts/session_summary.py`. The PC has
     every reply on the wire, not a sample, and the computing capacity to judge as many dimensions as needed.
2. **The wingman is a permanent part of the DShot test bench.** Every bench run with an ESC is recorded by it, and no
   scenario can be run without it. A session whose recording is missing has not been judged: its verdict is `unknown`.
   (Amendment 2026-11-12: the option that ran a scenario without the wingman is removed. Motivation: the wingman is an
   integral part of the quality assurance of the DShot driver, and no bench test with an ESC is run without it. A device
   script on the Pico alone, with no ESC, is not such a test and is recorded only when asked.)
3. **The scenario's decode thresholds are judged from the throttle step onward.** `expect.min_crc_valid_pct` and
   `expect.min_median_erpm` are judged on the PC, per line, on the replies from the first frame carrying a throttle of 48 or
   more, which is what "the motors spun as specified" means. On the wire that frame is located as the first of three
   consecutive frames of ours with the same throttle of 48 or more (`observer_start_summary.find_step`, the step the spin-up figure
   is measured from), so a single frame garbled by an overlapping reply cannot move the boundary. Replies before the step
   (the arming phase) are counted and reported per line as facts (`before_step`: replies, replies with a bad CRC, replies
   starting less than 10 us after our frame) and printed as a note when any had a bad CRC or started early; they are not
   judged against these thresholds.
4. **No compatibility with older session formats.** The harness and the PC tools read the format the current scenario
   runner writes, and only that. Session folders written before this decision (with the outcome `failed`, a `verdict` field
   or the Pico's "not judged" wording) are not supported, and no code path was added for them; the baseline restarts on AM32
   2.22 and those runs were dropped.

This is the principle of ADR-004 applied one level up. There the library reports facts and the application decides on them;
on the bench the main Pico is the application of the library, and it in turn reports facts to the PC, which decides.

## Alternatives considered

- **Keep both judges and align their scopes** (the PC judging from `ARMED`, or the Pico decoding more often). Two judges
  with different inputs disagree whenever the sample misses what the full record holds; aligning scopes would remove one
  cause of disagreement and keep the other. Decoding every capture on the device costs milliseconds per capture on the core
  that sets throttles, which changes the command loop's frame spacing (BUG-011). Rejected.
- **The Pico as the only judge.** It sees one capture per motor at a time and decodes a sample; it cannot see the wire. Its
  verdict could not include the frame checks, the stop, or a reply it did not sample. Rejected.
- **Judge the thresholds over the whole recording** (the state before this decision). The arming phase is not part of "the
  motors spun as specified", and reply activity during arming would fail runs whose spin met the specification. Arming has a
  proof of its own (the arming time within `CLEAN_ARM_THRESHOLD_MS`). Rejected.
- **Judge from `ARMED`, as the Pico did.** `ARMED` is the main Pico's state, not an event on the wire; the wire shows the
  throttle step directly, and between `ARMED` and the step the motors are still commanded to 0. Rejected.

## Consequences

- `run_scenario.py` no longer grades the decode thresholds: `check_decode_expect()` and `decode_not_judged()` are removed,
  together with `DecodeTally.check()`, `DecodeTally.not_judged()` and `MIN_SAMPLES`, which had no other use once
  `scripts/analyze_bidir_capture_log.py` stopped judging thresholds too. The record on the SD card has no `verdict` field.
- The outcome in the Pico's record is `completed` or `aborted: <reason>`, the reason being one line. A bail-out of
  `run_scenario.py` itself (`BailOutError`) gives its message alone, in the form actual value, comparison, limit, with the
  motor and its commanded throttle where one is concerned (`motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms`,
  `motor 2 throttle=215: 91000 eRPM in 5 decodes in a row > limit=64500 eRPM`, `largest_gap=163.2ms > max_gap_ms=150`),
  and draws no conclusion from it: the conclusion is the PC's. An error raised by the command loop on Core 1 keeps its
  exception type in the reason, because there the type is the information. `scripts/run_scenarios.py` classifies attempts as before: `Outcome: completed` with the arming time decides
  clean or reboot, the arming-timeout message decides refused, and any other abort is an error.
- The Pico's sampled decodes remain, as facts. The 2 s spin bail-out (`SpinWatch`) needs them, and they let the PC check
  that the on-device decoder agrees with an independent one on the same words (`analyze_bidir_capture_log.py`, which now
  checks only that agreement and exits non-zero on a disagreement).
- `expect.max_gap_ms` and `expect.min_record_rate_hz` remain bail-outs on the Pico. They measure how well the main Pico's own
  polling keeps up with the captures (harness health), not the ESC.
- A session without a wire recording has the verdict `unknown`; the note names the reason (the wingman started but no
  `signal_observation.bin` arrived, or a device script run on the Pico alone). `scripts/run_session.py` always starts the wingman for a
  scenario and stops before the main Pico runs anything when the wingman is not connected or does not start recording; a
  scenario cannot be run without it. A device script run on unused pins with no ESC (CLAUDE.md, "Pico-only runs need no
  permission") is not a bench run with ESCs and is recorded only with `--observe`.
- `summary.json` carries per bidirectional line `step_ms`, `before_step` and `from_step` next to the whole-recording counts.
  A commanded line with no step on the wire has its thresholds `not shown`, with a note.
- Proof "run finished cleanly" fails with the Pico's reason when the outcome is `aborted: ...`, and fails on any other
  outcome that is not `completed` (for example `running`, a record whose end was never written).
- Compatibility paths for older formats that predate this decision are not changed by it and remain in the tree:
  `load_scenario(..., historic=True)` in `scripts/analyze_bidir_capture_log.py` (with the `historic` branches of
  `tests/bench/scenario.py`), and the labelling of a session without `update_interval_us` as recorded on the unpaced loop.
  Under decision 4 they are candidates for removal.

## Verification

- PC tests (`tests/pc/test_session_summary.py`, `test_analyze_bidir_capture_log.py`, `test_decode_tally.py`) cover the
  window split, a synthetic line whose arming phase has bad-CRC and early replies (the session passes and the note names
  them), a line held at throttle 0 (no arming-phase block), the aborted outcome including an arming refusal, and the
  reasons for a missing recording.
- Bench runs of 2026-11-11 (AM32 2.22, four bidirectional motors on GPIO 6-9, DSHOT600, throttle 0 to 215 held 20 s, bare
  shafts, the wingman recording every run; none of them KPI evidence):
  - Five runs with the runner of this decision (reports completed or aborted, judges nothing; the PC verdict judges the
    thresholds from the throttle step): all five armed (2659, 2113, 2112, 2112, 2112 ms) and all five session verdicts pass
    with six proofs shown. The Pico console ended `Outcome: completed` / `=== Run completed ===`.
  - Arming phase on a real recording: in the run that armed in 2659 ms, channel 3 had 2151 replies before the throttle step
    at 2669.5 ms, 520 of them with a bad CRC and an early start. The session reported this as a note and did not judge it,
    as decided. In earlier runs of the same day, on the previous grading (CRC threshold over the whole recording), two runs
    that armed in 2636 and 2630 ms failed at 97.7 to 98.1 % CRC-valid; the wire showed arming-phase replies of the same kind (early start, bad CRC).
  - Abort with the ESC power switched off a few seconds into the hold (two runs, the second with the bail-out wording that
    states actual against limit): the Pico stopped the run 2004 and 2007 ms after the last real eRPM with outcome aborted
    (`aborted: motor 0 throttle=215: no real eRPM for 2007ms > limit=2000ms`, `=== Run aborted ===`); the batch line read `outcome=error`, and the session verdict was fail (the PC found 1879 to 1891 frames
    without a reply after a real eRPM on each line).
  - Refused arming with the ESCs unpowered from the start: `aborted: arming did not complete within 18000ms; per motor
    (replying_for_ms, last_reply_ms_ago), None = no reply: [None, None, None, None]`; batch `verdict=fail outcome=refused`;
    session verdict fail. The wire recording was valid and showed complete frames of ours on all four lines.
- Refinement after those runs (PC tests only): the "armed within the bound" proof reads failed, not "not shown", when the Pico
  reports that arming did not complete. The phrase is `ARMING_REFUSED_TEXT` in `scripts/arming_bound.py`, read by
  `scripts/run_scenarios.py` and `scripts/session_summary.py`; it was checked against the retained record of the refused run.
- Not exercised on the bench: the process exit status of an aborted run (only the batch runner's classification was seen); a session whose recording did not arrive (verdict "unknown"); the bail-outs for `max_gap_ms`,
  `min_record_rate_hz`, over-speed and a Core 1 error; Ctrl-C; a command loop that does not stop before `disarm()`.
- Whether the arming-phase activity observed on 2026-11-11 recurs, and its cause, are open; with this decision it is
  reported, not judged.

## Amendment 2026-11-12: the wingman's recording role is named the observer

**Decision.** The wingman's recording role, called "the spy" until now, is named "the observer". Its recording is
`signal_observation.bin` (formerly `spy.bin`), written under that name by `wingman_run.py` and read by `scripts/session_summary.py`,
`scripts/session_timeline.py` and `esc_mock_fixture.py`; the name is held in one place, `OBSERVATION_FILE` in
`tests/bench/wingman/tools/observer_edge_log.py`. The command-line flag `--spy` is `--observe`, the keys `"spy"` and `"spy_end"` in
`session.json` and `summary.json` are `"observer"` and `"observer_end"`, the board file `spy.py` is `observer.py`, the PC tools `spy_*.py` are
`observer_*.py`, and `SPY.md` is `OBSERVER.md`. The ESC mock keeps its name.

**Motivation.** A wingman does not spy: it observes what its partner does and reports it. The word also says what the role does, in line with
decision 1: it reports facts, the PC judges.

**Consequences.** Pure rename, no change of behaviour. No alias and no reader of the old names is kept (decision 4): a session folder
that holds `spy.bin` or a `session.json` with the key `"spy"` is not read. Records dated before 2026-11-12 keep the old word.

## Amendment 2026-11-12 (2): a session that did not start is a failed session

**Decision.** When the wingman does not start (not connected, never reports that it is recording, cannot serve the scenario) or the main
Pico does not start (the upload to it failed, or a scenario's console never shows its first line), the session is failed and still ends with
the standard output: `scripts/run_session.py` writes the fact to `session.json` as `start_failure` (`who`: `wingman` or `main_pico`,
`message`: why), puts the explaining text in `run.log`, and `scripts/session_summary.py` fails the proof "run finished cleanly" with
`the wingman did not start: <message>` or `the main Pico did not start: <message>`; the other proofs are `not shown`. The verdict is `fail`
and the command line exits 1. When the wingman does not start, nothing is uploaded to or run on the main Pico.

**Motivation.** A wingman that was not connected stopped `run_session.py` with a message after the session folder existed, leaving a folder
with `session.json` only and no verdict; a main Pico that did not start could read `unknown`, as if the run had started and its record
had gone missing. A run that never started is a failure of the session, not an unjudged run, and every attempted session has a
`summary.json` that says what happened.

**Consequences.**

- `session_summary.py` stays a function of the folder: it reads `start_failure` from `session.json`, so the verdict is the same when it is
  re-run. For a session that did not start it reads neither the Pico's record nor `signal_observation.bin`.
- A main Pico that started (its console shows `=== Scenario Capture ===`) but left no record is not a start failure; it keeps the verdict
  rules of decision 3.
- `run_scenarios.py`: a wingman start failure is an `error` attempt and is not retried; a failed upload keeps the classification
  `deploy_failed`, with its single retry; the closing `SESSION_VERDICTS` counts both as `fail` (before, the first stopped the attempt with
  no session summary and counted as `no_session`).
- Not exercised on the bench: the wingman unplugged, and an upload to an unconnected main Pico (PC tests use stand-ins).
