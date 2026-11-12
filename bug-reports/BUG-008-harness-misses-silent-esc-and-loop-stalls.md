# BUG-008: Harness health checks cannot see a silent ESC, a Core 1 stall, or "commanded but not running"

**Status:** PARTLY FIXED 2026-10-13 (items 2 and 3 done; healthy path bench-verified on two scenarios, failing path unit-tested only; items 1 and 5 still open - see "Update 2026-10-13"; since 2026-11-11 they are application work in the harness, not driver work - see "Update 2026-11-11")
**Severity:** Medium (diagnostics). BUG-002-type failures pass the harness's own checks and are
found only by reading logs afterwards.
**Component:** `tests/harness/run_scenario.py`, `tests/harness/scenarios/*.json`, `tests/harness/scenario.py`
**Found:** 2026-09-27, code review

## Summary

1. **The reply failsafe cannot trip with the frame receiver.**
   - `check_reply_failsafe()` (`run_scenario.py`) trips only if every captured record is
     all-zero words.
   - When the ESC does not reply, `dshot_bidir_rx_frame` falls through to our own next frame and
     pushes it, and that capture is non-zero. So a completely silent ESC keeps
     `total_nonzero_records` climbing and the failsafe never fires.
   - It is also any-motor, not per-motor: one live motor masks a silent one.
   - *Update 2026-11-11:* the check is removed (see "Done 2026-11-11" under the fix plan); silence is judged on the PC from the wire recording.
2. **Core 1 stalls are measured but never judged.**
   - `measured()` records the longest gap between `update()` calls, and the run prints it and
     stores it as `max_loop_gap_us` in the session summary.
   - But **no scenario sets `expect.max_loop_gap_ms`**, so `check_expect()` never checks it.
   - A gap of ≥0.5 s makes an armed AM32 reset itself
     ([`main.c#L1992-L2017`][am32-timeouts]), and the result looks exactly like BUG-002.
3. **"Commanded but not running" is only caught indirectly**, through `min_median_erpm`. A motor
   whose replies are all `not_running` (`0xFFF`) while its commanded throttle is ≥48 is BUG-002's
   exact signature. It deserves its own failure message, and it should fail as soon as it happens
   rather than at the end.

## Update 2026-10-13

- **Item 3 (not-running check): done.** `decode_tally.SpinWatch` replaces the permanent
  `real_spin_seen` latch. A bidirectional motor commanded at throttle 48 or above must decode one
  real eRPM every `NOT_RUNNING_LIMIT_MS` (2000 ms; the existing, bench-justified spin-up grace, not
  the 3 s first suggested below). It is a continuous streak, so a single glitch frame (BUG-009)
  restarts the clock once and cannot defeat the check for the rest of the run.
- **Item 2 (loop gap): done in a different form.** The per-tick loop-gap measurement this item
  assumed was retired with the rest of the per-tick instrumentation (BUG-002, 2026-10-06), so there
  is no `max_loop_gap_us` to judge. The check that exists is `expect.max_gap_ms`, the largest gap
  between records; `scenario.py` now requires it for every scenario with a bidirectional motor
  (the two unidirectional scenarios produce no records and are exempt).
- **Items 1 and 5 remain open:** the per-motor, echo-aware silent-ESC failsafe, and the
  reboot-after-`ARMED` flag.

## Update 2026-11-11

- **Items 1 and 5 are application work, not driver work.** The owner decided on 2026-11-11 that
  the library reports facts and the application decides on them and adds the resilience it needs
  (ADR-004, the 2026-11-11 note under "The library reports facts, not health"). The driver will
  not judge whether an ESC is silent or has rebooted after `ARMED`. For this bench the
  application is the test harness (`run_scenario.py`), so both checks belong there.
- **An independent check of the 227 figure (item 1).** A PC model run on 2026-11-11 fed the
  receiver program, as transcribed in `scripts/simulate_frame_receiver.py`'s `run()`, with the
  waveform `dshot_bidir_tx` sends for each throttle, at 64 receiver clock phases. At DSHOT600,
  218 of the 2,001 throttle values outside the command range (0 and 48 to 2047) give an
  own-frame capture that passes the CRC at some phase, and so do 9 of the values 1 to 47: 227 of
  2,048, the same count as item 1. At DSHOT300 the count is 197 of 2,001 (7 more in 1 to 47).
  Within today's bench ceiling (`MotorGroup` sends only 0 and 48 to 250), 13 values pass at
  DSHOT600 and 15 at DSHOT300, all between 60 and 122. Throttles 67, 72, 93
  and 98 are among them at both speeds. No value passes at every phase (at most 28 of 64), so
  whether a given frame passes depends on the clock phase as well as the throttle. For the four
  throttles above, every passing phase gave the same word. None of the passing captures equals
  AM32's not-running word. They decode to eRPM values from about 2,000 to 170,000 at DSHOT600 (to
  937,500 at DSHOT300), most between about 2,000 and 2,750. Throttles 0, 100 and 215 do not pass.
  These are model figures, not a bench measurement.
- **The echo model named in item 1 is gone.** `scripts/classify_reply_timeline.py` and its
  `EchoModel` were deleted in commit `18bf03a`; its successor, `scripts/session_timeline.py`,
  has no echo model. The receiver model's `run()` in `scripts/simulate_frame_receiver.py` can
  compute the echo words, as the check above did.
- **Item 1's `ready_last_us` cannot serve after `ARMED`.** BUG-003's evidence is now
  `ready_last_ms` in `MotorGroup`, and `update()` updates it only while `ARMING`.
- **Item 5's "all-zero words" no longer describes a reboot.** Since BUG-004 (2026-09-27) the
  receiver waits for the line to read high before it looks for a falling edge, so a line that
  reads low throughout produces no capture at all rather than all-zero words. If a reboot after
  `ARMED` holds the line low, as the reboots in BUG-002's captures did, it would show as captures
  that stop arriving (the token stops stepping), which the harness can time itself. This has not
  been checked on the bench.
- **Item 4 no longer exists.** `MotorGroup.publish_while_arming` was removed on 2026-10-17
  (ADR-005), and the harness's `arming.bin` was retired on 2026-10-06 (BUG-002).
- **Paths.** `tests/harness/` and `tests/unit/` no longer exist (tests restructured in commit
  `2cbde0c`). `run_scenario.py` is now `tests/bench/dshot_protocol/run_scenario.py`, its scenarios
  are in `tests/bench/dshot_protocol/scenarios/`, `scenario.py` is `tests/bench/scenario.py`, and
  the unit tests are in `tests/pc/`.

## Amendment 2026-11-11: the harness fails a run unless its test specification was met

**Decision.** Judging a run is the test harness's responsibility. The harness analyses every run
thoroughly and marks it failed unless the test specification was met: arming succeeded, every
motor spun as the scenario specifies, and the run finished cleanly.

**Motivation.** The library reports facts and judges nothing (ADR-004, amendment of 2026-11-11).
On the bench the harness is the application, so the judgement belongs to it. A run that passes
by default hides exactly the failures this report is about.

**Where the judgement is made today** (read from the code on 2026-11-11, not from a bench run):

| Case | On the Pico during the run (`run_scenario.py`) | On the PC after the run, from the spy's wire recording (`scripts/session_summary.py`) |
|---|---|---|
| The ESC never replies | Arming does not complete; the run fails and names the motor (`arming_status()`) | - |
| The ESC resets after `ARMED`, motor commanded to spin | `SpinWatch`: no real eRPM for 2 s stops the run | Fails: "not running" for `LOST_SPIN_FRAMES` (50) or more frames after a real eRPM |
| The ESC goes silent after `ARMED`, motor commanded to spin | `SpinWatch`, as above (limit below) | Fails since this amendment: `LOST_REPLY_FRAMES` (50) or more frames without a reply after a real eRPM. Before, a warning only |
| The ESC goes silent or resets on a motor held at throttle 0 | Not checked | A warning only (reply coverage) |

**Consequences.**

- Items 1 and 5 of this report are covered as the table shows. Item 5 has no check named "reset";
  a reset shows as "not running" while commanded and fails on both sides.
- Added with this amendment: `unanswered_after_first_real_erpm` per line in `summary.json`, and
  the failure reason for it. On the two sessions recorded on 2026-11-10 (AM32 2.22, four motors,
  throttle 215, about 19,170 real-eRPM replies per motor) the count is 0 on every line, so the
  threshold of 50 is far from a healthy run. The threshold itself is a choice, not a measurement.
- Known limit, not fixed: `SpinWatch` counts a CRC-valid decode that is not "not running" as a
  spinning motor. At the throttles where the receiver's capture of our own frame can pass the CRC
  (update above: 13 values between 60 and 122 at DSHOT600 within today's ceiling, in the model) a
  silent ESC can therefore restart its 2 s clock. The spy's recording is not affected, so the
  PC-side verdict still fails the run. The Pico does the minimum during a motor run (CLAUDE.md,
  "Rules for bench runs"), so no further check is added there.
- Done 2026-11-11: `check_reply_failsafe()`, `REPLY_FAILSAFE_GRACE_MS` and the counters that
  existed only for it (`total_nonzero_records`, the `nonzero=` column of the status lines, the
  "Passes with a real (non-all-zero) reply" summary line) are removed from `run_scenario.py`.
  Reason: the check fired only when no motor had a capture with a non-zero frame since `ARMED`.
  `ARMED` already requires replies from every bidirectional ESC (`MotorGroup`), and from then on
  the frame receiver pushes one word per frame sent, so a silent ESC yields the driver's own next
  frame, which is non-zero; it could fire only if no capture at all were published after `ARMED`.
  Silence after `ARMED` while commanded to spin is judged by `SpinWatch` on the Pico and by the
  spy's recording on the PC (table above). Bench runs of 2026-11-11 without the check (15 runs, AM32 2.22, four bidirectional
  motors, throttle 215 held 20 s): 14 armed; with the ESC
  power switched off during the hold, and with the ESCs unpowered from the start, the run ended aborted and the session verdict
  failed.
- Done 2026-11-11: the bail-out reasons of `run_scenario.py` state facts only, as the measured value against its limit
  (`motor 0 throttle=215: no real eRPM for 2004ms > limit=2000ms`; the over-speed reason likewise), without a conclusion or
  a pointer to a bug report, and without the `RuntimeError: ` prefix; the check formerly named
  `check_stuck_at_rest_failsafe` is `check_no_real_erpm`. Bench runs of 2026-11-11: with the ESC power switched off a few seconds
  into the hold the Pico printed `Outcome: aborted: motor 0 throttle=215: no real eRPM for 2007ms > limit=2000ms` and
  `=== Run aborted ===`, the traceback line read `BailOutError: ...`, the batch line read `outcome=error` and the session verdict
  was fail; with the ESCs unpowered from the start the reason was `aborted: arming did not complete within 18000ms; per motor
  (replying_for_ms, last_reply_ms_ago), None = no reply: [None, None, None, None]` and the batch line `verdict=fail outcome=refused`.
  Not exercised on the bench: the bail-outs for `max_gap_ms`, `min_record_rate_hz`, over-speed and a Core 1 error.
- Done 2026-11-11: `scripts/session_summary.py` gives a run with no wire recording the verdict
  "unknown" instead of "pass", with a note saying the motors' behaviour was not judged. A failed
  device outcome stays "fail" with or without a recording, a recording that exists but is
  invalid stays a failure, and a scenario with unidirectional motors only passes on a valid
  recording without replies. "unknown" is not "pass" for `run_scenarios.py` (it prints the value
  as `session_verdict=` and counts arming separately) and for `run_session.py` (the exit code is
  `run_test.py`'s, not the verdict's).

## Amendment 2026-11-11 (2): the PC verdict is built from required proofs

**Decision.** `scripts/session_summary.py` builds a session's verdict from an explicit list of
required proofs. Each proof has a name, a status (`shown`, `failed`, `not shown`, `not applicable`)
and a one-line detail, and is stored in `summary.json` under `proofs`. The verdict is `fail` if any
proof failed, `pass` only if every applicable proof is `shown`, and `unknown` otherwise. `reasons`
(why a run failed), `notes` (what was not shown) and the new `limitations` list are kept alongside.
The proofs:

1. The run finished cleanly: the Pico's outcome is `completed` (a command loop that did not stop
   before `disarm()` makes `run_scenario.py` report `failed: Core 1 loop did not stop`) and its own
   verdict is not a failure.
2. It armed: `run.log` has an arming time and it is below `CLEAN_ARM_THRESHOLD_MS` (now in
   `scripts/arming_bound.py`, imported by `run_scenarios.py` and `session_summary.py`; one copy).
3. The wire recording exists and is complete: `spy.bin` is valid and every motor pin of the
   scenario is a recorded line with at least one complete frame of ours.
4. Our frames were correct: no AM32 check has status FAIL on any line, and no line shows stray
   activity (`SLOW_GAPS_FLAGGED`).
5. Every bidirectional motor the scenario commands to spin did spin: it reached a real eRPM, kept
   reporting one (`LOST_SPIN_FRAMES`), kept answering (`LOST_REPLY_FRAMES`) and met the scenario's
   `min_crc_valid_pct` and `min_median_erpm` where set. A threshold that is set but cannot be
   judged for want of replies is `not shown`. A motor the scenario commands to spin whose throttle
   never reaches 48 on the wire fails.
6. The motors were stopped at the end: the last `STOP_TAIL_FRAMES` (5) frames of every driven line
   carry throttle 0.

`run_scenarios.py` keeps its per-run arming verdict and its keys unchanged and prints a closing
`SESSION_VERDICTS` line with the batch totals of the session verdicts.

**Motivation.** The previous verdict collected reasons to fail and reported `pass` when it found
none, so a missing piece of evidence passed silently. Judging a run is the harness's duty (amendment
of 2026-11-11 above): a pass has to be shown.

**Consequences.**

- Data behind proof 6 (the two sessions of 2026-11-10, four bidirectional motors, throttle 215):
  on every line the last 19 frames carry throttle 0 and the ESC still replies with a real eRPM
  that decays (about 46,000 at the first zero frame, 23,000 to 31,000 at the last frame). No line
  shows "not running" after the stop. The ESC reporting "not running" after the stop is therefore
  not required: the wire does not show it within the recorded tail.
- A bidirectional motor held at throttle 0 for the whole run has no spin proof; its replies are
  not required (arming already required them; reply coverage stays an AM32 check warning).
- Accepted constraint: a unidirectional motor sends no reply, so that it spun cannot be shown from
  the wire. It does not block a pass; every such run lists it under `limitations`, and visual
  confirmation by the operator is the only check until optical RPM measurement exists.
- A session played with the ESC mock keeps its standing (not evidence about an ESC, a motor or
  arming): proofs 2 and 5 are `not applicable`, the other proofs are judged, and a note says so. A
  device script run through `run_session.py` has no scenario outcome: proof 1 is `not applicable`,
  proof 2 is judged only when `run.log` has an arming time, and proof 5 needs a commanded line.
- Still unproven by a pass: that a unidirectional motor spun; that a motor reached the stop in
  fact (the wire shows the stop was commanded, not that the rotor stopped); the physical state of
  any motor beyond its eRPM replies; behaviour after the last recorded frame; the thresholds
  `STOP_TAIL_FRAMES`, `LOST_SPIN_FRAMES`, `LOST_REPLY_FRAMES` and `SLOW_GAPS_FLAGGED` are choices,
  not measurements. `run_session.py`'s exit code is still `run_test.py`'s, not the verdict's.
- Bench runs of 2026-11-11 (AM32 2.22, four bidirectional motors, DSHOT600, throttle 215 held 20 s, bare shafts, the wingman
  recording every run): without the wingman's recording ("unknown") not exercised; with a valid recording the verdicts of the
  runs above followed the proofs (pass with six proofs shown for completed runs, fail for the aborted and refused runs).
- Refinement 2026-11-11 (PC tests only): the "armed within the bound" proof reads failed, not "not shown", when the Pico reports
  that arming did not complete (`ARMING_REFUSED_TEXT` in `scripts/arming_bound.py`, read by `run_scenarios.py` and
  `session_summary.py`); checked against the retained record of the refused run.

## Amendment 2026-11-11 (3): a mock session is never a pass; the exit code follows the verdict

**Decision.** A session whose replies were played by the ESC mock never has the verdict `pass`: its
proofs "armed within the bound" and "motors spun as specified" are `not shown` (they were
`not applicable`), so a mock session with no failure reads `unknown`, and one with a failed proof
still reads `fail`. `scripts/run_session.py`, run as a command, exits 0 only when the session
verdict is `pass`, 1 for `fail` or a non-zero `run_test.py`, and 2 for `unknown`.

**Motivation.** A mock session is not evidence about an ESC, a motor or arming, and the rule of
amendment (2) is that a pass has to be shown. The exit code is what a script or a person running
the command sees first; it must not report success for a run that was not shown to meet its
specification.

**Consequences.**

- This replaces the statement in amendment (2) that proofs 2 and 5 are `not applicable` for a mock
  session; its other proofs are judged as before. The function `run_session()` still returns
  `run_test.py`'s code and the summary, so `run_scenarios.py` and its per-run arming `verdict=` are
  unchanged. Its `session_verdict=` shows `unknown` for a mock session.
- `run_session.py`'s exit code is no longer `run_test.py`'s alone (the "Done 2026-11-11" entry above
  that says so describes the state before this amendment).

## Amendment 2026-11-11 (4): the main Pico reports facts, the PC is the only judge

**Decision.** Recorded in ADR-008. The main Pico (`run_scenario.py`) reports facts and may bail out
of a run; a bail-out is reported as the outcome `aborted: <reason>`, a run that reached its end as
`completed`, and neither is a verdict. The session verdict is made only by
`scripts/session_summary.py`, from the wingman's recording. The wingman is a permanent part of the
bench; a session without its recording is `unknown`. The scenario's `min_crc_valid_pct` and
`min_median_erpm` are judged per line on the replies from the throttle step onward (the first of
three equal frames of ours with a throttle of 48 or more); replies before it are reported as facts.
Only the session format the current runner writes is read.

**Motivation.** The Pico judged the two decode thresholds on a 1-in-50 sample from `ARMED` onward;
the PC judged them on every reply over the whole recording. Observed on 2026-11-11 (four
bidirectional motors, bare shafts, AM32 2.22, a temporary scenario): two runs armed in 2636 ms and
2630 ms and spun up normally; the Pico passed both, the PC failed both at 97.7 % to 98.1 %
CRC-valid replies (threshold 99 %), because of bad-CRC replies confined to the first 0.44 s to
0.53 s of the recording (in the second run 407 to 495 per line, each starting less than 10 us after
our frame, none after the throttle step at 2644 ms). The session folders were not retained. Two
judges with different inputs and scopes disagreed; amendment (1) already placed the judgement in
the harness, and this amendment places it on the PC alone.

**Consequences.**

- Proof 1 ("run finished cleanly") is shown for the outcome `completed` and fails with the Pico's
  reason for `aborted: ...`. A command loop that did not stop before `disarm()` is now reported as
  `aborted: Core 1 loop did not stop before disarm()` (it was `failed: Core 1 loop did not stop`)
  and, like every abort, ends the process non-zero (before, that case exited zero). The Pico's
  console banner is `=== Run completed ===` or `=== Run aborted ===`.
- Removed: `check_decode_expect()` and `decode_not_judged()` in `run_scenario.py`, the `verdict`
  field of the Pico's record, `DecodeTally.check()`, `DecodeTally.not_judged()` and `MIN_SAMPLES`,
  the threshold checks of `scripts/analyze_bidir_capture_log.py` (it now checks only that the
  on-device decoder agrees with the PC's on the logged words), and `session_summary.py`'s reading
  of the Pico's verdict.
- Proof 5 judges the decode thresholds on `from_step`; `summary.json` carries `step_ms`,
  `before_step` (replies, bad CRC, early starts) and `from_step` per bidirectional line, and a
  note lists a line whose arming phase had bad-CRC or early replies. A commanded line with no
  step on the wire has its thresholds `not shown`.
- A session without `spy.bin` names the reason in its note: run with the spy off, or the wingman
  started but no `spy.bin` arrived. `run_session.py` already started the wingman by default for a
  scenario and stopped before the main Pico ran anything when the wingman was not connected.
- The gap and rate thresholds (`max_gap_ms`, `min_record_rate_hz`) stay as bail-outs on the Pico;
  they measure the main Pico's own polling, not the ESC.
- Bench runs of 2026-11-11 (five runs with the runner that reports and does not judge, plus an aborted run with the ESC power
  switched off and a refused arming with the ESCs unpowered): all five completed runs armed (2659, 2113, 2112, 2112, 2112 ms),
  printed `Outcome: completed` / `=== Run completed ===` and had a passing session verdict with six proofs shown; the aborted
  and refused runs printed `=== Run aborted ===` and had failing verdicts. Not exercised: a run without the wingman's recording,
  Ctrl-C, a loop that does not stop before `disarm()`.

## Fix plan

1. **Per-motor, CRC-based failsafe.** Replace the non-zero-words test with "each bidirectional
   motor has at least one CRC-valid reply by `REPLY_FAILSAFE_GRACE_MS`". Use the sampled decodes
   the tallies already make. With BUG-003 in place, use its `ready_last_us` instead.
   CRC alone is not enough. At some throttles (227 of 2,048 at DSHOT600, e.g. 67, 72, 93, 98),
   the receiver's capture of our *own* frame passes CRC when the ESC is silent. Count a reply only
   if it is also not one of those echo words (see `scripts/classify_reply_timeline.py`'s
   `EchoModel`, which computes them directly - no need to go via BUG-002's history for this). On
   the device, a per-throttle lookup of the few echo words is cheap.
2. **`max_loop_gap_ms` in every scenario** (e.g. 50 ms). Make `scenario.py` require it, like the
   other fail-fast fields. Also check the stored `max_loop_gap_us` of the existing BUG-002
   sessions; that data is already on disk.
3. **Not-running check.** For each bidirectional motor, fail when its sampled decodes have been
   `not_running` continuously for more than N seconds while its profile throttle is ≥48. Suggested
   N = 3 s, because a start can take a moment. The message should name BUG-002.
4. **Keep arming-phase captures in the log.** Done 2026-09-27: `MotorGroup.publish_while_arming`
   plus the harness's `arming.bin`, shown at negative times by `classify_reply_timeline.py`
   (BUG-002, R6).
5. **Flag a reboot after `ARMED`.** Fail when a bidirectional motor's words are all-zero (the
   line held low, AM32's startup tune) for more than ~300 ms. That is a reset, and one
   BUG-003's gate cannot catch once `ARMED` has passed.

## Confirmed instance, 2026-10-10 (BUG-009): a single glitch frame defeats the current check

*As of 2026-10-13 the permanent latch described here has been replaced (see "Update 2026-10-13" above); this section records the instance that motivated it.*

`run_scenario.py`'s `check_stuck_at_rest_failsafe` (the check that exists today, ahead of this
file's fix plan) sets `real_spin_seen[index] = True` the first time a decoded capture has `crc_ok`
true and `not_running` false, and **never resets it**. BUG-009 found a reproducible, CRC-valid,
non-sentinel glitch value (`data12=2`, decoding to a physically absurd `erpm=30,000,000`) - one
occurrence permanently marks that motor "confirmed spinning" for the rest of the run, even if it
then genuinely never spins (or stops) again. This is a confirmed, concrete instance of exactly the
gap this file already describes in general terms.

Note for whoever implements item 3's fix: a *continuous*-`not_running`-for-N-seconds check, unlike
the current permanent latch, is not defeated the same way - one glitch frame resets that streak, but
it starts counting again immediately and still fires N seconds later if the motor is truly stuck.
The latch's bug is permanence, not the one-glitch trigger itself; item 3's design already avoids it.

## Verification

- Unit (`tests/unit/test_scenario.py`, `test_decode_tally.py`):
  - a silent ESC, modelled as non-zero CRC-failing words only, trips the failsafe per motor;
  - a scenario without `max_loop_gap_ms` fails to load;
  - a tally of `not_running` under throttle ≥48 fails after N seconds.
- Bench: a BUG-002 run now ends with the not-running failure, instead of passing its live checks.

## Related

- BUG-002
- BUG-003
- BUG-009 - confirmed instance of the gap this file's item 3 (and its proposed fix) has.

[am32-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017
