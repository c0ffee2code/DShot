# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status:** OPEN — narrowed 2026-09-27: the failing ESC resets during our arming window and
comes back after ARMED, under non-zero throttle (see "Analysis of R3"); why it resets is open
**Severity:** Medium — intermittent, the ESC and telemetry link both stay healthy and the ESC
recovers normally afterward, but the motor silently fails to do the one thing it's told to do.
**Component:** unclear — could be driver timing, ESC-side arming state, or something environmental;
see "What's been ruled out" below for what it is *not*.

## Summary

A bidirectional motor occasionally replies with CRC-valid telemetry and never actually spins — the
decoded eRPM sits at 917 (payload `0xFFF`), AM32's fixed "motor not running" sentinel, for the rest
of the run. Per `specification/AM32_SOURCE_VERIFICATION.md` (findings 1-2), AM32 sends replies -
and this exact sentinel - whether the ESC is armed or disarmed, so a CRC-valid reply does **not**
mean the ESC ever armed; only a non-sentinel eRPM would show that. The same scenario, run again
immediately after with no other change, has spun cleanly every time this has been observed. Not
reproduced on demand; only ever seen as a sporadic result within an otherwise-passing regression
run.

## Symptom

1. Run a scenario with a bidirectional motor commanded to a real throttle (observed specifically
   on `two_channel_divergent_600`, DSHOT600, both channels bidirectional).
2. The scenario completes; the affected motor's decoded telemetry is mostly or entirely
   CRC-valid, but its median eRPM is 917 (at-rest) instead of a value consistent with the
   commanded throttle.
3. Visually/audibly: the motor does not spin. The ESC does not report any other error state.
4. After `disarm()`, the ESC recovers normally (plays its idle tune) regardless of whether the
   motor spun during the run — this part is unaffected and not in question (see BUG-001).
5. Re-running the identical scenario immediately after, with no other change, has spun the motor
   cleanly every time this has been tried.

## Evidence gathered

**2026-09-20 (pre-dates BUG-001's fix):** `two_channel_divergent_600` batch of runs — run 1:
motor 0 median 917 with 15 invalid decodes; run 2: both motors 917 with 35 invalid decodes on
motor 2; run 3: fully clean. Cause not established at the time; noted as something the user had
seen intermittently before this specific investigation even started.

**2026-09-25 (post BUG-001's fix, same scenario):** One run: motor 0 (channel 1) — 658 decoded,
643 CRC-valid (97.7%), **0 CRC failures, but 15 invalid decodes** (captures that couldn't even
be parsed as a complete frame, a different failure mode than a CRC mismatch), median eRPM 917.
Motor 2 (channel 3) spun normally in the same run (665/665 CRC-valid, median 34,562 eRPM). User
confirmed by direct observation: motor 0 did not spin; the ESC still recovered and played its
idle tune normally after `disarm()`. An immediate repeat of the identical scenario spun both
motors cleanly (645/645 CRC-valid each, both medians consistent with their commanded throttles).

**Pattern across both instances:** every no-spin observation has come with a nonzero count of
*invalid* decodes (frames that failed to parse, not frames that parsed but failed CRC) mixed
into an otherwise-CRC-valid run. Every clean run recorded 0 invalid decodes. This has not been
tested as a deliberate hypothesis — it's a correlation noticed across two independent sessions'
data, not yet chased.

**Channel:** seen on channel 1 (this session) and previously on both channel 1 and channel 3
(2026-09-20 batch) — not obviously specific to one channel or one PIO block.

**2026-09-26 (frame receiver, `two_channel_gc_600`, three consecutive runs):** run 1: both motors
stuck at 917, with CRC failures/invalid decodes mixed in (motor 0: 222/239 CRC-valid, 3 invalid;
motor 2: 222/239, 1 invalid) - matches the earlier invalid-decode correlation. Run 2 (immediate
retry): motor 0 spun cleanly (246/246 CRC-valid, median 21,490); motor 2 stuck at 917 with **0
CRC failures and 0 invalid decodes** - perfectly clean telemetry, motor still not spinning. Run 3
(immediate retry): both motors spun cleanly (246/246 each, medians 21,368 and 22,255). This is the
first time the symptom has been seen on the frame receiver, an entirely different capture/decode
pipeline from every prior instance (all previously on the sample receiver) - the same failure mode
surviving a full receiver rewrite is evidence against a receiver-implementation-specific cause and
for something at the ESC or arming-timing level, consistent with this report's existing leads. It
also weakens the invalid-decode correlation as a reliable indicator: run 2's clean-telemetry,
no-spin case had no invalid decodes at all.

**2026-09-26 (frame receiver, `two_channel_divergent_600`, one run):** motor 0 (channel 1,
commanded to ramp 60->200->300): offline decode of the full session gives 14,827/15,238 CRC-valid
(97.3%), median eRPM 917, **longest CRC-fail streak 25, largest single-motor gap 637.7ms** - both
notably worse than the earlier instances, and the first time a gap this large has been recorded
for this bug specifically (motor 2's gap in the same run was 18.7ms, in line with every clean
session). Motor 2 (channel 3, commanded to ramp 60->200->100, opposite direction) spun and
decoded normally throughout: 100% CRC-valid, median eRPM 32,538, tracking its own commanded
deceleration. User confirmed by direct observation: motor 1 (channel 1) did not spin. Immediately
preceded by a clean `two_channel_divergent_300` run on the same boot/session with both motors
100% CRC-valid and correctly divergent eRPMs (57,034 / 34,325) - so DSHOT300 was unaffected
back-to-back with the DSHOT600 failure.

Also notable: the device's own sampled tally and the offline replay of the *same* captures
disagreed slightly on motor 0's split between CRC failures and invalid decodes (device:
crc_ok=737/crc_fail=12; offline replay: crc_ok=742/crc_fail=7; both agree on invalid=12 and on
the 761/769 decoded counts). Both are legitimate reads of the same data - device-side sampling is
literally *every 20th* capture while the offline tool replays all of them - so this isn't itself
anomalous, but the size of the discrepancy (5 records) is larger than seen before and worth
keeping in mind if this gets investigated further.

This run followed a same-day driver change: `dshot_bidir_rx_rle`/`rle_rx_speed`/
`RLE_CYCLES_PER_BIT` were renamed to `dshot_bidir_rx_frame`/`frame_rx_speed`/
`FRAME_CYCLES_PER_BIT` (identifier rename only, verified by diff to be a 1:1 substitution with no
logic change, and by the clean DSHOT300 run using the identical renamed code moments earlier).
Noted for the record, not treated as a cause - the symptom, channel, and even the co-occurring
invalid-decode pattern all match the pre-rename 2026-09-25 and 2026-09-26 instances above.

## Offline re-analysis of stored captures (2026-09-26)

Prompted by `specification/AM32_SOURCE_VERIFICATION.md` findings 1-3, which show two AM32
mechanisms ("never armed" and "stuck-rotor protection") both reproduce this bug's signature. The
four sessions already on disk that correspond to occurrences above -
(`2026-09-25_22-20-07`, `2026-09-26_14-28-18`, `2026-09-26_14-29-42`, `2026-09-26_17-01-04`,
matched to this report's entries by their recorded CRC-valid/invalid counts) - were decoded
offline, per-record, with timestamps, using the project's own `analyze_frame`/`analyze_capture`
(no hardware involved; a script, not committed, reused `driver/gcr_decode.py` and
`driver/dshot_profiles.py` directly). This gives a finer view than the sampled/aggregate figures
already in this report.

**Consistent findings, 4/4 instances:**

- Every affected motor's log has exactly the same two-phase shape: a single contiguous block of
  CRC-**failing** replies at the very start (roughly 2.0-2.5s long), immediately followed by a
  switch to CRC-**valid** replies that decode to `data12 == 0xFFF` - not approximately, *exactly*
  0xFFF, with zero variation - for the entire rest of the run (up to 58s of continuous logging).
  The healthy motor in the same run shows normal, continuously-varying telemetry starting within
  about 8s of the profile beginning.
- This rules out a clean spin-up followed by a mid-run drop-out: there is no window of real,
  varying eRPM anywhere in the affected motor's log, in any instance. Whatever happens, happens
  before any real telemetry is ever seen.
- This does **not** distinguish "never armed" from "armed, then stuck-rotor-latched" - per AM32
  source, both end with `running=0`, which forces the same 0xFFF sentinel, and the CRC-failing
  prefix carries no decodable payload either way. Confirming which needs bench observation (the
  ESC's arming tune vs. its stuck-rotor behavior sound different - see "Suggested next steps"),
  not more log-reading.
- New fact, not previously recorded: in the one both-motors-affected session
  (`2026-09-26_14-28-18`), the CRC-fail-to-0xFFF transition happens at the **same microsecond** on
  both independently-decoded channels (1,984,441us and 1,988,204us respectively, both motors, to
  the microsecond). Two unrelated per-ESC coincidences landing on the same tick is implausible;
  this points at something shared on the Pico side of that moment (e.g. the arm-window-end / first
  non-zero-throttle transition, which `MotorGroup` applies to all motors in the same `update()`
  tick) rather than two independent ESC-side faults.
- The CRC-failing prefix's length (2.0-2.5s across all four instances) is close to AM32's own
  computed worst-case arming latency from finding 1 (>1s zero-throttle gate, plus a 600ms startup
  tune after a cold boot, plus margin - about 2s). Worth checking as a lead, not established as the
  cause: the prefix's replies fail CRC rather than being absent, which a coincidence in gate timing
  alone does not obviously explain.

**2026-09-27 update: reproduced on demand, 3 runs in a row.** With the bench up, `deploy.py` was
run against `two_channel_divergent_600.json`, `two_channel_gc_600.json` and
`single_channel_baseline.json` in sequence (the first bench session since `DEFAULT_ARM_DURATION_MS`
was raised 500->2000ms and the `not_running`/`AM32_NOT_RUNNING_DATA12` change landed - neither is
implicated, since every scenario still passes its own explicit `arm_duration_ms: 3000`). Both
two-bidirectional-motor DSHOT600 scenarios hit this bug again, back to back
(`2026-09-27_10-56-43`, `2026-09-27_11-01-37`); the DSHOT300 single-motor baseline
(`2026-09-27_11-02-16`) passed cleanly (100% CRC-valid, median eRPM 75949, real spin). Offline
re-analysis of the two new sessions shows the exact same signature as all four prior instances: one
CRC-failing prefix, then permanently exact `data12 == 0xFFF` for the rest of the run, no exceptions
- 6 for 6 sessions checked so far. New wrinkle: in `2026-09-27_10-56-43`, the two motors' prefixes
are no longer the same length or in sync (motor 0: 4.45s, motor 2: 1.98s) - the exact-synchronization
observed in the one prior both-motors-affected session was not a general rule.

This makes the failure look considerably more frequent than "intermittent, sporadic result within
an otherwise-passing run" (the original framing above) - it hit on the first attempt at each of the
last three two-bidirectional-motor DSHOT600 runs. Whether that is a real increase in rate (something
changed since this bug was first filed - the frame-receiver adoption, wiring, ESC firmware state
from repeated arm/disarm cycles) or small-sample noise is not established. Worth deliberately
tracking the occurrence rate on this exact scenario pair going forward rather than treating each
instance as a one-off anecdote.

**Same session, full 8-scenario suite run (after the scenario reorganization/rename/duration cut):**
`two_channel_divergent_600.json` (`2026-09-27_11-23-10`, the just-compressed 30s version) hit it
again on both motors - **7 for 7** capture sessions now show the identical signature (CRC-failing
prefix, then permanent exact `0xFFF`). The two motors' prefixes were close in length again this time
(1.985s and 1.982s, ~3ms apart), more like the one prior synced instance than the 4.45s/1.98s
mismatch seen earlier same day. All other 6 scenarios in the suite (both single-motor scenarios at
both speeds, `two_channel_divergent_300`, both `two_channel_gc_*`) passed cleanly with real,
plausible eRPM. The failure continues to look specific to the two-bidirectional-motor DSHOT600
combination, not to the scenario compression or any of today's code changes.

**2026-09-27, A/B retest: `arm_duration_ms` 3000 -> 2000ms, all 8 scenarios re-run.** Per
`AM32_SOURCE_VERIFICATION.md` finding 1, every scenario's `arm_duration_ms` was lowered from 3000ms
(the older, empirically-chosen figure) to 2000ms (the source-computed floor: >1s gate + 600ms
startup tune + margin), and the full suite re-run back to back. Result: **identical pass/fail
pattern to the 3000ms run**, scenario for scenario -
`single_channel_unidirectional_{300,600}`, `single_channel_bidirectional_{300,600}` and
`two_channel_divergent_300`/`two_channel_gc_300` all passed cleanly (real eRPM, matching the 3000ms
run's values closely); `two_channel_divergent_600` and `two_channel_gc_600` both hit BUG-002 again,
both motors, same exact signature. This is a clean, direct falsification of "the arm window is too
short" as a sufficient explanation for BUG-002 as observed here: shortening the Pico's own arm
window by a third neither fixed nor worsened it, on the exact same two scenarios, in the exact same
way. It doesn't rule out finding 1's mechanism as *a* real issue elsewhere, but it means finding
1(a) - "never armed because the Pico's own window was too short" - is not what's producing this
specific, repeatable failure.

Offline re-analysis of the two new instances adds a data point against the arming-gate theory more
generally: the CRC-failing prefix length, which had ranged 1.98-4.45s across all prior instances,
was much shorter this time - **887ms** (`two_channel_divergent_600`,
`2026-09-27_11-33-24`) and **76ms** (`two_channel_gc_600`, `2026-09-27_11-34-47`), both still
closely synced between the two motors (within a few ms of each other). A fixed ESC-clock arming
gate (finding 1's >1s-plus-tune mechanism) would be expected to produce a fairly consistent absolute
delay run to run, not one that swings from 76ms to 4.45s (a ~60x range) across 9 total instances.
That variability is easier to reconcile with finding 3(b), stuck-rotor protection: a real,
mechanical retry-until-timeout process (`bemf_timeout_happened` accumulating over failed start
attempts) is inherently variable in a way a fixed timer gate is not. Not proven - no bench
observation (tune vs. twitch) has been made yet - but the balance of evidence collected purely from
logs now leans toward (b) over (a) for this specific, repeatable failure.

**Sharpest characterization of the trigger condition to date:** across all 9 instances, the failure
has occurred if and only if the run had **both** DSHOT600 **and** two bidirectional motors running
simultaneously. Neither condition alone reproduces it - `single_channel_bidirectional_600` (DSHOT600,
one motor) has passed cleanly every time, and `two_channel_divergent_300`/`two_channel_gc_300`
(DSHOT300, two motors) have passed cleanly every time. Only the combination fails, and it has failed
100% of the times it's been tried (both scenarios, both arm durations tested).

## What's been ruled out

- **Not caused by BUG-001 (the disarm-hang fix), and not fixed by it either.** The no-spin
  symptom predates that fix (seen 2026-09-20, before the fix existed) and still occurred once
  after the fix landed. The ESC's post-disarm recovery — the thing BUG-001 actually fixes — was
  confirmed working correctly in the same run that showed the no-spin symptom, so the two are
  independent: this is not a side effect of BUG-001 or its fix.
- **Not a telemetry/decode-pipeline failure in the sense of "the link is broken."** CRC-valid
  rate is high (97.7% in the one precisely-measured instance) and the ESC is clearly receiving
  and replying to commands — this looks like the ESC is armed and communicating normally while
  simply not driving the motor, not like a corrupted or lost link.
- **Not power-related in any way visible so far** — no brownout, no voltage sag reported, ESC
  behaves normally in every other respect during the same run.
- **Not a clean spin-up that later drops out.** Offline re-analysis of all four stored instances
  (see above) found no window of real, varying eRPM anywhere in an affected motor's log — only a
  CRC-failing prefix followed immediately by the permanent at-rest sentinel. Whatever happens,
  happens before the motor is ever seen to actually turn.
- **Not explained by the Pico's own arm window being too short, at least not fully.** The
  2026-09-27 A/B retest (`arm_duration_ms` 3000ms vs. 2000ms, both source-derived-or-above per
  finding 1) produced the identical pass/fail pattern on the identical two scenarios. If "the
  Pico's own window doesn't give AM32 enough time" were the whole story, the two durations should
  not have failed identically - either the shorter one should fail more, or (if 2000ms already
  covers AM32's own gate, per finding 1) both should have passed. Neither happened.

## Untested leads

- **A pre-arm/late-arming timing window — weakened by the 2026-09-27 arm-duration A/B retest.**
  `BidirectionalDShot.__init__` applies the pull-up at construction time, before `arm()` is ever
  called, and scenario/SD-card setup happens in that gap. If that gap, combined with DSHOT600's
  tighter timing, ever pushes into the ESC's own 2-second unarmed signal-loss window, arming
  could begin while the ESC is mid-reboot - that was consistent with the 2.0-2.5s CRC-failing
  prefixes originally seen, close to AM32's own worst-case arming latency (finding 1). But shortening
  the Pico's own arm window (`arm_duration_ms` 3000ms -> 2000ms) neither fixed nor worsened the
  failure, and the prefix length turned out much more variable than a fixed arming-gate timer would
  predict (76ms to 4.45s across 9 instances - see "What's been ruled out"). Still possible as a
  contributing factor, but no longer the leading explanation on its own.
- **The invalid-decode correlation noted above** — largely explained by the offline re-analysis:
  the "invalid decodes" mixed into these runs sit inside the same CRC-failing prefix window
  identified above, not scattered randomly through the run. Superseded by that finding; no longer a
  separate lead to chase on its own.
- **The trigger condition, now the leading lead: DSHOT600 AND two bidirectional motors together,
  not either alone.** 9 for 9 instances have needed both conditions at once -
  `single_channel_bidirectional_600` (DSHOT600, one motor) and `two_channel_divergent_300`/
  `two_channel_gc_300` (DSHOT300, two motors) have never once failed, at either arm duration tested.
  `two_channel_divergent_600` and `two_channel_gc_600` have never once passed on a first attempt in
  this session's testing. This is no longer "could be coincidence given the small sample" - it's a
  100% reproduction rate on a specific combination, and 100% clean on every scenario missing either
  half of it. Worth investigating what's specific to two bidirectional motors' PIO/timing
  interaction *at DSHOT600's tighter bit period specifically* (half of DSHOT300's, per finding 4) -
  something in the frame receiver's shared timing/IRQ path, or in how two ESCs on the bench
  interact electrically at that speed, that doesn't show up with only one bidirectional motor or at
  the more forgiving DSHOT300 timing.

## Why this hasn't been investigated further

Per this project's own established practice ([[feedback_verify_spin_with_erpm]] in memory): a
no-spin cause must never be asserted without being able to see the ESC's beeps, its power draw,
or rule out a Pico-side hang — none of which is available from decoded telemetry alone. This
needs deliberate, repeated bench time with the operator watching and listening for the specific
failure, not inference from logs after the fact. Not chased further without a go-ahead, since
it's a different, lower-severity bug than BUG-001 and the fix for that was the priority.

## Suggested next steps, if picked up

**Reproduction is no longer the blocker.** As of 2026-09-27, `two_channel_divergent_600` and
`two_channel_gc_600` have failed 100% of the times they've been run this session (4 attempts
between them, across two different `arm_duration_ms` values), while every other scenario has passed
100% of the time. This is now a tractable, on-demand-reproducible investigation, not a wait-for-it
anecdote.

1. **Bench-observe one of the two known-failing scenarios directly** (`two_channel_gc_600` is
   shorter, 20s) - watch/listen with the operator present, per
   `AM32_SOURCE_VERIFICATION.md` finding 3's table: an arming tune (`playInputTune`) heard right
   around the transition favors "never armed" (re)arming mid-run; a motor twitch/buzz with no tune
   favors stuck-rotor protection; silence with neither favors something not yet in either AM32
   mechanism. The transition point varies run to run (76ms-4.45s after the profile starts per the
   offline re-analysis) so watch/listen for the whole run, not just a narrow window.
2. **Chase the sharpened trigger condition** (DSHOT600 + two bidirectional motors, not either
   alone) rather than the arm-window lead, which the 2026-09-27 A/B retest weakened. Candidates
   worth checking: whether the frame receiver's shared IRQ/timing path behaves differently with two
   bidirectional pairs active at DSHOT600's tighter bit period specifically (half of DSHOT300's,
   per finding 4), or whether it's electrical (two ESCs interacting on the bench's shared power/
   ground at that speed) rather than a Pico-side timing issue at all.
3. To directly separate "never armed" from "stuck-rotor protection" from a capture alone (no bench
   time): send a throttle-to-0 for ≥1.5s mid-run, then back up, per finding 3's own discriminator -
   "never armed" arms and then spins; stuck-rotor clears at once and retries the start. Needs a new
   scenario/profile, not existing captures. Given step 2's sharpened lead, build this specifically
   on `two_channel_gc_600` or `two_channel_divergent_600` rather than a new single-motor scenario -
   the failure has never been seen outside the two-bidirectional-motor DSHOT600 combination.

## Fix and investigation plan (2026-09-27)

Proposed after the AM32/Betaflight comparison
(`specification/AM32_ARMING_AND_BETAFLIGHT.md`, sections 4-5). Each step changes **one thing** and
is run on both failing scenarios (`two_channel_divergent_600`, `two_channel_gc_600`), 3 runs each.
Stop at the first step that clears the failure; it names the cause.

| Step | Change | If it clears BUG-002 | If not |
|------|--------|----------------------|--------|
| 0 | No change. Read `max_loop_gap_us` from the stored summaries of the failing sessions (already logged, never checked, BUG-008). | A gap ≥ 500 ms means Core 1 stalled long enough for an armed AM32 to reset itself. Chase the stall. | Go to step 1 |
| 1 | **BUG-004 change 1**: `dshot_bidir_tx` drives the last edge high before releasing | Cause = the released last edge. Close with BUG-004. | Keep it anyway (it matches Betaflight); go to step 2 |
| 2 | **BUG-004 change 2**: receiver `wait(1, pin, 0) [26]` instead of `nop() [26]` | Cause = our receiver triggering on its own frame tail. The ESC-side half still needs step 3 to explain the no-spin. | Go to step 3 |
| 3 | **Keep arming-phase replies in the log** (BUG-003 step 2 / BUG-008 step 4) | - | The log then shows whether the ESC was replying `0xFFF` before `ARMED` and whether its arming-tune dropout happened. That separates "never armed" from "armed, then lost our frames". |
| 4 | New scenario: **two unidirectional motors spinning at DSHOT600** | - | If it fails too, the cause is electrical or ESC-side, not bidirectional logic |
| 5 | **Two bidirectional motors at DSHOT600, only one spinning** (the other held at 0) | - | If it fails, a second released line alone is enough |
| 6 | **First profile step at throttle 200** instead of 60/100 | Cause = a low-throttle start (AM32 stuck-rotor protection, finding 3b). Adopt ≥200 starts (Betaflight idles at ~158). | - |

Whatever step 1-6 find, also land:

- **BUG-003** (evidence-gated arming): a disarmed ESC under non-zero throttle can no longer pass as
  `ARMED`.
- **BUG-008** (harness checks): this failure mode is reported during the run, by name, instead of
  being found in logs afterwards.

### Steps 0-2 run, 2026-09-27: none clear it

**Step 0 (retroactive, no extra bench time - already printed in every run this session):** every
run today, passing and failing alike, reports "longest gap between `update()` calls" in the 5.5-9ms
range. No failing run showed anything close to AM32's 500ms armed-signal-loss threshold, or even the
~360µs DSHOT600 detection-window limit sustained long enough to matter. No Core 1 stall signal.
Go to step 1 - already had no signal before step 1 was tried, consistent with step 1 not fixing it.

**Steps 1 and 2 (BUG-004's two changes), both implemented in `driver/dshot_pio.py`:**

- **Change 1 alone**, 3 runs each on `two_channel_divergent_600` and `two_channel_gc_600`: **6/6
  failed**, identical signature (CRC mostly fine, both motors' median eRPM still 917; one
  `two_channel_gc_600` run had only motor 2 affected, motor 0 spun cleanly).
- **Change 1 + 2 together**, 3 runs on `two_channel_divergent_600`: **3/3 failed**. One run again
  had an asymmetric result - motor 2 spun cleanly (33,557 eRPM, 100% CRC-valid) while motor 0
  stayed stuck at 917 in the same run.
- Regression check: all 6 previously-passing scenarios re-run once each with both changes in
  place - all still pass, eRPM values consistent with prior runs. The fix is safe, just not
  sufficient.

**Conclusion so far:** B4 (the released/floating edge) is not, by itself, what produces BUG-002.
The two changes are kept anyway (see BUG-004) since they're real, Betaflight-matching correctness
improvements with no downside. The newly-seen **asymmetric single-motor failures** (2 of the 9 runs
today) are a fact worth carrying into step 3+: whatever is happening is not always simultaneous
across both motors, even though the earlier same-microsecond-transition instances (2026-09-26)
suggested a shared cause. Next per the table: step 3 (keep arming-phase replies in the log) needs a
small code change to `capture_mailbox.py`/`motor_group.py` before it can be tried; step 4 (two
unidirectional motors at DSHOT600) needs only a new scenario and no driver change, so it is the
cheaper next experiment if picking this up again.

### Before the next bench run: classify the stored sessions (2026-09-27)

With B4 ruled out, the question is which AM32 state the ESC is in after `ARMED`. Three paths in
AM32's source end in the same permanent `0xFFF`, so the CRC counts cannot separate them. What the
ESC does **between ARMED and the first `0xFFF`** can:

| Path | ESC after ARMED | What our receiver records | AM32 source |
|------|-----------------|---------------------------|-------------|
| **A. Not listening at ARMED, never arms** | Silent while it reboots, is still before bidirectional detection, or is in an interrupts-off tune. When it comes up, throttle is already non-zero, so its arming counter keeps resetting. | **echo** (our own frame, see below), then `stop` | arming gate [`main.c#L1360-L1400`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400); replies start at the latch [`dshot.c#L86-L95`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L86-L95) |
| **B. Armed, start fails, stuck-rotor protection latches** | Replying and driving the motor. After ~100 start timeouts (throttle < 150) it cuts drive and holds until throttle returns to 0. | **garbled** (replies there but corrupted), then clean `stop` | [`main.c#L1144-L1149`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1144-L1149), [`#L2050-L2068`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2050-L2068), [`#L2281-L2294`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2281-L2294) |
| **C. Listening but never armed in the window** | Latched and replying before ARMED, but its >1 s zero-throttle gate had not completed. | `stop` from the first capture on | as A |

`not running` is `com_time = 65535` whenever `!running`
([`dshot.c#L284-L286`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L284-L286)).
That is true disarmed and armed alike, which is why the payload alone cannot say which path the
ESC took.

**Why "echo" is recognisable.**
- When the ESC does not reply, `dshot_bidir_rx_frame` waits past the release for the next low. That
  low is the first bit of our own next frame, and the receiver reconstructs that frame.
- Those words depend only on the throttle we sent. `scripts/simulate_frame_receiver.run()` gives the
  whole set for a throttle: 2-15 words at DSHOT600, with receiver phase the only variable, since
  TX and RX share the Pico's clock.
- AM32's `not running` reply (`0x52951`) is in none of the 2,048 throttles' sets.
- 22 of AM32's 2,304 possible replies do appear in some throttle's set. The classifier therefore
  checks CRC first.

**A hint already in the logs.** The prefixes were mostly CRC failures with a few GCR-invalid
decodes. At throttle 60 the echo set is 9 CRC-fail + 3 invalid words; at 100 it is 4 + 2. At 144,
172 and 200 every echo word would be GCR-invalid. This mix fits path A. It is not proof, because a
corrupted reply (path B) can land anywhere.

**To run on the existing BUG-002 sessions** (frame receiver only, so 2026-09-26 on):

```
python scripts/classify_reply_timeline.py captures/<session>
```

Sessions:
- `2026-09-26_14-28-18`, `2026-09-26_14-29-42`, `2026-09-26_17-01-04`;
- `2026-09-27_10-56-43`, `2026-09-27_11-01-37`, `2026-09-27_11-23-10`, `2026-09-27_11-33-24`,
  `2026-09-27_11-34-47`;
- today's nine BUG-004 runs;
- one passing `two_channel_*_300` run, as the healthy reference.

Each bidirectional motor gets:
- a timeline of `spin` / `stop` / `echo` / `garbled` segments;
- the longest stretch with no capture;
- one line naming the matching path.

**What each result means for the next step:**

- **Path A (echo, then stop).** The ESC was not listening at ARMED. **BUG-003** (arming gated on
  each motor's own replies) fixes the symptom. The cause is why the ESC is late or rebooting only
  with two bidirectional motors at DSHOT600. Step 3 (log the arming-phase captures, classified the
  same way) shows whether it latched, went silent at ~1.02 s for its arming tune, or went silent for
  ~2 s and rebooted.
- **Path B (garbled, then stop).** The ESC armed and failed to start. Run the table's steps 4-6:
  - two unidirectional motors at DSHOT600 (electrical/noise);
  - one bidirectional motor spinning, the other held at 0;
  - a first step at throttle 200 (stuck-rotor protection's timeout drops from 100 to 10 above
    throttle 150).

  Also listen for the motor twitching during the prefix.
- **Path C (stop only).** Same fix as A. Step 3 shows how far the ESC's arming got.

**Side finding (feeds BUG-008).** At some throttles our own echo passes CRC by chance:
- At DSHOT600, 227 of the 2,048 throttles. Up to 122 these are 1, 15, 20, 22, 25, 28, 30, 38, 45,
  67, 69, 72, 73, 75, 77, 84, 93, 94, 98, 107, 111 and 122.
- There the echo decodes as a "valid" reply: ~2,200-3,000 eRPM at those low throttles, anywhere
  from ~2,000 to ~170,000 eRPM higher up.

So a silent ESC can pass a CRC-based "ESC is replying" check at those throttles. BUG-008's
per-motor failsafe should count *non-echo* CRC-valid replies.

## Verification requests for the hardware session (2026-09-27)

The plan above needs bench data that can't be produced off the bench. Requests are listed cheapest
first; R0 and R1 need no bench time.

**Reporting back:**
- For each request, add a `### Results: R<n>` subsection at the end of this report with the table
  shown.
- Commit the sessions it produced with `git add -f captures/<session>`; `captures/` is
  git-ignored. Include `capture.bin`, `meta.txt` and `scenario.json`.
- Paste the classifier's output verbatim under the table.

`python scripts/classify_reply_timeline.py captures/<session>` labels every capture:
- `spin`: a real eRPM reply;
- `stop`: AM32's `0xFFF` not-running reply;
- `echo`: the ESC was silent and the receiver captured our own frame;
- `garbled`: anything else.

It also names path A, B or C from the table above.

### R0 - Bench facts (no running)

| Item | Why it matters |
|------|----------------|
| ESC firmware version and target name (AM32 configurator), channels 1 and 3 | Which MCU family's timings apply (F051 assumed from the reply rate) |
| `input type` (auto / DShot / **EDT arm**) | EDT-arm makes AM32 ignore throttle until DShot command 13, which we never send ([`dshot.c#L129-L136`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L129-L136), [`main.c#L743-L765`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L743-L765)) |
| `low voltage cutoff` (off / cell / absolute) and the PSU voltage | With cell cutoff the arming tune repeats once per detected cell (voltage / 3.7 V) with interrupts off: ~0.4 s per cell of silence at arming |
| `stuck rotor protection` on/off | Path B exists only if it is on |
| `sine startup` on/off and its changeover level; `startup power`; `minimum duty` | Sine startup moves the start threshold from 47 to 127, and startup power decides whether throttle 60/100 can start the motor at all |
| PSU current limit, and whether it hits constant-current when both motors start | Two simultaneous starts vs one is the other difference between the passing and failing scenarios |
| Signal lead routing for GPIO6/GPIO8: length, bundled with each other or with phase wires, ground return | Both bidirectional lines are released (weak pull-ups only) between frames and can pick up coupled noise |

### R1 - Classify stored sessions (no bench time)

Run the classifier on:
- **today's BUG-004 runs (after `6cbbc15`)** - the clean evidence;
- the older failing sessions listed in the section above;
- one passing `two_channel_gc_300` and one passing `single_channel_bidirectional_600` session, as
  references.

Caveat for sessions before `6cbbc15`: the old receiver waited a fixed delay after a released edge.
- A frame ending in a released, slowly rising "1" could make it capture its own tail.
- Throttle 100's frame `0x0C8B` ends in 1; throttle 60's frame `0x0780` ends in 0.
- So in those sessions, a silent ESC at throttle 100 can show as `garbled` instead of `echo`.

| Session | Scenario | Motor | Path (A/B/C/spin) | Transition time after ARMED | Longest stretch without capture |
|---------|----------|-------|-------------------|-----------------------------|----------------------------------|

### R2 - Watch and listen (no code change)

Run `two_channel_gc_600` 3 times. For the first 5 s after `Armed.` is printed, note for each motor
what you hear:

| Sound | Meaning | AM32 source |
|-------|---------|-------------|
| 3 short rising beeps, ~0.3 s total (repeated per cell with cell cutoff) | **Arming tune**: the ESC armed *after* our ARMED, while it was ignoring our throttle | [`sounds.c` `playInputTune`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c) |
| 3 longer rising beeps, ~0.6 s total | **Startup tune**: the ESC rebooted | [`sounds.c` `playStartupTune`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c) |
| Twitching or buzzing that stops after ~2 s | Failed start attempts, then stuck-rotor protection (path B) | as path B above |
| Nothing | Path A or C: disarmed and silent, or disarmed and replying | as path A above |

Also note the PSU current display during those 5 s.

| Run | Session | Motor 0 heard / seen | Motor 2 heard / seen | PSU current | Classifier path per motor |
|-----|---------|----------------------|----------------------|-------------|---------------------------|

### R3 - One motor spinning, both bidirectional (table step 5)

- Copy `two_channel_gc_600.json`. Change motor 2's (pin 8) profile to `hold` throttle 0 and delete
  its `min_median_erpm` entry. Keep both motors bidirectional.
- 3 runs; report the classifier output for both motors.
- If motor 0 still fails, a second bidirectional *line* is enough to trigger it, and a second
  spinning motor is not needed.

### R4 - Two unidirectional motors spinning (table step 4)

- Copy `two_channel_gc_600.json`. Set motors 0 and 2 to `"bidirectional": false`, remove their
  `rx_sm_id`, and set `"expect": {}`. With no telemetry, the loader rejects decode thresholds, and
  the record-rate check has nothing to count.
- 3 runs. Report by eye and ear whether each motor spins, and anything from R2's sound table.
- If they fail too, the cause is electrical or power, not bidirectional DShot.

### R5 - First step at throttle 200 (table step 6)

- Copy `two_channel_gc_600.json` with both motors on `hold` 200.
- 3 runs; classifier output for both motors.
- Above throttle 150, stuck-rotor protection gives up after ~10 failed starts instead of ~100
  ([`main.c#L2064-L2068`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2064-L2068)),
  and the start has more torque.

### R6 - Log the arming phase (table step 3; code change, only if R1/R2 point to path A or C)

Today every reply received while `ARMING` is dropped. Path A and C are about what the ESC did before
ARMED, so it has to be logged. Minimal change:
1. **`MotorGroup`**: a flag, default off, that makes `update()`'s ARMING branch call
   `drain_rx(True)` instead of `drain_rx(False)`. `raw_telemetry()` keeps returning `None` until
   ARMED, so the application contract is unchanged.
2. **`run_scenario.arm_group()`**: with the flag on, read each bidirectional motor's
   `latest_capture()` directly and write every new sequence as a record, throttles 0. Carry
   `last_seq` into the run loop.
3. **`meta.txt`**: add `armed_ticks_us`, the `ticks_us` at which `is_armed()` was first seen.
   `analyze_bidir_capture_log.py` must skip records before it, because its thresholds describe the
   ARMED phase. `classify_reply_timeline.py` should use it as time 0 (negative times = arming).

Then run `two_channel_gc_600` 3 times with the flag on, plus one passing `two_channel_gc_300` as
the reference. For each motor, report from `arm()` to ARMED + 5 s:
- **When the first CRC-valid reply appeared.** That is AM32's bidirectional latch, ~101 frames
  after it started listening.
- **Every `echo` stretch and its length.** On a healthy ESC, one stretch of ~0.3 s about 1.02 s
  after it started listening is the arming tune. ~0.7 s or more means a reboot. ~2 s with no
  reboot means it was deaf.
- **What it was sending at ARMED.**

### How the results decide the next step

| Result | Conclusion | Next |
|--------|-----------|------|
| R1/R2: path A or C, R6 shows no latch or a reboot before ARMED | The ESC is not armed when throttle starts, only in this configuration | Land **BUG-003**. Then find what delays or reboots the ESC with two bidirectional lines at DSHOT600, using R0's wiring and power facts. |
| R6: latched and replying, but no arming-tune dropout before ARMED | AM32's 1 s zero-throttle gate did not complete, although we sent zeros | Our zero frames are not reaching it intact (link errors at DSHOT600). Next is a scope on GPIO6/8 during arming. |
| R1/R2: path B, R4 fails too | Electrical or power | Wiring and PSU per R0; not a driver change |
| R1/R2: path B, R4 passes, R5 passes | A low-throttle start fails with two bidirectional motors | Start at ≥200 in the scenarios (Betaflight idles at ~158); keep investigating as noise on the released lines |
| R3 fails | One extra released bidirectional line is enough | Focus on the released-line window between frames (coupling), not on motor power |

---

**The steps below (steps 4 and 5, run before this session pulled the classification-plan commits
above) independently arrived at R3 and R4 - built as standalone new scenarios rather than by
editing `two_channel_gc_600.json` in place, but functionally the same experiments.
`### Results: R3` and `### Results: R4` below re-run the classifier against those sessions and
report per the format requested above.**

### Steps 4 and 5 run, 2026-09-27

**Step 4, `two_channel_unidirectional_600` (new scenario, mirrors `two_channel_gc_600`'s exact
wiring - sm_ids 0/8/4/9, pins 6-9, throttle 100 held for 20s - with both motor-mounted channels
flipped to plain unidirectional DShot600, no telemetry at all):** 1 run, operator-observed (no
telemetry proxy exists for this scenario). Both motors spun. Run completed cleanly, longest
`update()` gap 8.3ms.

**Caveat raised before running this** (see this investigation's own history): flipping both motors
to unidirectional removes two things at once, not one - the released/floating lines (B4), and all
RX-drain work from `update()`, which drops Core 1's tick well below the two-bidirectional-motor
tick that sits right at AM32's ~360µs (F051, DSHOT600) mid-frame-capture rejection limit (ADR-002's
~366µs average / 448µs worst for that configuration). **A clean pass here is therefore not
decisive on its own** - it does not distinguish "no released lines fixed it" from "a faster tick
fixed it" from "both". Only a *failure* here would have been decisive (electrical/ESC-side, ruling
out bidirectional logic entirely). It passed, so step 4 is inconclusive; step 5 below is what
actually narrows things down.

**Step 5, `two_channel_bidir_one_idle_600` (new scenario): both motor-mounted channels stay
bidirectional, but only channel 1 (pin 6) is commanded to a real throttle (100) - channel 3 (pin 8)
holds throttle 0 for the entire 20s run, so it never spins or switches phases, even though its line
still releases every frame and its receiver still runs (`MotorGroup.update()` drains every
bidirectional motor's RX FIFO regardless of that motor's commanded throttle - see
`driver/motor_group.py`).** 1 run: **BUG-002 reproduced, on the spinning motor.** Motor 0 (channel
1, commanded to 100): 251 decoded, 247 CRC-valid (98.4%), 3 CRC failures, 1 invalid, **median eRPM
917** - the exact signature. Motor 2 (channel 3, held at 0 throttle throughout): 259/259 CRC-valid,
median eRPM 917 - correct and expected, since a motor legitimately at rest replies the same
sentinel (finding 2); this motor's eRPM was deliberately excluded from `expect` for exactly that
reason and its clean CRC rate here is not itself informative about the bug. Longest `update()` gap
8.2ms, same range as every prior run, passing or failing.

**Why this result matters:** channel 3 never physically turned during this run. That rules out
"two motors' physical commutation/EMI" as a *necessary* trigger - the failing configuration's only
requirement, per this data, is two bidirectional TX/RX pairs both active at DSHOT600, independent
of whether either one is actually spinning. It directly confirms step 5's own predicted reading
from the fix plan table: "a second released line alone is enough."

**Offline replay, 3/3 step-5 runs, with a purpose-written timeline script (not committed - reused
`scripts/dshot_bidir_decode.py`'s `analyze_frame` directly against each session's raw
`capture.bin`) to find not just *whether* each motor had CRC failures, but *when*, and whether any
gap between two successfully-decoded replies was a real silence (no capture at all, any CRC) or
just a run of CRC failures with captures still arriving on schedule:**

**Superseded by `scripts/classify_reply_timeline.py`** (pulled from `origin` after this analysis was
first drafted - see "Results: R3" below for its verbatim output): that tool tells a real silence
apart from an **echo** (the receiver capturing our own next frame because the ESC stayed silent
past the release - recognisable because its words depend only on our own throttle, simulated via
`simulate_frame_receiver.run()`) instead of lumping both into "CRC failure". Re-reading the same
three sessions through it changes the count: **motor 0 matches AM32 path A - "silent
(echo)...then not running to the end...never armed" - in 3 of 3 runs, not 2 of 3.** The
run my ad hoc script called "no real silence" (`2026-09-27_14-22-14`) does have an echo window
(0.40-0.50s, 100% echo) - just shorter and without a single fully-dead (zero-word) stretch inside
it, which is why a cruder "any nonzero word = alive" check missed it. Once "echo" is counted
correctly, motor 0's outcome is unanimous, tool-verified, across all three runs: **it was never
listening when our `ARMED` transition started sending it throttle 100, came up (if it ever fully
"comes up" rather than staying mid-detection) under non-zero throttle, and so never armed** -
AM32 path A, not path B (stuck-rotor) or path C.

Motor 2's classified timeline also sharpens what a "clean" idle motor's echo window looks like:
in `2026-09-27_14-22-14` and `_14-22-57` it goes `stop` (already replying) → **echo, ~1.5-1.8s in,
~80% of that window** → `stop` again - a brief return to silence in the *middle* of otherwise
continuous replying, not a cold start. That is what AM32's **arming tune** looks like from the
receiver's side (interrupts off, so no reply, but only for the tune's duration, not a full
reboot) - motor 2 arming legitimately mid-run, consistent with the timing match to AM32's ~300ms
tune duration already noted. `2026-09-27_14-12-21`'s motor 2 shows no echo window at all (100%
`stop` throughout) - its own arming tune must have landed before this scenario's published window
starts (during our own `ARMING` phase, which discards every reply - see "Results: R3" and R6 below
for why that gap matters). The tool's built-in `reading()` heuristic has no case for this
stop-echo-stop shape and prints "no single AM32 state matches" for both instances - worth a small
addition if this script is extended further, but the segments themselves are unambiguous by eye.

**What still holds:** motor 0's silence/echo window and motor 2's arming-tune window are at
different elapsed times in every run (motor 0's within the first second, motor 2's around
1.5-1.8s) and are not simultaneous - so whatever makes the second motor's presence matter here is
not a single shared-instant corruption of both receivers at once.

**Follow-up experiment: is it Core 1's tick period?** Before this offline replay, the leading
candidate was that having two bidirectional motors drains two RX FIFOs every `update()` call,
slowing Core 1's tick - and that AM32 rejects a mid-frame capture once the frame-to-frame period
gets too long (`AM32_SOURCE_VERIFICATION.md` section 1, `~360µs` on this bench's F051-class ESCs).
Tested directly with a new scenario,
`single_channel_bidirectional_600_padded_tick` (identical to `single_channel_bidirectional_600`
except `core1_interval_us: 250` pads Core 1's sleep, on a **single** bidirectional motor, no second
motor at all): **passed cleanly** - 6179/6179 CRC-valid (100%), 0 fail streak, median eRPM 21,246 (a
real, correctly-spinning motor).

The scenario's own `_comment` guessed this would produce a ~400-425µs tick, based on ADR-002's
~175µs one-motor / ~366µs two-motor figures. **Those figures do not hold today.** The actual
average tick period, computed from each session's own published-capture count over its 20s ARMED
window (`captures published` in the summary, which only increments once ARMED - see
`driver/capture_mailbox.py`'s `drain(publish)` and `motor_group.py`'s `drain_rx(False)` during
arming):

| Session | Config | Captures published (20s) | Average tick period |
|---|---|---|---|
| `2026-09-27_14-12-21` (step 5) | 2 bidirectional (1 idle) | 26,561 (both motors) | **~753µs** |
| `2026-09-27_14-15-34` (padded) | 1 bidirectional, `core1_interval_us=250` | 21,562 | **~927µs** |

**The padded single-motor run's tick (927µs) is slower than step 5's two-motor tick (753µs), and
both are already several times past the ~360µs figure - yet the slower one passed and the faster
one failed.** This directly falsifies "the tick period alone, once it crosses AM32's mid-frame
window, is what triggers BUG-002" - the ADR-002 figures this bench previously relied on for that
argument are stale (from an earlier version of this code) and should not be cited for today's
timing without re-measuring. Combined with the offline replay above, the tick-period hypothesis is
now dropped in favor of the ESC-side-failure reading.

**Updated leading hypothesis, now tool-confirmed 3/3 (see "Results: R3" below):** motor 0 (the
commanded motor) matches AM32 path A in every step-5 run - not listening when our `ARMED`
transition started sending it throttle 100, so it either never completed detection/latching or
came back from a reset under non-zero throttle, and either way could never satisfy AM32's own
"more than 1s of continuous zero throttle" arming gate afterward, because `MotorGroup` never sends
it zero again once `ARMED`. This matches the upstream classification plan's own reading for path A
exactly (see the table above: "Land BUG-003. Then find what delays or reboots the ESC with two
bidirectional lines at DSHOT600.") and directly answers request **R3**: a second bidirectional
line is enough on its own, with no second motor spinning required - per the plan's own decision
table, that points at "the released-line window between frames (coupling), not motor power" as the
next thing to focus on, not another trigger-config sweep. Whether motor 0 is genuinely
resetting (as the ~600ms-scale silence windows in 2 of 3 runs suggest) or simply never getting
past AM32's own multi-stage detection/latch process before non-zero throttle arrives, is still
open - request **R6** (log the arming-phase replies) is what would settle that, since everything
before our own `ARMED` is currently invisible to every capture gathered so far
(`MotorGroup.update()` drains arming-phase replies with `drain_rx(False)`, so `CaptureMailbox`
never records them).

**Cheapest next check (no code change, no new scenario, same as request R2):** replicate
`two_channel_bidir_one_idle_600` again with the operator listening for two different tunes at two
different times - an early one within the first second of `ARMED` (motor 0, if it is genuinely
rebooting) and a later one around 1.5-1.8s in (motor 2's own normal arming tune, expected and
harmless - do not mistake it for a fault). **Most useful next code change: request R6** - log the
arming-phase replies (flagged, still hidden from the application) so motor 0's state going into
`ARMED` is finally on the record instead of inferred from what happens just after it.

### Results: R3 (one motor spinning, both bidirectional)

Built as a standalone scenario, `two_channel_bidir_one_idle_600.json`, rather than by editing
`two_channel_gc_600.json` in place (functionally the same experiment: channel 1 at throttle 100,
channel 3 held at 0, both bidirectional, DSHOT600, 20s). 3 runs, all reproduced the bug.

| Session | Motor | Path | Transition time after ARMED | Longest stretch without capture |
|---------|-------|------|------------------------------|----------------------------------|
| `2026-09-27_14-12-21` | 0 (throttle 100) | **A** (echo, then stop - never armed) | 0.953 s | 607.6 ms |
| `2026-09-27_14-12-21` | 2 (throttle 0) | stop only (already armed/replying at ARMED) | - | 14.9 ms |
| `2026-09-27_14-22-14` | 0 (throttle 100) | **A** (echo, then stop - never armed) | 0.532 s | 13.9 ms |
| `2026-09-27_14-22-14` | 2 (throttle 0) | stop→echo→stop (own arming tune mid-run, ~1.5-1.8s) | - | 13.9 ms |
| `2026-09-27_14-22-57` | 0 (throttle 100) | **A** (echo, then stop - never armed) | 0.860 s | 606.3 ms |
| `2026-09-27_14-22-57` | 2 (throttle 0) | stop→echo→stop (own arming tune mid-run, ~1.5-1.8s) | - | 16.6 ms |

**3/3: motor 0 matches path A.** A second bidirectional line is enough on its own - no second
motor spinning is required to reproduce this. Per the decision table above, this points at "the
released-line window between frames (coupling), not motor power" as the next focus.

Classifier output verbatim:

```
Session: captures\2026-09-27_14-12-21  (DSHOT600, bidirectional motors [0, 2], 5192 records, outcome=completed)

Motor 0: 5033 captures - stop 4943, echo 83, garbled 7
      0.00 -     0.30 s  echo     92.9% of    70 captures  throttle 100
      0.80 -     1.00 s  echo     58.1% of    31 captures  throttle 100
      1.00 -    20.00 s  stop    100.0% of  4932 captures  throttle 100
  longest stretch with no capture: 607.6 ms, from 0.270 s
  reading: silent until 0.953 s after ARMED, then 'not running' to the end: the ESC was not listening when throttle started (rebooting, before bidirectional detection, or in an interrupts-off tune), came up under non-zero throttle, and so never armed

Motor 2: 5192 captures - stop 5192
      0.00 -    20.00 s  stop    100.0% of  5192 captures  throttle 0
  longest stretch with no capture: 14.9 ms, from 0.000 s
  reading: 'not running' from the first capture to the last: the ESC was replying already at ARMED but never started the motor - it had not armed (AM32 needs >1 s of zero throttle after it starts listening), or it rejected our throttle frames

Session: captures\2026-09-27_14-22-14  (DSHOT600, bidirectional motors [0, 2], 5177 records, outcome=completed)

Motor 0: 5056 captures - stop 5036, echo 19, garbled 1
      0.40 -     0.50 s  echo    100.0% of    12 captures  throttle 100
      0.50 -    20.00 s  stop     99.8% of  5044 captures  throttle 100
  longest stretch with no capture: 13.9 ms, from 15.827 s
  reading: silent until 0.532 s after ARMED, then 'not running' to the end: the ESC was not listening when throttle started (rebooting, before bidirectional detection, or in an interrupts-off tune), came up under non-zero throttle, and so never armed

Motor 2: 5056 captures - stop 4957, echo 89, garbled 10
      0.40 -     0.50 s  echo     91.7% of    12 captures  throttle 0
      0.50 -     1.50 s  stop     94.2% of   259 captures  throttle 0
      1.50 -     1.80 s  echo     82.3% of    79 captures  throttle 0
      1.80 -    20.00 s  stop    100.0% of  4706 captures  throttle 0
  longest stretch with no capture: 13.9 ms, from 15.827 s
  reading: no single AM32 state matches; read the segments above

Session: captures\2026-09-27_14-22-57  (DSHOT600, bidirectional motors [0, 2], 5173 records, outcome=completed)

Motor 0: 5011 captures - stop 4945, echo 62, garbled 4
      0.00 -     0.20 s  echo     97.8% of    46 captures  throttle 100
      0.70 -     0.90 s  echo     56.7% of    30 captures  throttle 100
      0.90 -    20.00 s  stop    100.0% of  4935 captures  throttle 100
  longest stretch with no capture: 606.3 ms, from 0.177 s
  reading: silent until 0.860 s after ARMED, then 'not running' to the end: the ESC was not listening when throttle started (rebooting, before bidirectional detection, or in an interrupts-off tune), came up under non-zero throttle, and so never armed

Motor 2: 5051 captures - stop 4954, echo 88, garbled 9
      0.40 -     0.50 s  echo     92.3% of    13 captures  throttle 0
      0.50 -     1.50 s  stop     94.6% of   258 captures  throttle 0
      1.50 -     1.80 s  echo     80.5% of    77 captures  throttle 0
      1.80 -    20.00 s  stop    100.0% of  4703 captures  throttle 0
  longest stretch with no capture: 16.6 ms, from 2.246 s
  reading: no single AM32 state matches; read the segments above
```

### Results: R4 (two unidirectional motors spinning)

Built as a standalone scenario, `two_channel_unidirectional_600.json` (both motor-mounted channels
flipped to plain unidirectional, throttle 100, DSHOT600, 20s, no telemetry at all - so no
classifier output is possible here, per the request's own note). 4 runs total (sessions
`2026-09-27_14-10-12`, `_14-34-04`, `_14-34-39`, `_14-35-58`), all completed cleanly with no Core 1
stall or error (longest `update()` gap 7.9-8.4ms across all four). **Operator-confirmed both motors
spinning on 2 of the 4** (the first and last run; the operator was away from the bench for the
middle two and couldn't confirm by eye, though the harness itself reported the same clean
completion for those two as the confirmed ones). No failure in 4/4 attempts.

This scenario also changes Core 1's tick (no RX drain at all) at the same time as removing the
released lines, so a pass here is not decisive on its own between "no released lines" and "a faster
tick" - but R3's result (a second released line alone reproduces the bug with no tick-speed
confound, since the padded-tick control ran a *slower* single-bidirectional-motor tick with no
failure) already answers the tick-period half of that question independently, so R4's own result
here is best read as: no failure at all when neither line is released, consistent with R3's
"a released line is what matters" reading rather than contradicting it.

## Analysis of R3 (cloud session, 2026-09-27): every failing ESC rebooted around ARMED

### The "silences" were the line held low

The classifier used for R3 treated an all-zero word as "no capture" and skipped it. That was a bug
in the classifier, and it hid the key signal:
- `motor0_captures_published` equals `motor2_captures_published` to the unit in all three sessions
  (26561, 26648, 26655). Both receivers pushed a word on every frame, including during motor 0's
  "607 ms without a capture".
- Those words were all zero. The line was low from ~2 µs after our release to the end of the
  ~27 µs capture, on every frame. Nothing we transmit produces that, so the ESC side held the line
  low.

`classify_reply_timeline.py` now labels these words `low`, and reports the AM32 events it finds.
Re-run on the same three sessions, times from ARMED:

| Session | Motor (throttle) | Line held low | First reply | Arming tune (no replies) | Outcome |
|---------|------------------|---------------|-------------|--------------------------|---------|
| `14-12-21` | 0 (100) | 0.274 - 0.874 s (600 ms) | 0.953 s | none | never armed |
| `14-12-21` | 2 (0) | - | replying from the first record | before the first record, if any | - |
| `14-22-14` | 0 (100) | before 0 - 0.453 s | 0.532 s | none | never armed |
| `14-22-14` | 2 (0) | before 0 - 0.453 s | 0.536 s | 1.505 - 1.774 s | armed |
| `14-22-57` | 0 (100) | 0.180 - 0.779 s (599 ms) | 0.860 s | none | never armed |
| `14-22-57` | 2 (0) | before 0 - 0.452 s | 0.534 s | 1.505 - 1.773 s | armed |

Every stretch matches AM32's boot sequence, to the millisecond where it is measurable:

| What the receiver sees | Length | AM32 |
|------------------------|--------|------|
| Line held low | 599-600 ms | Startup tune, 3 × 200 ms with interrupts off ([`sounds.c#L118-L146`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L118-L146)). The signal pin gets its capture setup and pull-up only after the tune ([`main.c#L1893-L1933`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1933)). Why the line reads *low* rather than floating up to our pull-up is not established; the timing identifies the tune regardless. |
| `echo`, then the first `stop` | 79-83 ms | Detection, then >100 frames before the bidirectional latch ([`dshot.c#L86-L95`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L86-L95)); 100 frames at our 0.753 ms tick are 75 ms |
| `echo` between two runs of `stop` | ~270 ms, 1.05 s after the ESC came up | Arming tune after >1 s of zero throttle ([`main.c#L1360-L1400`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400), [`sounds.c#L219-L237`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L219-L237)) |

So R3 confirms path A and explains it. In 3 of 3 runs motor 0's ESC reset, came back up after
ARMED, found throttle 100, and never armed. Motor 2's ESC reset too in 2 of 3 runs, at the same
moment as motor 0's in `14-22-14`. Held at zero, it armed ~1 s later. So whichever ESC resets late
enough to come up under non-zero throttle is the one that "does not spin".

Corrections to the text above:
- "2 of 3 runs": it is 3 of 3. In `14-22-14` the reboot started before ARMED. The old classifier
  therefore printed no segment before 0.40 s.
- "motor 0's silence/echo window and motor 2's arming-tune window are ... not simultaneous": in
  `14-22-14` both ESCs were in their startup tune together. Their low stretches end within 1 ms of
  each other.
- Motor 2's mid-run `echo` is its arming tune after its own reboot. It is not a first arming from a
  healthy state.

### When the resets happen: inside our arming window, on a fixed clock

ARMED is 2.000 s after `arm()`. The low stretch starts at the reset plus the bootloader's and the
app's start-up time:
- 1.853 s after `arm()` for three ESC instances in two runs (motor 2 in `14-22-57`, both in
  `14-22-14`). Their starts are before the first record, so this is their end (0.452-0.453 s
  after ARMED, agreeing to within 1 ms) minus the 600 ms the two complete stretches measured.
- 2.180 s and 2.274 s for motor 0 in the other two runs.

Three of the five resets happen before ARMED. So the throttle step at ARMED is not the trigger;
something inside the arming window is. Timing that repeats to the millisecond across ESCs and runs
means a fixed chain that starts when our first frames arrive at `arm()`. The two later resets fit
the same chain started 0.33-0.42 s late, for example an ESC that was still in its own startup tune
when `arm()` began.

**A chain in AM32's source with that timing (leading hypothesis, not confirmed):**
1. At `arm()` the ESC detects our signal.
2. It counts zero-throttle captures **without validating them**: `zero_input_count++` runs for
   every capture whether or not it passed the frame check
   ([`signal.c#L166-L178`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L178)).
   So it arms ~1.05 s later even if it accepted none of our frames
   ([`main.c#L1360-L1400`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400)).
3. Its arming tune (~0.3 s) ends by clearing `signaltimeout`
   ([`sounds.c#L234`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L234)).
4. Armed, it resets after 0.5 s in which no capture passes the frame-length window of
   `computeDshotDMA()`
   ([`dshot.c#L72-L77`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L72-L77),
   [`main.c#L1992-L2004`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2004)).

That totals 1.05 + ~0.3 + ~0.52 s ≈ 1.87 s, plus the boot time, against 1.853 s measured. The
chain is close enough to test, but not a fit to the millisecond. The bench's firmware and
bootloader versions (R0) would pin the tune lengths. The disarmed 2 s reset
([`main.c#L2006-L2017`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2006-L2017))
does not fit as well: it counts from the ESC's last accepted frame or tune, and that depends on
what the ESC was doing before `arm()`, so it would not repeat to the millisecond.

The chain needs an ESC that, armed, stops accepting our frames. Two ways, which the arming-phase
log separates:
- **(a) It never accepted them.** AM32 learns the frame-length window once, from the 7th-14th
  capture after detection, at ±1/16, and never relearns it before a reset
  ([`signal.c#L166-L174`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174)).
  One misaligned capture among those 8 skews the window, and every correct frame after that is
  rejected. A misaligned capture is one with an extra edge on the line or a missed edge. The ESC
  then never latches bidirectional mode, so it never replies. **Signature: no `stop` at all
  between `arm()` and the reset.**
- **(b) It accepted them until it armed, then lost them.** **Signature: `stop` from ~75 ms after
  `arm()`, the arming tune at ~1.05 s, then `stop` or nothing until the reset at ~1.85 s.**

Either way, whether an ESC loses our frames is decided per ESC and per run. It happens with two
bidirectional DSHOT600 lines and not with one (R3 vs the single-motor scenarios, R4). Which
property of the second line causes it is the open question. Candidates are coupling onto a
released line, or edge timing at DSHOT600 (both two-motor DSHOT300 scenarios pass).

**This also explains the earlier A/B retest.** Suppose the reset lands at ~1.85 s and the ESC is
back ~0.68 s later. It then needs >1 s of zeros, so it can arm no earlier than ~3.6 s after
`arm()`. That fails with a 2000 ms window and with a 3000 ms window alike, which is what the
retest found. A 4500 ms window should pass if the ESC keeps our frames after its reboot. Motor 2
did in 2 of 2 runs.

### Consequence for BUG-003

A reply seen early in the arming window does not mean the ESC will still be there at ARMED. The
evidence gate must restart its 1.2 s count after any reply gap long enough to be a reboot. A gap of
≥ 450 ms is longer than the arming tune and shorter than a reboot's ~680 ms. See BUG-003's fix
plan, step 3.

## Verification requests, round 2 (2026-09-27)

Same reporting rules as round 1: add a `### Results: R<n>` subsection, commit the sessions with
`git add -f captures/<session>`, and paste the classifier output verbatim. This round's sessions
also contain `arming.bin`; commit it.

**R6 is now implemented** (this commit), so it no longer needs a code change on the bench side:
- `MotorGroup.publish_while_arming`: a diagnostic flag, default off. `raw_telemetry()` still
  returns `None` until ARMED.
- `run_scenario.py` sets the flag and logs every arming-phase capture to `arming.bin`, in
  `capture.bin`'s format.
- `meta.txt` gains `armed_ticks_us` and `motor<i>_captures_while_arming`.
- `capture.bin` still starts at ARMED, so `analyze_bidir_capture_log.py` and the scenario verdicts
  are unchanged. `motor<i>_captures_published` now counts from `arm()`; subtract
  `motor<i>_captures_while_arming` to get the ARMED-phase count.
- `classify_reply_timeline.py` shows the arming window at negative times.

`scripts/run_test.py --scenario <file>` uploads the changed driver and harness before each run,
as usual.

### R7 - The arming window, logged (bench, no code change)

Run `two_channel_bidir_one_idle_600` 3 times and `two_channel_gc_600` 2 times, plus
`single_channel_bidirectional_600` once as the healthy reference. For each bidirectional motor,
from the classifier's segments and events:

| Session | Motor | First class after `arm()` | First `stop` | Arming tune(s) | Line held low (reset) | Outcome |
|---------|-------|---------------------------|--------------|----------------|-----------------------|---------|

Times are relative to ARMED; `arm()` is at -2.000 s. What each hypothesis predicts for a failing
ESC:
- **(a) never accepted:** `echo` (or `garbled`) from -2.0 s with no `stop` at all, then `low` from
  about -0.15 s.
- **(b) accepted until armed:** `stop` from about -1.92 s, `echo` ~0.3 s at about -0.95 s (its
  arming tune), then `echo`, `garbled` or `stop`, then `low` from about -0.15 s.
- **Neither:** a `low` stretch at a different time, or one that does not follow an arming tune,
  means the chain above is wrong. That points to the IWDG (1.6 s nominal,
  [`peripherals.c#L139-L145`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/peripherals.c#L139-L145))
  or to power. Report `low` stretches that are not ~600 ms too.
- **Healthy ESC**, for comparison: `echo` ~80 ms, then `stop`, with its arming tune about 1.05 s
  after `arm()`, and no `low` at all.
- **Also note the first class right at `arm()`:** `low` means the ESC was already in its startup
  tune, and `echo` means it was listening or in its bootloader. A mix of `echo` and short `low` or
  `garbled` would be the bootloader answering our frames with its 0xC1/0xC2 NACK bytes.

### R8 - A 4500 ms arming window (bench, no code change)

Run `two_channel_bidir_one_idle_600_long_arm` (new; the same as `two_channel_bidir_one_idle_600`
except `arm_duration_ms: 4500`) 3 times.

| Session | Motor 0 outcome | Motor 0 reset(s) in the arming log | Motor 0 arming tune |
|---------|-----------------|------------------------------------|---------------------|

- **Prediction: motor 0 spins 3/3.** It resets at about -2.65 s, is back about 0.7 s later, and
  arms about 1.05 s after that, all before ARMED. That confirms the timing chain and gives a
  workaround until the cause is found; BUG-003 is the general form of that workaround.
- A second reset in the log means the loss of our frames repeats after a reboot. The cause then
  has to be found (R10) before any workaround can hold.

### R9 - Listen during R7 (no extra runs)

With the arming log, listening only confirms. The hypothesis predicts this for a failing ESC:
- the arming tune (3 short beeps) ~1 s after `Arming for 2000ms...` prints;
- the startup tune (3 longer beeps) ~0.8 s after that;
- no tune from that ESC afterwards.

Report only a sequence that differs.

### R10 - Only if R7 shows (a): find the extra edge

1. **Scope or logic analyser** on both signal lines at the ESC pads. Trigger on the first frame
   after `arm()` and capture 20 ms at ≥ 50 MS/s. Report any transition on a line between its own
   frames, when it happens relative to the other line's frame, and its amplitude.
2. **Pull-up test**, no code change. Fit a 2.2-4.7 kΩ pull-up from each bidirectional signal line
   to 3.3 V at the Pico end. That stiffens the released line about tenfold against coupling, and
   AM32's push-pull reply drives it easily. Re-run `two_channel_bidir_one_idle_600` 3 times.
   - Passes 3/3: coupling onto the released line is the cause. The fix is hardware (pull-ups,
     wiring), or frame staggering in `MotorGroup.update()`.
   - Still fails: look at edge timing at DSHOT600 instead.

### How round 2 decides the next step

| Result | Meaning | Next |
|--------|---------|------|
| R7 (a) + R8 passes | The ESC never takes our frames on its first boot; a reboot clears it | R10 for the cause. Land BUG-003 with the reboot-aware gate either way. |
| R7 (b) | The ESC loses our frames when it arms | Read AM32's armed input path (deferred `processDshot()`) against the arming log. R10 step 1 still applies. |
| R7: `low` without an arming tune before it | Not the armed-timeout chain | IWDG or power: R0's PSU facts, supply rail on the scope |
| R8 fails with one reset | Arming too slow even with 4.5 s | Read the log's timings; widen the window again only if the ESC is still arming at ARMED |
