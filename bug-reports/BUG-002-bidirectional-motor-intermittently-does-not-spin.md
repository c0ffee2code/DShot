# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status:** OPEN — not investigated to a root cause
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

## Untested leads

- **A pre-arm/late-arming timing window — sharpened by the 2026-09-26 offline re-analysis above,
  still not confirmed.** `BidirectionalDShot.__init__` applies the pull-up at construction time,
  before `arm()` is ever called, and scenario/SD-card setup happens in that gap. If that gap,
  combined with DSHOT600's tighter timing, ever pushes into the ESC's own 2-second unarmed
  signal-loss window, arming could begin while the ESC is mid-reboot. The offline re-analysis found
  a CRC-failing prefix of 2.0-2.5s at the start of every affected motor's log, close to AM32's own
  worst-case arming latency (finding 1: >1s gate + 600ms startup tune + margin) - consistent with
  the ESC still being mid-arm (or freshly rebooted and re-arming) when the Pico's own arm window
  had already closed and the throttle profile started sending non-zero commands, which per AM32
  source resets its arming counter on every non-zero-throttle tick and keeps it disarmed for the
  rest of the run. This does not yet explain why the prefix *fails CRC* rather than being silent or
  simply absent, which needs more thought or a bench capture to resolve.
- **The invalid-decode correlation noted above** — largely explained by the offline re-analysis:
  the "invalid decodes" mixed into these runs sit inside the same CRC-failing prefix window
  identified above, not scattered randomly through the run. Superseded by that finding; no longer a
  separate lead to chase on its own.
- **DSHOT600-specific:** every documented instance is at 600, none at 300 despite comparable
  total runtime at both speeds across this project's regression history - including a clean 300
  run immediately preceding the 2026-09-26 `two_channel_divergent_600` instance, on the same
  boot. Could be coincidence given the small sample, could be a real timing-margin issue specific
  to 600's tighter bit period.

## Why this hasn't been investigated further

Per this project's own established practice ([[feedback_verify_spin_with_erpm]] in memory): a
no-spin cause must never be asserted without being able to see the ESC's beeps, its power draw,
or rule out a Pico-side hang — none of which is available from decoded telemetry alone. This
needs deliberate, repeated bench time with the operator watching and listening for the specific
failure, not inference from logs after the fact. Not chased further without a go-ahead, since
it's a different, lower-severity bug than BUG-001 and the fix for that was the priority.

## Suggested next steps, if picked up

1. Try to reproduce on demand rather than waiting for it — repeat `two_channel_divergent_600`
   (and a DSHOT600 equivalent at other throttle profiles) enough times, watching both the
   invalid-decode count and the motor, to get a real occurrence rate instead of two anecdotes.
2. If reproduced, capture what the ESC's beeps/tone sound like during a no-spin run specifically
   — a silent failure and an ESC stuck in some other state would sound different. Per
   `AM32_SOURCE_VERIFICATION.md` finding 3's table: an arming tune (`playInputTune`) heard right
   around the transition favors "never armed" reconnecting mid-run; a motor twitch/buzz with no
   tune favors stuck-rotor protection. The offline re-analysis above narrows *when* to listen: the
   transition happens 2.0-2.5s after the throttle profile starts, not at an arbitrary point in the
   run.
3. Test the pre-arm timing window lead directly: deliberately delay the gap between motor
   construction and `arm()` past 2 seconds and see if that reproduces the symptom on demand.
4. To directly separate "never armed" from "stuck-rotor protection" from a capture alone (no bench
   time): send a throttle-to-0 for ≥1.5s mid-run, then back up, per finding 3's own discriminator -
   "never armed" arms and then spins; stuck-rotor clears at once and retries the start. Needs a new
   scenario/profile, not existing captures.
4. Once reproducible, this becomes a tractable investigation like BUG-001 was — right now it
   isn't, because it can't be reproduced at will.
