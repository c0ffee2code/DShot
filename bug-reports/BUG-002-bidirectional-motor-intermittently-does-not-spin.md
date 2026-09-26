# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status:** OPEN — not investigated to a root cause
**Severity:** Medium — intermittent, the ESC and telemetry link both stay healthy and the ESC
recovers normally afterward, but the motor silently fails to do the one thing it's told to do.
**Component:** unclear — could be driver timing, ESC-side arming state, or something environmental;
see "What's been ruled out" below for what it is *not*.

## Summary

A bidirectional motor occasionally arms, replies with mostly-CRC-valid telemetry, and never
actually spins — the decoded eRPM sits at 917, the documented at-rest sentinel value AM32 sends
for an armed motor that isn't turning, for the whole run. The same scenario, run again
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

## Untested leads

- **A pre-arm timing window, flagged during the BUG-001 investigation, not confirmed.**
  `BidirectionalDShot.__init__` applies the pull-up at construction time, before `arm()` is ever
  called, and scenario/SD-card setup happens in that gap. If that gap, combined with DSHOT600's
  tighter timing, ever pushes into the ESC's own 2-second unarmed signal-loss window, arming
  could begin while the ESC is mid-reboot. Weighed against this: AM32's own bootloader escapes
  quickly once real DShot frames start arriving (each failed-framing attempt bumps its internal
  counter toward a fast exit), so the more likely effect of this window, if it matters at all,
  is a **late** application start eating into the arm window — not a clean arm followed by a
  silent refusal to spin. Whether that's consistent with what's observed (motor arms, replies
  correctly, just doesn't spin) hasn't been reasoned through carefully. Flagged as a lead worth
  checking first, not a working theory.
- **The invalid-decode correlation noted above** — worth deliberately trying to reproduce with
  the invalid-decode count as the thing being watched, rather than noticing it after the fact.
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
   — a silent failure and an ESC stuck in some other state would sound different.
3. Test the pre-arm timing window lead directly: deliberately delay the gap between motor
   construction and `arm()` past 2 seconds and see if that reproduces the symptom on demand.
4. Once reproducible, this becomes a tractable investigation like BUG-001 was — right now it
   isn't, because it can't be reproduced at will.
