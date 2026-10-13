# BUG-009: Motors sometimes start rough

**Status:** OPEN. One specific telemetry artifact has a candidate mechanism, not yet fully
reconciled with the bench data. The mechanical question (does the motor actually turn unevenly,
separate from telemetry) is untouched.
**Recorded:** 2026-10-10, but present from the start, per the user - confirmed to predate every
code change made that day (W33 performance work, `@micropython.native`, BUG-002's `arm_group()`
fix). None of that day's own code changes are the cause.

## Before touching this: BUG-005 (now fixed) is why a ramp from 0 cannot be written

*Update 2026-10-13:* `MotorGroup.set_throttle()` now raises for 1-47 and scenario profiles that reach it fail to load, so a ramp from 0 can no longer walk through the command range by accident. A smooth-start experiment must jump from 0 to 48 or above. The text below is the original warning.

**Read `bug-reports/BUG-005-throttle-1-47-sent-as-dshot-commands.md` first.** It is OPEN,
independent of this investigation, and directly relevant: DShot values 1-47 are commands, not
throttle (beep, direction reverse, 3D-mode toggle, **EEPROM save**, programming mode), and this
driver does not clamp them. No current scenario hits it (none ramp from 0 - the lowest hold is 60),
but a "ramp gradually from 0 for a smoother start" experiment walks straight into it, and can
persist changes to the ESC across power cycles. Fix BUG-005, or at minimum clamp any experimental
ramp to 48+, before trying gradual starts here.

## What's confirmed: one specific CRC-valid glitch, reproducible and timed precisely

A standalone tool (`tools/measure_rough_start.py`) that jumps straight to one fixed throttle right
after arming and logs every capture (no on-device sampling) found captures decoding to `data12=2`
(`erpm = 60,000,000/2 = 30,000,000` - physically absurd, yet CRC-valid), byte-for-byte identical
(raw word `0x8bb89`) across every session that hit it (17 of 25 runs at `BASE_THROTTLE=65`, 6/10
without a pause, 11/15 with a 500ms pause added between arming and the first throttle command).

**Timing confirms it's tied to the throttle-crosses-arming-threshold moment, not to the arm event
itself.** Without the pause, every hit landed within ~2.5ms of `armed_ticks_us`. With the 500ms
pause added before the first nonzero throttle command, every hit shifted to 501,451-502,573us after
`armed_ticks_us` - matching the pause almost exactly. That shift is the strongest evidence in this
file: whatever produces `data12=2`, it happens at the moment throttle first goes nonzero, not at
`ARMED`.

**Candidate mechanism, from AM32's own source - re-verified at the pinned commit (`55c96847`).**
A first re-read (below, as "the first correction to this entry") concluded the zero-array story
didn't hold. A second, independent re-read (a side agent) found a gating condition the first read
missed, which reverses that conclusion back toward the original theory. This is a candidate again,
not a settled mechanism - two source reads disagreeing with each other once already is itself a
reason for caution, not confidence:
`commutation_intervals[6]`, the array AM32 averages to compute eRPM, is zero-initialized at boot
(line 432) and written only inside `commutate()` (line 904:
`commutation_intervals[step - 1] = commutation_interval`).

The first correction to this entry (below) worried that `startMotor()`'s one-shot `commutate()`
call would poison the array with `commutation_interval`'s declared initial value (12500, line 570)
instead of 0. **That worry doesn't apply on the normal boot path, because `startMotor()` is
gated off and never runs there at all:**
```c
// line 1205-1213, inside setInput()/the throttle-threshold check
if (input >= 47 + (80 * eepromBuffer.use_sine_start)) {
    if (running == 0) {
        allOff();
        if (!old_routine) {
            startMotor();
        }
        running = 1;
        ...
```
`old_routine` is declared `= 1` (line 458) and is only ever cleared to 0 *inside* `zcfoundroutine()`
- which itself only runs while `running` is already true (its call sites are all gated on
`running`). So on every fresh boot, `old_routine` is still 1 when throttle first crosses the start
threshold, `!old_routine` is false, **`startMotor()` is skipped**, and `running` flips to 1 directly
with `commutation_intervals[]` never touched by anything - genuinely `[0, 0, 0, 0, 0, 0]`. The first
`e_com_time` computed after that (any reply sent before the motor's first real commutation event)
is `(0+0+0+0+0+0+4) >> 1 = 2` - `data12=2`, exactly the anomaly. This also explains the
reproducibility: since every bench run resets the ESC (BUG-002's own well-established finding),
`old_routine` resets to its `=1` default every single run, reproducing the same gated-off path every
time.

This is a candidate explanation, not the best-supported one - it reasons from source alone, not
from a bench re-test designed around this specific claim, and it already reversed once on a second
read. If this gets revisited, the clean test is: does `data12=2` ever appear on a build/config
where `old_routine` would already be 0 or `use_sine_start` is set (changing the threshold and the
gate)? **Stop re-deriving this from source alone - the next real move on this specific question is
a bench test, not another re-read.**

**TX-echo checked, not fully ruled out.** This driver's RX receiver can capture its own TX frame
when the ESC stays silent, and some of those echoes happen to pass CRC by chance (`BUG-008`'s own
documented mechanism, `scripts/classify_reply_timeline.py`'s `EchoModel`). Computed the predicted
echo words directly for throttle 60, 65, 100 and 150 at DSHOT600: `0x8bb89` is not among them at any
of those throttles. That only means the model, as it stands, doesn't predict this word at these
throttles - the model itself has not been independently validated against a known-real echo from
this bench's own history, so this is evidence against the echo explanation, not a ruled-out
verdict. (`driver/gcr_decode.py`'s docstring says the same thing - keep both in agreement if either
changes.)

## What's NOT yet reconciled - do not call this settled

**Zero occurrences in 30 `two_channel_divergent_600.json` runs that set a nonzero throttle (60)
immediately after arming, at the same nominal `poll_ms=1`.** Across 30 full-resolution-checked
sessions, none ever produced `data12=2` (their worst early eRPM reading was 159,151, a different
and smaller-magnitude anomaly - see below). If the AM32 mechanism above is both correct and
genuinely throttle-independent, this is unexplained.

**The "lower effective poll rate" hypothesis is ruled out, by its own numbers, not just untested.**
Early-window record rates were measured directly: ~463/s for `measure_rough_start.py`, ~262/s for
`divergent_600` (56% of that). If the glitch's hit rate scaled proportionally with poll rate,
`measure_rough_start.py`'s observed 68% (17/25) would predict roughly 38% for `divergent_600` - and
observing literally 0 hits in 30 trials at a true 38% rate has probability under 1e-6. A real but
partial effective-poll-rate difference cannot produce a clean 0/30. Something else explains the gap
- possibly a genuine throttle dependence after all, or some other structural difference between the
two scripts not yet identified. **Next session's first task should be resolving this**, starting
with `measure_rough_start.py` at `BASE_THROTTLE=60` (matching `divergent_600`'s own first throttle
exactly) to see whether the hit rate collapses to match `divergent_600`'s 0/30 - that result would
itself argue for a genuine throttle dependence, since the polling-rate explanation is now excluded.

**Separately, `decode_every=20` (the on-device sampling every scenario uses for its pass/fail
verdict) cannot see any of this reliably** - it only decodes 1 in 20 replies, and none of the runs
that hit the `data12=2` spike printed `Test FAILED`. Full-resolution decode (every capture) is the
only check that has caught this so far; treat any scenario "pass" verdict on this motor's eRPM as
uninformative about brief spikes.

**The broader, smaller-magnitude eRPM-variance spikes in `two_channel_divergent_600.json`** (motor
2/channel 3's coefficient of variation up to 142.7%, max eRPM up to 159,151, in 3 of 20 runs from
one batch, spread across roughly the first 2 seconds rather than one single frame) are an open
question, not a confirmed instance of the same mechanism. A 6-slot moving average warming up does
not obviously explain noise spread across 2 full seconds rather than concentrated in the first few
commutation cycles; this needs its own check before assuming it's the same cause.

**That spike batch, and a separate 4-run batch where the same motor never spun at all in
isolation (throttle 65/100/150), are confounded**: the user was changing hardware on channel 3 (a
slotted-disk RPM sensor, an imbalanced prop) during that same window. Don't treat either result as
a clean software/firmware finding until re-run with a known, stable physical setup on that channel.

**Is there real mechanical roughness at all, separate from telemetry artifacts?** Nothing in this
investigation distinguishes "AM32's eRPM reporting is briefly unreliable" from "the motor is
actually turning unevenly." The independent slotted-disk + light-sensor RPM rig (in progress,
independent of DShot telemetry entirely) is the right tool to settle this - hold off on further
DShot-telemetry digging into "is it rough" until that's ready.

## Decided, not open

**The driver will not filter implausible eRPM values.** Considered a value-based floor in the
decoder, a time-based "not yet reliable" window in `MotorGroup`, and leaving it to applications;
the user chose the last: document the behavior (done, in `driver/gcr_decode.py`'s `decode_result()`
docstring) and leave filtering to whatever application needs it. Real applications don't poll eRPM
within milliseconds of arming anyway.

## Starting point for the next session

1. Resolve the echo-vs-AM32 question first (see above): validate `EchoModel` against a known-real
   echo before trusting it either way, and run `measure_rough_start.py` at `BASE_THROTTLE=60`.
2. Fix or work around BUG-005 before any gradual-ramp-from-0 experiment.
3. Re-run the channel-3 isolation and divergent-600 spike batches only once the physical setup on
   that channel is known and stable.
4. A full backup of `captures/` as it stood 2026-10-10 is at `captures Backup/` (sibling directory,
   repo root) - captures themselves are being dropped from git; use this backup, not git history,
   for any historical-capture analysis (e.g. bucketing spike timing by first-nonzero-throttle
   record instead of `armed_ticks_us`, or checking for eRPM sentinel re-entry after a motor has
   genuinely spun).

## Related

- BUG-002 - the arming-reliability investigation this spun off from, now resolved. A different
  problem; its `arm_group()` fix is unrelated to this one.
- BUG-005 - landmine for any fix attempted here (ramping from 0), see above.
- BUG-008 - tracks the harness safety-net gaps; this file's `data12=2` finding was a confirmed
  instance there. Since 2026-10-13 the permanent latch it defeated is replaced by a continuous
  "no real eRPM for 2 s while commanded" check (`decode_tally.SpinWatch`).
