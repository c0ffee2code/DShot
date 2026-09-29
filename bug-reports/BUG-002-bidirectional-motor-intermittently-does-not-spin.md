# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status: MITIGATED.** The failure this bug reports - a motor silently never spinning while its
ESC keeps replying with valid, CRC-good telemetry - has not recurred once across 132 bench runs.
That's because arming no longer trusts a timer alone; it waits for evidence that the ESC is
actually listening (BUG-003). What's still open is *why* the ESC so often rejects our frames when
our signal starts - that root cause is unconfirmed, and a motor stuck in that state for too long
still won't fly this run, just loudly instead of silently.

**Refusals (arming timing out rather than a motor failing to spin), across all 132 runs to date:**
6 (4.5%), all clustered in two batches - 3 of 38 on 2026-09-27 and 3 of 6 on one 2026-09-28 morning
batch - with 0 of 88 in every batch since. The 09-28 morning cluster's rate (3/6) against the
pooled 0/88 since is unlikely to be noise (one-sided Fisher p ≈ 0.0014), but its cause is
unestablished. Zero sessions have ever armed a motor that then failed to spin.

## Symptom

A bidirectional ESC, commanded a normal throttle after arming, sometimes never spins. It keeps
replying with CRC-valid telemetry the entire time - the constant `0xFFF`/917 eRPM frame AM32 sends
for "motor not running," whether armed or disarmed. Nothing on the wire looks broken: the link is
healthy, the ESC is clearly listening and replying, it simply never starts the motor.

## The fix

**BUG-003's evidence-gated arming** (`driver/motor_group.py`, commit `27722bb`) is what fixed the
silent-no-spin symptom. Before it, a group armed on a timer alone; a bidirectional ESC that
rebooted mid-arming would still get promoted to `ARMED` on schedule and then get sent nonzero
throttle before it had satisfied its own internal arming gate - so it never spun, despite replying
with valid telemetry throughout. Now `ARMED` additionally requires every bidirectional motor to
have replied steadily, with no gap longer than the reboot's own signature (`READY_GAP_MS`), for two
full seconds (`READY_SPAN_MS`). An ESC that's mid-reboot simply keeps the group in `ARMING`,
sending zeros, until it settles or the application's own timeout gives up and names the motor still
not ready (`arming_status()`).

A separate reliability fix landed the same day: the arming floor used to restart its own clock on
any `update()` gap over 10ms, on the mistaken assumption that AM32 resets its own arming counter
the same way - it doesn't. Removed (`ARM_GAP_TOLERANCE_MS` is gone); the floor is now plain elapsed
time since `arm()`.

The harness's own arming timeout (not the library's - `MotorGroup` waits forever for evidence, by
design) is 18s, sized off the ~2.46s reboot-cycle period to give margin for about 6 reboots.

## Established facts

- **Needs two concurrently-active bidirectional TX/RX pairs at DSHOT600.** Every recorded failure
  in this bug's history was on that configuration; DSHOT300 two-motor and every single-motor
  scenario (either speed) have passed cleanly in the same test suites where DSHOT600 two-motor
  failed. Not deterministic even there - two DSHOT600 two-motor runs back to back have gone one
  fail, one pass.
- **The reset fingerprint, consolidated across every measured instance:** the affected ESC goes
  silent 1855-1867ms after it starts listening (75 of 77 measured instances; 2 outliers at 1347ms) -
  AM32 arms on frames it never validated after >1s zero-throttle plus 30+ captures
  ([`main.c#L1360-L1400`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400)),
  plays its ~0.3s arming tune, then its armed 0.5s signal-loss timeout fires since no frame ever
  validated
  ([`main.c#L1992-L2004`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2004)),
  resetting it - that ~1.86s chain, then the reset's own 600-602ms startup tune (interrupts
  disabled). Before the reset, the ESC received our frames the whole time and rejected every one
  (the pre-reboot stretch is 85-90% our own transmission echoed back). One measured double-reset
  cycle was 2457ms apart, matching ~1.86s chain + ~0.60s tune. After a reset it often (not always)
  starts replying again within 61-96ms and arms about a second later.
- **Every logged reset instrumented under BUG-002 has been pre-first-contact.** With the extended
  `reboot_log()` (see "Instrumentation" below), across roughly 90 runs, the "gap" source (a reboot
  interrupting an *already-established* reply streak) has never fired - every reset recorded is
  "low" (before this motor's first-ever reply). Once an ESC has replied even once in a run, it has
  not been observed to reset again. The whole reset problem, on the data gathered so far, is
  first-contact rejection followed by AM32's own arm-then-0.5s-timeout chain - not an ESC dropping
  out after it was already accepted.
- **First-contact acceptance rate drifts over time, cause unconfirmed.**

  | How the ESC's listening period started | Accepted, 2026-09-27 17:57-18:34 | Accepted, 18:49-20:19 |
  |---|---|---|
  | First contact (already idle at `arm()`) | 0 of 28 | 12 of 28 |
  | After tune (mid-startup-tune at `arm()`) | 6 of 7 | 12 of 13 |
  | After reset (reset while arming) | 29 of 49 | 16 of 27 |

  Only the first row moved (p ≈ 5e-5); after-tune and after-reset stayed flat. A controlled
  power-cycle A/B the same day found power-cycling alone makes no difference (3 of 6 vs 4 of 8, both
  far above the 0/28 "before" baseline) - so the 18:48 shift tracks something else (bench/ESC state
  over a session, temperature, an unrelated drift), not the power cycle. Not pursued further at the
  time; the A-E bisection below was meant to test fixes directly instead. **Caution: its 21%
  first-contact baseline is now stale.** A `2026-09-29` re-measurement (`scripts/arming_stats.py`,
  see "A one-time startup-sequence delay" below) found first contact at 78-94% depending on
  configuration - re-baseline before trusting any of the bisection's thresholds.
- **The sustained-low refusal shape recurs - not a one-off.** 4 confirmed instances
  (`2026-09-27_18-55-34`, `2026-09-28_09-14-43`, `09-15-33`, `09-17-11`), on both motors, all
  DSHOT600 two-bidirectional-motor sessions. Each rebooted once around the usual 1.86s mark, then
  never played another arming tune or settled into clean replies - instead settling (in 2 of the 3
  09-28 instances within about a second, the third more gradually) into a sustained, dominant `low`
  state for the rest of the 13-18s window. A line held low that long is past both AM32's ~600ms
  startup tune and the bootloader's own ~1s NACK-escape window - neither mechanism on its own
  explains the duration. New packet-timing instrumentation confirmed this isn't us falling behind:
  all three 09-28 refusals kept sending frames at a normal ~914-930us cadence throughout.
- **Ruled out:** BUG-004/B4 (the released-edge receiver bug) - both halves implemented and
  bench-verified, but the 09-28 refusals happened *with* the fix in place, so it's tested and
  insufficient, not untested. BUG-001 (disarm-hang fix) - symptom predates it and recurred after.
  A broken telemetry link - 97.7% CRC-valid in the one precisely-measured instance, always the same
  two-phase shape (CRC-failing prefix, then permanently exact `0xFFF`), never general corruption.
  A clean spin-up that quietly drops out - no affected log ever shows real, varying eRPM before the
  failure. A fixed-length arming window fixing this at any length - 2000ms vs 3000ms produced
  identical results, because the reset lands ~1.9s in and recovery takes ~2.5s more.

## Open questions

1. **Why does an ESC reject our frames on first contact, and why does it need two DSHOT600
   bidirectional pairs?** Leading hypothesis (never directly observed): an edge left in the ESC's
   input capture around when our first frame arrives.
   - AM32 re-arms its capture right after its startup tune
     ([`main.c#L1893-L1933`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1933));
     an ESC that has waited on an idle line since then has also recorded any edge the line made
     meanwhile.
   - One left-over edge shifts every capture after it. The ESC then learns its one-time
     frame-length window from misaligned captures and rejects correct frames until it resets
     ([`signal.c#L166-L174`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174)).
   - Needs the second line to be involved: single-motor runs make the same edges on their own line
     and have never failed.

   See "Next: find the fix" below for the still-unrun bisection meant to narrow this down, and "A
   one-time startup-sequence delay" for a newly-found, unconfirmed candidate contributor.
2. **The sustained-low refusal shape's mechanism.** Possibly the bootloader (it answers unparseable
   bytes with 0xC1/0xC2 NACKs at 19200 baud, each holding the line low 52-260us,
   [`bootloader/main.c#L555-L565`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L555-L565))
   but unconfirmed - the bootloader should escape after ~100 NACKs (~1s,
   [`bootloader/main.c#L1275-L1286`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1275-L1286)),
   which doesn't match a 13-18s tail. The frame receiver's 21-bit-per-frame design cannot
   distinguish "line held continuously low" from "bursty NACK pulses" even in principle - only a raw
   pulse-width capture (a spare-PIO-block listener, or a logic analyzer) can settle this. A caution
   for any hardware change here, such as adding a pull-up: AM32's bootloader stays put if the line
   reads high and never low
   ([`bootloader/main.c#L1095-L1157`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1095-L1157)) -
   today the line between runs is driven low by `stop()`, then held by the RP2350's default
   pull-down; a pull-up before `arm()` would likely park a resetting ESC in its bootloader instead.
3. **What changed at 18:48 on 2026-09-27**, and whatever is behind 2026-09-29's much higher
   first-contact acceptance (see below) - not pursued as its own investigation, but a live confound
   for reading any A/B result against the old baseline.
4. **Pre-arm instrumentation, not yet built.** Nothing currently records anything between the
   Pico's reset and `arm()`, which is exactly the window open question 1's edge would form in.
   Candidates: log `ticks_us` at each pin-state transition from construction through
   `start()`/first TX put; have `run_test.py` stamp `run.log` with the PC's wall clock, to see how
   long each ESC sat in its post-disarm cycle before our first frame hit it (currently uncontrolled -
   "the usual spacing between runs").

## A one-time startup-sequence delay from the class-bin diagnostic (2026-09-29, unconfirmed candidate)

Adding ground-truth capture classification (`CaptureMailbox.enable_class_bins()`, below) measurably
changed the arming sequence's own timing. Investigated directly rather than assumed, since a
diagnostic that changes what it measures is exactly the kind of mistake this project has been
burned by before (see the `ARM_GAP_TOLERANCE_MS` removal under "The fix").

**The delay is real and precisely measured.** Comparing the time from `arm()` to the first captured
`arming.bin` record (isolates the cost to before `state=ARMING` is even set, so tick period after
that point can't be confounding it), across all 16 rounds per configuration (2 batches x 8 rounds,
interleaved, outcome doesn't affect this number so no runs excluded):

| Configuration | arm() -> first record (mean) |
|---|---|
| bins off, default tick | 3.35ms |
| bins off, padded tick (`core1_interval_us=130`) | 4.24ms |
| bins on | **22.79ms** |

Padding (a longer per-tick sleep, no classification) adds under 1ms here, confirming the extra
~19ms bins-on incurs is a one-time cost inside `arm()` - two `enable_class_bins()` calls plus two
`CaptureMailbox.reset()` zeroing loops (900 array elements each), not tick period accumulating.

**The mechanism, stated more carefully than the first pass.** `arm()`'s
`for motor in self.motors: motor.start()` loop runs one motor at a time, and only a bidirectional
motor's `start()` pays the zeroing cost. `state` stays `DISARMED` throughout this whole loop, and
`update()` no-ops while disarmed, so no frame reaches either ESC until `state` becomes `ARMING`,
at which point *both* motors get their first frame in the same `update()` call - there's no
evidence this staggers a frame arriving at one ESC versus the other. What does differ is each
motor's own *`start()`-to-`ARMING`* interval: motor 0's `start()` runs first in the loop, so by the
time `ARMING` is reached it has also waited out motor 2's `start()` (and its own zeroing again) -
roughly the full ~19-25ms delay. Motor 2's `start()` runs later, so it waits out much less -
roughly half that, ~10-13ms (these are reasoned from the loop order, not independently measured;
no per-motor timestamp exists yet to confirm the split directly). Whether that per-motor asymmetry,
or just the ~19ms shift in when our first frame reaches *either* ESC relative to `arm()`, matters to
AM32's first-contact acceptance is untested. There's no evidence of the delay creating a fresh edge
on the line either - the pull-up is applied once, in the constructor, so `start()`'s repeated
`pin.init(PULL_UP)` on an already-high line is unlikely to toggle anything; this hasn't been checked
against the PIO/GPIO FUNCSEL handoff directly, though.

**A same-day re-measurement with `scripts/arming_stats.py`** (extended to group by
`arming_class_bin_width_us`, which it didn't read before) gives real first-contact numbers instead
of a felt "far higher": first contact accepted 25 of 30 (83%) bins off, 25 of 32 (78%) padded, 28 of
32 (88%) bins on. Split per motor, the pattern is at least directionally consistent with the
mechanism above - motor 0 (the larger start()-to-ARMING delay) improved more than motor 2 between
bins-off and bins-on (87% -> 94%, vs motor 2's 80% -> 81%, essentially flat) - but with only 15-16
decided periods per cell this is a couple of periods' difference, not a finding.

**Formally, none of this reset-rate or acceptance difference is statistically significant at this
sample size.** The reset-rate split (3 of 16 bins-on vs pooled 6+6 of 32 bins-off) is a one-sided
hypergeometric p ≈ 0.16. Detecting an 18.75% vs 37.5% effect reliably needs roughly 70 runs per
configuration at 80% power - 16 per arm only rules out a much larger effect.

**Two claims from the first pass at this section were wrong and are corrected here:**
- **The 09-28-morning-to-09-29 refusal drop is real (3 of 6 -> 0 of 88 since, Fisher p ≈ 0.0014,
  now folded into "Status" above), but neither the `disarm()` pacing fix nor the class-bin
  diagnostic explains it** - the 09-28 *evening* batch (bins on, before the `disarm()` fix existed)
  already refused 0 of 39 that same day. The 09-28 morning batch looks like an isolated cluster, not
  a "before" state that later code changes fixed.
- **The bins-off tick-period rise even with the diagnostic fully disabled (09-28 morning
  ~906-936us -> 09-29 bins-off ~979-1049us) is more plausibly this session's own disabled-path
  overhead in the modified `drain()`/`update()`** (a few extra attribute reads and branches every
  call, even unused) **than environmental drift** - retracted as evidence of drift.

**Next test, not yet run:** with class bins off, interleave (a) a uniform sleep after all `start()`
calls, before `state=ARMING` (tests whether total delay before the first frame matters) against
(b) a sleep specifically between the two bidirectional motors' `start()` calls only (tests the
per-motor asymmetry specifically) - both against a freshly-run bins-off baseline, sized for the
effect actually seen (roughly 70 runs/configuration), not 20.

## Next: find the fix (the two-bidirectional-pairs trigger)

Wiring and power are ruled out: unidirectional DShot spins both motors cleanly, the same
neighbouring wire is active when nothing fails, and the other ESC's replies make no measurable
difference (63% vs 61% acceptance). What triggers rejection is specifically a second bidirectional
pair at DSHOT600. **Configurations, still unrun as an interleaved bisection:**

| | Scenario | What changes from A |
|---|---|---|
| A | `two_channel_arming_check_600` | nothing: the baseline |
| B | `two_channel_arming_check_300` | DSHOT300 |
| C | `two_channel_arming_check_600_same_block` | both bidirectional pairs on PIO0 instead of PIO0+PIO1 |
| D | `two_channel_arming_check_600_spaced` | motor 2's frame goes out 300us after motor 0's while arming (`arming_frame_gap_us`) |
| E | `single_channel_bidirectional_600` | control: one bidirectional pair only |

Procedure: 10+ rounds of A,B,C,D,E interleaved (not blocked - first-contact acceptance drifts), via
`scripts/run_test.py --scenario ...`, read with `scripts/arming_stats.py --since <session>`. **Must
be re-baselined first** - the 21%-acceptance baseline this was sized against is stale (see above).
Reading: E high + A low confirms the second pair as trigger (expected). B high -> DSHOT600-specific
timing fix. D high -> frame spacing is the fix, make it the default. C different from A -> per-block
PIO setup matters. Nothing moves -> bisect the receiver itself, then scope both lines directly.

## Instrumentation available

- **`scripts/classify_reply_timeline.py --from-arm`**: classifies every capture.bin/arming.bin
  record as spin/stop/echo/low/garbled from the sampled capture log. Works on a run that never
  armed. Sampled at whatever rate the application happened to poll - not ground truth.
- **Packet-send timing** (`run_scenario.py`'s `arming_call_stats`, commit `243c585`): count/min/max/
  avg time between `update()` calls while arming, measured on Core 1 - ground truth for TX cadence.
- **`CaptureMailbox.enable_class_bins()`** (`driver/capture_mailbox.py`): ground-truth classification
  of *every* capture `drain()` takes (not just published ones) into not_running/zero("low")/other,
  bucketed over time. Scenario-configurable via `arming_class_bin_width_us` (default on, 100000 =
  100ms bins; 0 disables it). **Costs ~19ms once per `arm()` plus ~50-130us/tick while enabled** -
  see "A one-time startup-sequence delay" above; account for this before comparing timing-sensitive
  results against a bins-off run.
- **`MotorGroup.reboot_log(index)`**: merges two sources into one sorted `(ms_since_arm,
  duration_ms, source)` list - `"gap"` (a reboot-length gap in an already-established not_running
  reply streak - the arming gate's own bookkeeping) and `"low"` (a run of low-classified captures
  ending - AM32's startup-tune signature, observed directly, catches resets before first contact
  too). A third label, `"low-open"`, reports a streak still running at call time - this is what
  makes a refusal's `reboot_log()` non-empty instead of silently missing the one state that matters
  most. **Caveats:** the `"low"` source needs `arming_class_bin_width_us` set. Sessions captured
  before 2026-09-29's fix synchronizing `class_bin_t0_us` across motors have each bidirectional
  motor's `"low"` timestamps relative to *that motor's own* `start()`, not a shared zero point - do
  not compare cross-motor timing from those older sessions. Neither the t0-sync fix nor `"low-open"`
  have been hardware-verified yet as of this writing.
- **`disarm()` round-pacing** (`DISARM_ROUND_GAP_US`): not a read instrument, but changes what state
  the ESC is in entering the *next* `arm()` - a real variable across any batch that spans a
  `disarm()`, see the confound discussion above.

## Verification

| Batch | Date | Config | Runs | Armed & spun | Refused |
|---|---|---|---|---|---|
| First verification + 10-run samples | 09-27 | 2 bidir | 26 | 23 | 3 |
| Post-fix references | 09-27 | 1 bidir | 2 | 2 | 0 |
| Power-cycle A/B | 09-27 | 2 bidir | 10 | 10 | 0 |
| Packet-timing verification batch | 09-28 morning | 2 bidir | 6 | 3 | 3 |
| Class-bin verification (various) | 09-28 evening | 2 bidir | 39 | 39 | 0 |
| Sanity check + confound A/B, two batches | 09-29 morning | 2 bidir | 49 | 49 | 0 |
| **Total** | | | **132** | **126** | **6** |

`ARMED` time is a direct function of reboot count before the last one settled: ~2.1-2.6s with none,
~4.0-4.8s with one, ~7.0s with two, ~9.5s with three. Every motor that reached `ARMED` then spun
with a real, distinct, CRC-valid eRPM - no exceptions across any batch.

**A genuinely unpowered ESC** produces its own distinct, easily-told-apart signature: 100%
echo/garbled captures, **no reboot (`low`) events at all**, for the entire window - unlike a stuck
reboot loop, which always shows `low` events.

## Evidence

Only the 2026-09-27 batch is committed. Everything from 2026-09-28 morning onward is pulled locally
only, not yet committed. Regenerate any capture-log-based table with:

```
python scripts/classify_reply_timeline.py --from-arm captures/<session>
```

(`--from-arm` works on a run that never armed too; add `--window-ms 20` for finer resolution on a
mixed/unstable pattern.)

Session ids, by batch:
- 09-27 first verification + 10-run samples: `17-57-01` through `18-04-36`, `18-27-22` through
  `18-31-57`, `18-49-51` through `18-59-14`. Refused: `17-58-01`, `18-28-28`, `18-55-34`.
  Excluded from the tally: `18-34-47` (12s-floor test, not a normal arming attempt) and `18-48-15`
  (ESC confirmed powered off - kept as the "unpowered" reference signature).
- 09-27 power-cycle A/B (does not replicate the 18:48 finding as power-cycle-caused): `20-09-49`
  through `20-19-17`, 10 sessions.
- 09-28 morning, packet-timing verification (3 of 6 refused - the sustained-low shape recurring):
  `09-10-50`, `09-14-09`, `09-14-43` (refused), `09-15-33` (refused), `09-16-23`, `09-17-11` (refused).
- 09-28 evening, class-bin verification: `20-09-06` through `20-41-14` (39 sessions).
- 09-29 sanity check: `07-33-06`. Confound A/B, round-robin bins-off / bins-off-padded / bins-on,
  repeated twice: batch 1 starts `07-33-32`, batch 2 starts `07-46-31` (24 sessions each, 8 rounds
  of the 3-way rotation per batch).

The full, unabridged investigation log (every hypothesis tried, every intermediate finding) is
preserved in git history: `git show 2c94f2f:bug-reports/BUG-002-bidirectional-motor-intermittently-does-not-spin.md`.

## Related

- BUG-001 (disarm-hang fix) - ruled out as cause or fix, see above.
- BUG-003 (open-loop arming) - the fix for this bug.
- BUG-004 (released-edge receiver bug) - implemented, bench-verified, does not clear this bug.
