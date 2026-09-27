# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status: MITIGATED.** The failure this bug reports - a motor silently never spinning while its
ESC keeps replying with valid, CRC-good telemetry - has not recurred once across 28 bench runs.
That's because arming no longer trusts a timer alone; it waits for evidence that the ESC is
actually listening (BUG-003). What's still open is *why* the ESC sometimes doesn't listen for the
first several seconds after `arm()` - that root cause is unconfirmed, and a motor stuck in that
state for too long still won't fly this run, just loudly instead of silently.

**The headline numbers, across all runs of the fix:** the mid-arming reset itself still happens in
most two-bidirectional-motor runs - 21 of the 26 two-motor sessions below show at least one motor
going quiet ~1.3-2.1s after `arm()` (the remaining 5 only show a motor still finishing a reboot
that was already in progress when `arm()` was called - a leftover from the previous session's
disarm, not a fresh mid-arming reset; those 5 all armed in 2.3-2.6s). What's changed is what
happens next: instead of silently arming and never spinning, the group now waits it out, turning
the reset into a 2-9s delay before a normal spin-up. Across all 28 bench sessions (26 with two
bidirectional motors, 2 with one), 25 armed and spun correctly, and 3 refused to arm rather than
spinning - all three on the two-motor configuration, all three because one motor kept failing to
settle within the timeout. Zero sessions armed a motor that then failed to spin. At the current
18s timeout, 1 run in 10 (two-motor configuration) still exceeds it.

## Symptom

A bidirectional ESC, commanded a normal throttle after arming, sometimes never spins. It keeps
replying with CRC-valid telemetry the entire time - the constant `0xFFF`/917 eRPM frame AM32 sends
for "motor not running," whether armed or disarmed. Nothing on the wire looks broken: the link is
healthy, the ESC is clearly listening and replying, it simply never starts the motor. This
happened intermittently enough, and looked healthy enough on the telemetry side, that it wasn't
obviously distinguishable from a disarmed or idle ESC until captured and analyzed directly.

## What the data shows about the cause

Established from bench experiments and 28 runs' worth of arming-phase capture logs (raw captures
are committed - see "Evidence" below for how to regenerate any of this):

- **It needs two concurrently-active bidirectional TX/RX pairs at DSHOT600 - not motor load, and
  not deterministic even there.** A dedicated scenario with one motor spinning and the second
  bidirectional line merely present but held at zero throttle reproduced it 3/3 times. Every
  recorded failure, across this bug's whole history, was on a two-bidirectional-motor DSHOT600
  scenario; the DSHOT300 two-motor scenario and every single-motor scenario (either speed) have
  passed cleanly in the same test suites where a DSHOT600 two-motor scenario failed - including,
  at least once, two DSHOT600 two-motor scenarios run back to back where one failed and the other
  passed. So the trigger needs this configuration, but doesn't reproduce every time it runs.
- **Ruled out: tick period alone.** A padded single-motor tick slower than what two real
  bidirectional pairs naturally produce still passed, while the faster two-motor tick failed -
  directly falsifying "crossing some tick-period threshold" as the trigger.
- **Ruled out: BUG-001 (the disarm-hang fix) or its absence.** The symptom predates that fix and
  recurred after it landed; the two are independent.
- **Ruled out: a broken telemetry link.** In the one precisely-measured instance, 97.7% of decoded
  replies were CRC-valid; per-record offline re-analysis of every affected log found the same
  two-phase shape (a CRC-failing prefix, then permanently exact, CRC-valid `0xFFF`), never a
  generally corrupted stream. This is an ESC that is armed and communicating normally while not
  driving the motor, not a broken link.
- **Ruled out: a clean spin-up that quietly drops out.** No affected log ever shows a window of
  real, varying eRPM - only silence or the not-running sentinel from the start. Whatever happens,
  happens before the motor is ever seen to turn at all.
- **A fixed-length arming window can't fix this, whatever length it is.** Changing
  `arm_duration_ms` from 2000ms to 3000ms produced the identical pass/fail pattern on the
  identical scenarios - because the reset lands around 1.9s into arming and recovery takes about
  2.5s more, so no single fixed timer reliably outlasts it. That's the reason the fix waits on
  evidence (BUG-003) rather than trying to guess a longer number.
- **What actually happens, per the arming-phase logs:** the affected ESC goes quiet for
  ~600-700ms partway through arming - the signature of AM32's own startup tune, played with
  interrupts disabled, meaning it just rebooted. Before that reboot, the ESC never once accepted
  our frames - the pre-reboot stretch is 85-90% our own transmission echoed back, the rest
  garbled, never once the ESC's actual reply - it was not "working, then dropping out," it had not
  yet started listening at all. After the reboot it usually settles down and starts replying
  normally within about a second. In one observed session (`2026-09-27_18-55-34`), a motor
  rebooted once and then never settled into clean replies for the rest of an 18s window instead -
  re-checked at finer time resolution, it shows short (20-40ms), scattered `low` blips recurring
  throughout the whole stretch, mixed with echo and occasional garbled captures - a persistently
  unstable reception, closer to a bootloader answering our frames with its own NACK bytes than to
  a clean recovery, though that specific mechanism is not confirmed. That's a second, less
  understood failure shape.
- **Not yet explained: why the ESC reboots during arming at all**, or why it's specifically the
  two-bidirectional-pairs configuration that triggers it. The cheapest untested lever is a
  hardware one: fitting a 2.2-4.7kΩ pull-up to 3.3V on each bidirectional signal line (stiffening
  the released line against coupling from the other one) and re-running the two-bidirectional
  scenario. That needs someone at the bench, not code.

## The fix

**BUG-003's evidence-gated arming is what fixed the silent-no-spin symptom** (`driver/motor_group.py`,
commit `27722bb`). Before it, a group armed on a timer alone; a bidirectional ESC that rebooted
mid-arming would still get promoted to `ARMED` on schedule and then get sent nonzero throttle
before it had satisfied its own internal arming gate - so it never spun, despite replying with
valid telemetry throughout. Now `ARMED` additionally requires every bidirectional motor to have
replied steadily, with no gap longer than the reboot's own signature, for two full seconds. An ESC
that's mid-reboot simply keeps the group in `ARMING`, sending zeros, until it settles or the
application's own timeout gives up and names the motor still not ready.

**A separate reliability fix landed the same day, not what fixed the original symptom:** the
arming *floor* used to restart its own clock on any gap over 10ms between `update()` calls, on the
mistaken assumption (already flagged as ungrounded in `specification/AM32_SOURCE_VERIFICATION.md`
before this was tested) that AM32 resets its own arming counter the same way. It doesn't. A bench
test that widened the floor to 12s to check whether patience alone resolves a reboot loop timed
out anyway, even though both ESCs' own reply-gate evidence showed them replying steadily for 10.1s
and 17.5s respectively by the time it gave up - well past the 2s the gate itself requires. The
likely explanation - fits the data, not independently confirmed - is that the floor's own clock
kept restarting on ordinary Core 1 scheduling gaps
before it could ever run 12s uninterrupted. Fixed by removing the gap-reset
(`ARM_GAP_TOLERANCE_MS` is gone); the floor is now plain elapsed time since `arm()`. That same 12s
floor test was never re-run after the fix, so this mechanism hasn't been directly re-confirmed -
only inferred from the one pre-fix run that exposed it.

**The harness's arming timeout (how long to wait before giving up) is the application's own
choice, not the library's** - `MotorGroup` will wait forever for evidence, by design. The bench
harness raised its own timeout from 10s to 18s, sized off the ~2.46s reboot-cycle period measured
across these runs (enough margin for about 6 reboots). **Whether this specific change has mattered
yet is unproven:** no post-fix run has taken longer than 9.46s to arm, so the extra headroom has
not yet been the difference between a run arming and timing out, and the refusal rate is the same
1-in-10 before and after this change. What does suggest it's the right precaution: the same
12s-floor test above showed one motor's reply-gate evidence wasn't satisfied until ~11.9s after
`arm()` (4 reboots) - past the old 10s ceiling, inside the new 18s one. That's a data point in
favor of the wider timeout, from a run that wasn't otherwise valid for the main tally (see
"Evidence" below), not proof the wider timeout has already earned its keep.

**A stuck-at-rest failsafe** (aborts a run early if a commanded motor never once shows a real,
non-sentinel eRPM) is a safety net for the harness, not part of the arming fix - it has not yet
been exercised by any run, since every run so far either armed cleanly or failed during arming
itself.

## Verification

| Batch | Config | Timeout ceiling | Runs | Armed & spun | Refused to arm |
|---|---|---|---|---|---|
| First verification batch, mixed scenarios | 2 bidirectional | 10s | 6 | 5 | 1 |
| First verification batch, reference | 1 bidirectional | 10s | 1 | 1 | 0 |
| 10-run sample, pre-gap-reset-fix | 2 bidirectional | 10s | 10 | 9 | 1 |
| 10-run sample, post-gap-reset-fix | 2 bidirectional | 18s | 10 | 9 | 1 |
| Post-fix reference | 1 bidirectional | 18s | 1 | 1 | 0 |
| **Total** | | | **28** | **25** | **3** |

`ARMED` time is a direct function of how many reboots happened before the last one settled: ~2.1
-2.6s with none observed in the window, ~4.0-4.8s with one, ~7.0s with two, ~9.5s with three. Every
motor that reached `ARMED` then spun with a real, distinct, CRC-valid eRPM - no exceptions.

**The 3 refusals, and how to tell them apart from each other and from a hardware problem:**
- Two (`2026-09-27_17-58-01`, `2026-09-27_18-28-28`) were a motor still cycling through repeated
  reboots (3-4 of them) when the deadline hit, never completing one full 2s settle. The traceback
  names the motor directly, e.g. `arming did not complete within 10000ms; per motor
  (replying_for_ms, last_reply_ms_ago), None = no reply: [(7466, 1), None, (89, 1), None]` - motor
  0 had been ready for 7.5s, motor 2 had barely started its latest attempt.
- One (`2026-09-27_18-55-34`) was the mixed echo/reboot pattern described above - not still
  cycling cleanly, just never stabilizing.
- All three refusals were motor 2 (the second bidirectional channel, pin 8). That's a real pattern
  worth keeping in mind, but the sample is small (3 events) and it isn't a general "this motor
  reboots more" rule - in several passing runs motor 0 rebooted more times than motor 2 did.
- **For contrast, a genuinely unpowered ESC** (confirmed - the user had powered it off) produces
  its own distinct signature: 100% echo/garbled captures, **no reboot (`low`) events at all**, for
  the entire window. That's how to tell "the ESC isn't there" apart from "the ESC is stuck
  rebooting" at a glance.

## Evidence

Every session referenced above is committed under `captures/<session>/` (`capture.bin`,
`arming.bin`, `meta.txt`, `scenario.json`). Regenerate any table above with:

```
python scripts/classify_reply_timeline.py --from-arm captures/<session>
```

(omit `--from-arm` for a session that never reached `ARMED`; add `--window-ms 20` for finer
resolution on a mixed/unstable pattern like `18-55-34`'s).

Most sessions also carry `run.log`, the run's console output, to cross-check against the tables
above - but not all the same way. The first verification batch (`17-57-01` through `18-04-36`) has
no `run.log` at all - that feature didn't exist yet. `18-27-22` through `18-50-45` were
reconstructed from output already visible in the session transcript (two, `18-34-47` and
`18-48-15`, carry an added `NOTE` line for context). `18-55-03` onward were written automatically
by `run_test.py` itself, live, during the run.

Session ids, by batch:
- First verification batch, mixed scenarios: `2026-09-27_17-57-01`, `17-58-01` (refused),
  `17-58-55`, `17-59-55`, `18-02-11`, `18-03-28`, `18-04-36` (reference)
- 10-run sample, pre-fix: `18-27-22`, `18-27-51`, `18-28-28` (refused), `18-29-02`, `18-29-34`,
  `18-30-01`, `18-30-30`, `18-30-59`, `18-31-31`, `18-31-57`
- 10-run sample, post-fix: `18-49-51`, `18-50-17`, `18-50-45`, `18-55-03`, `18-55-34` (refused),
  `18-56-36`, `18-57-06`, `18-57-33`, `18-58-00`, `18-58-34`
- Post-fix reference: `18-59-14`
- **Committed but excluded from the tally above:** `18-34-47` (the 12s-floor test, run on
  pre-gap-reset-fix code with a scenario file since deleted - this is the session that exposed the
  floor-starvation issue, not a normal arming attempt) and `18-48-15` (the ESC was confirmed
  powered off during this run - kept as the reference signature for "unpowered," not a driver
  failure).

The full, unabridged investigation log (every hypothesis tried, every intermediate finding, the
original verification-request scaffolding) is preserved in git history:
`git show 2c94f2f:bug-reports/BUG-002-bidirectional-motor-intermittently-does-not-spin.md`.

## Related

- BUG-001 (disarm-hang fix) - ruled out as cause or fix, see above.
- BUG-003 (open-loop arming) - the fix for this bug.
