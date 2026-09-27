# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status: MITIGATED.** The failure this bug reports - a motor silently never spinning while its
ESC keeps replying with valid, CRC-good telemetry - has not recurred once across 28 bench runs.
That's because arming no longer trusts a timer alone; it waits for evidence that the ESC is
actually listening (BUG-003). What's still open is *why* the ESC so often rejects our frames when
our signal starts - that root cause is unconfirmed, and a motor stuck in that state for too long
still won't fly this run, just loudly instead of silently.

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
  interrupts disabled, meaning it just rebooted. Before that reboot, the ESC never once replied -
  the pre-reboot stretch is 85-90% our own transmission echoed back, the rest garbled - so it was
  not "working, then dropping out": it received our frames and rejected every one.
- **It most likely armed anyway, and that is what resets it.** AM32 arms after >1 s of captures
  without validating any of them. Its gate needs only a detected input, a zero throttle (never
  updated by a rejected frame) and more than 30 captures
  ([`main.c#L1360-L1400`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400),
  [`signal.c#L166-L178`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L178)).
  - It then plays its ~0.3 s arming tune, invisible here because it was not replying anyway.
  - Its armed 0.5 s signal timeout then fires, as no frame passes validation
    ([`main.c#L1992-L2004`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2004)).
  - That chain is about 1.0 + 0.3 + 0.5 s. Measured: 60 of 62 resets came 1.854-1.859 s after the
    ESC started listening (the other 2 at 1.347 s).
  - The disarmed timeout would take at least 2.0 s.
- **After a reboot it often, not always, settles.** It then starts replying within 61-96 ms (its
  bidirectional latch) and arms about a second later.
- **Where the rejections concentrate: when our signal starts.** Every listening period in the 29
  sessions with an arming log (56 motor-runs; `18-48-15`, ESC unpowered, left out):

  | How the ESC's listening period started | Periods | Accepted | Rejected |
  |---|---|---|---|
  | At `arm()`, the ESC already listening on an idle line | 42 | 5 | 37 |
  | After a startup tune already playing at `arm()` | 14 | 13 | 1 |
  | After a reset inside the arming window | 63 | 37 | 25 (+1, `18-55-34`) |

  - An ESC that meets our first frame rejects it 37 times in 42. One that boots into our running
    signal rejects it 26 times in 76.
  - Each reboot is a fresh draw at about those odds, which is why some runs needed 3-4 resets.
  - Both ESCs behave alike. At `arm()`, motor 0 rejected 19 times and motor 2 18 times; after a
    reset, 13 and 12.
  - All 5 accepts at `arm()` came after the ESC power cycle (`18-48-15`): 5 of 14 after, 0 of 28
    before. No explanation yet.
- **Leading hypothesis (not observed): an edge left in the ESC's input capture when our first
  frame arrives.**
  - AM32 re-arms its capture right after its startup tune
    ([`main.c#L1893-L1933`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1933)).
    An ESC that has waited on an idle line since then has also recorded any edge the line made
    meanwhile.
  - One left-over edge shifts every capture after it. The ESC then learns its one-time
    frame-length window from misaligned captures and rejects correct frames until it resets
    ([`signal.c#L166-L174`](https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174)).
  - Candidates on our side: each `BidirectionalDShot` constructor enabling its pull-up shortly
    before `arm()`, and `start()`.
  - Single-motor runs have never failed, though, and they make the same edges on their own
    line. So the second line has to be involved: coupling from it around start-up, or during
    the first frames.
- **The `18-55-34` failure is a second, less understood shape, possibly the bootloader.**
  - **What the line did.** Motor 2 rebooted at 1.854 s after `arm()`, then rejected our frames
    again. At **4.32 s**, exactly when its next reset was due, no startup tune followed. For the
    remaining 13.7 s the line held a steady mix: ~62% echo, ~20% `low`, ~17% garbled. The `low`
    words were mostly single captures, a median 10.8 ms apart.
  - **Why the bootloader.** No firmware state we know of produces that mix, and the bootloader has
    no timeout. It answers bytes it cannot parse with 0xC1/0xC2 NACKs at 19200 baud
    ([`bootloader/main.c#L555-L565`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L555-L565)).
    Each NACK holds the line low in 52-260 µs stretches.
  - **Unconfirmed**, because the bootloader should have started the firmware after 100 NACKs,
    about a second at that rate
    ([`bootloader/main.c#L1275-L1286`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1275-L1286)).
- **Not yet explained: why the ESC rejects our frames**, or why it takes the
  two-bidirectional-pairs configuration. Next, at the bench:
  - **A scope on both signal lines comes first.** Capture from before the motors are constructed
    (before `Arming for ...` prints) through the first 20 ms of frames. Report every transition
    on either line before its first frame, and any transition on a line between its own frames.
    Also trigger on a reset, capturing the 100 ms before a startup tune, to see the bootloader's
    line check. A recurrence of `18-55-34`'s state is worth a capture too: bootloader bytes are
    52 µs per bit and obvious on a scope.
  - **Then the pull-up test,** with a caution. A 2.2-4.7kΩ pull-up to 3.3V on each bidirectional
    line stiffens the released line against coupling, but it also changes what the ESC is doing
    at `arm()`.
    - **The bootloader's rule.** After a software reset it checks the line for up to ~50 ms, and
      stays put if the line reads high and never low. That is the configurator's entry condition
      ([`bootloader/main.c#L1095-L1157`](https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1095-L1157)).
    - **Today.** The line between runs is low: `stop()` drives it low, and after the Pico's reset
      the RP2350's default pull-down holds it low. So an idle ESC boot-loops through its firmware,
      mid-tune at `arm()` in 14 of 56 motor-runs.
    - **With the pull-up.** The line sits high for the seconds between the Pico's reset and
      `arm()`. An ESC that resets in that time likely parks in its bootloader and meets our first
      frame there, so the arming logs must be checked for that before reading a pass or fail.
  - **Waiting longer before `arm()` has the same problem.** A line held idle-high with no frames
    lets an ESC reset into its bootloader.

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

(`--from-arm` also works on a session that never reached `ARMED`: newer ones record `arm()`'s time
on failure, and older ones use their first arming capture, within ~1 ms of it; add
`--window-ms 20` for finer resolution on a mixed/unstable pattern like `18-55-34`'s).

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
