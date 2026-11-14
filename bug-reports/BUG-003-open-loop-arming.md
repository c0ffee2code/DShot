# BUG-003: Arming is open-loop; nothing checks that the ESC actually armed

**Status:** FIX IMPLEMENTED AND BENCH-VERIFIED 2026-09-27 (a pre-fix round and two 10-run post-fix
samples: 28 runs total, 0 silent failures; not retained in BUG-002.md's later distillation). Fix plan steps 1, 2, 3, 5 and
7 are in; step 6 (a 3000 ms default for unidirectional-only groups) is not. It is the fix for
BUG-002 as well.

**Follow-up fix, same day:** the arming *floor* (separate from the reply gate this bug added) used
to restart its own clock on any gap over 10 ms between `update()` calls
(`MotorGroup.ARM_GAP_TOLERANCE_MS`), on the assumption that AM32 resets its arming counter the same
way. It does not (source-verified, `specification/AM32.md`#16) - only non-zero throttle or a real
reboot resets it, and the reboot case is what the reply gate below already handles. A bench test widening the floor to 12 s exposed the consequence directly: both ESCs had
long since satisfied the reply gate, but the floor itself never completed, because at least one
ordinary Core 1 scheduling gap landed somewhere in the window and restarted it. Removed the
gap-reset entirely; the floor is now plain elapsed time since `arm()`.

**As implemented** (small departures from the plan below):
- `CaptureMailbox.drain()` returns how many captures were `AM32_NOT_RUNNING_FRAME`.
  `MotorGroup.update()` keeps the evidence per motor in ticks_ms (`ready_first_ms`,
  `ready_last_ms`, `ready_seen`) from the tick's own `now`, so the mailbox needs no second clock
  call.
- `MotorGroup.READY_SPAN_MS = 2000`, `READY_GAP_MS = 450`, `READY_FRESH_MS = 50`;
  `bidir_ready(now)` checks them once the floor has passed.
- `MotorGroup.arming_status()`: per motor, `None`, or `(replying_for_ms, last_reply_ms_ago)`.
  The harness prints it when arming times out, now after `arm_duration_ms + 8000` ms (since
  raised to `arm_duration_ms + 16000`, `run_scenario.py`'s `ARM_TIMEOUT_MARGIN_MS`).
- `MotorGroup.wait_for_replies` (default `True`) keeps the gate on; set it `False` to turn the
  gate off. Only `tests/device/test_pio_lifecycle.py` does that, because it has no ESC attached.
  *Update 2026-11-13:* the attribute no longer exists. Bench code with no ESC attached uses
  `GroupWithoutReplyGate` (`tests/bench/group_without_reply_gate.py`), a `MotorGroup` whose
  `bidir_ready()` returns True.
- Unit tests: `ArmingGateTest` in `tests/unit/test_motor_group.py`, the not-running count in
  `test_capture_mailbox.py`, and the constant's derivation in `test_gcr_decode.py`.
- *Paths (2026-11-11):* `tests/unit/` is now `tests/pc/`, and `tests/device/test_pio_lifecycle.py`
  is now `tests/bench/pio_and_cores/test_pio_lifecycle.py` (tests restructured in commit
  `2cbde0c`). `CaptureMailbox` has two methods now: `drain()` publishes once armed and
  `discard()` drops replies while arming, and both return the not-running count (ADR-005,
  2026-10-17 update).
**Severity:** High - design gap. Any delay on the ESC side means the motor never arms, and nothing
reports it.
**Component:** `driver/motor_group.py` (`arm()`, `update()`), `driver/capture_mailbox.py`
**Found:** 2026-09-27, comparing this driver's arming logic against Betaflight's own arming guards:
Betaflight gates arming on evidence - `isDshotTelemetryActive()` - rather than elapsed time alone,
where this driver only checked elapsed time against its own clock.

## Summary

`MotorGroup` declares `ARMED` when `arm_duration_ms` has passed since `arm()`. It never learns
whether the ESC has armed:

- replies received while arming are drained and discarded unread
  (`update()`'s `ARMING` branch, `motors[i].drain_rx(publish)` where `publish` is
  `self.publish_while_arming`, default `False`);
- the application may send non-zero throttle on the very next tick.

AM32 arms only after **more than 1.02 s of uninterrupted zero throttle, counted from when *it*
detects our signal** ([`main.c#L1360-L1400`][am32-arming]). It detects our signal only after
anything that delays it has finished:
- a reboot;
- its 600 ms startup tune, played with interrupts off;
- a stay in the bootloader.

Any non-zero frame before the gate completes resets the count. The ESC then stays disarmed for as
long as throttle stays non-zero, while replying "not running" (`0xFFF`) the whole time. Nothing on
our side notices.

Betaflight never relies on a timer alone
([`core.c#L319-L331`][bf-grace], [`core.c#L434-L440`][bf-telem-gate],
[`dshot_command.c#L150-L165`][bf-streaming]). It:
- streams MOTOR_STOP from boot;
- refuses to arm for 5 s after boot and 3 s after the motors are enabled;
- with bidirectional DShot, refuses to arm until every motor has returned a valid eRPM frame.

## When it bites

- **`arm()` soon after `disarm()`.** `disarm()` does not disarm AM32. Zero throttle keeps it armed,
  and the signal cut makes it reset itself 0.5 s later. It then plays its startup tune, deaf for
  ~0.65 s, and needs its full 1.02 s gate again. An `arm()` issued ~0.5-1.5 s after a `disarm()`
  with the 2000 ms default can end with the ESC still disarmed when throttle arrives.
  `test_bidir_restart_cycles.py` avoids this only because it waits 3 s and arms for 3 s.
- **An ESC that stays in its bootloader, or has a long custom startup tune.**
- **BUG-002.** Its R3 captures show an ESC resetting inside our arming window, 1.85-2.3 s after
  `arm()`, and coming back ~0.7 s later, after `ARMED`, under non-zero throttle. It never arms.
  What makes the ESC reset is still open, but it is this open-loop gate that turns the reset into
  a motor that never spins. The earlier note here, "not what triggers BUG-002", was based on
  the 3000/2000 ms A/B retest; a reset at ~1.85 s explains why both windows failed.
- **Between harness sessions nothing waits.** After `disarm()` an AM32 ESC with no valid signal
  reboots every ~2.6 s (0.5 s armed timeout, then startup tune, then a 2 s disarmed timeout).
  The next `arm()` meets it at a random point in that loop: about a quarter of the time it is in
  its 600 ms startup tune and hears nothing.
  *Observation 2026-11-11:* 3 of 15 bench runs (AM32 2.22, four bidirectional motors, DSHOT600) armed in 2630 to 2659 ms,
  inside the 3.0 s bound; the other 12 armed in 2112 to 2113 ms; one run did not arm within 18 s (BUG-009, observations of
  2026-11-11). The cause of the slower arming is not established.

## Fix plan

1. **Name the evidence.** While AM32's motor is stopped, every valid reply is one fixed 21-bit
   frame: payload `0xFFF` with its inverted CRC, which `dshot_bidir_rx_frame` pushes as
   **`0x52951`**. Add it to `gcr_decode.py` as `AM32_NOT_RUNNING_FRAME`, next to
   `AM32_NOT_RUNNING_DATA12`, with a unit test that derives it from
   the port of `make_dshot_package()` in `tests/pc/am32_reply.py` (formerly `scripts/verify_am32_reply.py`).
2. **Record it without decoding.** In `CaptureMailbox.drain()`, in both modes, compare each word
   with that constant. On a match, if the previous match is more than `READY_GAP_US` (450 ms) old,
   set `ready_first_us = now`; then set `ready_last_us = now`. That is one integer compare per
   capture and no allocation, within ADR-002's hot-path rules. `reset()` clears both.
   Our own throttle-0 echoes cannot produce `0x52951`; checked with the receiver model for DSHOT300
   and DSHOT600.
3. **Gate `ARMED` on it.** `update()` promotes `ARMING` to `ARMED` only when all of these hold:
   - `arm_duration_ms` has elapsed. It stays as a floor.
   - For every bidirectional motor, `ready_first_us` is set and
     `now - ready_first_us >= READY_SPAN_US` (**2000 ms**). The motor has been replying "not
     running", without a gap of 450 ms or more, for two seconds.
   - For every bidirectional motor, `now - ready_last_us <= READY_FRESH_US` (~50 ms). It is still
     replying right now.

   Until then the group stays in `ARMING` and keeps sending zeros, which is safe.

   **Why 2000 ms, measured from the first reply.** AM32 replies ~75 ms after it detects us, at its
   bidirectional latch ([`dshot.c#L86-L95`][am32-latch]).
   - **It arms ~0.97 s after that first reply.** Measured on this bench: motor 2 in BUG-002's
     `14-22-14` and `14-22-57`, 0.969 s and 0.971 s. The source gate is >1 s from detection
     ([`main.c#L1360-L1400`][am32-arming]).
   - **It then plays its arming tune** (~0.3 s; 0.27 s measured).
   - **An armed ESC that is not taking our frames resets 0.5 s later**
     ([`main.c#L1992-L2004`][am32-armed-timeout]). The chain ends ~1.8 s after the first reply.

   The bench since (28 runs, see BUG-002.md) moved where the chain starts. Every reset came from
   an ESC that had never replied: it had not accepted one of our frames. It most likely went
   through this chain anyway, because AM32 arms without validating frames
   ([`main.c#L1360-L1400`][am32-arming]). Counted from when it started listening, the chain
   (~1.0 s + ~0.3 s tune + 0.5 s) fits the 1.854-1.867 s measured in 75 of 77 resets. The
   disarmed timeout would take at least 2.0 s.

   A span of replies cannot see that case, since there are no replies. The span still does its
   job: a reboot's silence is >= 680 ms (the startup tune plus latch, below), so an ESC still
   cycling breaks its streak and drops out before `ARMED`, not after.

   **Why a 450 ms gap restarts the count.**
   - **It tolerates the arming tune.** The tune's gap is ~300 ms, so an ESC that arms mid-count is
     not penalised.
   - **It catches a reboot.** A reboot is silent for ≥ 680 ms: the 600 ms startup tune, then the
     ~80 ms latch. After it, the ESC needs its whole gate again.
   - **A long tune is still safe.** With low-voltage cutoff the tune repeats once per cell
     (cells × 400 ms) and exceeds 450 ms. The count then restarts after an ESC that has already
     armed. That costs 2 s, while zeros keep flowing.

     *Note 2026-11-11:* two texts disagree on the tune's length. This bullet says 400 ms per
     cell and cites no source; the link `[am32-inputtune]` at the end of this file is defined but
     not used anywhere in the text. `specification/AM32.md` §16 says the arming tune is three
     100 ms beeps, about 300 ms, repeated once per cell with the cutoff on, and cites
     `Src/sounds.c#L219-L237` and `Src/main.c#L1376-L1389`. The "Why 2000 ms" step above also
     gives ~0.3 s (0.27 s measured). Which length is right is not settled here. Either way, with
     the cutoff on and two or more cells the silence is longer than 450 ms, and the count restarts
     after the ESC has armed. The count restarts only after arming (~1 s after the first reply) and
     the tune (at least ~0.6 s for two cells by either figure), and then needs 2 s more, so `ARMED`
     comes at least ~3.6 s after the first reply: past the published 3.0 s arming bound (README,
     Operational Envelope). The bench ESCs have
     the low-voltage cutoff off (vendor value, `specification/KM55A2_DEFAULTS.md`; the README's
     "ESC settings as configured", read after the 2.22 flash, lists it as unchanged), so this
     case does not arise on this bench. **Amendment 2026-11-11.** *Decision:* arming is tuned against AM32 2.22, or a newer release when
     one is available; no constant is changed now. *Motivation:* the figures in this report were
     read and measured on AM32 2.16, which the bench no longer runs. *Consequences:* the tune's
     length, `READY_GAP_MS` and the other gate constants are settled from the 2.22 source and from
     bench runs on that firmware; until then the disagreement above stays open.

   **Replayed on a bench round's captures from BUG-002's own investigation** (not retained in
   BUG-002.md's later distillation), this gate would have held both motors at zero until
   2.5-3.0 s after the old `ARMED`, that is, until each ESC had replied for 2 s after its reboot.
   In the same runs, the ESC that was held at zero armed 0.97 s after its first post-reboot reply.
   It then replied without a break for the remaining 18 s. The ESC that got throttle 100 had
   rebooted at the same moment, so held at zero it would have armed about a second before throttle
   arrived.

   **What it costs.** `ARMED` comes ~2.1 s after `arm()` for an ESC that was listening. It comes
   ~2.7 s after for one that was in its startup tune, and ~4.5 s after one reboot. An ESC that never
   replies keeps the group in `ARMING` for good. It is unpowered, not latched into bidirectional
   mode, or rebooting on every boot. The application sees `is_armed()` stay false and decides how
   long to wait. The library does not pick a timeout.
4. **Dropped: separate arming-tune confirmation.** The 2 s span already covers the tune, and the
   step as planned would have passed an ESC that armed and then reset (the chain above), since
   it only checked for the tune once rather than requiring a sustained span.
5. **Make it inspectable.** Add `MotorGroup.arming_status()`, per motor: replying since, last reply,
   gaps seen. The harness's `arm_group()` timeout grows to `arm_duration_ms + 8000` ms (since
   raised to `arm_duration_ms + 16000`, `run_scenario.py`'s `ARM_TIMEOUT_MARGIN_MS`), enough for
   two reboots. On a timeout its error prints the status, so a failed arm names the motor that
   never became ready.
6. **Unidirectional-only groups** have nothing to observe. Raise their default minimum to
   3000 ms, Betaflight's DShot detection delay. Document "wait ≥1.5 s after `disarm()` before
   `arm()`".

   *Update 2026-11-11:* still not done. `MotorGroup.DEFAULT_ARM_DURATION_MS` is 2000 for every
   group, unidirectional-only included, and nothing in `driver/motor_group.py` gives such a group
   a different default.
7. **Docs.**
   - CLAUDE.md ("Non-blocking arming"): `ARMED` means the evidence gate, not a timer. *Note 2026-11-12:* CLAUDE.md has no such section; non-blocking arming is in ADR-004 ("Non-blocking arming") and README.md, "Quick Start".
   - README: the "idle tune" heard after `disarm()` is AM32's startup tune after its own reset.

## Verification

- Unit (`tests/unit/test_motor_group.py` with `fakes.py`), feeding sentinel and non-sentinel words
  through a fake RX FIFO and a fake clock:
  - no `ARMED` before the floor, nor before 2000 ms of sentinels;
  - `ARMED` after both;
  - a 300 ms gap (arming tune) inside the span does not restart it;
  - a 700 ms gap (reboot) restarts it;
  - a stale sentinel blocks promotion;
  - one silent bidirectional motor holds the whole group in `ARMING`;
  - a unidirectional-only group stays time-based.
- Bench:
  - BUG-002's `two_channel_bidir_one_idle_600` and `two_channel_gc_600`, 3 runs each, with the
    arming log. Expected: every motor spins. Where an ESC resets during arming, the log shows
    `ARMED` only after its post-reboot arming tune.
  - `arm()` 0.7 s after `disarm()`. Before the fix: no spin and `0xFFF` forever. After: `ARMED` is
    delayed until the ESC is ready, then the motor spins.
  - The scenario suite still passes. Arming now takes ≥ 2.1 s instead of exactly 2.0 s.

## Related

- BUG-002 (arming-phase replies are also the data that separates its hypotheses)
- BUG-008 (harness checks that would have flagged the resulting state)

[am32-arming]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400
[am32-inputtune]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L219-L237
[bf-grace]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L319-L331
[bf-telem-gate]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L434-L440
[bf-streaming]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L150-L165
[am32-latch]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L86-L95
[am32-armed-timeout]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2004
