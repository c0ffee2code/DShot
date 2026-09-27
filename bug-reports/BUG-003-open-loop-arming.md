# BUG-003: Arming is open-loop; nothing checks that the ESC actually armed

**Status:** OPEN - fix proposed, not implemented
**Severity:** High - design gap. Any delay on the ESC side means the motor never arms, and nothing
reports it.
**Component:** `driver/motor_group.py` (`arm()`, `update()`), `driver/capture_mailbox.py`
**Found:** 2026-09-27, AM32/Betaflight comparison (`specification/AM32_ARMING_AND_BETAFLIGHT.md`, B1)

## Summary

`MotorGroup` declares `ARMED` when `arm_duration_ms` has passed since `arm()`. It never learns
whether the ESC has armed:

- replies received while arming are drained and discarded unread
  (`motor_group.py:296`, `drain_rx(False)`);
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
- It is **not** what triggers BUG-002 (see BUG-002's 3000/2000 ms A/B retest). It is why nothing
  detects or explains that state.

## Fix plan

1. **Name the evidence.** While AM32's motor is stopped, every valid reply is one fixed 21-bit
   frame: payload `0xFFF` with its inverted CRC, which `dshot_bidir_rx_frame` pushes as
   **`0x52951`**. Add it to `gcr_decode.py` as `AM32_NOT_RUNNING_FRAME`, next to
   `AM32_NOT_RUNNING_DATA12`, with a unit test that derives it from
   `scripts/verify_am32_reply.py`'s port of `make_dshot_package()`.
2. **Record it without decoding.** In `CaptureMailbox.drain(publish=False)`, the arming-phase
   path, compare each discarded word with that constant. Record `ready_first_ms` (first match) and
   `ready_last_ms` (latest match). That is one integer compare per capture and no allocation,
   within ADR-002's hot-path rules. `reset()` clears both.
3. **Gate `ARMED` on it.** For each bidirectional motor, `update()` promotes `ARMING` to `ARMED`
   only when all of these hold:
   - `arm_duration_ms` has elapsed. This stays as a minimum.
   - `ready_first_ms` is set, and `now - ready_first_ms >= ARM_AFTER_FIRST_REPLY_MS` (1200 ms).
     AM32 detects bidirectional mode ~101 frames *after* its arming count starts, so 1.2 s after
     the first valid reply its >1.02 s gate is complete.
   - `now - ready_last_ms <= READY_FRESH_MS` (~50 ms): it is still replying right now.
   - **A gap in replies of ≥ 450 ms clears `ready_first_ms`**, so the 1.2 s count starts again at
     the next reply. BUG-002's R3 captures show why: ESCs that had started replying reset ~1.85 s
     after `arm()`. They were silent for ~680 ms (startup tune plus the latch) and then needed
     their full >1 s gate again. Measuring from the first reply would have declared `ARMED` while
     the ESC was mid-reboot. The arming tune's ~300 ms gap stays under the threshold. With
     low-voltage cutoff it repeats once per cell and can exceed it, and that only delays `ARMED`
     by 1.2 s while zeros keep flowing, which is safe.

   Until then the group stays in `ARMING` and keeps sending zeros, which is safe.
4. **Optional stronger confirmation.** AM32 plays its arming tune with interrupts disabled
   ([`sounds.c#L219-L237`][am32-inputtune]), so its replies stop for ≥300 ms at the moment it arms
   (`cells × 400 ms` with low-voltage cutoff). A `not-running` stream, then a ≥250 ms gap, then
   `not-running` again is direct evidence of arming. Offer it as `confirm_arm_tune=True`. Keep it
   off by default: AM32's AT415 build plays the tune without blocking.
5. **Make it inspectable.** Add `MotorGroup.arming_status()`, per motor: ready since, last seen,
   tune seen. The harness's `arm_group()` timeout error prints it, so a failed arm says which motor
   never became ready.
6. **Unidirectional-only groups** have nothing to observe. Raise their default minimum to
   3000 ms, Betaflight's DShot detection delay. Document "wait ≥1.5 s after `disarm()` before
   `arm()`".
7. **Docs.**
   - CLAUDE.md ("Non-blocking arming"): `ARMED` means the evidence gate, not a timer.
   - README: the "idle tune" heard after `disarm()` is AM32's startup tune after its own reset.

## Verification

- Unit (`tests/unit/test_motor_group.py` with `fakes.py`), feeding sentinel and non-sentinel words
  through a fake RX FIFO and a fake clock:
  - no `ARMED` before first sentinel + 1200 ms;
  - `ARMED` after;
  - a stale sentinel blocks promotion;
  - a unidirectional-only group stays time-based.
- Bench:
  - `arm()` 0.7 s after `disarm()`. Before the fix: no spin, `0xFFF` forever. After: `ARMED` is
    delayed until the ESC is ready, then the motor spins.
  - The scenario suite still passes.

## Related

- BUG-002 (arming-phase replies are also the data that separates its hypotheses)
- BUG-008 (harness checks that would have flagged the resulting state)

[am32-arming]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400
[am32-inputtune]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L219-L237
[bf-grace]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L319-L331
[bf-telem-gate]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L434-L440
[bf-streaming]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L150-L165
