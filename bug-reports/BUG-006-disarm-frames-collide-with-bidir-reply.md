# BUG-006: `disarm()` queues bidirectional frames back to back into the ESC's reply window

**Status:** `disarm()`'s own collision FIXED (`DISARM_ROUND_GAP_US`, below). `update()`'s "same
exposure in principle" (see below) is OPEN and possibly live for a single-bidirectional-motor
group at max-rate pacing - unmeasured since the hot-path tightening that followed this bug.
**Severity:** Medium. The emergency stop relies on a single frame for bidirectional motors, and a
faster command loop would collide on every frame.
**Component:** `driver/motor_group.py` (`disarm()`), `driver/dshot_pio.py` (`dshot_bidir_tx`)
**Found:** 2026-09-27, from AM32's own reply-drive-window timing (see below, `specification/AM32.md`#13) and a Betaflight Pico comparison

## Summary

After each frame, AM32 drives the line for `23 + buffer_padding` reply periods, and it does not
listen during that time ([`IO.c#L68-L77`][am32-io], [`dshot.c#L322-L329`][am32-gcr]). On F051:

| | Drive window | Frame start to next frame start |
|---|---|---|
| DSHOT300 | ~78 µs | ≥ ~135 µs |
| DSHOT600 | ~48 µs | ≥ ~80 µs |

`dshot_bidir_tx` pulls the next queued word as soon as it releases the line.

`MotorGroup.disarm()` queues `DISARM_FRAMES = 4` zeros per motor in one go (`motor_group.py:237`).
For a bidirectional motor:
- frames 2-4 start a few µs after frame 1, while the ESC is driving its reply;
- the two push-pull drivers fight over the line, and the ESC is not listening;
- **only frame 1 can land.**

So the "margin against a frame lost to noise" (`motor_group.py:98-100`) does not exist for
bidirectional motors. If frame 1 is lost, the motor coasts at its last throttle until AM32's 0.5 s
signal-loss reset.

`update()` has the same exposure in principle. `UPDATE_INTERVAL_US = 0` is safe only because
MicroPython's tick (~175-450 µs) is slower than 135 µs.

**Betaflight's Pico driver never sends into a pending exchange.** It checks the state machine's
program counter and skips the cycle ([`dshot_pico.c#L191-L228`][bf-skip]).

**Status update (2026-10-11).** `MotorGroup.disarm()`'s own collision is fixed: it now paces its
`DISARM_FRAMES` rounds with `DISARM_ROUND_GAP_US` (200us) between them, matching this bug's fix
plan item 2 below (implemented as one flat constant rather than a per-speed-computed
`min_frame_period_us`, but 200us clears both this table's floors). No dedicated bench verification
of this specific fix (frames 2-4 decoding as valid replies, this bug's own "Verification" ask) was
found in the commit history that introduced it - every scenario run disarms a bidirectional motor
without reported failure, which is incidental exercise, not the targeted check this bug asked for.

The `update()` exposure above is still open, and its margin against this table's own floor is
unmeasured on the current code: the last single-bidirectional-motor `update()` tick figure on
record is ~175us, from ADR-002's 2026-09-19 measurement - several code versions before BUG-003's
per-tick arming-gate work, drain-before-send, and the `@micropython.native` hot-path tightening
that followed this bug (ADR-002's "Further hot-path tightening" section). None of those changes'
combined effect on a single-bidirectional-motor group's tick has been measured since. A direct
check is cheap: one bench run of `single_channel_bidirectional_300.json` (DSHOT300 has the
tighter ~135us floor) at full CRC-valid would settle whether this is live. Not run as part of this
sweep - offered to the user rather than run unasked.

## Fix plan

1. **`dshot_profiles.py`: a per-speed reply guard from AM32's numbers.** Compute it as
   `(23 + padding) × reply period + margin`, with padding 7 at DSHOT300 and 14 at DSHOT600, and
   the period from `BIDIR_PROFILES`. That gives ~90 µs (DSHOT300) and ~60 µs (DSHOT600).
   `BidirectionalDShot` exposes `min_frame_period_us = frame_us + reply_guard_us`.
2. **`MotorGroup.disarm()`.** Send the `DISARM_FRAMES` rounds with
   `utime.sleep_us(max(min_frame_period_us))` between rounds whenever the group has a
   bidirectional motor. Unidirectional motors go out in the same rounds.
   - Blocking grows from ~0.3 ms to ~0.6 ms, still inside `disarm()`'s documented cost.
   - Fix the `DISARM_FRAMES` comment.
3. **Document the tick floor.** A bidirectional motor must not be sent to more often than
   `min_frame_period_us`. State it in `MotorGroup`'s docstring and CLAUDE.md.
4. **Optional, measure first (ADR-002 hot-path rules).** Enforce the floor in `update()` by
   skipping a bidirectional motor whose last send was less than `min_frame_period_us` ago. That is
   Betaflight's skip-a-cycle behaviour, at the cost of one `ticks_us()` per bidirectional motor
   per tick.

## Verification

- Unit: fake state machines and a fake clock (`tests/unit/fakes.py`). Assert the spacing between
  `disarm()`'s puts when the group has bidirectional motors, and no added delay for
  unidirectional-only groups.
- Bench:
  - disarm from a spinning bidirectional motor, and check that it stops at once (ADR-004's check);
  - the RX captures of frames 2-4 decode as valid replies, not collision garbage.

## Related

- BUG-007 (shutdown ordering)
- ADR-002 part 3 (the stalled-drain test hit this same collision)

[am32-io]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/IO.c#L68-L77
[am32-gcr]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L322-L329
[bf-skip]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_pico.c#L191-L228
