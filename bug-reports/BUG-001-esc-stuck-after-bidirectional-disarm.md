# BUG-001: ESC stuck silent after a bidirectional DShot session ends

**Status:** RESOLVED (2026-09-25)
**Severity:** High — a bidirectional motor's ESC could not recover without a hard power cycle or
a Pico reset, after every single use, not just an edge case.
**Component:** `driver/dshot_pio.py`, `BidirectionalDShot.stop()`
**Fix:** commit `7cdb8cb`, verified `git show ffae59d..ca1db7d` on the (now merged) branch
`fix/bidir-disarm-line-state`

## Summary

After a program using `BidirectionalDShot` called `disarm()` (directly, or via
`MotorGroup.disarm()`), the ESC on that channel went completely silent instead of returning to
its normal "waiting for signal" idle tune. It stayed silent indefinitely — the only ways
observed to recover it were a full Pico chip reset (`machine.reset()`, not a software-level
`mpremote ... reset`) or power-cycling the ESC. This affected **every** bidirectional motor,
every time, not an intermittent or two-motor-specific condition (see "Investigation history"
below for how that was determined).

## Symptom

1. Build a `BidirectionalDShot` motor, arm it (directly or through `MotorGroup`), optionally
   spin it, then `disarm()`.
2. The ESC goes silent. It does not play its normal idle/waiting-for-signal tune.
3. It stays silent indefinitely, regardless of how long you wait.
4. A unidirectional motor on the same board, disarmed the same way, recovers normally and plays
   its idle tune as expected.
5. A hard Pico reset (or power-cycling the ESC) clears it — the ESC then plays its startup tune
   and returns to normal.

## Root cause

`BidirectionalDShot.stop()` deactivated the TX/RX state machines but never drove the signal pin
to a defined level — it left the pin **released**, held high only by the pull-up resistor
configured once at construction. Nothing drove the line again until the next `start()`.

AM32 ESC firmware (the target bidirectional-capable firmware for this project — see
`CLAUDE.md`), confirmed by reading its source directly
(`am32-firmware/AM32` and `am32-firmware/AM32-bootloader`, `master`/`main`, 2026-09-25):

1. Self-reboots via `NVIC_SystemReset()` after 0.5s (armed) or 2s (unarmed) with no valid
   DShot-timed signal (`Src/main.c`). This is normal, constant idle behavior for any AM32 ESC —
   confirmed by ear: a healthy idle ESC's "waiting for signal" sound is its startup tune
   repeating every ~2-3 seconds, which *is* this reboot loop.
2. On that reboot, AM32's bootloader (`checkForSignal()` in `AM32-bootloader/bootloader/main.c`)
   finds no path to the application while the Pico holds the line permanently high.
3. Parked in the bootloader, its receive loop (`serialreadChar()`) waits for a UART start bit
   with **no general timeout** for a line that never goes low. It hangs forever — confirmed by
   reading the function directly and checking for a hidden watchdog rescue elsewhere in `main()`
   (none found).

A unidirectional motor's line is always actively driven, even when "stopped" (frozen at its last
level, not released), so it never hits this — which is why only bidirectional motors were ever
affected, and why the failure went unnoticed until it was specifically looked for.

**A stuck ESC can be rescued after the fact** by forcing the line low (or a genuine chip reset,
which removes the pull-up along with everything else): the bootloader escapes via a *different*
~20ms timeout inside `serialreadChar()`, once the line actually goes low and stays low. This is
the mechanism a Pico hard-reset was using to "fix" it, and it's also how the fix below works.

(Which exact bootloader build is flashed on this project's Skystar KM55A2 4-in-1 ESC was not
independently confirmed — the mechanism above is read from AM32's reference source, not this
board's binary.)

## Fix

`BidirectionalDShot.stop()` now waits a fixed 300µs after `drain()` returns (a generous margin,
not computed from the ESC's actual reply timing — nothing guarantees the reply has finished by
then, but this runs once per shutdown and is not performance-sensitive), then drives the line
low via SIO instead of releasing it. `BidirectionalDShot.start()` reclaims the pin for PIO
(re-applies the pull-up, re-initializes both state machines) before resuming, so this is safe to
call repeatedly, including the very first `start()` after construction.

Two design alternatives were considered and rejected: keeping the pin under PIO ownership and
forcing it low with `sm.exec()` (never validated on hardware); and doing this at the
application/harness level instead of inside the driver (pushes a correctness requirement onto
every future caller, and doesn't even avoid touching pin ownership once a re-arm is needed).
`stop()` was chosen over `disarm()`-only sequencing so every caller — including the low-level
single-motor usage example — gets the fix automatically.

Full rationale: `decision/ADR-002-bidirectional-dshot.md`, "Implementation Update (2026-09-25)"
section.

## Verification

All run on the actual bench hardware (Skystar KM55A2 4-in-1 AM32 ESC), 2026-09-25:

| Check | Result |
|---|---|
| `telemetry_settled_300` / `telemetry_settled_600` (single bidirectional motor) | ESC recovers audibly, no reset, both speeds |
| `two_channel_divergent_300` / `_600` (two bidirectional motors — the scenario that first showed the bug) | Both channels recover audibly, no reset, both speeds |
| `tests/device/test_bidir_restart_cycles.py` (3 arm/spin/disarm cycles in one session, run twice) | 6/6 cycles, 100% CRC-valid telemetry with real spin every cycle — confirms `start()`'s pin-reclaim works repeatedly, not just once |
| `tests/device/test_pio_lifecycle.py` (no ESC; 30 build/arm/disarm cycles + 15 re-arms) | Disarmed bidirectional line now reads low (assertion updated); every re-arm captured fresh telemetry |
| `smoke_unidirectional` (run twice) | Unaffected path confirmed unaffected |
| 3 new ordering unit tests (`tests/unit/test_dshot_packet.py`) | Lock in that `stop()` deactivates before touching the pin, `start()` re-applies the pull-up and reclaims before activating, and a unidirectional motor's `stop()` never touches its pin |

## Investigation history

The original 2026-09-22 framing was "two bidirectional motors running at once" get stuck. That
turned out to be an acoustic artifact: every earlier "one bidirectional motor: fine" test had a
companion unidirectional motor on the same board, whose own recovery tune was masking that the
tested bidirectional channel was equally stuck the whole time. Confirmed on the bench (F3):
disarming a single bidirectional motor, with an ordinary unidirectional motor still on the
board, leaves that one channel silent while the other keeps beeping normally.

Ruled out along the way, recorded so they aren't re-investigated:
- **RP2350 silicon erratum E9** ("increased leakage current on Bank 0 GPIO when pad input is
  enabled"): real, but it can only hold a pulled-*down* pad high through leakage current, and
  this driver's pin uses a pull-*up*. It fully explained a side diagnostic that happened to use
  a forced pull-down, not this symptom.
- **Two channels contending for one shared ESC CPU**, or **cross-talk between the two signal
  lines**: both require two bidirectional motors running together. Ruled out by the bench fact
  that one motor alone fails identically (F3), not by argument.
- **A `disarm()`/`update()` race** on another core: real, and fixed regardless (commit
  `5342e9c`), but the ESC still got stuck after that fix landed.
- **Releasing the line to `Pin.IN, PULL_UP` in `stop()`** instead of leaving it alone: tried and
  reverted 2026-09-24 — a released line is exactly the failing state, so this "fix" changed
  nothing.

Full trail, including everything tried: `git show ffae59d..ca1db7d`.

## Related, separate, not resolved by this fix

`two_channel_divergent_600` intermittently shows a bidirectional motor replying with valid
telemetry but not spinning (observed once across several post-fix runs; an immediate repeat was
clean). The ESC recovered normally after disarm in every case regardless of whether the motor
spun. Not investigated further. One untested lead: the pull-up is applied at
`BidirectionalDShot.__init__`, before `arm()` is ever called, which is a window this fix doesn't
touch.
