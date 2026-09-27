# BUG-005: Throttle values 1-47 are transmitted, and AM32 executes them as DShot commands

**Status:** OPEN - fix proposed, not implemented
**Severity:** High (latent) - no current scenario hits it, but any throttle ramp from 0 does.
Consequences include ESC EEPROM writes.
**Component:** `driver/motor_group.py` (`clamp_throttle()`, `set_throttle()`),
`tests/harness/throttle_profile.py`
**Found:** 2026-09-27, code review against AM32 and Betaflight

## Summary

In DShot, values 1-47 are commands, not throttle. The library lets them through:

- `MotorGroup.clamp_throttle()` only limits to 0..2047 (`motor_group.py:393`).
- `throttle_profile.py` accepts hold values and ramp targets in 0..2047 (`:37`, `:48`), and
  ramps step through every value in between.

`update()` repeats the current value on every tick, at ~2.7 kHz. Any value held for more than
~3 ms therefore goes out six or more times, which is what AM32 needs to execute a command.

**What AM32 does with them** ([`dshot.c#L129-L234`][am32-cmds]):

- A frame carrying 1-47 **forces throttle to 0**, so the motor stops or stutters.
- While the ESC is **armed and the motor stopped** (e.g. just armed, before the ramp crosses 47),
  it executes:
  - 1-5: beep, on the first frame;
  - 6: ESC info, over the serial-telemetry UART;
  - 7/8: spin direction, stored;
  - 9/10: **3D mode off/on**, which changes the whole throttle mapping;
  - **12: save settings to EEPROM**;
  - 13/14: EDT on/off, which changes the reply format `gcr_decode` expects;
  - 20/21: direction for this session;
  - **36: programming mode.** The next two valid frames give an EEPROM position and a value, and
    **AM32 ignores all throttle** until a frame of exactly 37 commits them, or a CRC failure exits
    ([`dshot.c#L110-L128`][am32-prog]).

**A ramp from 0 to 200** walks through all of these in order: direction reversed (8), 3D on (10),
**save (12, persisting both)**, EDT on (13) then off (14), then programming mode (36). The ESC's
behaviour then changes in ways that survive a power cycle.

**Betaflight never sends 1-47 as throttle.** Its armed output is constrained to
`[48 + idle, 2047]`, and below-range output becomes 0 with the comment "Prevent getting into
special reserved range" ([`mixer.c#L472-L480`][bf-mixer-range]).

## Fix plan

1. **`MotorGroup.clamp_throttle()`: map 1..47 to 0** (stop). This matches Betaflight's
   below-range handling and never produces a command. Document in `set_throttle()`: "0 = stop,
   48-2047 = throttle, 1-47 are DShot commands and are sent as 0". Mapping to 48 (minimum spin)
   instead is possible, but it would make a controller's small output spin the motor. Stop is the
   safer default.
2. **Low-level `DShotPIO.send_throttle_command()`** stays 0..2047, because ADR-003 needs the
   command range. Add a docstring warning; when commands are implemented, give them their own
   entry point.
3. **`throttle_profile.py`** rejects:
   - any hold value or ramp target in 1..47;
   - any ramp whose steps would pass through 1..47.

   A ramp "from 0" must be written as hold 0 → hold/ramp from 48.
4. **`scenario.py`**: the same rule applies at load time, so a bad scenario fails before anything
   is armed.

## Verification

- Unit:
  - `clamp_throttle` maps 1..47 to 0 and leaves 0 and 48..2047 alone;
  - profile validation rejects holds, targets and ramp steps in 1..47.
- Grep every scenario JSON and device test for throttle values 1..47. None today; the smallest is
  60.

## Related

- ADR-003 (DShot commands)
- `specification/DSHOT_PROTOCOL.md` "Special Commands"

[am32-cmds]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L129-L234
[am32-prog]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L110-L128
[bf-mixer-range]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/flight/mixer.c#L472-L480
