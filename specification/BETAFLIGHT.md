# Betaflight: A Reference DShot Host Implementation

How Betaflight, a widely-deployed flight-controller firmware, drives DShot and arms its ESCs. See
`DSHOT.md` for the protocol itself and `AM32.md` for ESC-side firmware knowledge; this page is
host-side only.

**Source, pinned commit:** [`betaflight/betaflight` @ `e5071ce4`][bf]

## 1. Arming guards

Two independent conditions both have to clear before Betaflight will arm, each addressing a
different risk:

**Power-on arm grace**, 5 seconds after boot by default, and - with DShot in use - additionally
requiring DShot's own "streaming commands" to be enabled:
```c
bool graceTimeElapsed =
    (getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME)
    && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000);
#ifdef USE_DSHOT
graceTimeElapsed = graceTimeElapsed && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled());
#endif
```
[`src/main/fc/core.c#L319-L330`][bf-grace], default value (5) at
[`src/main/config/config.c#L124`][bf-grace-default]. `dshotStreamingCommandsAreEnabled()` itself
folds in a second, 3-second delay measured from when the motors were first enabled - the two
aren't separate guards, the streaming check *is* the detection-delay check:
```c
#define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000
bool dshotStreamingCommandsAreEnabled(void) {
    ...
    goodMotorDetectDelay = motorGetMotorEnableTimeMs()
        && (cmpTimeMs(millis(), motorGetMotorEnableTimeMs()) > DSHOT_PROTOCOL_DETECTION_DELAY_MS);
    ...
    return motorIsEnabled() && goodMotorDetectDelay;
}
```
[`src/main/drivers/dshot_command.c#L37,#L148-L163`][bf-detect-delay],
[`#L150-L165`][bf-streaming]

**Bidirectional-telemetry arming gate**, independent of the grace/streaming check above: with
DShot telemetry in use, arming is disabled outright until telemetry is confirmed active:
```c
if (useDshotTelemetry && !isDshotTelemetryActive()) {
    setArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
} else {
    unsetArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
}
```
[`src/main/fc/core.c#L434-L439`][bf-telem-gate]

## 2. Continuous command streaming

Betaflight never simply "doesn't send" while disarmed - every control-loop pass writes the
disarmed motor value (0) to every motor, and that same unconditional per-loop write path is what
sends it out:
```c
// Disarmed mode
if (!ARMING_FLAG(ARMED)) {
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = motor_disarmed[i];
    }
}
```
[`src/main/flight/mixer.c#L485-L489`][bf-mixer-disarmed] (the per-loop assignment),
`motor_disarmed[]` is zero-initialized and stays so unless the CLI's own "motor" test command
writes into it - a raw value of 0.0 maps, after rounding, to DShot's own `MOTOR_STOP` command
value (0), one and the same as a literal stop command. This write happens every PID-loop iteration
(`mixTable()` then an unconditional `writeMotors()`), armed or not - it just changes *which* value
gets written.

## 3. Armed idle throttle floor

The lowest output Betaflight will command once armed is not 48 (DShot's own protocol floor) but a
configurable idle above it:
```c
const float motorIdlePercent = CONVERT_PARAMETER_TO_PERCENT(motorConfig->motorIdle * 0.01f);
*outputLow = DSHOT_MIN_THROTTLE + motorIdlePercent * DSHOT_RANGE;
```
[`src/main/drivers/dshot.c#L60-L71`][bf-idle]. `DSHOT_MIN_THROTTLE` is 48 and `DSHOT_RANGE` is
1999 (`DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE`) [`src/main/drivers/dshot.c#L59-L73`][bf-endpoints].
The default `motorIdle` config value is 550 -
that field is a percent-times-100 encoding, i.e. 5.50% - giving
`48 + 0.055 * 1999 ≈ 157.9`, an idle floor around **158**.

## 4. Disarm behavior

Disarming clears the armed flag and does bookkeeping; it does not itself stop or cut anything.
[`src/main/fc/core.c#L506-L556`][bf-disarm] The same unconditional per-loop path from §2 keeps
running regardless of the armed flag, so once disarmed it simply starts writing the disarmed value
(0) again, every loop - the signal itself is never cut, only its value changes.

## 5. Pico port

Betaflight's Raspberry Pi Pico driver and its PIO programs make several deliberate choices:

**Skip rather than collide.** Before sending a new frame, the driver checks the PIO state
machine's own program counter. If the state machine is still mid-transmit or mid-receive from the
previous frame, that cycle is skipped entirely rather than queued; only after two consecutive
stuck cycles does it forcibly restart the state machine:
```c
uint pc = pio_sm_get_pc(motor->pio, motor->pio_sm);
...
if (waitCount[motorIndex] >= 2) {
    // Stuck for >125us - ESC definitely didn't respond, restart
    pio_sm_restart(motor->pio, motor->pio_sm);
} else {
    // SM is mid-transmit or mid-receive - skip
}
```
[`src/platform/PICO/dshot_pico.c#L151-L220`][bf-pico-skip]

**Drive high, then release; wait for high, then for the edge.** The transmit PIO program ends
both the "1" and "0" bit paths by driving the line actively HIGH, holds that for a short settling
delay (~280ns, 21 cycles at 75MHz), and only then releases the pin to the pull-up. The receive
program correspondingly waits for the line to actually read high before it starts waiting for the
falling edge that marks the reply's start - not a fixed delay followed directly by an edge wait:
```c
outone_tx:
    set    pins, 1               [31]
    ...
receive:
    nop                          [20]       ; ~280ns settling delay
    set    pindirs, 0                       ; release to the pull-up
wait_one:
    wait   1 pin 0                          ; wait for line HIGH (idle)
wait_zero:
    wait   0 pin 0               [16]       ; wait for the falling edge (reply start)
```
[`src/platform/PICO/dshot.pio#L84-L112`][bf-pio-tx]

**Pull-up enabled**: `gpio_set_pulls(pin, true, false);` [`src/platform/PICO/dshot_bidir_pico.c#L74`][bf-pullup]

**`0x0FFF` (the "not running" sentinel) decodes as 0 eRPM**, not left as a raw value:
```c
if (value == 0x0fff) { return 0; }
```
[`src/main/drivers/dshot.c#L205-L209`][bf-erpm]

**Transmit duty-cycle split**: a "1" bit is 84 cycles LOW / 41 cycles HIGH (67%/33%); a "0" bit is
the reverse (33%/67%) - a 125-cycle bit, split asymmetrically either way.
[`src/platform/PICO/dshot.pio#L84-L98`][bf-pio-duty]

## 6. Telemetry reply CRC

Betaflight's own reply decoder checks the whole 16-bit decoded value (12 data bits + its 4-bit
CRC) at once: XOR the value's bytes together, then XOR the result's nibbles, and the frame is
valid only if that comes out as `0xf` (all ones) - which is the self-checking form of an
*inverted* CRC, not the plain XOR a command frame uses:
```c
uint32_t csum = decodedValue;
csum = csum ^ (csum >> 8);
csum = csum ^ (csum >> 4);
if ((csum & 0xf) != 0xf || decodedValue > 0xffff) { /* reject */ }
```
[`src/main/drivers/dshot_bitbang_decode.c#L92-L96`][bf-reply-crc]. The Pico port's own decoder
(§5) checks the identical self-check on its own captured value:
[`src/platform/PICO/dshot_bidir_pico.c#L289-L296`][bf-reply-crc-pico]. See `AM32.md`#12 for an ESC
that also sends this inverted form.

## Elsewhere

See `AM32.md` for how an actual ESC firmware satisfies (or doesn't need) the guards and timing
choices above, and `DSHOT.md` for the wire protocol these both implement.

[bf]: https://github.com/betaflight/betaflight/tree/e5071ce4ee436a7d8778e4596b6398f3d705d114
[bf-grace]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L319-L330
[bf-grace-default]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/config/config.c#L124
[bf-detect-delay]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L148-L163
[bf-telem-gate]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L434-L439
[bf-mixer-disarmed]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/flight/mixer.c#L485-L489
[bf-idle]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L60-L71
[bf-endpoints]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L59-L73
[bf-streaming]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L150-L165
[bf-disarm]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L506-L556
[bf-pico-skip]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_pico.c#L151-L220
[bf-pio-tx]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L84-L112
[bf-pullup]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_bidir_pico.c#L74
[bf-erpm]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L205-L209
[bf-pio-duty]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L84-L98
[bf-reply-crc]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_bitbang_decode.c#L92-L96
[bf-reply-crc-pico]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_bidir_pico.c#L289-L296
