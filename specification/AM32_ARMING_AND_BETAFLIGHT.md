# AM32 arming, and our DShot implementation vs. Betaflight

This document has two parts:

- **AM32's arming sequence, end to end**, read from source.
- **A comparison of this driver with Betaflight**, the reference flight-controller
  implementation (its generic DShot code and its Raspberry Pi Pico port). The comparison looks for
  the logic error behind BUG-002.

**Date:** 2026-09-27.

**Sources.** All links are permalinks.

- [`am32-firmware/AM32` @ `55c96847`][am32]
- [`am32-firmware/AM32-bootloader` @ `578ff29c`][bl]
- [`betaflight/betaflight` @ `e5071ce4`][bf] (2026-09-26)

For per-line evidence on the protocol itself, see [`AM32_SOURCE_VERIFICATION.md`](AM32_SOURCE_VERIFICATION.md).

## Summary

Two separate problems. The second is the best Pico-side suspect for BUG-002.

**1. Design gap: `MotorGroup` arms open-loop (B1).**

- `ARMED` means "`arm_duration_ms` has passed since `arm()`", not "the ESC is armed".
- Every reply received while arming is drained and discarded unread.
- The harness sends non-zero throttle on the very next tick.

AM32 arms only after more than 1.02 s of uninterrupted zero throttle. That second is counted from
when *it* detects our signal, and any non-zero frame before then keeps it disarmed indefinitely
while it replies `0xFFF`.

Betaflight never relies on a timer alone. It:
- streams MOTOR_STOP from power-up;
- refuses to arm for 5 s after boot and 3 s after the motors are enabled;
- with bidirectional DShot, refuses to arm until every motor has returned a valid eRPM frame
  (section 3).

Your 3000 vs 2000 ms A/B retest (BUG-002, 2026-09-27) shows this gap is **not what triggers
BUG-002**. It is still why nothing on our side notices, or recovers, when an AM32 ends up not
running under non-zero throttle.

**2. BUG-002 suspect: our frames end on a released, slowly rising edge, and our receiver trusts a
fixed delay after it (B4).**

- After a frame whose last bit is "1" (every zero-throttle frame, and throttle 100),
  `dshot_bidir_tx` releases the pin while it is still low. The line then rises only through the
  two weak pull-ups.
- Two things look at that edge:
  - AM32 measures each frame's length to it;
  - our receiver starts looking for the reply's falling edge a fixed 27 cycles after the release:
    **2.18 µs at DSHOT600, 4.35 µs at DSHOT300**.
- If the rise is slow, or switching noise pulls the released, weakly held line low, our receiver
  fires on the tail of our own frame. That capture fails CRC, which is exactly BUG-002's prefix.
- The same disturbance can shift the ESC's 32-edge capture.
- Betaflight avoids both halves: its Pico driver drives the line high, waits ~280 ns and then
  releases, and its receiver waits for *high* before it waits for *low*.

This fits the trigger. DSHOT600 halves both margins. Two bidirectional motors means two released
lines side by side with two motors switching; the passing one-motor scenarios have one. It costs
nothing to fix. It is a hypothesis to test first, not a proven cause: section 5 lists what it does
not yet explain, and the experiments that settle it.

Other deviations from Betaflight (section 4):
- B2: `disarm()` makes AM32 reset rather than disarm.
- B3: our frame pacing ignores the ESC's reply window.
- B5: scenario start throttles sit below Betaflight's idle floor.

---

## 1. AM32's arming sequence, end to end

Times assume AM32's STM32F051 build, which matches this bench (verification finding 4), and our
frames arriving continuously.

| # | Stage | What AM32 does | Duration | Source |
|---|-------|----------------|----------|--------|
| 0 | Bootloader | `checkForSignal()` samples the line: with a pull-down (up to 4,000 reads, jumps to the app after more than 450 lows), then with a pull-up (a line that is never low **parks the ESC in the bootloader**), then floating (any low jumps to the app) | ~50 ms with our frames present; indefinite if the line sits high with no frames | [`bootloader/main.c#L1095-L1157`][bl-checkforsignal] |
| 1 | App init and startup tune | `playStartupTune()` runs **with interrupts disabled, before input capture is enabled**. Frames sent now are ignored entirely. | 600 ms by default, longer with a custom tune | [`main.c#L1893-L1910`][main-startup], [`sounds.c#L118-L146`][sounds-startup] |
| 2 | Input detection | The first 32 captured edges are classified into a speed band: the shortest interval and `(edge30 − edge0) / 32` must fit the band. Success sets `inputSet`. | One frame (a few more if capture started mid-frame, see below) | [`signal.c#L201-L265`][signal-detect] |
| 3 | Frame-length window | Frames 7-14 after `inputSet` are averaged. From then on only frames within **±1/16** of that average count, and only they reset the signal-loss timer. | 8 frames | [`signal.c#L166-L174`][signal-average] |
| 4 | Bidirectional detection | A frame counts when the pin reads high right after it, and only if its length is inside the window. After more than 100 such frames the mode latches and **replies start**. | ~101 frames (~20-45 ms at our tick) | [`dshot.c#L74-L100`][dshot-detect] |
| 5 | **Arming gate** | In the ~20 kHz loop, `armed_timeout_count` increases while `inputSet` holds and throttle is 0. Arming requires the count to exceed `LOOP_FREQUENCY_HZ` and more than 30 zero frames. **Non-zero throttle resets the count.** | **>1.02 s from `inputSet`** (F051: TIM6 at 48 MHz / 48 / 51 = 19.6 kHz) | [`main.c#L1360-L1400`][main-arming] |
| 6 | Arming tune | `armed = 1`, then `playInputTune()`, three rising tones **with interrupts disabled**. With low-voltage cutoff enabled it plays once per detected cell, plus 100 ms each, all inside the loop's interrupt. | ≥300 ms, or `cells × 400 ms` | [`main.c#L1376-L1389`][main-armtune], [`sounds.c#L219-L237`][sounds-input] |
| 7 | Motor start | Armed and not in sine-stepping mode: `input ≥ 47`, or **`input ≥ 127` if sine start is enabled**, starts the motor. | — | [`main.c#L1204-L1213`][main-start] |

**Things that reset or stall the sequence:**

- **Non-zero throttle before stage 5 completes.** `else armed_timeout_count = 0;`, with no timeout
  and no recovery until throttle returns to 0.
- **`zero_input_count ≤ 30` when the count expires.** AM32 then clears `inputSet` and re-runs
  stage 2.
- **No frame of valid length for 2.04 s while disarmed, or 0.5 s while armed.** AM32 then calls
  `allOff()` and `NVIC_SystemReset()`, which goes back to stage 0
  ([`main.c#L1992-L2017`][main-timeouts]).
- **AM32 has no "disarm on zero throttle".** Once armed, only a reset disarms it: signal loss, or
  low-voltage cutoff.

**How long arming takes, from our first frame:**

| ESC state when our frames start | ESC armed at |
|---|---|
| Listening (its idle "waiting for signal" phase) | **~1.05 s** |
| Just started its startup tune | **~1.7 s** |
| Parked in the bootloader by a line held high without frames | bootloader escape + 1.7 s |
| Rebooting because of a reset during our window | reset time + ~0.7 s + 1.05 s |

**A capture that starts mid-frame.** AM32 captures exactly 32 edges and never looks for the gap
between frames. A capture that begins mid-frame therefore spans the inter-frame gap. Stage 2 then
rejects it at DSHOT600 once our frame-to-frame period exceeds:
- **~360 µs** on F051 (5.33 MHz capture clock, `60 × 32` ticks);
- ~288 µs on F421;
- versus ~600 µs / ~480 µs at DSHOT300.

This recovers by itself: each retry skips the edges that arrive during the interrupt, so capture
drifts into alignment within a few frames. It is still worth knowing that the multi-motor tick ADR-002
measured (~366 µs average, 448 µs worst) sits right on the DSHOT600 limit.

**A Pico-side arming signal.** Stage 6 disables interrupts. So an AM32 (F051/F421 build) that
has been replying during our arming window **goes silent for at least 300 ms at the moment it
arms**, then resumes. That dropout is visible from the Pico: a run of RX captures that fail CRC
between two runs of valid `0xFFF`. It is the only signal from which the Pico can observe arming
on AM32 without a microphone. It would not work on the AT415 build, which plays the tone without
blocking.

---

## 2. How our library arms, step by step

| Step | Our code | Consequence on AM32 |
|------|----------|---------------------|
| Construction | `BidirectionalDShot.__init__` sets `Pin.PULL_UP`; no frames yet | If the ESC reboots now, stage 0 may see a high line and park. The harness keeps this gap short. |
| `arm()` | Starts the state machines; frames begin on the next `update()` | The ESC's gate starts whenever *it* reaches stage 5, not now |
| ARMING | Zeros every tick (~175-450 µs); `drain_rx(False)` **discards every reply** | We never learn whether the ESC is listening, latched bidirectional mode, or played its arming tune |
| Window closes | `arm_duration_ms` (default 2000, scenarios 2000) after `arm()` → `ARMED` | Declared on a clock, not on evidence |
| First ARMED tick | Sends `throttles[]`; the harness sets the profile (60 or 100) at once | If the ESC's gate had not finished, it now never will |
| `disarm()` | 4 zeros queued at once, `drain()`, drive the line low | AM32 stays **armed** at zero throttle, then resets itself 0.5 s later on signal loss. The "idle tune" heard after `disarm()` is that reset's startup tune. |

---

## 3. What Betaflight does

**Before arming: continuous MOTOR_STOP.** While disarmed, the mixer writes the disarmed value (0)
to every motor on every loop ([`mixer.c#L486-L488`][bf-mixer-disarmed]). The ESC has therefore
been receiving zeros since the motors were enabled at boot.

**Three arming guards, all of which must clear before the pilot can arm:**

- **`pwr_on_arm_grace`, 5 s after boot by default** ([`config.c#L124`][bf-grace-default],
  [`core.c#L319-L331`][bf-grace]):

  ```c
  bool graceTimeElapsed =
      (getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME)
      && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000);
  #ifdef USE_DSHOT
  // With DSHOT, we also require DSHOT to be ready
  graceTimeElapsed = graceTimeElapsed && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled());
  #endif
  ```

- **DShot "protocol detection" delay, 3 s after the motors are enabled**
  ([`dshot_command.c#L37`][bf-detect-delay], [`#L150-L165`][bf-streaming]):

  ```c
  #define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000
  // ...
  goodMotorDetectDelay = motorGetMotorEnableTimeMs() && (cmpTimeMs(millis(), motorGetMotorEnableTimeMs()) > DSHOT_PROTOCOL_DETECTION_DELAY_MS);
  ```

- **With bidirectional DShot, arming is blocked until every motor has produced valid eRPM
  telemetry** ([`core.c#L434-L440`][bf-telem-gate], [`dshot.c#L342-L359`][bf-telem-active]):

  ```c
  // If Dshot Telemetry is enabled and any motor isn't providing telemetry, then disable arming
  if (useDshotTelemetry && !isDshotTelemetryActive()) {
      setArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
  }
  ```

  On AM32, a valid reply means the ESC is listening and has latched bidirectional mode (stages
  2-4). Combined with the zeros it has been streaming all along, the gate has long finished by
  the time the pilot flips the switch.

**Armed idle.** Betaflight's lowest armed output is `48 + motorIdle × 1999`, with a default
`motorIdle` of 5.5 %, i.e. **~158** ([`pg/motor.c#L83`][bf-idle], [`dshot.c#L59-L73`][bf-endpoints]).
Our scenarios start motors at 60 or 100. AM32's stuck-rotor protection uses a different allowance
on each side of 150: 100 stall timeouts below it, 10 above (verification finding 3).

**Disarm.** Betaflight keeps streaming MOTOR_STOP ([`mixer.c#L103-L107`][bf-stopmotors]); it never
cuts the signal, so an AM32 behind it stays armed and never reboots. Ours cuts the signal, and the
ESC reboots every time.

**Pico port: never sending into the reply window.** Betaflight's Pico driver checks each state
machine's program counter. It sends a new frame only when the previous transmit-and-receive has
finished. If the machine is mid-transmit or mid-receive it **skips that cycle**, and it restarts
the machine only after two consecutive stuck cycles ([`dshot_pico.c#L191-L228`][bf-pico-skip]).
Ours queues up to 4 frames in the TX FIFO and sends them back to back.

**Pico port: frame end.** The last bit ends with the line **driven high**. The program waits 21
cycles (~280 ns) and only then releases the pin ([`dshot.pio#L84-L104`][bf-pio-tx]). Ours releases
the pin while it is still low when the last bit is a "1", leaving the final rising edge to the
pull-ups.

**Pico port: other details, all compatible with ours.**
- Transmit duty is 67 %/33 %, against our 75 %/37.5 %. Both classify correctly against AM32's
  ~49 % threshold. Betaflight's margins are more symmetric: about 16 % on each side, against our
  ~11 % for a "0" and ~26 % for a "1".
- The receiver waits for high, then for a falling edge ([`dshot.pio#L109-L112`][bf-pio-rx]); ours
  waits a fixed predelay, then for a falling edge.
- Pull-up on ([`dshot_bidir_pico.c#L74`][bf-pullup]).
- `0x0FFF` is decoded as 0 eRPM ([`dshot.c#L206-L221`][bf-erpm]).

---

## 4. Our logic vs. Betaflight: the deviations that matter on AM32

| ID | Severity | Deviation | Effect on AM32 | Fix |
|----|----------|-----------|----------------|-----|
| **B1** | **High** (design); not BUG-002's trigger per the A/B retest | **Open-loop arming.** `ARMED` = timer since `arm()`; replies received while arming are discarded; nothing requires the ESC to be listening; throttle may go non-zero on the next tick. Betaflight has three guards (section 3). | If the ESC's gate is not done when the window closes, it never arms, silently. `DEFAULT_ARM_DURATION_MS = 2000` is only ~0.3 s above AM32's worst normal case (1.7 s); 2000 ms leaves no room for a bootloader stay or a longer tune. | Gate `ARMED` on evidence for bidirectional motors. Each motor must have returned CRC-valid replies, and more than 1.2 s must have passed since its **first** valid reply (detection comes ~101 frames after `inputSet`, so this guarantees the >1.02 s gate). Optionally, also require the arming-tune dropout from section 1 to confirm. Keep a floor of ~3 s, Betaflight's detection delay, for groups with no bidirectional motor, where nothing is observable. Stay in ARMING, with a timeout error, until the conditions hold. |
| **B2** | Medium | **`disarm()` cuts the signal instead of streaming zeros.** | AM32 stays armed for 0.5 s, then resets and plays its startup tune, and is deaf for ~0.65 s. An `arm()` shortly after `disarm()` lands in that reboot and pushes the gate later (see B1). `test_bidir_restart_cycles.py` is safe only because it waits 3 s and arms for 3 s. | Accept it, since cutting the signal is the emergency-stop contract (ADR-004). Document that re-arming must wait for the ESC's reboot, or let B1's evidence gate absorb it. |
| **B3** | Medium | **No reply-window pacing.** `dshot_bidir_tx` sends queued words back to back, and `disarm()` queues 4 zeros at once. Betaflight skips a cycle while a motor's exchange is still in progress. | Frames 2-4 of `disarm()` start while the ESC is driving its reply (~48 µs at DSHOT600, ~78 µs at DSHOT300). Only frame 1 can land. Any future loop faster than ~80/135 µs would collide on every frame. | Pace `disarm()`'s bidirectional zeros at least one frame plus the drive window apart. Document a minimum tick for bidirectional motors. |
| **B4** | **Medium-High** (BUG-002 suspect) | **The last edge is released, not driven, and our receiver trusts a fixed delay after it.** After a final "1" bit, `dshot_bidir_tx` releases the pin while it is low, so the rising edge comes from the pull-ups alone. `dshot_bidir_rx_frame` then waits a fixed 27 cycles (2.18 µs at DSHOT600, 4.35 µs at DSHOT300) and triggers on the first low. Betaflight drives the line high, waits ~280 ns, releases ([`dshot.pio#L84-L104`][bf-pio-tx]), and its receiver waits for high before low ([`#L109-L112`][bf-pio-rx]). | **Our receiver:** a slow or noise-disturbed rise still reads low after the predelay, so the receiver captures our own frame tail and the capture fails CRC. The margin at DSHOT600 is half of DSHOT300's. **AM32:** measures frame length to that edge against a ±1/16 window learned from zero-throttle frames (which end in "1"). A frame ending in "0" is rejected if the edge lags more than ~1.1 µs at DSHOT600 (~2.2 µs at DSHOT300). Of our start values, 60 ends in "0" and 100 in "1". A noise edge on the released line also shifts AM32's 32-edge capture by one. | Free, same instruction count: in the "1" tail, raise the IRQ with a driven-high side-set *before* releasing: `irq(rel(1)).side(1)`, then `set(pindirs, 0).side(1) [1]`, then `jmp("frame_start").side(1)`. The line is then high when released, which covers the receiver too. A `wait(1, pin)` in the receiver would need an instruction the block does not have. |
| B5 | Test design | Profiles start at throttle 60 or 100, below Betaflight's ~158 idle floor and AM32's 150 threshold. | Low-throttle starts are the fragile case, and stuck-rotor protection latches until throttle returns to 0, which our profiles never do. | Start bidirectional scenarios at ≥ 150; 200 is the clean choice, because its frame ends in "1" like the zero frames, so the test is not confounded with B4 (160 ends in "0"). Or add a step that holds zero after `ARMED`. |

These parts of our logic are **not** at fault; they match AM32 or Betaflight:
- CRC and inverted CRC;
- bit encoding;
- telemetry bit = 0;
- pull-up;
- GCR decode;
- `0xFFF` handling (`not_running`, `a0d25d4`);
- RX synchronisation.

---

## 5. BUG-002 in the light of this

**The data**, from BUG-002 (9/9 instances, updated 2026-09-27):

- Fails **every time** with DSHOT600 and two bidirectional motors spinning. Passes every time
  otherwise: one bidirectional motor at DSHOT600, or two at DSHOT300.
- `arm_duration_ms` 3000 and 2000 fail identically.
- From `ARMED`, the replies fail CRC for **76 ms to 4.45 s**, in sync across the two motors to a
  few ms. They then become exact `0xFFF` for the rest of the run, and the motor does not spin.

**What AM32's source rules out:**

| Candidate | Ruled out by |
|-----------|--------------|
| The Pico's arming window too short (B1 as trigger) | The A/B retest |
| An ESC reset, at least in the 76 ms instance | Reset to first reply takes ≥ ~0.65 s: bootloader, then the 600 ms startup tune with interrupts off (section 1) |
| Stuck-rotor protection alone, at least in the 76 ms instance | At throttle 100 it needs more than 100 stall timeouts of 22.5 ms (~2.3 s) before it stops the motor (verification finding 3) |
| An ESC-internal timer | Two independent ESC MCUs (on the 4-in-1, one per channel) change state within a few ms of each other. That points at something they share: the frames and the wiring on our side, or the power and ground. |

**What remains.** From about 76 ms after `ARMED` onward, each ESC is replying but **never running**
the motor under non-zero throttle. On AM32 that means one of:
- the ESC is not armed;
- or its `input` never reaches the start threshold, because it is not decoding our non-zero frames
  (section 1, stage 7).

The CRC-failing prefix is the part our own code can produce.

**How B4 would produce it.** The failing configuration is the only one with two released,
high-impedance lines (GPIO6 and GPIO8) side by side while two motors switch nearby. In the passing
one-motor scenarios, GPIO8 is a unidirectional line, always driven. The failing configuration also
halves both of B4's margins.
- **Our side:** a disturbed release edge makes our receiver capture our own frame tail, giving a
  CRC-failing prefix that lasts as long as the disturbance does.
- **ESC side:** an extra edge shifts AM32's 32-edge capture, so it decodes garbage or rejects frames
  on length.

**What B4 does not yet explain** is why the ESC never recovers once the lines are quiet. AM32
re-aligns its capture within a frame or two once our frames are clean.

The remaining link could be either of these:
- the ESC never armed (route 1 of B1, with the arming zeros themselves disturbed);
- a decoded-garbage frame or stuck-rotor state that latches until throttle returns to 0, which our
  profiles never do.

**Experiments that separate these, cheapest first.** Each changes one thing, on the failing
`two_channel_*_600` scenarios.

1. **B4 fix only.** The free PIO change in section 4. If the failure disappears, B4 is the cause.
2. **Keep the arming-phase replies** (log them flagged, still hidden from the application). The
   offline decoder then shows whether the ESC was replying before `ARMED`, and whether the
   arming-tune dropout (section 1) happened. That separates "never armed" from "armed, then lost
   our frames". It is also the data B1's fix needs.
3. **Two unidirectional motors spinning at DSHOT600.** Two motors switching, no released lines. If
   this fails, the cause is electrical or ESC-side, not our bidirectional logic.
4. **Two bidirectional motors at DSHOT600, only one spinning.** Two released lines, one motor's
   noise. If this fails, the second released line matters on its own.
5. **Start throttle 200** (B5). It is above 150, and its frame ends in "1" like the zero frames,
   so it is not confounded with B4's length effect.
6. **Check `loop_times[1]`**, the longest gap between `update()` calls, which the harness already
   records. Compare it with AM32's 0.5 s armed signal-loss limit and the ~360 µs DSHOT600
   detection limit (section 1).

[am32]: https://github.com/am32-firmware/AM32/tree/55c96847a0cddfee9852eb65d2b10e58f563b3d7
[bl]: https://github.com/am32-firmware/AM32-bootloader/tree/578ff29cb6774c5ce491075ec9b7f05e9781acd6
[bf]: https://github.com/betaflight/betaflight/tree/e5071ce4ee436a7d8778e4596b6398f3d705d114

[bl-checkforsignal]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1095-L1157
[main-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1910
[sounds-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L118-L146
[signal-detect]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L201-L265
[signal-average]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174
[dshot-detect]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L74-L100
[main-arming]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400
[main-armtune]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1376-L1389
[sounds-input]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L219-L237
[main-start]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1204-L1213
[main-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017

[bf-mixer-disarmed]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/flight/mixer.c#L486-L488
[bf-stopmotors]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/flight/mixer.c#L103-L107
[bf-grace-default]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/config/config.c#L124
[bf-grace]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L319-L331
[bf-detect-delay]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L37
[bf-streaming]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot_command.c#L150-L165
[bf-telem-gate]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/fc/core.c#L434-L440
[bf-telem-active]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L342-L359
[bf-idle]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/pg/motor.c#L83
[bf-endpoints]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L59-L73
[bf-erpm]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/main/drivers/dshot.c#L206-L221
[bf-pico-skip]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_pico.c#L191-L228
[bf-pio-tx]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L84-L104
[bf-pio-rx]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L109-L112
[bf-pullup]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot_bidir_pico.c#L74
