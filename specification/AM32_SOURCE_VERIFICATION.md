# DShot / Bidirectional DShot vs. AM32 firmware source

Verification of this driver against AM32's own source, 2026-09-26. Per CLAUDE.md, where
a generic spec and AM32's source disagree, the source wins.

**Sources read**
- `am32-firmware/AM32` @ `55c96847` (2026-09-25): `Src/dshot.c`, `Src/signal.c`,
  `Src/main.c`, `Src/sounds.c`, `Inc/targets.h`, `Mcu/*/Src/IO.c`, `Mcu/*/Src/*_it.c`
- `am32-firmware/AM32-bootloader` @ `578ff29c` (2026-09-17): `bootloader/main.c`

Every AM32 constant this report relies on (the 1 s arming gate, the 0.5 s / 2 s signal-loss
timeouts, the 101-frame bidirectional detection, the per-MCU reply-timer periods, the
`armed` gate on DShot commands) has been unchanged since AM32's initial combined commit
(`31e751f`, 2023-08-04). The conclusions therefore hold for any AM32 release the bench ESC
could be running.

**Reproduce the receive-path checks:** `python scripts/verify_am32_reply.py` (no hardware,
no captures). It ports `make_dshot_package()` line for line, pushes every payload AM32 can
emit through the project's own PIO model of `dshot_bidir_rx_frame`
(`scripts/simulate_frame_receiver.py`), and decodes the result with `gcr_decode`.

## Summary

| # | Area | Verdict |
|---|------|---------|
| — | TX packet, CRC, bit timing, inverted polarity, bidir detection, speed bands | **Correct** |
| — | Reply format: marker, GCR table, differential coding, inverted CRC, eRPM | **Correct**: all 2,304 distinct AM32 payloads decode exactly |
| — | `stop()` driving the bidirectional line low (BUG-001 fix) | **Correct**, matches the bootloader |
| 1 | Arming window | **Wrong for AM32**: the default 500 ms is below AM32's hard 1 s minimum |
| 2 | "A reply shows the ESC is armed" | **Wrong**: AM32 replies while disarmed. The 917 eRPM value means "not running", not "armed". |
| 3 | BUG-002 | Findings 1 and 2 reproduce its exact signature. This is a testable hypothesis, not a proven root cause. |
| 4 | Reply bit rate | Correct value, **wrong explanation**: the rate is set by AM32's per-MCU timer, not by oscillator drift. The receiver tuning only fits some AM32 MCU families. |
| 5 | Reply turnaround / ESC drive window | **Doc wrong** (~4.7 µs). The driver does not enforce a gap, so `disarm()`'s back-to-back frames collide with the ESC's reply. |
| 6 | Signal-loss timeout | AM32 uses **500 ms** when armed, 2 s when disarmed (CLAUDE.md quotes 100-250 ms from BLHeli_S) |
| 7 | DShot commands | AM32 executes them **only while armed**, which contradicts the README's BEEP1 note |

## Confirmed correct

| Item | Project | AM32 |
|------|---------|------|
| Packet | `throttle << 1 \| 0`, 4-bit CRC (`dshot_pio.py:345-356`) | `tocheck` = bits 0-10, `dpulse[11]` = telemetry bit, CRC over the three nibbles (`dshot.c:84-85,102`) |
| Bidir CRC | Inverted (`dshot_pio.py:352-353`) | `checkCRC = ~checkCRC + 16` once `dshot_telemetry` is set (`dshot.c:98-100`) |
| Bit decision | Low/high duty 75 % for "1", 37.5 % for "0" | `pulse > frametime >> 5`, i.e. about 48 % of a bit (`dshot.c:74-83`). Margin is about 11 % of a bit on a "0" and 26 % on a "1". |
| Bidir detection | Inverted, idle-high TX, needed during arming | Pin reads high after a frame, counted more than 100 times, only while `!armed` (`dshot.c:87-97`). This happens before the CRC check, so the rejected frames sent before detection still count. The flag is never cleared, so it stays latched until the ESC reboots. |
| Telemetry bit = 0 | Leaves it 0 (`dshot_pio.py:334-337`) | A reply goes out on **every** captured 32-edge frame once detected, whether armed (`signal.c:121-131`, `stm32f0xx_it.c:108-120`) or disarmed (`signal.c:138-149`). `dpulse[11]` only requests serial (KISS) telemetry (`dshot.c:107-109`). |
| Speeds | DSHOT300/600 only | `checkDshot()` has two bands (`signal.c:201-228`). DSHOT300 uses `buffer_padding` 7, DSHOT600 uses 14. |
| Throttle range | 0 stop, 48-2047 throttle | `>47` throttle, `1-47` command, `0` stop (`dshot.c:129-155`). In AM32's 3D mode (`bi_direction`) the range splits 48-1047 / 1048-2047 (`main.c:1105-1135`); that is an ESC setting, not a driver concern. |
| Reply frame | 21 bits, marker 0 on top, `data ^ data>>1`, GCR lookup, inverted CRC only | `gcr[bp+1]` is the marker. Each following level is `bit ^ previous` (`dshot.c:339-344`). Output is active-low (F421 `cctrl = 0x3`). CRC is `csum = ~csum` (`dshot.c:310`). The GCR table matches entry for entry. |
| eRPM | `period_us = m << e`, `eRPM = 60e6 / period` | `e_com_time` = sum of 6 commutation intervals in µs (`main.c:1943`), normalised to `eee mmmmmmmmm` (`dshot.c:291-301`) |
| Pull-up | Pico `PULL_UP` on the shared pin | AM32 enables its own input pull-up (`main.c:1933`). The Pico's is redundant but harmless. |
| Bidir stop | Drive the line low after the last reply (`dshot_pio.py:583-600`) | After the timeout comes `NVIC_SystemReset()`. `checkForSignal()` jumps to the app only if the pin reads low. A pulled-up line that never goes low keeps the ESC in the serial loop (`bootloader/main.c:1097-1157`). |

## Findings

### 1. The default arming window is shorter than AM32's arming gate

AM32 arms in `tenKhzRoutine()` (20 kHz): `armed_timeout_count` counts while `inputSet` holds
and `adjusted_input == 0`. Arming happens only once the count exceeds `LOOP_FREQUENCY_HZ`
(**more than 1 s**) and `zero_input_count > 30` (`main.c:1359-1398`). A **non-zero throttle at any
point resets the count** (`else armed_timeout_count = 0`), so an ESC that has not finished its
second of zeros **stays disarmed for as long as throttle is non-zero**.

- `MotorGroup.DEFAULT_ARM_DURATION_MS = 500` (`motor_group.py:81`) cannot arm AM32 on its own.
  An application that sets throttle as soon as `is_armed()` goes true leaves the ESC disarmed.
- The bench works because every scenario uses `"arm_duration_ms": 3000`.
- README "Verified Parameters" says "500ms (down to 300ms) armed cleanly, confirmed via genuine
  telemetry replies". Per source that is not possible from a cold start, and replies do not
  indicate arming (see finding 2). The ESC most likely armed during zero throttle that
  followed the window.
- `ARM_GAP_TOLERANCE_MS`'s rationale ("the ESC resets its own arming counter when commands stop
  arriving", `motor_group.py:84`) is not how AM32 behaves. A gap leaves `adjusted_input` at 0, so
  the count keeps running. Only non-zero throttle, or a gap long enough to hit the 2 s reboot,
  resets it. Restarting the window on a gap is harmless but not source-grounded.

**Suggested fix.** Raise the default to cover the gate plus a late ESC start: at least 2 s.
AM32's default startup tune alone takes 600 ms with interrupts disabled (`sounds.c:118-146`),
and a custom `eepromBuffer.tune` can take longer. A source-grounded alternative for
bidirectional groups: treat the ESC as armed at **first CRC-valid reply + more than 1 s**. Detection
takes 101 frames after `inputSet`, so the gate is already counting by the time the first reply
appears.

### 2. A reply does not mean the ESC is armed; 0xFFF means "not running"

Once `dshot_telemetry` latches, AM32 replies to every frame whether armed or not (see
"Confirmed correct"). `make_dshot_package()` substitutes `com_time = 65535` whenever
`!running` (`dshot.c:284-286`). That encodes to payload `0xFFF`, which decodes to
**917.3 eRPM**. It is the same value for a disarmed ESC and for an armed ESC whose motor has
not started. Betaflight decodes `0xFFF` as 0 eRPM.

Contradicted text: `motor_group.py:74` ("A telemetry reply only shows that the ESC is armed"),
README:181, the premise of BUG-002 ("arms, replies... never actually spins"), and ADR-005:44
(true for an armed ESC, but a disarmed one replies identically). Optionally, `gcr_decode`
could report `0xFFF` as 0 eRPM or as a `stopped` flag rather than 917.

### 3. BUG-002 matches "never armed" exactly

Findings 1 and 2 together produce BUG-002's signature: CRC-valid replies at 917 for the whole
run, no spin, no error state, and normal recovery after `disarm()`. That is what happens when
the ESC's 1 s gate has not completed by the time the profile's non-zero throttle arrives. The
count then resets on every tick and never finishes, while replies keep flowing.

AM32's source does not show why the 3 s window would sometimes be too short. With defaults the
budget is:
- the bootloader escape under DShot frames, which is quick: garbage reads push `invalid_command`
  past 100;
- the 600 ms startup tune;
- the gate of more than 1 s.

That comes to about 2 s. A custom startup melody, or anything else that delays the ESC's
application, would close the gap. This is a hypothesis with cheap tests:
- In a no-spin run, listen for the arming tune (`playInputTune`, 3 rising tones). By this
  hypothesis it will be absent.
- Mid-run, return a stuck motor to throttle 0 for at least 1.5 s. By this hypothesis it will
  arm and then spin when throttle returns.

### 4. The reply bit rate is AM32's per-MCU timer design, not oscillator drift

AM32 clocks the reply from the input-capture timer switched to PWM, one period per bit:
`(output_timer_prescaler + 1) * (ARR + 1) / f_timer`. The prescaler is 1 at DSHOT300 and 0 at
DSHOT600, or 3/1 above 100 MHz (`signal.c:201-228`). The ARR is fixed per MCU family.
`scripts/verify_am32_reply.py` prints:

| Family | Timer MHz | ARR+1 | DSHOT300 period | vs. nominal 375k | Receiver as tuned |
|--------|-----------|-------|-----------------|------------------|-------------------|
| F051/F031 (`IO.c:71`) | 48 | 62 | 2.5833 µs | +3.2 % | 100 % |
| F421 (`IO.c:28`) | 120 | 77 | 2.5667 µs | +3.9 % | 100 % |
| F415 / V203 | 144 / 48 | 96 / 64 | 2.6667 µs | 0 % | 100 % |
| G431 | 160 | 109 | 2.7250 µs | −2.1 % | ~80 % |
| L431 | 80 | 111 | 2.7750 µs | −3.9 % | ~44 % |
| E230 (`IO.c:63`) | 72 | 101 | 2.8056 µs | −5.0 % | ~23 % |
| G071/G031 (`IO.c:65`) | 64 | 93 | 2.9062 µs | −8.2 % | ~0 % |

DSHOT600 periods are exactly half these, with the same percentages. The bench's measured
periods (`BIDIR_PROFILES`) are 2.5798 µs and 1.2908 µs. These match **F051 to 0.14 % and
0.07 %**, are within 0.6 % of F421, and rule out E230 (8 % off). Note that AM32's only
Skystars KM55 target is `SKYSTARS_KM55_E230` (`targets.h:535`), so the bench board is either a
different MCU revision or was flashed with another target. The AM32 configurator shows which.

Consequences:
- The "~3 % faster than nominal because ESC oscillators run a few percent off" explanation is
  wrong. It appears in `dshot_profiles.py:34,46`, the `gcr_decode.py` docstrings, and
  `DSHOT_PROTOCOL.md:250-266`. `DSHOT_PROTOCOL.md:146-149`'s "not traced this far" can now be
  closed.
- `BIDIR_PROFILES` is effectively a per-MCU-family constant computable from source, not a
  per-unit calibration.
- In the project's own model, `dshot_bidir_rx_frame` reads 100 % of frames between 14.8 and
  16.6 receiver cycles per bit (tuned value 16). That leaves only about 3.7 % headroom toward
  slower replies. As tuned it would fail on E230, G071/G031 and L431 AM32 ESCs, and be
  marginal on G431. This is not a bug for the bench's ESC. It is a scoping fact: the profile
  fits F051/F031/F421/F415/V203-class AM32 hardware only.

### 5. Reply turnaround and the ESC's drive window

On the frame's 32nd edge (its last rising edge), AM32's DMA interrupt calls
`sendDshotDma()`. The ESC then **drives** the line for `23 + buffer_padding` reply periods:
- `buffer_padding + 1` idle-high periods;
- the marker and 20 data bits;
- one trailing idle period.

After that, `receiveDshotDma()` re-arms input capture (`dshot.c:322-345`, `IO.c` DMA count
`23 + buffer_padding`). On F051:

| | DSHOT300 (bp 7) | DSHOT600 (bp 14) |
|---|---|---|
| Frame's last edge → reply marker | 8 × 2.583 = **~20.7 µs** + ISR | 15 × 1.292 = **~19.4 µs** + ISR |
| ESC drives the line for | 30 × 2.583 = **~77.5 µs** | 37 × 1.292 = **~47.8 µs** |
| Minimum frame-start spacing (frame + window + ISR) | **~135 µs** | **~80 µs** |

- `DSHOT_PROTOCOL.md:239-245` says hardware found "~4.7µs fixed delay before a reply begins".
  That figure is the receiver's predelay, a lower bound. ADR-002:392-396 already says so, and
  the source puts the marker about 20 µs after the frame.
- ADR-002:1982 sized the reply as "~54us ... plus a ~4us predelay". The ESC actually holds the
  line for about 78 µs at DSHOT300.
- **Nothing in the driver enforces the spacing.** `MotorGroup.disarm()` queues
  `DISARM_FRAMES = 4` zeros per motor at once (`motor_group.py:91-92`), and `dshot_bidir_tx`
  sends queued words back to back. For a bidirectional motor, frames 2-4 start about 3 µs
  after frame 1, while the ESC is driving its reply. That means bus contention, and the ESC is
  not listening. Only frame 1 can land, so the "margin against a frame lost to noise" does not
  exist for bidirectional motors. If frame 1 is lost, the motor coasts at its last throttle
  until the 0.5 s timeout. `UPDATE_INTERVAL_US = 0` likewise relies on MicroPython's tick
  (~175 µs for one bidirectional motor, per ADR-002) being slower than 135 µs.

**Suggested fix.** In `disarm()`, pace a bidirectional motor's zeros at least one frame plus the
drive window apart, instead of queuing them together. Also document the minimum tick for
bidirectional motors. Enforcing it in PIO would need instructions the block no longer has.

### 6. Signal-loss timeout is 500 ms when armed

`main.c:1992-2017`: once `signaltimeout` exceeds `LOOP_FREQUENCY_HZ >> 1` (**0.5 s**) while armed,
or `<< 1` (**2 s**) while disarmed, AM32 calls `allOff()` and then `NVIC_SystemReset()`. The
timeout is reset by any frame whose duration is in range, **even one that fails CRC**
(`dshot.c:76-77`). Update CLAUDE.md's "100-250ms (BLHeli_S)" figure and ADR-004:106,170 to
AM32's 500 ms. `disarm()`'s "over a hundred times longer" still holds.

### 7. DShot commands run only while armed

`dshot.c:157`: `if ((dshotcommand > 0) && (running == 0) && armed)`, with 6 repeats needed
except for beeps 1-5. A BEEP1 that beeped therefore means the ESC **was** armed, which
contradicts README:183-187. For ADR-003:
- command 13 (EDT enable) switches some replies to EDT frames (`dshot.c:246-281`), which
  `gcr_decode` would misread as eRPM;
- the `EDTARM_IN` input type ignores throttle until EDT is enabled (`main.c:759-762`).

## Minor notes

- AM32 replies even to frames that fail its CRC, so a reply does not confirm the command was
  accepted.
- Bidirectional detection is latched until the ESC reboots. Switching a channel from
  `BidirectionalDShot` to `UnidirectionalDShot` needs an ESC reboot first.
- When the last bit is a "1", `dshot_bidir_tx`'s final rising edge comes from the pull-ups
  (release happens with the pin low). AM32 measures frame length to that edge. The effect is
  well inside AM32's ±1/16 frame-time window, and the bench shows no problem.
