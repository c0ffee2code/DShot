# DShot / Bidirectional DShot vs. AM32 firmware source

This file checks the driver against AM32's own source (2026-09-26). Per CLAUDE.md, where a generic
spec and AM32's source disagree, the source wins. Every claim below quotes the AM32 code it rests
on and links the exact lines.

**Sources.** Links are permalinks, so the line numbers stay valid when AM32 moves on.

- [`am32-firmware/AM32` @ `55c96847`][am32] (2026-09-25): `Src/dshot.c`, `Src/signal.c`, `Src/main.c`,
  `Src/sounds.c`, `Inc/targets.h`, `Mcu/*/Src/IO.c`, `Mcu/*/Src/peripherals.c`, `Mcu/*/Src/*_it.c`
- [`am32-firmware/AM32-bootloader` @ `578ff29c`][bl] (2026-09-17): `bootloader/main.c`

**Which AM32 versions this covers.** The bench ESC's AM32 release is not recorded. Every constant
used below has the same value in every tagged AM32 release, from `v2.05` (2023-12-02) to `v2.21`
(2026-08-07). The timeouts took their current `LOOP_FREQUENCY_HZ`-based form in
[`b0d47ec`][hist-b0d47ec] (2023-08-27), before the first tag. The constants checked are:

- the 1 s arming gate;
- the 0.5 s armed / 2 s disarmed signal-loss timeouts;
- the 101-frame bidirectional detection;
- `buffer_padding` 7/14;
- the reply-timer periods;
- the inverted reply CRC;
- the `armed` gate on DShot commands.

The initial combined commit ([`31e751f`][hist-31e751f], 2023-08-04) differed in one respect: it
used a 2.5 s disarmed timeout.

**Reproduce the receive-path checks:** `python scripts/verify_am32_reply.py` needs no hardware and
no captures. It ports `make_dshot_package()` line for line and pushes every payload AM32 can emit
through the project's own PIO model of `dshot_bidir_rx_frame`
(`scripts/simulate_frame_receiver.py`). It then decodes the result with `driver/gcr_decode.py`.

**Bench ESC hardware.** The measured reply rate identifies the bench ESC as F051-class; see
finding 4. Timings below are given for AM32's STM32F051 build unless stated otherwise:
48 MHz ([`targets.h`][targets-f051]), running on the internal HSI oscillator
([`peripherals.c`][per-f051-hsi]).

## Summary

| # | Area | Verdict |
|---|------|---------|
| A | Command frames: packet, CRC, bit timing, inverted polarity, bidir detection, speed bands, throttle range | **Correct** |
| B | Telemetry reply: marker, GCR table, differential coding, inverted CRC, eRPM | **Correct**: all 2,304 distinct AM32 payloads decode exactly |
| C | `stop()` driving the bidirectional line low (BUG-001 fix) | **Correct**, matches the bootloader |
| 1 | Arming window | **Wrong for AM32**: the default 500 ms is below AM32's hard >1 s gate |
| 2 | "A reply shows the ESC is armed" | **Wrong**: AM32 replies while disarmed. 917 eRPM (`0xFFF`) means "not running". |
| 3 | BUG-002 | Two AM32 mechanisms reproduce its signature: never armed, or stuck-rotor protection. Both are testable hypotheses, not proven causes. |
| 4 | Reply bit rate | Correct value, **wrong explanation**: set by AM32's per-MCU timer, not oscillator drift. The receiver tuning fits only some AM32 MCU families. |
| 5 | Reply turnaround / ESC drive window | **Doc wrong** (~4.7 µs). No gap is enforced, so `disarm()`'s back-to-back frames collide with the reply. |
| 6 | Signal-loss timeout | AM32 uses **500 ms** armed and 2 s disarmed; CLAUDE.md quotes BLHeli_S's 100-250 ms |
| 7 | DShot commands | AM32 executes them **only while armed and stopped**, which contradicts the README's BEEP1 note |

---

## A. Command frames (Pico → ESC): confirmed correct

### A1. Packet layout and CRC

The project sends an 11-bit throttle, a telemetry bit of 0, and a 4-bit CRC over the 12-bit value
(`driver/dshot_pio.py:345-356`). AM32 samples 16 pulses into `dpulse[]`, rebuilds the throttle from
bits 0-10, and XORs the three nibbles. [`Src/dshot.c#L84-L85`][dshot-crc],
[`#L102`][dshot-tocheck]:

```c
uint8_t calcCRC = ((dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3 | (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2 | (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1 | (dpulse[3] ^ dpulse[7] ^ dpulse[11]));
uint8_t checkCRC = (dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15]);
// ...
int tocheck = (dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7 | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3 | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);
```

### A2. Bit decision threshold and frame-length window

The project uses a 75 % pulse for "1" and 37.5 % for "0", 8 PIO cycles per bit. AM32 captures the
frame's 32 edges with timer input capture and DMA. It measures the frame from the first edge to the
last and calls a pulse "1" when it exceeds 1/32 of that span, about 48 % of a bit. That leaves a
margin of about 11 % of a bit on a "0" and 26 % on a "1". The capture only counts if the frame
length is in a window; the same check resets the signal-loss timer, before the CRC is known (see
finding 6). [`Src/dshot.c#L74-L83`][dshot-frame]:

```c
dshot_frametime = dma_buffer[31] - dma_buffer[0];
halfpulsetime = dshot_frametime >> 5;
if ((dshot_frametime > dshot_frametime_low) && (dshot_frametime < dshot_frametime_high)) {
    signaltimeout = 0;
    for (int i = 0; i < 16; i++) {
        // ...
        const uint16_t pdiff = dma_buffer[(i << 1) + 1] - dma_buffer[(i << 1)];
        dpulse[i] = (pdiff > halfpulsetime);
    }
```

The window starts wide open ([`Src/signal.c#L31-L32`][signal-frametime]: `high = 50000`, `low = 0`).
While disarmed, AM32 then learns it from the average of 8 zero-throttle frames and tightens it to
**±1/16 of that average**. [`Src/signal.c#L166-L174`][signal-average]:

```c
if (dshot && (average_count < 8) && (zero_input_count > 5)) {
    average_count++;
    average_packet_length = average_packet_length + (uint16_t)(dma_buffer[31] - dma_buffer[0]);
    if (average_count == 8) {
        dshot_frametime_high = (average_packet_length >> 3) + (average_packet_length >> 7);
        dshot_frametime_low = (average_packet_length >> 3) - (average_packet_length >> 7);
    }
}
```

Frames ending in a "0" bit are about 2.4 % shorter than ones ending in "1". That is well inside
±6.25 %.

### A3. Bidirectional detection and the inverted CRC

The project's `dshot_bidir_tx` idles the line HIGH, and `BidirectionalDShot` must be used from the
start of arming (`driver/dshot_pio.py:42-46`). AM32 detects bidirectional mode **from the line
level, not the CRC**. After each captured frame it reads the pin. If the pin reads high, it counts
the frame, and after more than 100 such frames while disarmed it latches `dshot_telemetry`. Only
then does it expect the inverted CRC. [`Src/dshot.c#L87-L100`][dshot-detect]:

```c
if (!armed) {
    if (dshot_telemetry == 0) {
        if (getInputPinState()) { // if the pin is high for 100 checks between
                                  // signal pulses its inverted
            high_pin_count++;
            if (high_pin_count > 100) {
                dshot_telemetry = 1;
            }
        }
    }
}
if (dshot_telemetry) {
    checkCRC = ~checkCRC + 16;
}
```

Consequences:
- The first ~101 frames fail AM32's CRC. That is harmless, because the project sends zeros while
  arming.
- Nothing ever clears `dshot_telemetry`, so bidirectional mode stays latched until the ESC reboots.

### A4. The telemetry bit can stay 0

The project always sends 0 (`driver/dshot_pio.py:334-337`). In AM32 the bit only requests
**serial** (KISS) telemetry on the separate telemetry wire. [`Src/dshot.c#L104-L109`][dshot-telembit]:

```c
if (calcCRC == checkCRC) {
    signaltimeout = 0;
    dshot_goodcounts++;
    if (dpulse[11] == 1) {
        send_telemetry = 1;
    }
```

`send_telemetry` feeds `makeTelemPackage()` / `send_telem_DMA()` on the UART
([`Src/main.c#L2115-L2121`][main-serialtelem]). The bidirectional reply is triggered by the
capture itself; see B1.

### A5. Speed detection: DSHOT300 and DSHOT600 only

The project offers DSHOT300 and DSHOT600 (`driver/dshot_profiles.py:13-21`). AM32 classifies the
first 32 captured edges into one of two bands. It uses the shortest edge-to-edge interval
(`smallestnumber`) and the average interval, and each band sets that speed's capture prescaler,
reply prescaler and reply padding. [`Src/signal.c#L201-L228`][signal-checkdshot]:

```c
if ((smallestnumber >= 1) && (smallestnumber < 4) && (average_signal_pulse < 60)) {   // DSHOT600 (and 1200)
    ic_timer_prescaler = 0;
    if (CPU_FREQUENCY_MHZ > 100) { output_timer_prescaler = 1; } else { output_timer_prescaler = 0; }
    dshot = 1;
    buffer_padding = 14;
    // ...
}
if ((smallestnumber >= 4) && (smallestnumber <= 8) && (average_signal_pulse < 100)) {  // DSHOT300
    dshot = 1;
    ic_timer_prescaler = 1;
    if (CPU_FREQUENCY_MHZ > 100) { output_timer_prescaler = 3; } else { output_timer_prescaler = 1; }
    buffer_padding = 7;
    // ...
}
```

The prescaler `if`/`else` is condensed onto one line and the band comments are ours.
On F051 detection runs at `ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6`, a 5.33 MHz capture clock
([`Mcu/f051/Src/IO.c#L16`][io-f051-icpsc]):
- DSHOT600's shortest pulse (25 % of 1.67 µs, 0.42 µs) is about 2.2 ticks, so band 1.
- DSHOT300's (0.83 µs) is about 4.4 ticks, so band 2.
- DSHOT1200 falls into band 1 as well, which is incidental, not a supported mode.

### A6. Stop, commands and throttle values

The project sends 0 to stop and 48-2047 as throttle (`driver/motor_group.py`). AM32's input
handling is in [`Src/dshot.c#L129-L155`][dshot-input]:

```c
if (tocheck > 47) {
    if (EDT_ARMED) {
        newinput = tocheck;
        dshotcommand = 0;
        command_count = 0;
        return;
    }
}
if ((tocheck <= 47) && (tocheck > 0)) {
    newinput = 0;
    dshotcommand = tocheck; //  todo
}
if (tocheck == 0) {
    // ...
    newinput = 0;
    dshotcommand = 0;
    command_count = 0;
}
```

- `EDT_ARMED` is 1 for the `AUTO_IN` and `DSHOT_IN` input types. The `EDTARM_IN` type ignores
  throttle until EDT is enabled ([`Src/main.c#L743-L769`][main-inputtype]).
- **A command frame (1-47) also forces throttle to 0.** Sending one while spinning stops the motor.
- Without 3D mode, the throttle passes straight through: `adjusted_input = newinput`
  ([`Src/main.c#L1140-L1142`][main-adjusted]).
- With AM32's 3D mode (`eepromBuffer.bi_direction`) the range splits into 48-1047 and 1048-2047
  ([`Src/main.c#L1105-L1139`][main-3d]). That is an ESC setting, not a driver concern.

---

## B. Telemetry reply (ESC → Pico): confirmed correct

### B1. A reply goes out on every captured frame once bidirectional mode is latched

Once `dshot_telemetry` is set, the capture interrupt sends the reply straight away and decodes the
frame afterwards. It does this whether the ESC is armed or disarmed, and whether the frame's CRC is
good or bad. It is independent of the telemetry bit.

Armed, F051 capture DMA interrupt ([`Mcu/f051/Src/stm32f0xx_it.c#L108-L120`][it-f051]):

```c
if (armed && dshot_telemetry) {
    DMA1->IFCR |= DMA_IFCR_CGIF5;
    DMA1_Channel5->CCR = 0x00;
    if (out_put) {
        receiveDshotDma();
        compute_dshot_flag = 2;
    } else {
        sendDshotDma();
        compute_dshot_flag = 1;
    }
    EXTI->SWIER |= LL_EXTI_LINE_15;
    return;
}
```

Disarmed, via `transfercomplete()` ([`Src/signal.c#L138-L149`][signal-disarmed-bidir]):

```c
if (inputSet == 1) {
    if (dshot_telemetry) {
        if (out_put) {
            make_dshot_package(e_com_time);
            computeDshotDMA();
            receiveDshotDma();
            return;
        } else {
            sendDshotDma();
            return;
        }
```

The packet sent is built after the previous reply (`compute_dshot_flag = 2`, then
`make_dshot_package()` in [`processDshot()`][main-processdshot]). A reply therefore carries data
one frame old.

### B2. Payload: eRPM encoding

The project decodes `period_us = mantissa << exponent` and `eRPM = 60e6 / period_us`
(`driver/gcr_decode.py:288-291`). AM32 measures `e_com_time`, one electrical revolution in µs:
six commutation intervals in 0.5 µs units, halved ([`Src/main.c#L1943`][main-ecom]):

```c
e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1; // COMMUTATION INTERVAL IS 0.5US INCREMENTS
```

It clamps that to 65535 ([`Src/main.c#L1581-L1585`][main-clamp]), substitutes 65535 when the
motor is not running, and normalises it to `eee mmmmmmmmm`, so the mantissa's top bit is 1 whenever
the exponent is non-zero. [`Src/dshot.c#L283-L302`][dshot-erpm]:

```c
} else {
    if (!running) {
        com_time = 65535;
    }
    // ...
    for (int i = 15; i >= 9; i--) {
        if (com_time >> i == 1) {
            shift_amount = i + 1 - 9;
            break;
        } else {
            shift_amount = 0;
        }
    }
    dshot_full_number = ((shift_amount << 9) | (com_time >> shift_amount));
}
```

### B3. Reply CRC is inverted

The project accepts only the inverted polarity (`driver/gcr_decode.py:239-247`). AM32 computes it
in [`Src/dshot.c#L303-L313`][dshot-csum]:

```c
uint16_t csum = 0;
uint16_t csum_data = dshot_full_number;
for (int i = 0; i < 3; i++) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
}
csum = ~csum; // invert it
csum &= 0xf;

dshot_full_number = (dshot_full_number << 4) | csum; // put checksum at the end of 12 bit dshot number
```

### B4. GCR table, marker and differential line levels

The project reads 21 bits with the marker at the top, decodes `data ^ data >> 1`, then looks up the
GCR symbols (`driver/gcr_decode.py:219-236`). AM32's table is entry-for-entry identical
([`Src/dshot.c#L20-L23`][dshot-gcr]):

```c
const char gcr_encode_table[16] = {
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111
};
```

AM32 then builds one timer compare value per bit. Each is "full" or 0, with the marker first and
every later level equal to `bit XOR previous level`. On F051 the "full" value is 64, and
[`Src/dshot.c#L322-L329`][dshot-levels-f051] has:

```c
gcr[1 + buffer_padding] = 64;
for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
    gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6))
        << 6; // exclusive ored with number before it multiplied by 64 to match
              // output timer.
}
gcr[buffer_padding] = 0;
```

The compare value 64 exceeds the timer's `ARR = 61`, so for that bit the PWM output is active for
the whole period. The channel is configured active-low (`CCER = 0x3`,
[`Mcu/f051/Src/IO.c#L68-L71`][io-f051]), so active means the line is driven **LOW**. The marker is
therefore LOW (0), and a "1" GCR bit is a level change. `gcr[0..buffer_padding]` is 0: the line is
idle HIGH before the marker, which matters for finding 5.

`scripts/verify_am32_reply.py` runs this encoder for all 2,304 payloads AM32 can emit and gets
**0 mismatches** from `gcr_decode.analyze_frame()`.

### B5. Line pull-up

The project enables the Pico's `Pin.PULL_UP` on the shared pin. AM32 enables its own input pull-up
at startup ([`Src/main.c#L1930-L1934`][main-pullup]):

```c
#ifdef NEUTRONRC_G071
    setInputPullDown();
#else
    setInputPullUp();
#endif
```

The Pico's pull-up is redundant but harmless.

---

## C. Stopping a bidirectional motor (BUG-001 fix): confirmed correct

`BidirectionalDShot.stop()` drives the line low after the last reply (`driver/dshot_pio.py:583-600`).
With no valid frames, AM32 reboots after its signal-loss timeout (finding 6). Its bootloader's
`checkForSignal()` then decides whether to start the application.
[`bootloader/main.c#L1095-L1157`][bl-checkforsignal]:

```c
#define low_pin_count_threshold 450		// count signal pin is low before determining jump to main firmware
#define pull_down_pin_count_interations 4000		// greater interations extend grace period for input devices booting with signal pin high
// ...
  gpio_mode_set_input(input_pin, GPIO_PULL_DOWN);
  // ... up to 4000 reads, 10 us apart
  if (low_pin_count > low_pin_count_threshold) {		// pulled low & majority stayed low - jump to application
    jump();
  }
  gpio_mode_set_input(input_pin, GPIO_PULL_UP);
  // ... 500 reads
  if (low_pin_count == 0) {
    return;		// pulled high & never low in history - stay in bootloader only
  }
  // ... floating: 500 reads
  if (low_pin_count > 0) {
    jump();		// floating & low at least once - jump to application
  }
```

A line held permanently high leaves the ESC in the bootloader's serial loop. That loop only leaves
if the line stays low for 20 ms ([`#L841-L853`][bl-serialread]), or after more than 100 failed
reads, each of which bumps `invalid_command` ([`#L1042-L1046`][bl-receivebuffer],
[`#L1276-L1280`][bl-mainloop]). Driving the line low takes the first exit, which is exactly what
the fix does.

---

## Findings

### 1. The default arming window is shorter than AM32's arming gate

**AM32.** Arming happens in `tenKhzRoutine()`, which runs at `LOOP_FREQUENCY_HZ` = 20 kHz
([`Inc/targets.h#L5736-L5738`][targets-loop]; F051's timer is `TIM6->PSC = 47`,
[`peripherals.c#L302-L303`][per-f051-tim6]). While the input is detected (`inputSet`) and throttle
is zero, `armed_timeout_count` increases. The ESC arms once the count **exceeds one second's worth**
and more than 30 zero frames have been seen. **Any non-zero throttle resets the count.**
[`Src/main.c#L1360-L1400`][main-arming]:

```c
if (!armed) {
    if (cell_count == 0) {
        if (inputSet) {
            if (adjusted_input == 0) {
                armed_timeout_count++;
                if (armed_timeout_count > LOOP_FREQUENCY_HZ) { // one second
                    if (zero_input_count > 30) {
                        armed = 1;
                        // ... arming tune
                    } else {
                        inputSet = 0;
                        armed_timeout_count = 0;
                    }
                }
            } else {
                armed_timeout_count = 0;
            }
        }
    }
}
```

After a reboot the ESC does not even listen until its startup tune has played. The tune runs before
input capture is enabled ([`Src/main.c#L1893-L1910`][main-startup]), with interrupts off, for 600 ms
by default. A custom tune (`eepromBuffer.tune`) can take longer. [`Src/sounds.c#L118-L146`][sounds-startup]:

```c
void playStartupTune()
{
    __disable_irq();
    // ...
        SET_PRESCALER_PWM(55); // frequency of beep
        delayMillis(200); // duration of beep
        // ... two more 200 ms beeps
    __enable_irq();
}
```

**Project.**
- `MotorGroup.DEFAULT_ARM_DURATION_MS = 500` (`driver/motor_group.py:81`) cannot arm AM32 on its
  own. An application that sets throttle as soon as `is_armed()` turns true keeps the ESC disarmed
  indefinitely.
- The bench works only because every scenario sets `"arm_duration_ms": 3000`.
- README "Verified Parameters" (`README.md:181`) says "500ms (down to 300ms) armed cleanly,
  confirmed via genuine telemetry replies". Per source that cannot happen from a cold start, and
  replies do not indicate arming (finding 2). The ESC most likely armed during zero throttle
  after the window closed.
- `ARM_GAP_TOLERANCE_MS` rests on "the ESC resets its own arming counter when commands stop
  arriving" (`driver/motor_group.py:84`). AM32 does not do that. During a gap `adjusted_input`
  stays 0, so the count keeps running; only non-zero throttle, or a gap long enough to hit the 2 s
  reboot, resets it. Restarting the window after a gap is harmless but not grounded in the source.

**Suggested fix.** Raise the default to at least 2 s: the >1 s gate, plus the 600 ms startup tune,
plus margin. A source-grounded alternative for bidirectional groups is to treat the ESC as armed
at **the first CRC-valid reply plus more than 1 s**. Detection takes 101 frames after `inputSet`
(A3), so the gate is already running by the time the first reply arrives.

### 2. A reply does not mean the ESC is armed; `0xFFF` (917 eRPM) means "not running"

**AM32.** Replies start as soon as bidirectional mode latches, armed or not (B1). When the motor is
not running, `make_dshot_package()` substitutes 65535 ([`Src/dshot.c#L284-L286`][dshot-notrunning]):

```c
if (!running) {
    com_time = 65535;
}
```

That normalises to payload `0xFFF` (mantissa 511, exponent 7, 65,408 µs), which decodes to
**917.3 eRPM**. A disarmed ESC and an armed one whose motor has not started send the same value.
Betaflight decodes `0xFFF` as 0 eRPM.

**Project text this contradicts:**
- `driver/motor_group.py:74`: "A telemetry reply only shows that the ESC is armed".
- `README.md:181`: "confirmed via genuine telemetry replies".
- BUG-002's premise: "arms, replies with mostly-CRC-valid telemetry".
- ADR-005:44: "An armed ESC replies with a constant at-rest value". This is true, but a disarmed
  ESC replies identically.

**Suggested fix.** Optionally, have `gcr_decode` report `0xFFF` as 0 eRPM, or as a `stopped` flag,
rather than 917.

### 3. BUG-002: two AM32 mechanisms produce exactly its signature

BUG-002's signature is CRC-valid replies at 917 eRPM, no spin, no error state, and normal recovery
after `disarm()`. AM32 can produce that in two ways.

**(a) Never armed.** This is findings 1 and 2 combined. If the >1 s gate has not completed when the
profile's first non-zero throttle arrives, the count resets on every tick. The ESC then stays
disarmed for the rest of the run while replying `0xFFF`. With AM32 defaults, the time from the
first frame to armed is about 2 s:
- escaping the bootloader under DShot frames is fast;
- the startup tune takes 600 ms;
- the gate takes more than 1 s.

That fits inside the scenarios' 3 s window. A custom startup melody, or anything else that delays
the ESC's application, would close the gap.

**(b) Stuck-rotor protection.** An armed ESC whose motor fails to start gives up and stays stopped.
Each time the motor is `running` but no back-EMF zero-crossing arrives for 45,000 interval-timer
ticks, AM32 counts a timeout. On F051 a tick is 0.5 µs (`TIM2->PSC = 23` at 48 MHz,
[`peripherals.c#L292`][per-f051-tim2]), so that is 22.5 ms. [`Src/main.c#L2281-L2284`][main-stuck-count]:

```c
if (INTERVAL_TIMER_COUNT > 45000) {
  zero_throttle_brake_active = 0;   // reset zero throttle brake on back emf timeout (rotation stop)
  if(running){
    bemf_timeout_happened++;
```

Past the limit, the ESC forces `input = 0`. The limit is 100 timeouts below throttle 150 (about
2.3 s of failed starts) and 10 above. [`Src/main.c#L1144-L1148`][main-stuck],
[`#L2064-L2068`][main-stuck-limit]:

```c
if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
    allOff();
    maskPhaseInterrupts();
    input = 0;
    bemf_timeout_happened = 102;
// ...
    if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
        bemf_timeout = 100;
    } else {
        bemf_timeout = 10;
    }
```

Once tripped, the protection **latches until throttle returns to 0**. Nothing else in the harness's
profiles clears it. [`Src/main.c#L2049-L2057`][main-stuck-clear]:

```c
if ((zero_crosses > 1000) || (adjusted_input == 0)) {
    bemf_timeout_happened = 0;
}
if (zero_crosses > 100 && adjusted_input < 200) {
    bemf_timeout_happened = 0;
}
if (eepromBuffer.use_sine_start && adjusted_input < 160) {
    bemf_timeout_happened = 0;
}
```

Every BUG-002 instance so far was in a scenario that starts its motors at throttle 60
(`two_channel_divergent_*`) or 100 (`two_channel_gc_*`). Both are below 150 and held for 5-20 s,
and neither profile ever returns to 0.

**Telling them apart:**

| Check | (a) Never armed | (b) Stuck-rotor |
|-------|-----------------|-----------------|
| Arming tune (`playInputTune`, 3 rising tones, repeated per cell if low-voltage cutoff is on; [`sounds.c#L219-L234`][sounds-input]) at the end of the arming window | absent | present |
| Motor in the run's first seconds | silent | twitches or buzzes while trying to start |
| Offline replay of the run's first ~3 s of replies | only `0xFFF` from start to end | non-`0xFFF` values while `running`, then `0xFFF` |
| Throttle back to 0 for ≥1.5 s mid-run, then up again | arms, then spins | clears at once, then retries the start |

The offline-replay row can be checked against captures already on disk, without new bench time.

### 4. The reply bit rate comes from AM32's per-MCU timer, not oscillator drift

**AM32.** The reply is clocked by the same timer that captured the command, switched to PWM output
with one timer period per bit. The period is therefore
`(output_timer_prescaler + 1) × (ARR + 1) / f_timer`. The prescaler is set per speed band (A5); the
ARR is a fixed constant in each MCU family's `IO.c`. On F051 that is
[`Mcu/f051/Src/IO.c#L68-L71`][io-f051]:

```c
IC_TIMER_REGISTER->CCMR1 = 0x60;
IC_TIMER_REGISTER->CCER = 0x3;
IC_TIMER_REGISTER->PSC = output_timer_prescaler;
IC_TIMER_REGISTER->ARR = 61;
```

Other families use `IC_TIMER_REGISTER->pr = 76; // 76 to start` (F421, [`IO.c#L28`][io-f421]),
`TIMER_CAR(IC_TIMER_REGISTER) = 100;` (E230, [`IO.c#L63`][io-e230]), and so on. The full table,
from `scripts/verify_am32_reply.py`:

| Family | Timer MHz | ARR+1 | DSHOT300 bit | vs. nominal 375 kbit/s | Receiver as tuned |
|--------|-----------|-------|--------------|------------------------|-------------------|
| F051 / F031 ([`targets.h`][targets-f051], [`IO.c`][io-f051], [`IO.c`][io-f031]) | 48 | 62 | 2.5833 µs | +3.2 % | 100 % |
| F421 ([`targets.h`][targets-f421], [`IO.c`][io-f421]) | 120 | 77 | 2.5667 µs | +3.9 % | 100 % |
| F415 ([`targets.h`][targets-f415], [`IO.c`][io-f415]) | 144 | 96 | 2.6667 µs | 0 % | 100 % |
| V203 ([`targets.h`][targets-v203], [`IO.c`][io-v203]) | 48 | 64 | 2.6667 µs | 0 % | 100 % |
| G431 ([`targets.h`][targets-g431], [`IO.c`][io-g431]) | 160 | 109 | 2.7250 µs | −2.1 % | ~80 % |
| L431 ([`targets.h`][targets-l431], [`IO.c`][io-l431]) | 80 | 111 | 2.7750 µs | −3.9 % | ~44 % |
| E230 ([`targets.h`][targets-e230], [`IO.c`][io-e230]) | 72 | 101 | 2.8056 µs | −5.0 % | ~23 % |
| G071 / G031 ([`targets.h`][targets-g071], [`IO.c`][io-g071], [`IO.c`][io-g031]) | 64 | 93 | 2.9062 µs | −8.2 % | ~0 % |

The prescaler is 1 at DSHOT300 and 0 at DSHOT600 for timers up to 100 MHz, 3 and 1 above that. So
every DSHOT600 period is exactly half the DSHOT300 one, with the same percentages.

**Bench.** The measured periods in `BIDIR_PROFILES` are 2.5798 µs and 1.2908 µs:
- They match **F051 to 0.14 % and 0.07 %**. That residual is the HSI oscillator's share.
- They are within 0.6 % of F421.
- They rule out E230, which is 8 % off. E230 runs from its internal RC oscillator too
  (`__SYSTEM_CLOCK_72M_PLL_IRC8M_DIV2`, [`system_gd32e23x.c#L50`][clk-e230]), but 8 % is far beyond
  RC tolerance.

AM32's only Skystars KM55 target is an E230 build ([`Inc/targets.h#L535-L541`][targets-km55]):

```c
#ifdef SKYSTARS_KM55_E230
#define FIRMWARE_NAME "KM55A BUTTER"
#define FILE_NAME "SKYSTARS_KM55_E230"
```

So the bench board is either a different MCU revision or was flashed with another target. The AM32
configurator shows which.

**Consequences:**
- The explanation "~3 % faster than nominal because ESC oscillators run a few percent off" is
  wrong. It appears in:
  - `driver/dshot_profiles.py:34,46`;
  - the `driver/gcr_decode.py` docstrings;
  - `specification/DSHOT_PROTOCOL.md`'s reply-rate paragraph (now corrected there).
- `BIDIR_PROFILES` is effectively a per-MCU-family constant that can be computed from source, not a
  per-unit calibration.
- In the project's own model, `dshot_bidir_rx_frame` reads 100 % of frames between 14.8 and 16.6
  receiver cycles per bit; the tuned value is 16. That leaves only about 3.7 % headroom for slower
  replies. As tuned, it fails on E230, G071/G031 and L431 AM32 ESCs and is marginal on G431. This
  is not a bug for the bench's ESC. It is a scoping fact: the profile fits F051/F031/F421/F415/V203
  AM32 hardware only.

### 5. Reply turnaround and the ESC's drive window

**AM32.** The DMA completes on the frame's 32nd edge, its last rising edge. The capture interrupt
then calls `sendDshotDma()` (B1), which re-programs the timer as a PWM output. From that point the
ESC drives the line for `23 + buffer_padding` reply periods. [`Mcu/f051/Src/IO.c#L77`][io-f051-dma]:

```c
DMA1_Channel4->CNDTR = 23 + buffer_padding;
```

Those periods are:
- `buffer_padding + 1` idle-high periods (`gcr[0..buffer_padding] = 0`, B4);
- the marker;
- 20 data bits;
- one trailing idle period (`gcr[buffer_padding + 22]`, never written).

`buffer_padding` is 7 at DSHOT300 and 14 at DSHOT600 (A5). Afterwards `receiveDshotDma()` re-arms
input capture. On F051:

| | DSHOT300 (padding 7) | DSHOT600 (padding 14) |
|---|---|---|
| Frame's last edge → reply marker | 8 × 2.583 = **~20.7 µs** + interrupt latency | 15 × 1.292 = **~19.4 µs** + latency |
| The ESC drives the line for | 30 × 2.583 = **~77.5 µs** | 37 × 1.292 = **~47.8 µs** |
| Minimum frame start → next frame start (frame + window + latency) | **~135 µs** | **~80 µs** |

**Project.**
- `DSHOT_PROTOCOL.md`'s timing section said hardware measurement found "~4.7µs fixed delay before
  a reply begins". That figure is the receiver's predelay, a lower bound; ADR-002:392-396 already
  says so. The source puts the marker about 20 µs after the frame. Corrected there.
- ADR-002:1982 sized the reply as "~54us ... plus a ~4us predelay". The ESC actually holds the line
  for about 78 µs at DSHOT300.
- **Nothing in the driver enforces the spacing.**
  - `MotorGroup.disarm()` queues `DISARM_FRAMES = 4` zeros per motor in one go
    (`driver/motor_group.py:91-92`), and `dshot_bidir_tx` sends queued words back to back.
  - For a bidirectional motor, frames 2-4 therefore start a few µs after frame 1, while the ESC is
    driving its reply. That is bus contention, and the ESC is not listening. Only frame 1 can land.
  - So the "margin against a frame lost to noise" does not exist for bidirectional motors. If frame
    1 is lost, the motor coasts at its last throttle until the 0.5 s timeout.
  - `UPDATE_INTERVAL_US = 0` likewise relies on MicroPython's tick (~175 µs for one bidirectional
    motor, per ADR-002) staying slower than ~135 µs.

**Suggested fix.** In `disarm()`, space a bidirectional motor's zeros at least one frame plus the
drive window apart instead of queuing them together. Also document the minimum tick for
bidirectional motors. Enforcing it in PIO would need instructions the block no longer has.

### 6. The signal-loss timeout is 500 ms when armed

**AM32.** `signaltimeout` counts at 20 kHz ([`Src/main.c#L1569`][main-sigtimeout-inc]) and is
cleared by any frame whose length passes the window, **before the CRC is checked** (A2). When it
runs out, the ESC switches all phases off and resets itself. [`Src/main.c#L1992-L2017`][main-timeouts]:

```c
if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
    if (armed) {
        allOff();
        // ...
        NVIC_SystemReset();
    }
    if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
        allOff();
        // ...
        NVIC_SystemReset();
    }
}
```

Until then the motor keeps its last throttle. There is no earlier ramp-down.

**Project.**
- CLAUDE.md quotes "100-250ms, measured on the now-unsupported BLHeli_S ESC"; ADR-004:106,170 do the
  same. For AM32 it is 500 ms, from source.
- `disarm()`'s claim that the timeout is "over a hundred times longer" than its own stop still
  holds.

### 7. DShot commands run only while armed and stopped

**AM32.** Commands 1-47 are executed only when the ESC is armed and the motor is not running.
Beeps 1-5 act on the first frame; everything else needs 6 consecutive identical frames.
[`Src/dshot.c#L157-L166`][dshot-cmd]:

```c
if ((dshotcommand > 0) && (running == 0) && armed) {
    if (dshotcommand != last_command) {
        last_command = dshotcommand;
        command_count = 0;
    }
    if (dshotcommand <= 5) { // beacons
        command_count = 6; // go on right away
    }
    command_count++;
    if (command_count >= 6) {
```

What AM32 does with each command:
- **Implemented:** 1-5 (beeps), 6 (ESC info, sent over the serial-telemetry UART,
  [`Src/main.c#L2122-L2125`][main-escinfo]), 7/8 (direction, EEPROM), 9/10 (3D off/on),
  12 (save settings), 13/14 (EDT on/off), 20/21 (direction for this session only), and 36.
  Command 36 is AM32's own programming mode: the next two valid frames give an EEPROM position and a
  value, and a following 37 commits them.
- **Ignored:** everything else, including the LED commands (22-29), 32-35 and the telemetry
  requests (42-47) ([`Src/dshot.c#L168-L230`][dshot-cmd-switch]).

**Project.**
- A BEEP1 that beeped means the ESC **was** armed. That contradicts `README.md:183-187` ("regardless
  of whether its arm state machine has ever been satisfied").
- For ADR-003:
  - command 13 switches some replies to EDT frames, which `gcr_decode` would misread as eRPM (EDT
    frame format is in `DSHOT_PROTOCOL.md`);
  - a command frame also zeroes throttle (A6).

---

## Minor notes

- **A reply does not confirm the command was accepted.** AM32 replies to every captured 32-edge
  frame, including ones that fail its CRC (B1).
- **Bidirectional mode stays latched until the ESC reboots** (A3). Switching a channel from
  `BidirectionalDShot` to `UnidirectionalDShot` needs an ESC reboot first.
- **The last rising edge can be slow.** When the frame's last bit is a "1", `dshot_bidir_tx`
  releases the pin while it is still low, so the pull-ups make the final rising edge. AM32 measures
  the frame's length to that edge (A2). The effect is well inside the ±1/16 window, and the bench
  shows no problem.

[am32]: https://github.com/am32-firmware/AM32/tree/55c96847a0cddfee9852eb65d2b10e58f563b3d7
[bl]: https://github.com/am32-firmware/AM32-bootloader/tree/578ff29cb6774c5ce491075ec9b7f05e9781acd6
[hist-b0d47ec]: https://github.com/am32-firmware/AM32/commit/b0d47ece8acf3620a03bd1fa22376b453ad135b1
[hist-31e751f]: https://github.com/am32-firmware/AM32/commit/31e751f58bff45125da1f3c55f710d6b96fcdc58

[dshot-gcr]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L20-L23
[dshot-frame]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L74-L83
[dshot-crc]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L84-L85
[dshot-detect]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L87-L100
[dshot-tocheck]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L102
[dshot-telembit]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L104-L109
[dshot-input]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L129-L155
[dshot-cmd]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L157-L166
[dshot-erpm]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L283-L302
[dshot-csum]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L303-L313
[dshot-levels-f051]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L322-L329

[signal-frametime]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L31-L32
[signal-disarmed-bidir]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L138-L149
[signal-average]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174
[signal-checkdshot]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L201-L228

[main-inputtype]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L743-L769
[main-3d]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1105-L1139
[main-adjusted]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1140-L1142
[main-stuck]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1144-L1148
[main-arming]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400
[main-sigtimeout-inc]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1569
[main-processdshot]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1574-L1590
[main-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1910
[main-pullup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1930-L1934
[main-ecom]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1943
[main-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017
[main-stuck-clear]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2049-L2057
[main-serialtelem]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2115-L2121
[main-stuck-count]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2281-L2284

[sounds-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L118-L146
[sounds-input]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L219-L234

[targets-km55]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L535-L541
[targets-f051]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5383-L5385
[targets-g071]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5435-L5437
[targets-g431]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5504-L5506
[targets-e230]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5540-L5542
[targets-f421]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5567-L5569
[targets-f415]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5601-L5603
[targets-l431]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5621-L5623
[targets-v203]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5685-L5689
[targets-loop]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Inc/targets.h#L5736-L5738

[io-f051-icpsc]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/IO.c#L16
[io-f051]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/IO.c#L68-L71
[io-f031]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f031/Src/IO.c#L59
[io-f421]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f421/Src/IO.c#L25-L28
[io-f415]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f415/Src/IO.c#L35
[io-v203]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/v203/Src/IO.c#L40
[io-g431]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/g431/Src/IO.c#L64
[io-l431]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/l431/Src/IO.c#L71
[io-e230]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/e230/Src/IO.c#L60-L67
[io-g071]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/g071/Src/IO.c#L64-L65
[io-g031]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/g031/Src/IO.c#L65
[it-f051]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/stm32f0xx_it.c#L108-L120
[per-f051-hsi]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/peripherals.c#L80-L85
[per-f051-tim2]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/peripherals.c#L292
[per-f051-tim6]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/peripherals.c#L302-L303
[clk-e230]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/e230/Src/system_gd32e23x.c#L50

[main-clamp]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1581-L1585
[main-stuck-limit]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2064-L2068
[main-escinfo]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L2122-L2125
[dshot-cmd-switch]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L168-L230
[dshot-notrunning]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L284-L286
[io-f051-dma]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/IO.c#L77

[bl-checkforsignal]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1095-L1157
[bl-serialread]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L841-L853
[bl-receivebuffer]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1042-L1046
[bl-mainloop]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1276-L1280
