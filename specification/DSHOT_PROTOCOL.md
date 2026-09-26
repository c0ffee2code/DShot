# DShot Protocol Specification

Compiled from:
- https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- https://www.betaflight.com/docs/development/API/Dshot
- https://ardupilot.org/copter/docs/common-dshot-escs.html
- https://github.com/betaflight/betaflight/files/2704888/Digital_Cmd_Spec.txt
- AM32 firmware source, this project's ground truth:
  [`am32-firmware/AM32` @ `55c96847`][am32] (`Src/dshot.c`, `Src/signal.c`, `Src/main.c`,
  `Mcu/*/Src/IO.c`) and [`am32-firmware/AM32-bootloader` @ `578ff29c`][bl].
  Where a generic description and AM32's source disagree, the source wins, and the sections below
  say so. The line-by-line evidence (code snippets and permalinks) is in
  [`AM32_SOURCE_VERIFICATION.md`](AM32_SOURCE_VERIFICATION.md), cited below as "verification §X".

## Overview

DShot (Digital Shot) is a digital protocol for communication between flight controllers and ESCs. Unlike PWM-based protocols, DShot transmits discrete values with error checking, eliminating calibration requirements and providing consistent throttle resolution.

## Packet Structure

Each DShot frame consists of **16 bits**:

```
| 11-bit Throttle | 1-bit Telemetry | 4-bit CRC |
|  S S S S S S S S S S S  |        T        | C C C C |
     MSB              LSB
```

- **Throttle (bits 15-5)**: 11-bit value (0-2047)
- **Telemetry (bit 4)**: Request telemetry from ESC (1 = request, 0 = no request).
  In AM32 this bit only requests *serial* (KISS) telemetry on the separate telemetry wire. A
  bidirectional reply is sent after every frame regardless of it (verification §A4), so this
  driver always sends 0.
- **CRC (bits 3-0)**: 4-bit checksum for error detection

## Throttle Value Ranges

| Range | Purpose |
|-------|---------|
| 0 | Disarmed / Motor Stop |
| 1-47 | Special Commands (see below) |
| 48-2047 | Throttle (2000 resolution steps) |

In AM32, a frame carrying 1-47 also **forces throttle to 0**, so sending a command stops a
spinning motor. In AM32's 3D mode (the `bi_direction` ESC setting), 48-1047 and 1048-2047 are the
two directions (verification §A6).

## DShot Speed Variants

| Variant | Bitrate | T1H (µs) | T0H (µs) | Bit Period (µs) | Frame Length (µs) | Max Update Rate |
|---------|---------|----------|----------|-----------------|-------------------|-----------------|
| DShot150 ¹ | 150 kbit/s | 5.00 | 2.50 | 6.67 | 106.67 | 9.375 kHz |
| DShot300 | 300 kbit/s | 2.50 | 1.25 | 3.33 | 53.33 | 18.75 kHz |
| DShot600 | 600 kbit/s | 1.25 | 0.625 | 1.67 | 26.67 | 37.5 kHz |
| DShot1200 ¹ | 1200 kbit/s | 0.625 | 0.3125 | 0.83 | 13.33 | 75 kHz |

¹ Listed for reference only. AM32 supports DShot300/600 (see "ESC Compatibility"), and so does
this driver.

Frame length is 16 bit periods, and the maximum update rate is the frames that fit back to back.
Bidirectional DShot cannot run that fast: the ESC's reply occupies the line after every frame (see
"Bidirectional DShot > Timing").

### Bit Encoding

Each bit is encoded by the duration of the HIGH portion of the pulse:
- **Bit "1"**: HIGH for 75% of bit period (T1H)
- **Bit "0"**: HIGH for 37.5% of bit period (T0H)

The ratio between T1H and T0H is always 2:1. In bidirectional DShot the levels are swapped: the
LOW portion carries the same durations (see "Signal Inversion").

### How AM32 reads a command frame

AM32 does not sample at fixed points. It timestamps the frame's 32 edges with timer input capture
and DMA, then decides each bit from the whole frame's measured length. From
[`Src/dshot.c#L74-L85`][dshot-frame] (verification §A2):

- **Threshold.** A pulse is a "1" when it is longer than 1/32 of the span from the frame's first
  edge to its last. That is about 48 % of a bit, which leaves a margin of about 11 % of a bit on a
  "0" (37.5 %) and 26 % on a "1" (75 %).
- **Frame-length window.** While disarmed, AM32 averages 8 zero-throttle frames. After that, only
  frames within **±1/16** of that average count. The same check resets the signal-loss timer, and
  it runs *before* the CRC is known.
- **Speed detection.** At startup AM32 classifies the first 32 edges into one of two bands, using
  the shortest edge-to-edge interval and the average one. The bands are DShot600 (DShot1200
  incidentally lands here too) and DShot300. The band then fixes the capture resolution and the
  reply timing ([`Src/signal.c#L201-L228`][signal-checkdshot]).
- **Alignment.** Each capture is exactly 32 edges and does not look for the gap between frames.

So the absolute accuracy of the transmitter's clock matters little once the band is detected.
What matters is the ratio of pulse widths within each frame.

### PIO Clock Frequency Calculation

For RP2040 PIO implementation with 8 clock cycles per bit:
```
PIO_frequency = bitrate × 8
```

| Variant | PIO Frequency |
|---------|---------------|
| DShot150 ¹ | 1,200,000 Hz |
| DShot300 | 2,400,000 Hz |
| DShot600 | 4,800,000 Hz |
| DShot1200 ¹ | 9,600,000 Hz |

### RP2040 PIO Implementation Example

The RP2040/RP2350 PIO (Programmable I/O) subsystem can generate precise DShot waveforms in hardware. Here's the MicroPython implementation from `driver/dshot_pio.py`:

```python
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def dshot():
    wrap_target()
    label("start")
    out(x, 1)            .side(0)    [1]  # 2 cycles: shift bit into X, pin LOW
    jmp(not_x, "zero")   .side(1)    [2]  # 3 cycles: if X=0 goto "zero", pin HIGH
    jmp("start")         .side(1)    [2]  # 3 cycles: (X=1 path) stay HIGH, loop
    label("zero")
    jmp("start")         .side(0)    [2]  # 3 cycles: (X=0 path) go LOW, loop
    wrap()
```

The bidirectional transmitter, `dshot_bidir_tx` in the same file, has the same 8-cycle bit timing
with every level inverted. It also releases the pin after each frame so the ESC can reply.

#### Decorator Configuration

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `sideset_init` | `PIO.OUT_LOW` | Output pin starts LOW |
| `out_shiftdir` | `PIO.SHIFT_LEFT` | Bits shift out MSB first |
| `autopull` | `True` | Auto-refill output shift register when empty |
| `pull_thresh` | `16` | Trigger autopull after 16 bits consumed |

#### PIO Instructions Explained

| Instruction | Purpose |
|-------------|---------|
| `wrap_target()` | Loop start marker (program jumps here after `wrap()`) |
| `wrap()` | Loop end marker (automatically jumps to `wrap_target()`) |
| `label("name")` | Creates a named jump target |
| `out(x, 1)` | Shifts 1 bit from output shift register into X scratch register |
| `jmp(condition, "label")` | Conditional or unconditional jump |
| `.side(n)` | Side-set: controls output pin simultaneously with main instruction |
| `[n]` | Delay: adds n extra clock cycles after instruction |

#### Waveform Generation

Each bit takes exactly **8 clock cycles**, achieving the required duty cycles:

```
Bit "1" (75% duty cycle):          Bit "0" (37.5% duty cycle):
        ┌──────────────┐                   ┌──────┐
        │   6 cycles   │                   │  3   │
        │     HIGH     │ 2 cycles          │ HIGH │   5 cycles
────────┘              └───LOW───  ────────┘      └─────LOW─────
```

**Execution path for bit = 1:**
1. `out(x,1).side(0)[1]` → 2 cycles, pin LOW, X=1
2. `jmp(not_x,"zero").side(1)[2]` → 3 cycles, pin HIGH, condition false (X≠0)
3. `jmp("start").side(1)[2]` → 3 cycles, pin HIGH, loop back

**Execution path for bit = 0:**
1. `out(x,1).side(0)[1]` → 2 cycles, pin LOW, X=0
2. `jmp(not_x,"zero").side(1)[2]` → 3 cycles, pin HIGH, jumps to "zero"
3. `jmp("start").side(0)[2]` → 3 cycles, pin LOW, loop back

## CRC Calculation

### Standard DShot
```c
// value = (throttle << 1) | telemetry_bit
crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
```

### Bidirectional DShot (inverted CRC)
```c
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
```

Verified against AM32's firmware source in both directions:
- **Command frames.** AM32 XORs the 12-bit value's three nibbles, and once it has detected
  bidirectional mode it compares against the inverted CRC (`checkCRC = ~checkCRC + 16`,
  [`Src/dshot.c#L84-L100`][dshot-detect]). Until then it expects the plain CRC, so inverted frames
  sent before detection are rejected (see "Signal Inversion").
- **Replies.** `make_dshot_package()` computes the same nibble XOR and inverts it
  (`csum = ~csum;`, [`Src/dshot.c#L303-L313`][dshot-csum]).

AM32's `gcr_encode_table[16]` is identical to this project's GCR table
(`driver/gcr_decode.py`/`scripts/dshot_bidir_decode.py`). The reply bit rate is covered under
"Reply bit rate" below.

## Packet Assembly

1. Take 11-bit throttle value (0-2047)
2. Shift left by 1 bit, OR with telemetry bit: `(throttle << 1) | telemetry`
3. Calculate 4-bit CRC
4. Combine: `(throttle_with_telemetry << 4) | crc`
5. Result is 16-bit packet, transmitted MSB first

## Arming Sequence

ESCs require an arming period of zero throttle before accepting throttle commands. The length is
firmware-specific; generic write-ups quote ~300 ms for Bluejay, which does not apply to AM32.

**AM32**, from [`Src/main.c#L1360-L1400`][main-arming] (verification finding 1):

1. **After a reboot, the ESC plays its startup tune first.** It lasts 600 ms by default, or longer
   with a custom tune. It runs with interrupts disabled and *before* input capture is enabled, so
   frames sent during it are ignored ([`Src/sounds.c#L118-L146`][sounds-startup],
   [`Src/main.c#L1893-L1910`][main-startup]).
2. **The first 32 captured edges select the input type and speed** (see "How AM32 reads a command
   frame").
3. **The arming gate.** In the 20 kHz loop, a counter increases while throttle is 0. The ESC arms
   once it exceeds `LOOP_FREQUENCY_HZ`, i.e. **more than 1 s of continuous zero throttle**, provided
   more than 30 zero frames were received. It then plays its arming tune (`playInputTune`, three
   rising tones), once per detected battery cell if low-voltage cutoff is enabled.
4. **Any non-zero throttle before that resets the counter**, and the ESC stays disarmed as long as
   throttle stays non-zero. A pause in frames does *not* reset the counter: throttle is still
   zero. Only a pause long enough to trigger the 2 s signal-loss reboot starts everything over.
5. Bidirectional detection (below) completes during this window, after about 101 frames.

A zero-throttle window must therefore last more than 1 s after the ESC starts listening; after a
reboot, allow ≥2 s. The driver's default is shorter (`MotorGroup.DEFAULT_ARM_DURATION_MS`, 500 ms,
verification finding 1). Every test scenario uses 3000 ms.

## Special Commands (0-47)

**AM32 executes a command only while armed and with the motor stopped**
([`Src/dshot.c#L157-L166`][dshot-cmd]):
- beeps 1-5 act on the first frame;
- every other command needs **6 consecutive** identical frames;
- a zero or throttle frame resets the count.

The Repeat and Wait After columns are the generic (Betaflight) figures. The AM32 column is from
AM32's command `switch` ([`Src/dshot.c#L168-L230`][dshot-cmd-switch]).

### Commands Requiring Motors Stopped

| Code | Command | Repeat | Wait After | AM32 |
|------|---------|--------|------------|------|
| 0 | MOTOR_STOP | - | - | Stop (throttle 0) |
| 1 | BEEP1 | 1x | 260ms | Beep |
| 2 | BEEP2 | 1x | 260ms | Beep |
| 3 | BEEP3 | 1x | 260ms | Beep |
| 4 | BEEP4 | 1x | 280ms | Beep |
| 5 | BEEP5 (extended) | 1x | 1020ms | Beep |
| 6 | ESC_INFO | 1x | 12ms | Info packet sent on the serial-telemetry UART, not the signal line; needs 6 frames in AM32 |
| 7 | SPIN_DIRECTION_1 | 6x | - | Sets the stored direction to normal; persist with 12 |
| 8 | SPIN_DIRECTION_2 | 6x | - | Sets the stored direction to reversed; persist with 12 |
| 9 | 3D_MODE_OFF | 6x | - | Sets `bi_direction = 0`; persist with 12 |
| 10 | 3D_MODE_ON | 6x | - | Sets `bi_direction = 1`; persist with 12 |
| 11 | SETTINGS_REQUEST | - | Not implemented | Ignored |
| 12 | SAVE_SETTINGS | 6x | 35ms | Saves EEPROM, then beeps |
| 13 | EDT_ENABLE (Extended Telemetry) | 6x | - | Enables EDT (see below) |
| 14 | EDT_DISABLE | 6x | - | Disables EDT |
| 20 | SPIN_DIRECTION_NORMAL | 6x | - | Direction for this session only (not stored) |
| 21 | SPIN_DIRECTION_REVERSED | 6x | - | Direction for this session only (not stored) |
| 22 | LED0_ON | 1x | - | Ignored |
| 23 | LED1_ON | 1x | - | Ignored |
| 24 | LED2_ON | 1x | - | Ignored |
| 25 | LED3_ON | 1x | - | Ignored |
| 26 | LED0_OFF | 1x | - | Ignored |
| 27 | LED1_OFF | 1x | - | Ignored |
| 28 | LED2_OFF | 1x | - | Ignored |
| 29 | LED3_OFF | 1x | - | Ignored |
| 30 | AUDIO_STREAM_MODE | - | Not implemented | Ignored |
| 31 | SILENT_MODE | - | Not implemented | Ignored |
| 32 | SIGNAL_LINE_TELEMETRY_DISABLE | 6x | - | Ignored |
| 33 | SIGNAL_LINE_TELEMETRY_ENABLE | 6x | - | Ignored |
| 34 | SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY | 6x | - | Ignored |
| 35 | SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY | 6x | - | Ignored |
| 36 | *(AM32 only)* programming mode | 6x | - | The next two valid frames give an EEPROM position and a value; 37 then writes it to RAM; persist with 12 |

### Telemetry Request Commands

Generically these can be sent at any time. **AM32 ignores all of them.** Like any command frame they
also force throttle to 0, so on AM32 they must not be sent while the motor is meant to spin.

| Code | Command | Resolution | Max Value |
|------|---------|------------|-----------|
| 42 | TEMPERATURE_TELEMETRY | 1°C per LSB | 4095°C |
| 43 | VOLTAGE_TELEMETRY | 10mV per LSB | 40.95V |
| 44 | CURRENT_TELEMETRY | 100mA per LSB | 409.5A |
| 45 | CONSUMPTION_TELEMETRY | 10mAh per LSB | 40.95Ah |
| 46 | ERPM_TELEMETRY | 100 eRPM per LSB | 409,500 eRPM |
| 47 | ERPM_PERIOD_TELEMETRY | 16µs per LSB | 65,520µs |

## Bidirectional DShot

Bidirectional DShot enables ESC-to-FC communication on the same signal wire, primarily for eRPM telemetry.

### Requirements
- **DShot300 or higher** (DShot150 not supported)
- Compatible ESC firmware. Generically that means BLHeli_32, BLHeli_S with Bluejay/JESC, or AM32;
  this project supports AM32 only.
- Signal line must support bidirectional communication. AM32 enables its own pull-up on the signal
  input ([`Src/main.c#L1930-L1934`][main-pullup]). This driver adds the Pico's pull-up too, which is
  redundant but harmless.

### Signal Inversion

Bidirectional DShot uses **inverted signal levels**:
- **Standard:** the line idles LOW, and each bit is a HIGH pulse whose length encodes the bit.
- **Bidirectional:** the line idles HIGH, and each bit is a LOW pulse. A LOW of 75 % of the bit is
  "1" and 37.5 % is "0".

The bit values and duty ratios are unchanged; only the levels are swapped. The CRC is inverted as
well (see "CRC Calculation").

**How AM32 detects bidirectional mode.** Generic write-ups say the inverted CRC signals the mode.
AM32's source does the opposite: it detects the mode **from the line level** and only *then*
expects the inverted CRC ([`Src/dshot.c#L87-L100`][dshot-detect], verification §A3).
- After each captured frame, AM32 reads the pin. If it reads HIGH, the frame counts toward
  detection. After more than 100 such frames, it switches to bidirectional mode.
- This happens **only while disarmed**, so a bidirectional transmitter must be in use from the
  start of arming.
- The first ~101 frames fail AM32's CRC check. That is harmless while they carry zero throttle.
- Detection is **latched until the ESC reboots**. A channel cannot be switched back to standard
  DShot without rebooting the ESC.

### Timing

```
|<-- FC transmits -->|<-- turnaround -->|<-- ESC reply: marker + 20 bits -->|<- 1 idle ->|
     16 bits          (padding + 1)        21 reply periods (GCR)             period
                       reply periods,
                       line held HIGH by the ESC
|<------------------ the ESC drives the line for (23 + padding) reply periods ------------------>|
```

The FC transmits its 16-bit command, then releases the line. When AM32 captures the frame's last
edge, its interrupt immediately reprograms the same timer as a PWM output and starts **driving**
the line (verification finding 5, [`Mcu/f051/Src/IO.c#L68-L77`][io-f051]). In order, it sends:
- `buffer_padding + 1` idle-HIGH reply periods;
- the marker;
- 20 data bits;
- one trailing idle period.

Only then does it switch back to listening. `buffer_padding` is **7 at DShot300 and 14 at
DShot600**, set by the speed detection ([`Src/signal.c#L201-L228`][signal-checkdshot]).

Worked figures for AM32's STM32F051 build, which matches this bench's ESC (see "Reply bit rate"):

| | DShot300 | DShot600 |
|---|---|---|
| Frame's last edge → reply marker | 8 × 2.583 µs ≈ **20.7 µs** + interrupt latency | 15 × 1.292 µs ≈ **19.4 µs** + latency |
| The ESC drives the line for | 30 × 2.583 µs ≈ **77.5 µs** | 37 × 1.292 µs ≈ **47.8 µs** |
| Minimum spacing from one frame's start to the next | **~135 µs** (~7.4 kHz) | **~80 µs** (~12.5 kHz) |

Two consequences:

- **The generic ~30 µs turnaround is the right order of magnitude.** On AM32 it is a fixed number
  of reply periods, about 20 µs on F051. An earlier version of this document called it "~4.7µs". That
  figure was the receiver's own predelay, a lower bound on when to start listening
  (`dshot_bidir_rx_frame`'s `nop()[26]`), not a measured turnaround.
- **A frame sent inside the ESC's drive window is lost.** Two push-pull outputs fight over the
  line, and the ESC is not listening. It also corrupts the reply in flight. The transmitter must
  leave at least the spacing above between frames. `dshot_bidir_tx` does not enforce this itself;
  it sends queued words back to back.

The reply carries data one frame old: AM32 sends the packet it built after the previous reply,
then decodes the new frame ([`Src/main.c#L1574-L1590`][main-processdshot]).

### Reply bit rate

Generically the reply runs at **5/4 × the command bitrate**: 375 kbit/s (2.67 µs) for DShot300 and
750 kbit/s (1.33 µs) for DShot600. **AM32 does not produce exactly that.**

AM32 clocks the reply from the timer that captured the command, switched to PWM with one timer
period per bit. The period is therefore `(output_timer_prescaler + 1) × (ARR + 1) / f_timer`:
- the prescaler comes from the speed detection: 1 at DShot300 and 0 at DShot600, or 3 and 1 on
  MCUs above 100 MHz;
- the ARR is a fixed constant per MCU family.

The result is a deterministic, per-family offset from 5/4 (verification finding 4):

| AM32 MCU family | Timer | ARR+1 | DShot300 reply bit | vs. 375 kbit/s |
|-----------------|-------|-------|--------------------|----------------|
| F051 / F031 | 48 MHz | 62 | 2.5833 µs (387.1 kbit/s) | +3.2 % |
| F421 | 120 MHz | 77 | 2.5667 µs | +3.9 % |
| F415, V203 | 144 / 48 MHz | 96 / 64 | 2.6667 µs | 0 % |
| G431 | 160 MHz | 109 | 2.7250 µs | −2.1 % |
| L431 | 80 MHz | 111 | 2.7750 µs | −3.9 % |
| E230 | 72 MHz | 101 | 2.8056 µs | −5.0 % |
| G071 / G031 | 64 MHz | 93 | 2.9062 µs | −8.2 % |

DShot600 periods are exactly half, with the same percentages.

**On this bench.** The measured reply is 387.6 kbit/s (2.5798 µs) at DShot300 and 1.2908 µs at
DShot600. It was confirmed at four different sampling rates (see
`decision/ADR-002-bidirectional-dshot.md`'s fixed-ratio RX sampling section). That matches AM32's
F051 timing to within 0.14 %; the MCU's internal RC oscillator accounts for the remainder.
- An earlier version of this document attributed the whole ~3 % offset to oscillator error. The
  source shows that it is AM32's design.
- AM32's only Skystars KM55 target is an E230 build (−5.0 %), which the measurement rules out. The
  bench board is running an F051-class build; F421, 0.6 % off, is the only other close match.

The driver uses the measured period (`driver/dshot_profiles.py`'s `BIDIR_PROFILES`). From it,
`frame_rx_speed()` sets `dshot_bidir_rx_frame`'s clock to 16 cycles per reply bit. In the
project's PIO model that receiver reads 100 % of frames between 14.8 and 16.6 cycles per bit. That
covers the F051/F031/F421/F415/V203 families but not E230, G071/G031 or L431; G431 is marginal
(`scripts/verify_am32_reply.py`). `scripts/dshot_bidir_decode.py`, the PC-side reference, keeps a
brute-force period search for analysing captures from an unfamiliar ESC.

### eRPM Response Frame

The ESC returns a 16-bit value encoded using GCR (Group Code Recording):
- **12-bit eRPM data**: 3-bit exponent + 9-bit mantissa
- **4-bit CRC**, inverted (see below)

GCR maps each 4-bit nibble to a 5-bit symbol, turning 16 bits into 20. A marker bit brings the
frame to 21 bits on the wire.

**On the wire (AM32, [`Src/dshot.c#L315-L345`][dshot-gcr-levels]).** After the idle-HIGH padding
comes:
- a **LOW marker bit**, always 0;
- 20 bits in which a GCR "1" **changes** the line level and a "0" **keeps** it;
- then the line is idle HIGH again.

A receiver reads 21 line levels, marker first. XOR-ing each level with the one before it recovers
the 20 GCR bits; for the first data bit, the one before is the marker. Those bits then go through
the table lookup below. AM32's `gcr_encode_table[16]` matches this project's table exactly, and
`scripts/verify_am32_reply.py` decodes all 2,304 payloads AM32 can emit without a mismatch.

**Payload (AM32, [`Src/dshot.c#L283-L302`][dshot-erpm]).**
- The value is the period of one electrical revolution in µs (`e_com_time`, the sum of six
  commutation intervals), clamped to 65535.
- It is normalised to `eee mmmmmmmmm`: shifted right until it fits 9 bits, with the shift stored
  as the exponent. So whenever the exponent is non-zero, the mantissa's top bit is 1.
- **When the motor is not running, AM32 sends 65535**, which encodes to `0xFFF` (65,408 µs,
  917 eRPM). This covers a disarmed ESC as well as an armed one whose motor has not started.
- Betaflight decodes `0xFFF` as 0 eRPM. Treat it as "stopped", not as a speed and not as "armed".
  Replies start as soon as bidirectional mode is detected, before the ESC arms (verification
  finding 2).

**Response CRC polarity: AM32 deviates from generic community documentation here, confirmed at
the source level.**
- brushlesswhoop.com describes this response CRC as "calculated exactly as it is with uninverted
  DSHOT... sent back... uninverted", i.e. the plain, non-complemented nibble-XOR formula.
- AM32 does the opposite: `make_dshot_package()`, the function that builds this reply, explicitly
  inverts it with `csum = ~csum; // invert it`.
- This matches the bench data exactly: every CRC-valid capture pulled so far (798/798) used the
  inverted polarity, and none the plain one. `driver/gcr_decode.py`'s `check_crc()` accepts only
  the inverted polarity for this reason.

Generic DShot write-ups describe the BLHeli_S/Bluejay-era convention, which AM32 does not follow.
Consistent with this project's own rule (see CLAUDE.md), when a generic spec and AM32's source
disagree, the source wins.

#### GCR Decoding
```c
gcr_decoded = value ^ (value >> 1);   // over the 21 levels, marker included
// then split the low 20 bits into four 5-bit symbols and map each back to a nibble
// through the inverse of gcr_encode_table - this lookup is not optional
```

#### eRPM Calculation
```c
period_us = mantissa << exponent;  // in microseconds; 0xFFF means "not running"
erpm = 60000000 / period_us;       // electrical RPM
rpm = erpm / (motor_poles / 2);    // mechanical RPM
```

### Extended DShot Telemetry (EDT)

Modern alternative to separate telemetry wire. Uses eRPM frame bandwidth to transmit additional data:
- Temperature
- Voltage
- Current
- Debug values

Supported by Bluejay, BLHeli_32 and AM32.

**AM32** ([`Src/dshot.c#L246-L281`][dshot-edt]):
- **Enabling.** DShot command 13 enables EDT, and 14 disables it. Like every command, it needs the
  ESC armed and stopped, and 6 frames. The setting is not stored: EDT is off after every reboot.
- **Framing.** Enabling sends `0xE00` once and disabling sends `0xEFF`. In between, AM32 never
  sends two EDT frames in a row, so eRPM frames stay interleaved. On eligible replies it sends:
  - current as `0x6nn`, 1 A per LSB, every 40th;
  - voltage as `0x4nn`, 0.25 V per LSB, every 200th;
  - temperature as `0x2nn`, 1 °C per LSB, every 200th.
- **Telling EDT from eRPM.** AM32's eRPM mantissa always has its top bit (bit 8) set when the
  exponent is non-zero. An EDT frame has bit 8 clear and a non-zero top nibble, so the two can be
  told apart.
- **In this driver.** EDT is not supported: `gcr_decode` would misread these frames as eRPM. Since
  the driver never sends command 13, AM32 never produces them.

## ESC Compatibility

| ESC Firmware | DShot150 | DShot300 | DShot600 | DShot1200 | Bidirectional |
|--------------|----------|----------|----------|-----------|---------------|
| BLHeli_S (EFM8BB1) | ✓ | ✓ | ✗ | ✗ | ✗ |
| BLHeli_S (EFM8BB2/BB21) | ✓ | ✓ | ✓ | ✓ | With Bluejay/JESC |
| BLHeli_32 | ✓ | ✓ | ✓ | ✓ | ✓ |
| KISS | ✓ | ✓ | ✓ | ✓ | ✓ |
| AM32 | ✗ | ✓ | ✓ | ✗² | DShot300/600 only |

² Corrected 2026-08-29: this row previously claimed full ✓ support, generalized from other
firmwares rather than checked against AM32 itself. AM32's own README ("Dshot(300, 600) motor
protocol support") and wiki.am32.ca ("Compatible with PWM and BiDirectional DShot300/600
protocols") both document DShot300/600 only; DShot150 and DShot1200 aren't mentioned as
supported at all. `Src/signal.c`'s `checkDshot()` explains why a DShot1200 signal can still
produce a CRC-valid bidirectional reply on real hardware. It has no distinct DShot1200 code path;
it just sorts the detected input rate into two coarse bands with loose pulse-width thresholds, and
DShot1200 happens to fall inside DShot600's band. That is undocumented incidental behavior, not a
supported mode. See `bidirectional_dshot_review.md`'s W1 item for the hardware measurement that
surfaced this.

## Implementation Notes

### Signal Integrity
- Higher DShot rates are more susceptible to noise
- DShot600 is the most commonly used variant (balance of speed and reliability)
- Use short signal wires and proper grounding
- Consider DShot300 for long cable runs (DShot150 is not an option on AM32)

### Timing Accuracy
- Generic guidance allows ±10% bit-timing tolerance. On AM32 what matters is set by "How AM32
  reads a command frame":
  - each pulse is judged against about 48 % of the frame's own average bit;
  - a frame's length must stay within ±1/16 of the length AM32 learned while arming.
- PIO-based implementations provide precise timing
- Software bit-banging may struggle at DShot600+

### Update Rate and Signal Loss
- Send commands continuously. AM32 has no minimum rate beyond its signal-loss timeout, which is
  **0.5 s while armed and 2 s while disarmed**. On timeout it switches all phases off and reboots
  ([`Src/main.c#L1992-L2017`][main-timeouts], verification finding 6).
- Until the timeout the motor **keeps its last throttle**. Any frame of valid length resets the
  timer, even one that fails CRC.
- Bidirectional mode caps the frame rate at the ESC's reply cycle, not a fixed fraction of the
  standard rate: about 135 µs per frame at DShot300 and 80 µs at DShot600 on F051 (see "Timing"). Frames sent
  faster collide with the reply.

### Idle / Stopped Line State
- Simply pausing outbound frames is not the same as making an ESC's own signal-loss timeout
  resolve cleanly. Some ESC firmware's recovery path depends on the line actually reaching a low
  level at some point, not just on no more valid frames arriving. A bidirectional transmitter's
  idle state leaves the line released, held passively high by the receiver's pull-up, when a driver
  simply deactivates its state machine. That can leave such an ESC unable to find its way back to
  a normal idle state on its own.
- Confirmed against AM32's source: after a signal-loss timeout AM32 reboots its own MCU, and the
  bootloader that runs on every reboot starts the application only once the line reads low. Its
  `checkForSignal()` ([`bootloader/main.c#L1095-L1157`][bl-checkforsignal]):
  - first samples the line with a pull-down, up to 4,000 reads 10 µs apart, and jumps to the
    application once more than 450 reads are low;
  - otherwise, with the pull-up on, a line that never goes low leaves the ESC in the bootloader's
    serial loop.
- That loop only exits if the line stays low for 20 ms, or after more than 100 failed reads.
  Driving the line to a defined low level when stopping, rather than only releasing it, avoids this
  and is a safe default for any DShot TX implementation, bidirectional or not. This driver's
  `BidirectionalDShot.stop()` does exactly that (verification §C).

[am32]: https://github.com/am32-firmware/AM32/tree/55c96847a0cddfee9852eb65d2b10e58f563b3d7
[bl]: https://github.com/am32-firmware/AM32-bootloader/tree/578ff29cb6774c5ce491075ec9b7f05e9781acd6
[dshot-frame]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L74-L85
[dshot-detect]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L84-L100
[dshot-cmd]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L157-L166
[dshot-cmd-switch]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L168-L230
[dshot-edt]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L246-L281
[dshot-erpm]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L283-L302
[dshot-csum]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L303-L313
[dshot-gcr-levels]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L315-L345
[signal-checkdshot]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L201-L228
[main-arming]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1360-L1400
[main-processdshot]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1574-L1590
[main-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1893-L1910
[main-pullup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1930-L1934
[main-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017
[sounds-startup]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/sounds.c#L118-L146
[io-f051]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Mcu/f051/Src/IO.c#L68-L77
[bl-checkforsignal]: https://github.com/am32-firmware/AM32-bootloader/blob/578ff29cb6774c5ce491075ec9b7f05e9781acd6/bootloader/main.c#L1095-L1157
