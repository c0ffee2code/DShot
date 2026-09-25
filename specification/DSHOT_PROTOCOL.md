# DShot Protocol Specification

Compiled from:
- https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- https://www.betaflight.com/docs/development/API/Dshot
- https://ardupilot.org/copter/docs/common-dshot-escs.html
- https://github.com/betaflight/betaflight/files/2704888/Digital_Cmd_Spec.txt
- https://github.com/am32-firmware/AM32 - `Src/dshot.c` (`make_dshot_package()`)
  cross-checked directly against the CRC and GCR sections below (see notes there)

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
- **Telemetry (bit 4)**: Request telemetry from ESC (1 = request, 0 = no request)
- **CRC (bits 3-0)**: 4-bit checksum for error detection

## Throttle Value Ranges

| Range | Purpose |
|-------|---------|
| 0 | Disarmed / Motor Stop |
| 1-47 | Special Commands (see below) |
| 48-2047 | Throttle (2000 resolution steps) |

## DShot Speed Variants

| Variant | Bitrate | T1H (µs) | T0H (µs) | Bit Period (µs) | Frame Length (µs) | Max Update Rate |
|---------|---------|----------|----------|-----------------|-------------------|-----------------|
| DShot150 | 150 kbit/s | 5.00 | 2.50 | 6.67 | 106.72 | ~9.4 kHz |
| DShot300 | 300 kbit/s | 2.50 | 1.25 | 3.33 | 53.28 | ~18.8 kHz |
| DShot600 | 600 kbit/s | 1.25 | 0.625 | 1.67 | 26.72 | ~37.5 kHz |
| DShot1200 | 1200 kbit/s | 0.625 | 0.313 | 0.83 | 13.28 | ~75.3 kHz |

### Bit Encoding

Each bit is encoded by the duration of the HIGH portion of the pulse:
- **Bit "1"**: HIGH for 75% of bit period (T1H)
- **Bit "0"**: HIGH for 37.5% of bit period (T0H)

The ratio between T1H and T0H is always 2:1.

### PIO Clock Frequency Calculation

For RP2040 PIO implementation with 8 clock cycles per bit:
```
PIO_frequency = bitrate × 8
```

| Variant | PIO Frequency |
|---------|---------------|
| DShot150 | 1,200,000 Hz |
| DShot300 | 2,400,000 Hz |
| DShot600 | 4,800,000 Hz |
| DShot1200 | 9,600,000 Hz |

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

Verified 2026-09-09 byte-for-byte against AM32's actual firmware source
(`Src/dshot.c`, `make_dshot_package()`) - AM32 XORs the payload's three
4-bit nibbles, inverts, masks to 4 bits, matching this formula exactly.
AM32's `gcr_encode_table[16]` is likewise identical to this project's own
GCR encode table (`driver/gcr_decode.py`/`scripts/dshot_bidir_decode.py`).
The specific 5/4 response-bitrate figure below, by contrast, was NOT
independently confirmed at this level - AM32's response timing comes from
per-target timer/DMA setup that wasn't traced this far; that number
remains sourced from brushlesswhoop.com only.

## Packet Assembly

1. Take 11-bit throttle value (0-2047)
2. Shift left by 1 bit, OR with telemetry bit: `(throttle << 1) | telemetry`
3. Calculate 4-bit CRC
4. Combine: `(throttle_with_telemetry << 4) | crc`
5. Result is 16-bit packet, transmitted MSB first

## Arming Sequence

ESCs require an arming period before accepting throttle commands:
- Send throttle value **0** continuously
- Duration varies by firmware (typically **300ms** for Bluejay)
- After arming, ESC accepts throttle values 48-2047

## Special Commands (0-47)

### Commands Requiring Motors Stopped

| Code | Command | Repeat | Wait After |
|------|---------|--------|------------|
| 0 | MOTOR_STOP | - | - |
| 1 | BEEP1 | 1x | 260ms |
| 2 | BEEP2 | 1x | 260ms |
| 3 | BEEP3 | 1x | 260ms |
| 4 | BEEP4 | 1x | 280ms |
| 5 | BEEP5 (extended) | 1x | 1020ms |
| 6 | ESC_INFO | 1x | 12ms |
| 7 | SPIN_DIRECTION_1 | 6x | - |
| 8 | SPIN_DIRECTION_2 | 6x | - |
| 9 | 3D_MODE_OFF | 6x | - |
| 10 | 3D_MODE_ON | 6x | - |
| 11 | SETTINGS_REQUEST | - | Not implemented |
| 12 | SAVE_SETTINGS | 6x | 35ms |
| 13 | EDT_ENABLE (Extended Telemetry) | 6x | - |
| 14 | EDT_DISABLE | 6x | - |
| 20 | SPIN_DIRECTION_NORMAL | 6x | - |
| 21 | SPIN_DIRECTION_REVERSED | 6x | - |
| 22 | LED0_ON | 1x | - |
| 23 | LED1_ON | 1x | - |
| 24 | LED2_ON | 1x | - |
| 25 | LED3_ON | 1x | - |
| 26 | LED0_OFF | 1x | - |
| 27 | LED1_OFF | 1x | - |
| 28 | LED2_OFF | 1x | - |
| 29 | LED3_OFF | 1x | - |
| 30 | AUDIO_STREAM_MODE | - | Not implemented |
| 31 | SILENT_MODE | - | Not implemented |
| 32 | SIGNAL_LINE_TELEMETRY_DISABLE | 6x | - |
| 33 | SIGNAL_LINE_TELEMETRY_ENABLE | 6x | - |
| 34 | SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY | 6x | - |
| 35 | SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY | 6x | - |

### Telemetry Request Commands (Can Be Sent Anytime)

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
- Compatible ESC firmware (BLHeli_32, BLHeli_S with Bluejay/JESC, AM32)
- Signal line must support bidirectional communication

### Signal Inversion

Bidirectional DShot uses **inverted signal levels**:
- Standard: HIGH = 1, LOW = 0
- Bidirectional: HIGH = 0, LOW = 1

The inverted CRC signals to the ESC that bidirectional mode is active.

### Timing

```
|<-- FC Transmit -->|<-- Gap -->|<-- ESC Response -->|
     16 bits           ~30µs         21 bits (GCR)
```

- FC transmits 16-bit command
- ~30µs gap for line turnaround (a generic figure from community
  documentation, not confirmed at the AM32 source level - actual hardware
  measurement on this project's bench found AM32's real reply turnaround
  to be much shorter, ~4.7µs fixed delay before a reply begins, not 30µs;
  see decision/ADR-002-bidirectional-dshot.md's RX redesign section. Don't
  treat 30µs as a value to design a receiver's timing around.)
- ESC responds with 21-bit GCR-encoded eRPM frame, sent at **5/4 × the
  command bitrate** (e.g. DShot300's 300 kbit/s command rate implies a
  375 kbit/s / 2.67µs-bit-period response; DShot600 → 750 kbit/s)

This is the nominal, firmware-design bit rate - actual ESCs commonly run
their MCU off an internal RC oscillator rather than a crystal, so the
real bit period on a given unit can sit a few percent off this number.
Measured and confirmed on this project's bench: DSHOT300's real reply
bitrate sits consistently at ~388,000 bps, not the nominal 375,000 -
confirmed independently across four different sampling rates taken in one
session (see decision/ADR-002-bidirectional-dshot.md's fixed-ratio RX
sampling section), which rules out sampling-rate-dependent measurement
error. A receiver can't assume the nominal value holds exactly. This
driver measures the real period once per DShot speed
on real hardware and uses that fixed, verified value
(`driver/dshot_profiles.py`'s `BIDIR_PROFILES`, consumed by
`driver/gcr_decode.py`'s `estimate_bit_period_fixed`) rather than
searching for it fresh on every reply - `scripts/dshot_bidir_decode.py`
(the PC-side reference tool) keeps a brute-force search
(`estimate_bit_period`) available for analyzing a capture from an
unfamiliar or historical rate.

### eRPM Response Frame

The ESC returns a 16-bit value encoded using GCR (Group Code Recording):
- **12-bit eRPM data**: 3-bit exponent + 9-bit mantissa
- **4-bit CRC**

GCR encoding expands 16 bits to 21 bits for improved noise immunity.
The encode table itself matches AM32's `gcr_encode_table[16]` in
`Src/dshot.c` exactly (verified 2026-09-09) - see the CRC section above.

**Response CRC polarity - AM32 deviates from generic community
documentation here, confirmed at the source level (2026-09-09).**
brushlesswhoop.com describes this response CRC as "calculated exactly as
it is with uninverted DSHOT... sent back... uninverted" - i.e. the plain,
non-complemented nibble-XOR formula. AM32's actual firmware does the
opposite: `Src/dshot.c`'s `make_dshot_package()` (the function that
builds this exact eRPM response frame) explicitly inverts it -
`csum = ~csum; // invert it`. This matches this project's own hardware
data exactly: every real CRC-valid capture pulled from this bench so far
(798/798) used the inverted polarity, 0 the plain one - `driver/
gcr_decode.py`'s `check_crc()` accepts only inverted for this reason.
Generic DShot write-ups describe BLHeli_S/Bluejay-era convention; AM32
does not follow it here. Consistent with this project's own rule (see
CLAUDE.md): when a generic spec and AM32's source disagree, the source
wins.

#### GCR Decoding
```c
gcr_decoded = value ^ (value >> 1);
```

#### eRPM Calculation
```c
period_us = mantissa << exponent;  // in microseconds
erpm = 60000000 / period_us;       // electrical RPM
rpm = erpm / (motor_poles / 2);    // mechanical RPM
```

### Extended DShot Telemetry (EDT)

Modern alternative to separate telemetry wire. Uses eRPM frame bandwidth to transmit additional data:
- Temperature
- Voltage
- Current
- Debug values

Supported by: Bluejay, BLHeli_32, AM32

## ESC Compatibility

| ESC Firmware | DShot150 | DShot300 | DShot600 | DShot1200 | Bidirectional |
|--------------|----------|----------|----------|-----------|---------------|
| BLHeli_S (EFM8BB1) | ✓ | ✓ | ✗ | ✗ | ✗ |
| BLHeli_S (EFM8BB2/BB21) | ✓ | ✓ | ✓ | ✓ | With Bluejay/JESC |
| BLHeli_32 | ✓ | ✓ | ✓ | ✓ | ✓ |
| KISS | ✓ | ✓ | ✓ | ✓ | ✓ |
| AM32 | ✗ | ✓ | ✓ | ✗¹ | DShot300/600 only |

¹ Corrected 2026-08-29: this row previously claimed full ✓ support, generalized from other
firmwares rather than checked against AM32 itself. AM32's own README ("Dshot(300, 600) motor
protocol support") and wiki.am32.ca ("Compatible with PWM and BiDirectional DShot300/600
protocols") both document DShot300/600 only - DShot150 and DShot1200 aren't mentioned as
supported at all. `Src/signal.c`'s `checkDshot()` confirms why a DShot1200 signal can still
produce a CRC-valid bidirectional reply on real hardware despite that: it has no distinct
DShot1200 code path, it just bins detected input rate into two coarse bands with loose
pulse-width thresholds, and DShot1200 happens to fall inside the same band as DShot600 -
undocumented incidental behavior, not a supported mode. See `bidirectional_dshot_review.md`'s
W1 item for the hardware measurement that surfaced this.

## Implementation Notes

### Signal Integrity
- Higher DShot rates are more susceptible to noise
- DShot600 is the most commonly used variant (balance of speed and reliability)
- Use short signal wires and proper grounding
- Consider DShot150/300 for long cable runs

### Timing Accuracy
- Bit timing must be within ±10% tolerance
- PIO-based implementations provide precise timing
- Software bit-banging may struggle at DShot600+

### Update Rate
- Send throttle commands continuously (typically every 1-2ms)
- ESCs may shut down if no valid command received within timeout
- Bidirectional mode roughly halves effective update rate due to response wait

### Idle / Stopped Line State
- Simply pausing outbound frames is not the same as making an ESC's own signal-loss timeout
  resolve cleanly. Some ESC firmware's recovery path depends on the line actually reaching a
  low level at some point, not just on no more valid frames arriving - a line left merely
  released (its receiver's pull-up holding it passively high, as a bidirectional TX's idle state
  naturally does when a driver simply deactivates its state machine) can leave that ESC unable
  to find its way back to a normal idle state on its own.
- Confirmed against AM32 firmware source (`am32-firmware/AM32`'s `Src/main.c` and
  `am32-firmware/AM32-bootloader`): it reboots its own MCU after a signal-loss timeout, and the
  bootloader that runs on every such reboot only returns to the application once the line has
  gone low - held permanently high, it waits indefinitely. Driving the line to a defined low
  level when stopping, rather than only releasing it, avoids this and is a safe default for any
  DShot TX implementation, bidirectional or not.
