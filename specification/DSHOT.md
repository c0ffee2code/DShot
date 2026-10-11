# DShot Protocol

The wire protocol itself, as generically documented - vendor and host agnostic. See `AM32.md` for
how one specific ESC firmware implements and sometimes deviates from what's described here, and
`BETAFLIGHT.md` for how one specific flight-controller host drives it.

**Sources:**
- https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- https://www.betaflight.com/docs/development/API/Dshot
- https://ardupilot.org/copter/docs/common-dshot-escs.html
- https://github.com/betaflight/betaflight/files/2704888/Digital_Cmd_Spec.txt

## Overview

DShot (Digital Shot) is a digital protocol for communication between flight controllers and ESCs.
Unlike PWM-based protocols, it transmits discrete values with error checking, eliminating
calibration and giving consistent throttle resolution.

## Packet structure

Each frame is 16 bits:

```
| 11-bit Throttle | 1-bit Telemetry | 4-bit CRC |
|  S S S S S S S S S S S  |    T    | C C C C |
     MSB              LSB
```

- **Throttle** (bits 15-5): 11-bit value, 0-2047.
- **Telemetry** (bit 4): requests a telemetry reply (1 = request, 0 = no request) - generically.
- **CRC** (bits 3-0): 4-bit checksum.

## Throttle value ranges

| Range | Meaning |
|---|---|
| 0 | Disarmed / motor stop |
| 1-47 | Special commands (below) |
| 48-2047 | Throttle, 2000 resolution steps |

## Speed variants

| Variant | Bitrate | T1H | T0H | Bit period | Frame length | Max update rate |
|---|---|---|---|---|---|---|
| DShot150 | 150kbit/s | 5.00us | 2.50us | 6.67us | 106.67us | 9.375kHz |
| DShot300 | 300kbit/s | 2.50us | 1.25us | 3.33us | 53.33us | 18.75kHz |
| DShot600 | 600kbit/s | 1.25us | 0.625us | 1.67us | 26.67us | 37.5kHz |
| DShot1200 | 1200kbit/s | 0.625us | 0.3125us | 0.83us | 13.33us | 75kHz |

Frame length is 16 bit periods; the maximum update rate is frames sent back to back. Bidirectional
DShot cannot run that fast in practice - the ESC's reply occupies the line after every frame (see
"Bidirectional timing" below).

## Bit encoding

Each bit is encoded by the duration of the HIGH portion of the pulse: a "1" is HIGH for 75% of the
bit period (T1H), a "0" for 37.5% (T0H) - always a 2:1 ratio.

## Arming

Generically, an ESC requires a window of continuous zero throttle before it will accept real
throttle commands, and this window's exact length is firmware-specific - see `AM32.md` for one
firmware's actual gate, and `BETAFLIGHT.md`§2 for how a reference host satisfies it from the
sending side: it streams a zero/disarmed command every control-loop pass rather than going silent,
which is what keeps a zero-throttle window continuous from the ESC's point of view.

## CRC

**Command frames**, generic formula:
```
value = (throttle << 1) | telemetry_bit
crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
```

**Bidirectional command frames** invert that same formula before sending:
```
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F
```
This CRC inversion is a separate change from the signal-level inversion described below ("Signal
inversion") - bidirectional mode inverts both, but independently.

**Bidirectional reply frames**: brushlesswhoop.com states the reply CRC "is calculated exactly as
it is with uninverted DShot, [and] sent back to the flight-controller uninverted" - the same
formula as a plain command frame, not the inverted one above. Betaflight's own API docs reproduce
this same text almost verbatim, so it's one source's claim repeated, not independent confirmation
from a second source; the other two listed sources are silent on the point. See `AM32.md`#12 and
`BETAFLIGHT.md`#6 for two real implementations that both send and expect the reply CRC inverted
instead, diverging from what's documented here.

## Special commands (0-47)

Per the community Digital Cmd Spec reference:

| Code | Command | Repeat | Wait after |
|---|---|---|---|
| 0 | MOTOR_STOP | - | - |
| 1 | BEEP1 | 1x | 260ms |
| 2 | BEEP2 | 1x | 260ms |
| 3 | BEEP3 | 1x | 260ms |
| 4 | BEEP4 | 1x | 280ms |
| 5 | BEEP5 | 1x | 1020ms |
| 6 | ESC_INFO | 1x | 12ms |
| 7 | SPIN_DIRECTION_1 | 6x | - |
| 8 | SPIN_DIRECTION_2 | 6x | - |
| 9 | 3D_MODE_OFF | 6x | - |
| 10 | 3D_MODE_ON | 6x | - |
| 11 | SETTINGS_REQUEST | - | - |
| 12 | SAVE_SETTINGS | 6x | 35ms |
| 13 | EXTENDED_TELEMETRY_ENABLE | 6x | - |
| 14 | EXTENDED_TELEMETRY_DISABLE | 6x | - |
| 20 | SPIN_DIRECTION_NORMAL | 6x | - |
| 21 | SPIN_DIRECTION_REVERSED | 6x | - |
| 22-29 | LED0-3 ON/OFF | 1x | - |
| 30 | AUDIO_STREAM_MODE | - | - |
| 31 | SILENT_MODE | - | - |
| 32 | SIGNAL_LINE_TELEMETRY_DISABLE | 6x | - |
| 33 | SIGNAL_LINE_TELEMETRY_ENABLE | 6x | - |
| 34 | SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY | 6x | - |
| 35 | SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY | 6x | - |

| Code | Telemetry request | Resolution | Max value |
|---|---|---|---|
| 42 | TEMPERATURE | 1C/LSB | 4095C |
| 43 | VOLTAGE | 10mV/LSB | 40.95V |
| 44 | CURRENT | 100mA/LSB | 409.5A |
| 45 | CONSUMPTION | 10mAh/LSB | 40.95Ah |
| 46 | ERPM | 100eRPM/LSB | 409,500eRPM |
| 47 | ERPM_PERIOD | 16us/LSB | 65,520us |

Which of these any given firmware actually implements, and under what conditions, is
firmware-specific - see `AM32.md`.

## Bidirectional DShot

### Requirements

- DShot300 or higher.
- Firmware support on the ESC side.
- A signal line that both sides can drive and release.

### Signal inversion

Bidirectional DShot inverts the idle level and the pulse sense from standard DShot:
- **Standard**: line idles LOW, each bit is a HIGH pulse whose length encodes the bit.
- **Bidirectional**: line idles HIGH, each bit is a LOW pulse - 75% of the bit period LOW is "1",
  37.5% is "0". The underlying bit values and duty ratios are unchanged; only the levels are
  swapped.

### Timing

Generically, the ESC's reply follows a fixed turnaround after the command frame (community
figures put this around ~30us), and the two directions must not overlap - a transmitter has to
leave room for the full round trip (command frame, turnaround, reply, idle) before sending the
next frame. See `AM32.md` for one firmware's actual turnaround and drive-window timing, which
differs from this generic figure.

### Reply bit rate

Generically, the reply runs at 5/4 the command bitrate: 375kbit/s for DShot300, 750kbit/s for
DShot600. See `AM32.md` for a firmware whose actual reply rate is not exactly this.

### eRPM response frame

The reply is a 16-bit value: 12 bits of eRPM data (a 3-bit exponent + 9-bit mantissa) plus a 4-bit
CRC, encoded with GCR (Group Code Recording).

**GCR encoding.** Each 4-bit nibble maps to a 5-bit symbol, turning 16 bits into 20; a marker bit
brings the frame to 21 bits on the wire. A receiver reads 21 line levels (marker first), recovers
the 20 GCR bits by XOR-ing each level with the one before it (the marker serves as the "previous"
level for the first data bit), then maps each of the four resulting 5-bit groups back to a nibble
through the inverse of the encode table.

**eRPM payload**, generically:
```
period_us = mantissa << exponent
erpm = 60_000_000 / period_us
rpm = erpm / (motor_poles / 2)
```

### Extended DShot Telemetry (EDT)

A later convention that reuses eRPM reply frames' bandwidth to carry additional telemetry
(temperature, voltage, current, debug values) instead of eRPM on some replies, interleaved with
normal eRPM frames. See `AM32.md` for one firmware's actual EDT framing.

## Implementation notes

- Send commands continuously - an ESC's signal-loss recovery, and its exact timeout, is
  firmware-specific (see `AM32.md`).
- Higher DShot rates are more sensitive to signal integrity (wire length, grounding) than lower
  ones.
- Bit-timing tolerance - how far a real pulse can drift from the T1H/T0H figures above and still
  be read correctly - is a firmware decision, not something this protocol itself specifies (see
  `AM32.md` for one firmware's actual frame-length acceptance window).
