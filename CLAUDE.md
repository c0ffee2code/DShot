# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DShot driver for Raspberry Pi Pico, part of a flight control systems test bench. The test bench consists of 2 drone motors on a swinging lever, 2 ESCs, power distribution board, power supply, and Adafruit sensors (including magnetic encoder). This repository focuses specifically on the DShot protocol implementation.

Original implementation from https://github.com/jrddupont/DShotPIO (GNU GPL v3.0 license).

**Supported ESC targets (design constraint):** This is a pet/exploration project — it does
not aim to support the endless universe of ESCs. Exactly two firmware families are in scope:
**BLHeli_S** (cheap, old, unidirectional DShot only in stock form) and **AM32** (modern,
bidirectional-capable). AM32 has a further advantage: it is open source
(https://github.com/am32-firmware/AM32), so behavior is verified against its actual
firmware source rather than guessed from generic protocol articles — when a generic spec and AM32's source disagree, the source wins. Do not add
abstraction layers or configuration surface for hypothetical other ESC families.

## Project Goals

| Goal | Status | Details |
|------|--------|---------|
| **Improve arming sequence** | Done | ADR-001: continuous 1kHz commands solve timing issues |
| **Invert core assignment to client** | Done | ADR-004: library exposes `update()`, application owns the loop |
| **DShot commands** | Blocked | ADR-003: Several different ESCs required for testing |
| **Bidirectional DShot** | Deferred | ADR-002: Needs Bluejay firmware or BLHeli_32/AM32 ESCs |

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

## Code Style

- **No `_` prefix for visibility.** This is MicroPython on a microcontroller, not a published CPython package — attributes and methods are named plainly (`self.motors`, `self.state`, `runner.loop()`). The underscore convention buys nothing here and just adds noise. `_thread` (a stdlib module) and `__init__` are unaffected, as is `for _ in range(n)` for a throwaway loop variable.
- **snake_case** throughout `driver/`, including method names.
- Prefer plain attributes over accessor methods. Keep a method only when it does real work — `get_all_throttles()` converts an `array` to a list, `is_armed()` compares against a state constant.
- Avoid f-strings on error paths in `driver/`; they allocate, and that code may run on a core with a constrained stack.

## Architecture

### Core Components

**`driver/dshot_pio.py`** - Low-level PIO driver:

1. **PIO Assembly (`dshot` function)**: State machine program generating DShot waveforms. Each bit takes 8 clock cycles with specific high/low timing to encode 0s and 1s.

2. **`DSHOT_SPEEDS` class**: Protocol variant constants, restricted to DSHOT300/600 — the only speeds AM32 documents support for (its README and wiki.am32.ca; DSHOT150 and DSHOT1200 are deliberately not offered, see the class's own comment). Values are clock frequencies: `bit_rate * 8_cycles_per_bit`.

3. **`DShotPIO` class**: Main driver. Creates a PIO state machine on the specified pin (inactive until `start()`), provides `send_throttle_command(throttle)` to send 16-bit packets (11-bit throttle + 1-bit telemetry + 4-bit CRC), and `stop()` to deactivate.

**`driver/motor_throttle_group.py`** - Multi-motor facade (see ADR-004):

1. **`MotorThrottleGroup` class**: Owns the PIO state machines and throttle values for a group of motors. Provides `arm()`, `disarm()`, `update()`, `is_armed()`, `set_throttle()`.

2. **Core-agnostic by design**: The library does **not** spawn threads or pick a core. The application calls `update()` at least every 1ms from wherever its architecture dictates. Do not add `_thread` to anything under `driver/` — that inversion is the whole point of ADR-004.

3. **Non-blocking arming**: `arm()` opens the arming window and returns; `update()` completes it. The application polls `is_armed()`. This is what lets the library work in a cooperative single-core loop as well as on a dedicated core.

4. **`update()` is inert while disarmed**: a safety requirement, not an optimisation. Writing to deactivated state machines would fill the TX FIFO and block the calling core forever.

5. **`disarm()` transmits its own zeros**, then drains them, then deactivates — in that order. It is the one method in the facade that blocks (a few hundred microseconds). Deactivating alone is not a stop: the motor keeps spinning at its last throttle until the ESC's own 100-250ms signal-loss timeout expires. Do not "optimise" the transmit away.

6. **Lock-free design**: Shared throttle array allows one core to update values while another sends commands. See ADR-001 for technical details on atomic writes.

**`tests/harness/`** - Example application code and bench infrastructure, deliberately *not* part of the library: `core1_runner.py` (a Core 1 loop that drives `update()` at 1kHz; projects copy and adapt it), `bidir_capture_runner.py`/`bidir_capture_sink.py` (dual-core raw bidir RX capture + SD logging, see `tests/test_bidir_rx_capture.py`), and the ported `sdcard.py`/`pcf8523.py` drivers for the PicoBell Adalogger SD+RTC breakout. Kept separate from the runnable `test_*.py` scripts directly under `tests/` so the two aren't mixed together.

### DShot Protocol

See `specification/DSHOT_PROTOCOL.md` for complete protocol documentation including:
- Packet structure and bit timing for all DShot variants
- Special commands (0-47) with repeat counts and timing requirements
- Bidirectional DShot and eRPM telemetry
- CRC calculation

## Usage Example

**Recommended (multi-motor with reliable timing):**
```python
from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner  # application code, see tests/
import utime

motors = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)

# The application picks the core - here, a dedicated Core 1 loop
runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)
runner.start()

motors.arm()                    # non-blocking
while not motors.is_armed():
    utime.sleep_ms(10)

motors.set_throttle(0, 100)     # Motor 0
motors.set_throttle(1, 150)     # Motor 1

motors.disarm()                 # commands zero, then cuts the signal
runner.stop()
```

**Low-level (single motor):**
```python
from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS

motor = DShotPIO(0, Pin(4), DSHOT_SPEEDS.DSHOT600)
motor.start()  # Activate PIO state machine
motor.send_throttle_command(100)  # Must call continuously at 1ms intervals
motor.stop()   # Deactivate
```
