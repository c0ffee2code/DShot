# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DShot driver for Raspberry Pi Pico, part of a flight control systems test bench. The test bench consists of 2 drone motors on a swinging lever, 2 ESCs, power distribution board, power supply, and Adafruit sensors (including magnetic encoder). This repository focuses specifically on the DShot protocol implementation.

Original implementation from https://github.com/jrddupont/DShotPIO (GNU GPL v3.0 license).

## Project Goals

| Goal | Status | Details |
|------|--------|---------|
| **Improve arming sequence** | Done | ADR-001: Dual-core facade solves timing issues |
| **DShot commands** | Deferred | ADR-003: Beeps and spin direction - not needed for test bench |
| **Bidirectional DShot** | Deferred | ADR-002: Needs Bluejay firmware or BLHeli_32/AM32 ESCs |

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

## Architecture

### Core Components

**`driver/dshot_pio.py`** - Low-level PIO driver:

1. **PIO Assembly (`dshot` function)**: State machine program generating DShot waveforms. Each bit takes 8 clock cycles with specific high/low timing to encode 0s and 1s.

2. **`DSHOT_SPEEDS` class**: Protocol variant constants (DSHOT150/300/600/1200). Values are clock frequencies: `bit_rate * 8_cycles_per_bit`.

3. **`DShotPIO` class**: Main driver. Initializes PIO state machine on specified pin, provides `sendThrottleCommand(throttle)` to send 16-bit packets (11-bit throttle + 1-bit telemetry + 4-bit CRC).

**`driver/motor_throttle_group.py`** - Dual-core facade (see ADR-001):

1. **`MotorThrottleGroup` class**: Manages multiple motors with guaranteed 1kHz command rate via Core 1 dedicated loop. Provides `arm()`, `setThrottle()`, `emergencyStop()`.

2. **Lock-free design**: Shared throttle array allows Core 0 to update values while Core 1 continuously sends commands.

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

motors = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)
motors.start()  # Start Core 1 loop
motors.arm()    # Arm ESCs
motors.setThrottle(0, 100)  # Motor 0
motors.setThrottle(1, 150)  # Motor 1
```

**Low-level (single motor):**
```python
from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS

motor = DShotPIO(0, Pin(4), DSHOT_SPEEDS.DSHOT600)
motor.start()  # Activate PIO state machine
motor.sendThrottleCommand(100)  # Must call continuously at 1ms intervals
```
