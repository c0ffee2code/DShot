# DShot Driver for Raspberry Pi Pico

DShot protocol implementation for Raspberry Pi Pico/Pico 2 (RP2040/RP2350) using PIO, built for pet project - flight control test bench.

## Features

- **DShot150/300/600/1200** protocol support via PIO state machines
- **Scheduling-agnostic** - your application decides which core runs the command loop
- **Lock-free design** for low-latency throttle updates across cores
- **MicroPython** runtime (no external dependencies)

## Hardware

This driver was developed and tested on a flight control test bench:

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Controller** | Raspberry Pi Pico 2 | RP2350, dual ARM Cortex-M33, 150MHz |
| **Motors** | BetaFPV Lava Series 1104 (×2) | 7200KV, 5g weight |
| **ESC** | JHEMCU Brushless Wing Dual 40A 2-in-1 | 40A×2, 2-6S (7.4-27V), 6.2g |
| **Firmware** | BLHeli_S | G-H-30 V16.7 |

### Test Bench Configuration

```
                    ┌─────────────┐
                    │  Pico 2     │
                    │  (RP2350)   │
                    └──┬───────┬──┘
                 GPIO4 │       │ GPIO5
                       ▼       ▼
              ┌────────────────────────┐
              │  JHEMCU 2-in-1 ESC     │
              │  (BLHeli_S firmware)   │
              └────┬──────────────┬────┘
                   ▼              ▼
            ┌──────────┐   ┌──────────┐
            │ Motor 1  │   │ Motor 2  │
            │ 1104     │   │ 1104     │
            │ 7200KV   │   │ 7200KV   │
            └──────────┘   └──────────┘
```

## Architecture

The driver uses a three-layer architecture. The library is deliberately
core-agnostic: it exposes `update()`, and **your application decides where that
runs** - a dedicated Core 1 thread, a timer IRQ, or its own main loop. See
[ADR-004](decision/ADR-004-client-owned-command-loop.md) for why, and
[ADR-001](decision/ADR-001-dual-core-motor-control.md) for the timing
requirements that drive it.

```
┌─────────────────────────────────────┐
│            Application              │
│   UI, control algorithms, sensors   │
│   OWNS THE COMMAND LOOP             │
└──────────────────┬──────────────────┘
                   │ update()  ── at least every 1ms
                   │ set_throttle()
                   ▼
┌─────────────────────────────────────┐
│    MotorThrottleGroup Facade        │
│  Throttle state, arming sequence,   │
│  PIO lifecycle. Core-agnostic.      │
└──────────────────┬──────────────────┘
                   │ send_throttle_command()
                   ▼
┌─────────────────────────────────────┐
│          DShotPIO Driver            │
│     PIO state machine, encoding     │
└─────────────────────────────────────┘
```

ESCs disarm if commands stop arriving, so whatever context you choose must call
`update()` at least every millisecond. On this test bench that means a dedicated
Core 1 thread, keeping Core 0 free for the display and buttons - see
`tests/core1_runner.py` for a ready-made example to copy into your project.

## Quick Start

### Single Motor (Low-Level Driver)

```python
from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

motor = DShotPIO(0, Pin(4), DSHOT_SPEEDS.DSHOT600)  # SM 0, GPIO 4
motor.start()  # Activate PIO state machine

# Arm ESC (send throttle=0 for 500ms)
for _ in range(500):
    motor.send_throttle_command(0)
    utime.sleep_ms(1)

# Run motor
while True:
    motor.send_throttle_command(100)
    utime.sleep_ms(1)

motor.stop()  # Deactivate - the ESC times out and the motor cannot spin
```

### Multiple Motors (Recommended)

```python
from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner  # your code - see tests/core1_runner.py
import utime

# Create group with Pin objects (DShotPIO instances created internally)
motors = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)

# You choose where the command loop runs. This one dedicates Core 1.
runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)
runner.start()

# Arming is non-blocking - poll while doing something useful
motors.arm()
while not motors.is_armed():
    utime.sleep_ms(10)

# Control motors independently
motors.set_throttle(0, 100)  # Motor 1
motors.set_throttle(1, 150)  # Motor 2

# Or update all at once
motors.set_all_throttles([100, 150])

# Commands zero throttle and then cuts the signal, whether or not the loop
# is still alive. The only call in the API that blocks - for ~0.3ms.
motors.disarm()
runner.stop()
```

### Without a Second Core

The same group works when you pump it from your own main loop - useful when
Core 1 is busy or unavailable:

```python
motors.arm()
while True:
    motors.update()          # must happen at least every 1ms
    ...your work here...
    utime.sleep_us(motors.UPDATE_INTERVAL_US)
```

## Verified Parameters

Tested with specific hardware (JHEMCU 40A ESC + test bench motors). May differ with other ESC/motor combinations.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Protocol | DShot600 | Best balance of speed and reliability |
| Minimum throttle | 70 | Hardware-specific; values 50-69 unreliable on test bench |
| Command interval | 1ms | Required for reliable operation |
| Arming duration | 500ms | Works with BLHeli_S firmware |

## Project Structure

```
├── driver/                          # the library - core-agnostic
│   ├── dshot_pio.py                 # Low-level PIO driver
│   └── motor_throttle_group.py      # Multi-motor facade
├── tests/                           # application code
│   ├── core1_runner.py              # Example Core 1 loop (copy into your project)
│   ├── test_dshot_single_motor.py   # Single motor test
│   ├── test_motor_throttle_group.py # Multi-motor test
│   └── demo_manual_control.py       # Interactive demo with display
├── specification/
│   └── DSHOT_PROTOCOL.md     # Protocol documentation
└── decision/
    └── ADR-00N-*.md          # Architecture decision records
```

Nothing under `driver/` imports `_thread` or picks a core. `core1_runner.py`
lives in `tests/` because it is an application concern, not a DShot one.

## DShot Protocol

See [DSHOT_PROTOCOL.md](specification/DSHOT_PROTOCOL.md) for complete protocol documentation including:

- Packet structure (11-bit throttle + 1-bit telemetry + 4-bit CRC)
- Bit timing for all DShot variants
- Special commands (0-47)
- Bidirectional DShot and eRPM telemetry

## Roadmap

| Feature | Status | Dependencies |
|---------|--------|--------------|
| **DShot commands** | Blocked | Several different ESCs required for testing |
| **Bidirectional DShot** | Deferred | ESC firmware: Bluejay, BLHeli_32, or AM32 |
| **Extended telemetry (EDT)** | Blocked | Bidirectional DShot + compatible firmware |

See `decision/` folder for Architecture Decision Records (ADRs).

## License

GNU General Public License v3.0

Original DShot PIO implementation from [jrddupont/DShotPIO](https://github.com/jrddupont/DShotPIO).

## References

- [DShot Protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [BetaFPV Lava 1104 Motors](https://betafpv.com/products/lava-series-1104-brushless-motors)
- [JHEMCU Dual 40A ESC](https://www.jhemcu.com/e_productshow/?81-JHEMCU-BRUSELESS-WING-DUAL-40A-2IN1-ESC-81.html)
