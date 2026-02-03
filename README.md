# DShot Driver for Raspberry Pi Pico

DShot protocol implementation for Raspberry Pi Pico/Pico 2 (RP2040/RP2350) using PIO, built for pet project - flight control test bench.

## Features

- **DShot150/300/600/1200** protocol support via PIO state machines
- **Dual-core architecture** for reliable motor control
- **Lock-free design** for low-latency throttle updates
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

The driver uses a three-layer architecture (see [ADR-001](decision/ADR-001-dual-core-motor-control.md)):

```
┌─────────────────────────────────────┐
│         Client Code (Core 0)        │
│   UI, control algorithms, sensors   │
└──────────────────┬──────────────────┘
                   │ setThrottle()
                   ▼
┌─────────────────────────────────────┐
│    MotorThrottleGroup Facade (Both)    │
│  Core 0: API, throttle updates      │
│  Core 1: 1kHz command transmission  │
└──────────────────┬──────────────────┘
                   │ sendThrottleCommand()
                   ▼
┌─────────────────────────────────────┐
│       DShotPIO Driver (Core 1)      │
│     PIO state machine, encoding     │
└─────────────────────────────────────┘
```

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
    motor.sendThrottleCommand(0)
    utime.sleep_ms(1)

# Run motor
while True:
    motor.sendThrottleCommand(100)
    utime.sleep_ms(1)
```

### Multiple Motors (Recommended)

```python
from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup

# Create group with Pin objects (DShotPIO instances created internally)
motors = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)
motors.start()  # Start 1kHz command loop on Core 1
motors.arm()    # Arm all ESCs

# Control motors independently
motors.setThrottle(0, 100)  # Motor 1
motors.setThrottle(1, 150)  # Motor 2

# Or update all at once
motors.setAllThrottles([100, 150])

# Cleanup
motors.stop()
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
├── driver/
│   ├── dshot_pio.py          # Low-level PIO driver
│   └── motor_throttle_group.py  # Dual-core facade
├── tests/
│   ├── test_dshot_single_motor.py  # Single motor test
│   ├── test_motor_throttle_group.py   # Multi-motor test
│   └── demo_manual_control.py      # Interactive demo with display
├── specification/
│   └── DSHOT_PROTOCOL.md     # Protocol documentation
└── decision/
    └── ADR-001-*.md          # Architecture decision record
```

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
