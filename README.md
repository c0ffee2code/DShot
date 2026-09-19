# DShot Driver for Raspberry Pi Pico

DShot protocol implementation for Raspberry Pi Pico/Pico 2 (RP2040/RP2350) using PIO, built for pet project - flight control test bench.

## Features

- **DShot300/600** protocol support via PIO state machines (restricted to what AM32 documents support for)
- **Scheduling-agnostic** - your application decides which core runs the command loop
- **Lock-free design** for low-latency throttle updates across cores
- **MicroPython** runtime (no external dependencies)

## Hardware

This driver was developed and tested on a flight control test bench, against
two different ESC firmware families with meaningfully different timing
requirements (see "Verified Parameters" below):

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Controller** | Raspberry Pi Pico 2 | RP2350, dual ARM Cortex-M33, 150MHz |
| **Motors** | BetaFPV Lava Series 1104 (×2) | 7200KV, 5g weight |
| **ESC (1)** | JHEMCU Brushless Wing Dual 40A 2-in-1 | 40A×2, 2-6S (7.4-27V), 6.2g |
| **Firmware (1)** | BLHeli_S | G-H-30 V16.7 |
| **ESC (2)** | Skystar RC KM55A2 (4-in-1) | AM32 firmware |

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
                   │ update()  ── as fast as UPDATE_INTERVAL_US allows
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
`update()` continuously, without long or irregular gaps. How fast depends on
the ESC firmware - see "Verified Parameters" below; some ESCs need
near-back-to-back frames just to complete arming. On this test bench that
means a dedicated Core 1 thread, keeping Core 0 free for the display and
buttons - see `tests/harness/core1_runner.py` for a ready-made example to copy into
your project.

## Quick Start

### Single Motor (Low-Level Driver)

```python
from machine import Pin
from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS
import utime

motor = UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600)  # SM 0, GPIO 4
motor.start()  # Activate PIO state machine

# Arm ESC (send throttle=0 back-to-back for ~500ms). Some ESC firmware needs
# near-continuous frames to arm at all - see "Verified Parameters" below.
arm_start = utime.ticks_ms()
while utime.ticks_diff(utime.ticks_ms(), arm_start) < 500:
    motor.send_throttle_command(0)

# Run motor
while True:
    motor.send_throttle_command(100)

motor.stop()  # Deactivate - the ESC times out and the motor cannot spin
```

### Multiple Motors (Recommended)

```python
from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner  # your code - see tests/harness/core1_runner.py
import utime

# Create group with Pin objects (UnidirectionalDShot instances created internally)
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
    motors.update()          # must happen continuously, without long gaps
    ...your work here...
    utime.sleep_us(motors.UPDATE_INTERVAL_US)
```

## Verified Parameters

Timing requirements are ESC-firmware-dependent, not just protocol-dependent -
the two ESCs tested needed meaningfully different arming behavior. The
library's defaults (`MotorThrottleGroup.UPDATE_INTERVAL_US`,
`DEFAULT_ARM_DURATION_MS`) target the more demanding of the two, since a
faster/longer hold is always safe for the less demanding one too.

| Parameter | JHEMCU / BLHeli_S | Skystar KM55A2 / AM32 |
|-----------|---------------------|------------------------|
| Protocol | DShot600 | DShot300 |
| Minimum throttle | 70 (50-69 unreliable) | 100 confirmed working |
| Command interval | 1ms (1kHz) tolerant | Back-to-back required (0us / no sleep) - a clean, jitter-free 1kHz was not enough; even sleep-paced 250us (4kHz) failed once real per-call overhead was added, but max-rate (no sleep) arms reliably |
| Arming duration | 500ms | 500ms - an earlier finding claimed 500ms never completed the ESC's own arm confirmation and set this to 3000ms, but that test predated a board-reset bug fix (see `driver/motor_throttle_group.py`'s `DEFAULT_ARM_DURATION_MS`); re-tested 2026-09-12 with the corrected workflow and 500ms (down to 300ms) armed cleanly, confirmed via genuine telemetry replies |

The AM32 ESC gave no indication via its beep pattern alone that arming was
failing - it decodes individual commands correctly (confirmed via the DShot
`BEEP1` special command) regardless of whether its arm state machine has
ever been satisfied. The only reliable signal was the ESC's own "3 short
beeps, then 2 deeper beeps" arm confirmation tone; current draw at the power
supply (near-zero until genuinely armed and driving) was the second
confirming signal.

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
