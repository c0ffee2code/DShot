# DShot Driver for Raspberry Pi Pico

DShot protocol implementation for Raspberry Pi Pico/Pico 2 (RP2040/RP2350) using PIO, built for pet project - flight control test bench.

## Features

- **DShot300/600** protocol support via PIO state machines (restricted to what AM32 documents support for)
- **Scheduling-agnostic** - your application decides which core runs the command loop
- **Lock-free design** for low-latency throttle updates across cores
- **MicroPython** runtime (no external dependencies)

## Hardware

**AM32 is the only supported ESC firmware** (see CLAUDE.md's "Supported ESC targets") -
no other ESC exists on this project's bench. An earlier BLHeli_S ESC was used before the
project narrowed to AM32 only; its measurements live on in ADR-001/ADR-004 as the reasoning
trail for some of the library's conservative defaults, not as current hardware.

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Controller** | Raspberry Pi Pico 2 | RP2350, dual ARM Cortex-M33, 150MHz |
| **Motors** | BetaFPV Lava Series 1104 (×2) | 7200KV, 5g weight |
| **ESC** | Skystar RC KM55A2 (4-in-1) | AM32 firmware |

### Test Bench Configuration

See `tests/harness/scenarios/*.json` for the source-of-truth wiring.

```
                    ┌─────────────┐
                    │   Pico 2    │
                    │  (RP2350)   │
                    └─┬──┬──┬──┬──┘
                 GPIO6│  │  │  │GPIO9
                      │7 │  │8 │
                      ▼  ▼  ▼  ▼
              ┌─────────────────────────┐
              │  Skystar KM55A2 (4-in-1) │
              │      AM32 firmware       │
              │  ch1  ch2  ch3  ch4      │
              └──┬───────────┬───────────┘
                 ▼           ▼
            ┌──────────┐   ┌──────────┐
            │ Motor 1  │   │ Motor 2  │
            │ 1104     │   │ 1104     │
            │ 7200KV   │   │ 7200KV   │
            └──────────┘   └──────────┘
```

ch2 (GPIO7) and ch4 (GPIO9) are wired but idle in every scenario - only ch1/ch3 drive motors.

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
│    MotorGroup Facade        │
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
the ESC firmware - see "Verified Parameters" below; AM32 needs near-back-to-back
frames just to complete arming. On this test bench that
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

# Arm ESC (send throttle=0 back-to-back for ~500ms). AM32 needs
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
from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner  # your code - see tests/harness/core1_runner.py
import utime

# One motor object per motor (1-4). You pick the state machine and pin for each;
# use BidirectionalDShot instead where you want eRPM telemetry.
motors = MotorGroup([
    UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600),
    UnidirectionalDShot(1, Pin(5), DSHOT_SPEEDS.DSHOT600),
])

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

# Stop the loop first: while it's running, Core 1 can call update() at the same
# time disarm() sends/drains/stops on the same state machines from this core,
# and nothing serialises the two. Then disarm() commands zero throttle and
# cuts the signal - the only call in the API that blocks, for ~0.3ms.
runner.stop()
motors.disarm()
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

Timing requirements are ESC-firmware-dependent, not just protocol-dependent. The library's
defaults (`MotorGroup.UPDATE_INTERVAL_US`, `DEFAULT_ARM_DURATION_MS`) are tuned to AM32's
measured requirements, which are demanding enough (see "Command interval" below) that they
need no further headroom.

| Parameter | AM32 (Skystar KM55A2) |
|-----------|------------------------|
| Protocol | DShot300 |
| Minimum throttle | 100 confirmed working |
| Command interval | Back-to-back required (0us / no sleep) - a clean, jitter-free 1kHz was not enough; even sleep-paced 250us (4kHz) failed once real per-call overhead was added, but max-rate (no sleep) arms reliably |
| Arming duration | 2000ms (`driver/motor_group.py`'s `DEFAULT_ARM_DURATION_MS`), grounded in AM32 source rather than bench measurement - see below |

Earlier revisions of this table claimed 500ms (down to 300ms) armed cleanly, "confirmed via
genuine telemetry replies". That reasoning was wrong: per AM32's source
(`specification/AM32_SOURCE_VERIFICATION.md`, findings 1-2), a telemetry reply proves nothing
about arm state - AM32 replies whether armed or disarmed, with the same at-rest sentinel eRPM
either way - and a bench re-test at 300/500/1000ms confirmed the ESC was replying but the motor
never actually spun at any of those durations (eRPM stayed at rest for the whole run; it only
reached a real value in a 3000ms control). AM32's own arming gate needs more than 1s of continuous
zero throttle after it starts listening, plus a 600ms startup tune after a cold boot, hence the
2000ms default with margin. Likewise, the ESC's beep pattern was previously read as decoding
commands "regardless of whether its arm state machine has ever been satisfied" - per source, AM32
only executes DShot commands (including `BEEP1`) while armed, so a `BEEP1` that audibly beeped
actually confirms the ESC **was** armed at that moment, not that arm state is irrelevant to command
execution. The ESC's own "3 short beeps, then 2 deeper beeps" arm confirmation tone, and current
draw at the power supply (near-zero until genuinely armed and driving), remain the reliable signals
that the motor is actually armed and spinning.

## Project Structure

```
├── driver/                          # the library - core-agnostic
│   ├── dshot_pio.py                 # PIO programs and the UnidirectionalDShot / BidirectionalDShot motors
│   ├── motor_group.py               # MotorGroup: multi-motor facade (arm, update, telemetry)
│   ├── capture_mailbox.py           # One-slot latest-capture store shared between cores
│   ├── gcr_decode.py                # On-device decoder for the ESC's GCR telemetry reply
│   └── dshot_profiles.py            # DShot speeds and the tuned receiver profile for each
├── tests/
│   ├── harness/                     # bench regression suite: scenario runner + JSON scenarios,
│   │                                # plus core1_runner.py, an example Core 1 loop to copy
│   ├── unit/                        # PC unit tests (python -m unittest discover -s tests/unit)
│   └── device/                      # on-Pico checks: two-core mailbox stress (no ESC needed),
│   │                                # and a multi-cycle arm/disarm restart check (needs one)
├── scripts/                         # deploy, pull/analyse captures, PC-side reference decoder
├── specification/
│   └── DSHOT_PROTOCOL.md            # Protocol documentation
├── decision/
│   └── ADR-00N-*.md                 # Architecture decision records
└── bug-reports/
    └── BUG-00N-*.md                 # Resolved/tracked bug reports
```

Nothing under `driver/` imports `_thread` or picks a core. `core1_runner.py`
lives in `tests/harness/` because it is an application concern, not a DShot one.

## DShot Protocol

See [DSHOT_PROTOCOL.md](specification/DSHOT_PROTOCOL.md) for complete protocol documentation including:

- Packet structure (11-bit throttle + 1-bit telemetry + 4-bit CRC)
- Bit timing for all DShot variants
- Special commands (0-47)
- Bidirectional DShot and eRPM telemetry

## Roadmap

| Feature | Status | Dependencies |
|---------|--------|--------------|
| **DShot commands** | Blocked | ADR-003: several different ESCs required for testing |
| **Bidirectional DShot** | Done | ADR-002: frame receiver bench-validated on AM32, in production use |
| **Extended telemetry (EDT)** | Blocked | Needs AM32-side EDT support to test against |

See `decision/` folder for Architecture Decision Records (ADRs).

## License

GNU General Public License v3.0

Original DShot PIO implementation from [jrddupont/DShotPIO](https://github.com/jrddupont/DShotPIO).

## References

- [DShot Protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [BetaFPV Lava 1104 Motors](https://betafpv.com/products/lava-series-1104-brushless-motors)
- [AM32 Firmware](https://github.com/am32-firmware/AM32) - this project's ground truth for ESC behavior
