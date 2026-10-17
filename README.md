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
| **Motors** | BETAFPV LAVA Series 1104 (×2) | 7200KV, 5.3g, 12 rotor magnets (6 pole pairs) |
| **ESC** | Skystar RC KM55A2 (4-in-1) | AM32 firmware |
| **Optical RPM sensor** | Adafruit STEMMA reflective photo interrupt sensor (TCRT1000) | GPIO 14, 3.3V; 2-hole printed disk on motor 1's shaft |

The 12-magnet count is not on BETAFPV's product page; it was counted by eye on the rotor and then
confirmed by measurement: the optical RPM sensor gave eRPM / shaft RPM = 6.02-6.05 at throttle 60-100
(`tests/bench/rpm_accuracy/test_esc_rpm_vs_optical_rpm.py`). The ESC reports **electrical** RPM, so shaft RPM = eRPM / 6 here
(about 3,400 RPM at throttle 100, which the ESC reports as about 20,600 eRPM). Each scenario file
carries the motor's name, KV and magnet count in a `hardware` block, and a session's `meta.txt`
records them.

### Test Bench Configuration

See `tests/bench/dshot_protocol/scenarios/*.json` for the source-of-truth wiring.

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
│         MotorGroup Facade           │
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
the ESC firmware - see "Verified Parameters" below; on this bench, AM32 arms reliably both with
`Core1Runner`'s deadline-anchored pacing at 1045us (20/20, then 13/13 clean in later spot checks)
and with `MotorGroup.UPDATE_INTERVAL_US`'s own default, max-rate/no-pacing (0us). On this test
bench that means a dedicated Core 1 thread, keeping Core 0 free for the display and buttons -
see `tests/bench/core1_runner.py` for a ready-made example to copy into your project.

## Quick Start

### Single Motor (Low-Level Driver)

```python
from machine import Pin
from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS
import utime

motor = UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600)  # SM 0, GPIO 4
motor.start()  # Activate PIO state machine

# Arm ESC (send throttle=0 back-to-back for 2000ms, matching
# MotorGroup.DEFAULT_ARM_DURATION_MS - AM32's own gate needs more than 1s of
# continuous zero throttle after a 600ms startup tune; see "Verified
# Parameters" below).
arm_start = utime.ticks_ms()
while utime.ticks_diff(utime.ticks_ms(), arm_start) < 2000:
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
from core1_runner import Core1Runner  # your code - see tests/bench/core1_runner.py
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
# time disarm() resets the same state machines from this core, and nothing
# serialises the two. disarm() transmits nothing: it cuts the signal and resets
# every motor, and the ESC's own timeout stops the motor (AM32: up to 500ms at
# the last throttle). To stop sooner, set_throttle(i, 0) and let update() send
# it before calling disarm().
if not runner.stop():   # waits for the loop to really exit; False = it did not
    print("Core 1 still in update():", runner.error)
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

## Operational Envelope

What this driver guarantees, and what it does not.

| Guarantee | Value |
|-----------|-------|
| Arming success rate | **≥ 99%** of `arm()` attempts reach `ARMED` |
| Arming duration | **≤ 3.0 s** from `arm()` to `ARMED`, for every configured motor |
| Throttle range | `0` (stop) or `48`-`2047`; `MotorGroup` rejects `1`-`47` (DShot commands, not supported yet) |
| ESC firmware | AM32 only |

An attempt that has not reached `ARMED` within 3.0 s counts as a failure, even if it arms later
after an ESC reboot. Not guaranteed: DShot commands, other ESC firmware, motor behaviour after
arming.

**Status of the measurement:** in progress.

## Verified Parameters

Timing requirements are ESC-firmware-dependent, not just protocol-dependent. The library's
defaults (`MotorGroup.UPDATE_INTERVAL_US`, `DEFAULT_ARM_DURATION_MS`) are tuned to AM32's
measured requirements.

| Parameter | AM32 (Skystar KM55A2) |
|-----------|------------------------|
| Protocol | DShot300 and DShot600, both bench-verified (the command-interval and arming-duration rows below don't depend on which) |
| Minimum throttle | 60 confirmed spinning (DShot300, ADR-002; BUG-009 tracks an open rough-start question at low throttle) - 1-47 are special commands, not throttle: `MotorGroup.set_throttle()` rejects them (see `specification/DSHOT.md`, BUG-005) |
| Command interval | Two mechanisms both bench-confirmed reliable post-BUG-002: `Core1Runner`'s deadline-anchored pacing at 1045us (20/20 clean, then 13/13 in later spot checks, see BUG-002.md) and max-rate/no-pacing (`UPDATE_INTERVAL_US=0`, the library default). An earlier (2026-08-23) bring-up failure at "1kHz/500ms" was never a clean test of pacing specifically - its 500ms arm window is well under AM32's own >1.6s gate (`specification/AM32.md`#16) by itself, so that result doesn't isolate what role pacing played, if any |
| Arming duration | The published promise is >=99% within 3.0 s (see "Operational Envelope"). Window: 2000ms (`driver/motor_group.py`'s `DEFAULT_ARM_DURATION_MS`), grounded in AM32 source rather than bench measurement - see below. With bidirectional motors it is a floor: `ARMED` also waits until every ESC has replied "not running" for 2s without a reboot-length gap (`READY_SPAN_MS`, bug-reports/BUG-003) - measured about 2.03s on the 100-run batch of 2026-10-14 (99 of 100 runs between 2026 and 2028ms, one at 2516ms), several seconds if an ESC reboots while arming |

Earlier revisions of this table claimed 500ms (down to 300ms) armed cleanly, "confirmed via
genuine telemetry replies". That reasoning was wrong: per AM32's source (`specification/AM32.md`#8
and #9), a telemetry reply proves nothing about arm state - AM32 replies whether armed or
disarmed, with the same at-rest sentinel eRPM either way - and a bench re-test at 300/500/1000ms
confirmed the ESC was replying but the motor
never actually spun at any of those durations (eRPM stayed at rest for the whole run; it only
reached a real value in a 3000ms control). AM32's own arming gate needs more than 1s of continuous
zero throttle after it starts listening, plus a 600ms startup tune after a cold boot, hence the
2000ms default with margin. Likewise, the ESC's beep pattern was previously read as decoding
commands "regardless of whether its arm state machine has ever been satisfied" - per source, AM32
only executes DShot commands (including `BEEP1`) while armed, so a `BEEP1` heard on this bench
(sent as throttle value 1 through the low-level `send_throttle_command()`, not a dedicated command
API - BUG-005's exact path; `MotorGroup` has refused that value since 2026-10-13) that audibly beeped actually confirms the ESC **was** armed at that moment,
not that arm state is irrelevant to command
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
│   ├── pc/                          # PC unit tests (python -m unittest discover -s tests/pc)
│   └── bench/                       # tests that run on the Pico with the bench attached
│       ├── core1_runner.py          # Core 1 update() loop + fast_stop, shared by the groups below
│       ├── session_recorder/        # SD-card/RTC session recorder
│       ├── dshot_protocol/          # scenario runner + JSON scenarios, single-motor ESC probes
│       ├── pio_and_cores/           # PIO and two-core checks (Pico only, no ESC)
│       └── rpm_accuracy/            # ESC eRPM vs physical RPM from the reflective sensor
├── scripts/                         # deploy, pull/analyse captures, PC-side reference decoder
├── specification/
│   ├── DSHOT.md                      # Wire protocol (vendor/host-agnostic)
│   ├── AM32.md                       # AM32 firmware facts
│   └── BETAFLIGHT.md                 # Betaflight reference-host facts
├── decision/
│   └── ADR-00N-*.md                 # Architecture decision records
└── bug-reports/
    └── BUG-00N-*.md                 # Resolved/tracked bug reports
```

Nothing under `driver/` imports `_thread` or picks a core. `core1_runner.py`
lives in `tests/bench/` because it is bench infrastructure; an application is free to copy it, but nothing in the DShot library depends on it.

## DShot Protocol

See [DSHOT.md](specification/DSHOT.md) for the wire protocol itself, including:

- Packet structure (11-bit throttle + 1-bit telemetry + 4-bit CRC)
- Bit timing for all DShot variants
- Special commands (0-47)
- Bidirectional DShot and eRPM telemetry

See [AM32.md](specification/AM32.md) for how this project's target ESC firmware implements (and
sometimes deviates from) that protocol, and [BETAFLIGHT.md](specification/BETAFLIGHT.md) for how
a reference flight-controller host drives it.

## Roadmap

| Feature | Status | Dependencies |
|---------|--------|--------------|
| **DShot commands** | TODO | No driver-side command API yet (BUG-005's prerequisite is done: `MotorGroup` rejects 1-47, so a command API needs its own entry point, not `set_throttle()`). AM32 itself implements commands (`specification/AM32.md`#7), and this bench has already heard a `BEEP1` while armed (see above - sent as throttle value 1, not through a real command API) - ADR-003's original "several different ESCs required" blocker no longer applies now that AM32 is the only ESC in scope |
| **Bidirectional DShot** | Done | ADR-002: frame receiver bench-validated on AM32, in production use |
| **Extended telemetry (EDT)** | TODO | Current AM32 source already implements it (`specification/AM32.md`#10) - whether this bench's own flashed build does is unconfirmed (see ADR-002's MCU-identification note). This driver's own decoder doesn't yet discriminate an EDT frame from an eRPM frame either way |

See `decision/` folder for Architecture Decision Records (ADRs).

## License

GNU General Public License v3.0

Original DShot PIO implementation from [jrddupont/DShotPIO](https://github.com/jrddupont/DShotPIO).

## References

- [DShot Protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [BetaFPV Lava 1104 Motors](https://betafpv.com/products/lava-series-1104-brushless-motors)
- [AM32 Firmware](https://github.com/am32-firmware/AM32) - this project's ground truth for ESC behavior
