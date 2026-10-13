# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DShot driver for Raspberry Pi Pico, part of a flight control systems test bench. The test bench consists of 2 drone motors on a swinging lever, 2 ESCs, power distribution board, power supply, and Adafruit sensors (including magnetic encoder). This repository focuses specifically on the DShot protocol implementation.

Original implementation from https://github.com/jrddupont/DShotPIO (GNU GPL v3.0 license).

**Supported ESC targets (design constraint):** This is a pet/exploration project — it does
not aim to support the endless universe of ESCs. **AM32** is the sole firmware family in
scope (modern, bidirectional-capable). AM32 has a further advantage: it is open source
(https://github.com/am32-firmware/AM32), so behavior is verified against its actual
firmware source rather than guessed from generic protocol articles — when a generic spec and AM32's source disagree, the source wins. Do not add
abstraction layers or configuration surface for hypothetical other ESC families.
**BLHeli_S** was an earlier bench target (see ADR-001/ADR-004 for the timing data it produced -
README's own "Verified Parameters" table is AM32-only now) but is no longer supported — do not
add code paths or timing accommodations to keep it working.

## Project Goals

| Goal | Status | Details |
|------|--------|---------|
| **Improve arming sequence** | Done | Evidence-gated arming on a 2000ms floor, not a timer alone (BUG-002/BUG-003, AM32.md#16); ADR-001's own 2026-08-23 bring-up finding used a 500ms arm window, well under AM32's own >1.6s gate by itself, so it never isolated command pacing as a cause - see README's "Verified Parameters" for what pacing has actually been bench-confirmed since |
| **Invert core assignment to client** | Done | ADR-004: library exposes `update()`, application owns the loop |
| **DShot commands** | TODO | No driver-side command API yet (BUG-005's prerequisite is done: `MotorGroup` rejects 1-47, so a command API needs its own entry point, not `set_throttle()`). AM32 itself implements commands (AM32.md#7) and this bench has already heard a `BEEP1` while armed (sent as throttle value 1, not through a real command API) - ADR-003's original "several different ESCs required" blocker no longer applies now that AM32 is the only ESC in scope |
| **Bidirectional DShot** | Done | ADR-002: frame receiver (`dshot_bidir_rx_frame`) bench-validated on AM32 and adopted as the sole production receiver (2026-09-26) |

## Operational Envelope (what this driver guarantees)

| Guarantee | Value |
|-----------|-------|
| Arming success rate | **≥ 99%** of `arm()` attempts reach `ARMED` |
| Arming duration | **≤ 3.0 s** from `arm()` to `ARMED`, for every configured motor |
| Throttle range | `0` (stop) or `48`-`2047`. `1`-`47` are DShot commands; `MotorGroup` rejects them (BUG-005) |
| ESC firmware | AM32 only |

An attempt that does not reach `ARMED` within 3.0 s counts as a failure, including one that
arms later after an ESC reboot. Callers should treat it as a failure to handle, not wait
indefinitely. Where the bound sits: across the 698 sessions archived on the bench (mostly earlier
code), clean arms took 2.08-2.59 s and arms that needed an ESC reboot took 4.06 s or more, so 3.0 s
falls in the empty gap between the two. That archive justifies the bound, not the 99% rate.
Evidence for the rate on current code: 41 of 41 arming-only bench runs on 2026-10-13, across the
last four revisions of this code (10, 10, 20 and 1 runs). It is not yet enough to support 99%
(about 300 clean attempts are needed); the long batch is waiting to be scheduled. The bound is
measured by `scripts/run_scenarios.py` (`CLEAN_ARM_THRESHOLD_MS`); keep that constant, README.md
and this table in agreement. Not guaranteed: DShot commands, other ESC firmware, behaviour of the
motor after arming (BUG-009 is open).

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

## Communicating with the user

Explain work as goal → step → why, in plain language.

- Say what you will do and how it moves the user's current goal *before* naming files,
  scripts, bug numbers, backlog IDs or metrics. Describe any such name in prose on first
  mention; the user does not hold these labels in their head between sessions.
- Do not resurface an older plan's "next item" unless you can say how it serves the goal the
  user is pursuing now.
- Keep "you need to decide" items in their own block, apart from information, with each
  option's consequence in the user's terms.
- Say whether a number comes from the user's memory, your memory, or the repo - and check the
  repo first.

## Code Style

- **No `_` prefix for visibility.** This is MicroPython on a microcontroller, not a published CPython package — attributes and methods are named plainly (`self.motors`, `self.state`, `runner.loop()`). The underscore convention buys nothing here and just adds noise. `_thread` (a stdlib module) and `__init__` are unaffected, as is `for _ in range(n)` for a throwaway loop variable.
- **snake_case** throughout `driver/`, including method names.
- Prefer plain attributes over accessor methods. Keep a method only when it does real work — `get_all_throttles()` converts an `array` to a list, `is_armed()` compares against a state constant.
- Avoid f-strings on error paths in `driver/`; they allocate, and that code may run on a core with a constrained stack.

## Investigating a bench-behavior shift over time

When two batches of bench runs, taken hours or days apart, behave differently, check for an actual
code change before reaching for "thermal drift," "power sag," "day-to-day factor," or any other
unnamed environmental cause. Those can be real, but they're rarely the first thing to check, and
naming one without evidence just launders a gap in the investigation as a finding.

**Committed timestamps lag deployed code.** Code is routinely written, deployed to the Pico, and
bench-tested *before* it's committed - so "what commit was HEAD at that wall-clock time" is not
"what code was actually running on the bench." A commit's own message often says so directly (e.g.
"bench-verified: 10 more runs") - that verification happened before the commit, not after. When
comparing two batches, check `git log --date=iso-local -- driver tests/harness` for every commit
between them, read what each one actually changed, and check whether it's mechanistically capable
of causing the observed shift - not just correlated in time with it. A real example from BUG-002's
2026-09-30 investigation: a 09-27 evening jump in first-contact acceptance lined up with a commit
timestamped minutes *after* the jump, because the change had been bench-verified first and
committed once confirmed.

## Architecture

### Core Components

**`driver/dshot_pio.py`** - Low-level PIO driver:

1. **PIO Assembly (`dshot` function)**: State machine program generating DShot waveforms. Each bit takes 8 clock cycles with specific high/low timing to encode 0s and 1s.

2. **`DSHOT_SPEEDS` class**: Protocol variant constants, restricted to DSHOT300/600 — the only speeds AM32 documents support for (its README and wiki.am32.ca; DSHOT150 and DSHOT1200 are deliberately not offered, see the class's own comment). Values are clock frequencies: `bit_rate * 8_cycles_per_bit`.

3. **`DShotPIO` base class, with `UnidirectionalDShot` and `BidirectionalDShot` subclasses**: Main driver. Creates a PIO state machine on the specified pin (inactive until `start()`), provides `send_throttle_command(throttle)` to send 16-bit packets (11-bit throttle + 1-bit telemetry + 4-bit CRC), and `stop()` to deactivate. `BidirectionalDShot` additionally owns the RX state machine that captures the ESC's telemetry reply, and a `CaptureMailbox` (`driver/capture_mailbox.py`, pure Python) that assembles replies from the RX FIFO and holds the latest one for another core to read. The receiver is `dshot_bidir_rx_frame` (see ADR-002's "run-length capture" section): it reconstructs the frame in the state machine itself and hands the CPU one already-decoded word per reply, for `gcr_decode.analyze_frame()`. This TX/RX pair fills its PIO block on its own (adopted as the sole production receiver 2026-09-26, after bench validation — see the backlog) — it cannot share one with a unidirectional motor, though two bidirectional motors may share one block (identical programs load once per block). The older sample receiver, `dshot_bidir_rx` (128 raw samples per reply, reconstructed on the CPU by `gcr_decode.analyze_capture()`), backed a standalone calibration/diagnostic tool (`tools/calibrate_bidir_rx.py`) that measured a new ESC unit's `BIDIR_PROFILES` entry, since the frame receiver has no bit-period search of its own for that. Both were removed 2026-10-11: the tool never worked (it captured TX's own echo rather than a real ESC reply) and the project settled on one ESC, so the use case never came up. The raw-sample reconstruction math it depended on (`find_edges`, `reconstruct_frame`, `sample_cycle`) moved to `scripts/simulate_frame_receiver.py`, which still needs it to verify `dshot_bidir_rx_frame`'s own PIO logic against real captures.

**`driver/motor_group.py`** - Multi-motor facade (see ADR-004):

1. **`MotorGroup` class**: Takes 1-4 already-built `UnidirectionalDShot`/`BidirectionalDShot` motors (the application picks their state machines and pins) and owns the throttle values and lifecycle for the group. Provides `arm()` (only valid while disarmed), `disarm()`, `update()`, `is_armed()`, `set_throttle()`, `set_all_throttles()`, `raw_telemetry(index)`, `decode_telemetry(index, words)` and `arming_status()`. It rejects motors that share a state machine or a pin. `update()` also drains every bidirectional motor's RX FIFO into that motor's one-slot latest capture (discarded while arming, unless the diagnostic `publish_while_arming` flag is set); `raw_telemetry()` hands the latest capture out only once armed and holds no data itself. Decoding (`decode_telemetry()`) and retry-on-CRC-failure are the application's business, on its own schedule. The harness no longer logs the arming phase or wraps `update()` with per-tick measurement (retired 2026-10-06, BUG-002: real overhead - ~90us/call even idle - on the exact loop whose timing the investigation cares about; see BUG-002.md's "Telemetry overhead and the timed window" section).

2. **Core-agnostic by design**: The library does **not** spawn threads or pick a core. The application calls `update()` at least every 1ms from wherever its architecture dictates. Do not add `_thread` to anything under `driver/` — that inversion is the whole point of ADR-004.

3. **Non-blocking arming**: `arm()` opens the arming window and returns; `update()` completes it. The application polls `is_armed()`. This is what lets the library work in a cooperative single-core loop as well as on a dedicated core. With bidirectional motors, `ARMED` is evidence-gated, not a timer: the window is only a floor, and every bidirectional ESC must also have replied AM32's not-running frame (`gcr_decode.AM32_NOT_RUNNING_FRAME`, counted by `CaptureMailbox.drain()`) for `READY_SPAN_MS` (2s) with no gap over `READY_GAP_MS` (450ms, a reboot restarts the count). An ESC that never replies keeps the group `ARMING`, sending zeros, until the application gives up (`arming_status()` names the motor). This is BUG-002's fix: an ESC that reset during a timer-only window came back after `ARMED` under throttle and never armed (see BUG-003). `wait_for_replies = False` turns the gate off for bench tests with no ESC attached only.

4. **`update()` is inert while disarmed**: a safety requirement, not an optimisation. Writing to deactivated state machines would fill the TX FIFO and block the calling core forever.

5. **`disarm()` transmits nothing and returns every motor to a fresh, never-run state** (decided 2026-10-13). It deactivates each motor's state machines, re-initialises them (TX FIFO empty, no queued frame survives to the next `start()`; a bidirectional motor also flushes its RX FIFO and stored capture), and leaves the pin a plain output **driven low**. The ESC's own signal-loss timeout then stops the motor and resets it — AM32's is 500ms armed, 2s disarmed (see specification/AM32.md#15) — so a motor keeps its last throttle for up to 500ms after `disarm()`. An application that needs a faster stop sets throttle 0 and lets `update()` send it before calling `disarm()`; the library does not stream zeros on the way out. `disarm()` pays one fixed ~300us wait per bidirectional motor so an in-flight ESC reply is not cut off by the pin hand-off. Its `self.state` check keeps a concurrent `update()` from corrupting the group's own state, but nothing serialises its state-machine calls against an `update()` still running on another core — stop the loop first (see the usage example). Driving the line low rather than releasing it matters: a released, floating-high line leaves some ESC firmware (AM32 included) unable to time out of its own post-disarm reboot cleanly. `start()` re-initialises again and reclaims the pin for PIO. **Convention for bench code:** any script that spins motors ends its session with `fast_stop(group, runner)` from `tests/harness/core1_runner.py` (throttle 0 while the loop runs, then stop the loop, then `disarm()`), and treats a `False` result as a failed run. Scripts driving a single motor without `MotorGroup` send their own zero frames before `stop()` (see `tests/device/test_frame_arming_echo.py`); scripts that never command a throttle (`tools/measure_arming_tick_timing_once.py`, `tests/device/test_pio_lifecycle.py`) need neither.

6. **Lock-free design**: Shared throttle array allows one core to update values while another sends commands. See ADR-001 for technical details on atomic writes.

**`tests/`** has three parts, each with its own way of running:

- **`tests/harness/`** - the bench regression suite: example application code plus the JSON scenarios that verify the library on real hardware. `run_scenario.py` runs a scenario the way an application uses the library: it builds the motors (bidirectional ones use the frame receiver, the only one `BidirectionalDShot` has; since it fills its own PIO block, `scenario.py` rejects a scenario JSON that puts a unidirectional motor on the same block), wraps them in `MotorGroup`, runs `update()` on Core 1 through `core1_runner.py` (a Core 1 loop projects copy and adapt), and on Core 0 follows each motor's throttle profile and samples `raw_telemetry()`. Every 20th new capture is decoded on the device (`decode_tally.py` tallies real replies, CRC failures and non-replies) and judged against the scenario's `expect` thresholds, so a run prints its own verdict; the raw captures go to the PicoBell Adalogger's SD card (`bidir_capture_sink.py`, `capture_sink.py`, the ported `sdcard.py`/`pcf8523.py`). `scenario.py`/`throttle_profile.py` load and validate a scenario fail-fast before anything is armed, and `scenarios/*.json` is the scenario library (each file declares all 4 motors' wiring and a throttle profile per motor). Run one with `python scripts/run_test.py --scenario tests/harness/scenarios/<file>.json` (uploads via `scripts/deploy.py`, then runs and streams output), pull the session with `scripts/pull_captures.py` (which deletes each pulled session from the SD card) and analyse it with `scripts/analyze_bidir_capture_log.py`, which decodes everything logged, checks the same thresholds and cross-checks the device's sampled tally. `scripts/arming_stats.py` reads the arming logs of a batch of sessions and prints, per configuration, how often each ESC accepted our frames. `scripts/run_scenarios.py` runs one or more scenarios repeatedly (interleaved round by round when given several), tallying each attempt as armed/refused/error from `run_test.py`'s own printed output rather than a hand-rolled shell loop's string-matching - see its own docstring for why that distinction matters.
- **`tests/unit/`** - PC-run unit tests of the pieces that need no hardware (mailbox, decoder, command packet, `MotorGroup` against fake state machines, scenario loading): `python -m unittest discover -s tests/unit`. `fakes.py` stands in for `machine`, `rp2` and `utime`.
- **`tests/device/`** - on-Pico checks that don't fit the scenario harness. `test_capture_slot_stress.py` needs the Pico's two real cores but no ESC or motor power; `test_stop_clears_queues.py` and `test_pio_lifecycle.py` need the Pico only (the first proves on real PIO that `stop()` leaves the send queue empty and that repeated start/stop does not exhaust the program store). `test_bidir_restart_cycles.py` needs an ESC and motors, to arm/spin/disarm a bidirectional motor three times in one session and prove `start()` reclaims the pin correctly after `stop()`, which no scenario run does (`run_test.py` resets the Pico before every scenario, so a scenario only ever calls `start()` once). `test_frame_stalled_drain.py` and `test_frame_arming_echo.py` also need an ESC and a motor and probe the frame receiver under a stalled drain and during arming. Run any of them with `python scripts/run_test.py <file>.py` (the bare file name, not a path).

`tools/` holds on-Pico scripts that aren't tests - no pass/fail assertion, just data collection for a person to read (`measure_arming_tick_timing_once.py`, `measure_rough_start.py`). Also run with `python scripts/run_test.py <file>.py`.

`scripts/deploy.py` only uploads the driver/harness files and resets the board - it runs nothing on the Pico. `scripts/run_test.py` is the one that spins motors: it calls `deploy.py`'s upload+reset, then runs the chosen script via `mpremote run` and streams its output live. `scripts/run_scenarios.py` wraps `run_test.py` for a batch of runs (`/run-scenarios`, mirroring `/deploy`) rather than a one-off shell loop. All three are on this project's Claude Code permission allow-list (`.claude/settings.local.json`), since none is interactive and `run_test.py`/`run_scenarios.py` are the ones that touch hardware.

### DShot Protocol

See `specification/DSHOT.md` for the wire protocol itself (packet structure, bit timing for all
variants, special commands 0-47, bidirectional DShot and eRPM telemetry, CRC), `specification/AM32.md` for how this project's target ESC firmware implements it, and `specification/BETAFLIGHT.md` for how a reference flight-controller host drives it.

## Usage Example

**Recommended (multi-motor with reliable timing):**
```python
from machine import Pin
from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner  # application code, see tests/
import utime

motors = MotorGroup([
    UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600),
    UnidirectionalDShot(1, Pin(5), DSHOT_SPEEDS.DSHOT600),
])

# The application picks the core - here, a dedicated Core 1 loop. A real
# interval_us (unlike UPDATE_INTERVAL_US=0 here, which means "no fixed
# pacing") is an absolute period, held by anchoring each deadline to the
# previous one rather than sleeping a pad after the call - immune to
# update()'s own cost changing, matching how a flight controller's own
# fixed-period scheduler paces DShot output (verified against Betaflight's
# source - see Core1Runner's own docstring for the exact citation).
# There used to be a second, pad-based mode, selected by a fixed_schedule
# flag; removed 2026-10-09 (see Core1Runner's own docstring) because it only
# ever worked by retuning a magic constant to one specific measurement.
runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)
runner.start()

motors.arm()                    # non-blocking
while not motors.is_armed():
    utime.sleep_ms(10)

motors.set_throttle(0, 100)     # Motor 0
motors.set_throttle(1, 150)     # Motor 1

if not runner.stop():           # stop the loop first, and check it really exited
    print("Core 1 still in update():", runner.error)
motors.disarm()                 # cuts the signal and resets; the ESC stops the motor itself
```

**Low-level (single motor):**
```python
from machine import Pin
from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS

motor = UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600)
motor.start()  # Activate PIO state machine
# ... arm first (send_throttle_command(0) for 2000ms, see README's Quick Start) ...
motor.send_throttle_command(100)  # Must call continuously at ~1ms intervals
motor.stop()   # Deactivate
```
