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
| **Smooth start** | In progress | KPI re-scoped 2026-11-02 (owner): **all four motors spin up with Gemfan 2218 props**, measured by the share of runs in which all four spin up and hold and by the spin-up duration; RPM disks and heavier or draggier props are paused (channel 4 did not spin up with the disks on either of the owner's AM32 ESCs). Definition and baseline: BUG-009 ("Current KPI", "Result 3") |

## Operational Envelope (what this driver guarantees)

| Guarantee | Value |
|-----------|-------|
| Arming success rate | **≥ 99%** of `arm()` attempts reach `ARMED` |
| Arming duration | **≤ 3.0 s** from `arm()` to `ARMED`, for every configured motor |
| Throttle range | `0` (stop) or `48`-`250`. `1`-`47` are DShot commands (BUG-005) and above `250` is a **temporary bench ceiling** (owner, 2026-10-19: nothing higher is tested yet); `MotorGroup.set_throttle()` and `send_throttle_command()` reject both. Raise `THROTTLE_CEILING` in `driver/dshot_pio.py` (mirrored in `tests/bench/throttle_profile.py`) when higher throttles are tested |
| ESC firmware | AM32 only |
| AM32 input match (what our frames look like to AM32) | Measured on the wire with no ESC (2026-10-28): AM32's own input threshold would read 0 of 74,752 bits per run differently from ours, the worst frame length is 2.7% off the window AM32 learns (limit 6.25%), no wrong CRC, largest gap between frames 1.1 ms (limit 500 ms). With one ESC attached (single motor, throttle 100) the input figures are the same and every frame is answered outside the ESC's arming silence. Not covered: other motors/channels, several ESCs, other throttles, cold ESC power-on. Figures, method and tools: `specification/BENCH_EVIDENCE.md`. |
| Start quality | **Pending - not yet guaranteed.** Scope (owner, 2026-11-02): all four motors, Gemfan 2218 3-Blade props, no RPM disks, no heavier or draggier props (the RPM-disk and reflective-sensor tests were removed 2026-11-02, standing backlog item W41). Target **≥ 99%** of runs (armed, then a jump from `0` to throttle 215 on all four motors, held for the run) that **succeed: all four motors spin up and hold, and the slowest spin-up is within 0.5 s** (a drone with fewer than four motors turning does not fly, so one motor short is a failed run). A motor spins up when it reaches 80% of its own settled speed and the ESC never cuts it; the spin-up duration is the time from the first nonzero throttle frame to that speed, limit **0.5 s** (owner, 2026-11-02, provisional: set from the 28-run baseline). About 300 clean runs would support 99% (exact one-sided 95% bound, the arming evidence's convention); the check also compares each motor's settled speed with the other three, so a motor stuck at a low reading does not pass, counts runs that never reached the step apart, and covers only the ESC's own eRPM over back-to-back warm restarts (`scripts/start_kpi.py`). The four motors start together, so a run, not a motor-start, is one trial. Both are in each session's `summary.json` (`start`, per-line `spin_up_ms`), which also says whether the session counts toward the KPI; `scripts/start_kpi.py` holds the definition and prints the totals (the batch runner prints them at the end of every batch). Baseline so far, 2026-11-01, 28 runs: all four spun up and held in 28 of 28, but 26 of 28 runs fully succeed (93%, exact one-sided 95% lower bound 79%), because two GPIO 6 spin-ups took 0.9 and 1.05 s; per-motor medians 0.09-0.16 s; BUG-009 "Result 3". Second batch 2026-11-02, 32 runs: 30 of 32 fully succeed, both failures a slow GPIO 6 spin-up; the cause is not found (BUG-009 "Result 4" to "Result 6") |

An attempt that does not reach `ARMED` within 3.0 s counts as a failure, including one that
arms later after an ESC reboot. Callers should treat it as a failure to handle, not wait
indefinitely.

**Evidence.** The 3.0 s bound sits in the empty gap between clean arms (2.08-2.59 s) and arms that needed an ESC reboot
(4.06 s or more) across 698 archived sessions; that archive justifies the bound, not the rate. The 99% rate is **not yet
supported**: 104 of 104 passing runs support about 97%, and about 300 clean runs on the current code are needed (the
long batch waits for the owner to schedule it; bidirectional scenario runs count). Scope of the earlier evidence: one
scenario, one bench, an already-powered ESC, DSHOT600, two bidirectional motors. Full record:
`specification/BENCH_EVIDENCE.md`.

The bound is measured by `scripts/run_scenarios.py` (`CLEAN_ARM_THRESHOLD_MS`); keep that constant,
README.md and this table in agreement. The batch runner also flags arms within 500 ms of the bound as
"marginal"; that is a reporting aid, not part of the promise and not a failure. Not guaranteed:
DShot commands, other ESC firmware, behaviour of the motor after arming (BUG-009 is open).

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

**Pico-only runs need no permission** (owner, 2026-10-20). Code that relies on the Pico 2 but uses no real ESC and no motor - a test on unused GPIO with nothing attached, a PIO/DMA/timing experiment, a driver or harness change that can be exercised without an ESC - is written, deployed and run on the Pico without asking the owner first. Anything that can put a signal on an ESC's input or spin a motor (GPIO 6-9 with an ESC attached, any throttle above 0) still needs the owner's go. Say in the report which pins a run used.

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
comparing two batches, check `git log --date=iso-local -- driver tests/bench tests/harness tests/device` for every commit
between them, read what each one actually changed, and check whether it's mechanistically capable
of causing the observed shift - not just correlated in time with it. A real example from BUG-002's
2026-09-30 investigation: a 09-27 evening jump in first-contact acceptance lined up with a commit
timestamped minutes *after* the jump, because the change had been bench-verified first and
committed once confirmed.

## No hardware speculation

Do not offer physical causes for a bench result - wiring, connectors, crosstalk between signal wires,
supply sag or inrush, grounds, solder joints, mechanical coupling through the lever, a "bad" ESC or
motor - unless a measurement in this repo supports it. The Pico sees only DShot frames and replies;
it cannot see the power or the wiring, so a hardware cause cannot be inferred from its
data. This rule is stricter than the section above, which covers environmental explanations of a shift
between batches.

- Report what the data shows (counts, timings, which motor on which channel and when) and what has been
  ruled out by a test, in those words.
- When asked "why", propose only hypotheses the repo can test in software (order of motors in a
  group, simultaneous against staggered starts, throttle, polling pattern, PIO block assignment), and
  say which result would confirm or reject each.
- Hardware actions (inspect, swap, measure the supply) are the user's to choose. List them as options
  without naming a suspected cause. The user knows the bench and holds the hardware facts; ask for
  them instead of guessing.
- A quantity taken from AM32's source (a timeout, a limit) may be quoted as a source fact. Saying that
  it explains a bench result is a hypothesis and is marked as one.

## Permanent scenarios need the owner's permission

A scenario file that is checked into git (everything under `tests/bench/dshot_protocol/scenarios/`, the baseline pair included) is a permanent part of
the project. **Do not add, rename, edit or delete one without the owner's explicit go** (owner, 2026-11-02: the start-quality configs had grown into
many files that differed in every aspect, and nobody could say any more what was being tested). Say which question the file answers when asking.

**Temporary scenarios are free.** For checking a hypothesis, a bug or a fix, write the scenario to a temporary folder (the session scratchpad or the OS
temp folder), run it from there, and never commit it. Nothing is lost: every run keeps its own copy as `scenario.json` in its session folder. If a
temporary scenario turns out worth keeping, ask first; do not move it into the repo on your own.

## Architecture

### Core Components

**`driver/dshot_pio.py`** - Low-level PIO driver:

1. **PIO Assembly (`dshot` function)**: State machine program generating DShot waveforms. Each bit takes 8 clock cycles with specific high/low timing to encode 0s and 1s.

2. **`DSHOT_SPEEDS` class**: Protocol variant constants, restricted to DSHOT300/600 — the only speeds AM32 documents support for (its README and wiki.am32.ca; DSHOT150 and DSHOT1200 are deliberately not offered, see the class's own comment). Values are clock frequencies: `bit_rate * 8_cycles_per_bit`.

3. **`DShotPIO` base class, with `UnidirectionalDShot` and `BidirectionalDShot` subclasses**: Main driver. Creates a PIO state machine on the specified pin (inactive until `start()`), provides `send_throttle_command(throttle)` to send 16-bit packets (11-bit throttle + 1-bit telemetry + 4-bit CRC), and `stop()` to deactivate. `BidirectionalDShot` additionally owns the RX state machine that captures the ESC's telemetry reply, and a `CaptureMailbox` (`driver/capture_mailbox.py`, pure Python) that assembles replies from the RX FIFO and holds the latest one for another core to read. The receiver is `dshot_bidir_rx_frame` (see ADR-002's "run-length capture" section): it reconstructs the frame in the state machine itself and hands the CPU one already-decoded word per reply, for `gcr_decode.analyze_frame()`. This TX/RX pair fills its PIO block on its own (adopted as the sole production receiver 2026-09-26, after bench validation — see the backlog) — it cannot share one with a unidirectional motor, though two bidirectional motors may share one block (identical programs load once per block). The older sample receiver, `dshot_bidir_rx` (128 raw samples per reply, reconstructed on the CPU by `gcr_decode.analyze_capture()`), backed a standalone calibration/diagnostic tool (`tools/calibrate_bidir_rx.py`) that measured a new ESC unit's `BIDIR_PROFILES` entry, since the frame receiver has no bit-period search of its own for that. Both were removed 2026-10-11: the tool never worked (it captured TX's own echo rather than a real ESC reply) and the project settled on one ESC, so the use case never came up. The raw-sample reconstruction math it depended on (`find_edges`, `reconstruct_frame`, `sample_cycle`) moved to `scripts/simulate_frame_receiver.py`, which still needs it to verify `dshot_bidir_rx_frame`'s own PIO logic against real captures.

**`driver/motor_group.py`** - Multi-motor facade (see ADR-004):

1. **`MotorGroup` class**: Takes 1-4 already-built `UnidirectionalDShot`/`BidirectionalDShot` motors (the application picks their state machines and pins) and owns the throttle values and lifecycle for the group. Provides `arm()` (only valid while disarmed), `disarm()`, `update()`, `is_armed()`, `set_throttle()`, `set_all_throttles()`, `raw_telemetry(index)`, `decode_telemetry(index, value)` and `arming_status()`. It rejects motors that share a state machine or a pin. `update()` also drains every bidirectional motor's RX FIFO into that motor's one-slot latest capture (`discard_rx()` while arming, when replies are not yet trusted, `drain_rx()` once armed); `raw_telemetry()` hands the latest capture out only once armed and holds no data itself: the capture is one integer, the 21-bit reply frame with a 9-bit token above it that steps on every publish (1..511, then 1 again), so a caller keeps the last value it saw and knows a capture is new when the value differs; there is no timestamp, and a read is one load that is never refused (ADR-005's 2026-10-17 update). Decoding (`decode_telemetry()`) and retry-on-CRC-failure are the application's business, on its own schedule. The harness no longer logs the arming phase or wraps `update()` with per-tick measurement (retired 2026-10-06, BUG-002: real overhead - ~90us/call even idle - on the exact loop whose timing the investigation cares about; see BUG-002.md's "Telemetry overhead and the timed window" section).

2. **Core-agnostic by design**: The library does **not** spawn threads or pick a core. The application calls `update()` every `UPDATE_INTERVAL_US` (1045us), at a steady period, from wherever its architecture dictates. **There is no unpaced mode** (`update()` back to back, as fast as it returns): it was the default until 2026-10-19 and was removed, and `Core1Runner` rejects an interval of 0. Do not reintroduce one, and do not add a per-test or per-scenario period. Do not add `_thread` to anything under `driver/` — that inversion is the whole point of ADR-004.

3. **Non-blocking arming**: `arm()` opens the arming window and returns; `update()` completes it. The application polls `is_armed()`. This is what lets the library work in a cooperative single-core loop as well as on a dedicated core. With bidirectional motors, `ARMED` is evidence-gated, not a timer: the window is only a floor, and every bidirectional ESC must also have replied AM32's not-running frame (`gcr_decode.AM32_NOT_RUNNING_FRAME`, counted by `CaptureMailbox.drain()`) for `READY_SPAN_MS` (2s) with no gap over `READY_GAP_MS` (450ms, a reboot restarts the count). An ESC that never replies keeps the group `ARMING`, sending zeros, until the application gives up (`arming_status()` names the motor). This is BUG-002's fix: an ESC that reset during a timer-only window came back after `ARMED` under throttle and never armed (see BUG-003). `wait_for_replies = False` turns the gate off for bench tests with no ESC attached only.

4. **`update()` is inert while disarmed**: a safety requirement, not an optimisation. Writing to deactivated state machines would fill the TX FIFO and block the calling core forever.

5. **`disarm()` transmits nothing and returns every motor to a fresh, never-run state** (decided 2026-10-13). It deactivates each motor's state machines, re-initialises them (TX FIFO empty, no queued frame survives to the next `start()`; a bidirectional motor also flushes its RX FIFO and stored capture), and leaves the pin a plain output **driven low**. The ESC's own signal-loss timeout then stops the motor and resets it — AM32's is 500ms armed, 2s disarmed (see specification/AM32.md#15) — so a motor keeps its last throttle for up to 500ms after `disarm()`. An application that needs a faster stop sets throttle 0 and lets `update()` send it before calling `disarm()`; the library does not stream zeros on the way out. `disarm()` pays one fixed ~300us wait per bidirectional motor so an in-flight ESC reply is not cut off by the pin hand-off. Its `self.state` check keeps a concurrent `update()` from corrupting the group's own state, but nothing serialises its state-machine calls against an `update()` still running on another core — stop the loop first (see the usage example). Driving the line low rather than releasing it matters: a released, floating-high line leaves some ESC firmware (AM32 included) unable to time out of its own post-disarm reboot cleanly. `start()` re-initialises again and reclaims the pin for PIO. **Convention for bench code:** any script that spins motors ends its session with `fast_stop(group, runner)` from `tests/bench/core1_runner.py` (throttle 0 while the loop runs, then stop the loop, then `disarm()`), and treats a `False` result as a failed run. Scripts driving a single motor without `MotorGroup` send their own zero frames before `stop()` (see `tests/bench/dshot_protocol/test_frame_arming_echo.py`); scripts that never command a throttle (`tests/bench/pio_and_cores/test_pio_lifecycle.py`) need neither.

6. **Lock-free design**: Shared throttle array allows one core to update values while another sends commands. See ADR-001 for technical details on atomic writes.

**`tests/`** is split first by whether it needs hardware, then by what it checks. Each folder has one rule; a file that
breaks its folder's rule goes elsewhere, not into a catch-all. How each script works, the scenario format, the session
folder contents and the spy's tools are in `tests/README.md`.

- **`tests/pc/`** - runs on a PC under plain Python with `fakes.py` (`python -m unittest discover -s tests/pc`). Nothing that touches the Pico.
- **`tests/bench/dshot_protocol/`** - does the DShot link and the arming/telemetry behaviour hold on this ESC across motor counts and configurations? The scenario runner, its JSON scenarios, the single-motor ESC probes. Not: physical RPM, code another group also needs, anything without an ESC.
- **`tests/bench/pio_and_cores/`** - how the driver uses the RP2350's PIO blocks and its two cores. Pico only, no ESC or motor power. Not: anything that arms an ESC.
- **`tests/bench/wingman/`** - the second Pico (the wingman) and everything it relies on, grouped by the machine the code runs on: the **spy** records every edge of the ESC lines, the **ESC mock** answers our frames with recorded ESC replies so the reply path runs with no ESC and no motor (`run_session.py --esc-mock`, the ESC disconnected). **Design principle (owner, 2026-11-01): the wingman takes everything it needs to know about the main Pico (pins, which lines are bidirectional, DSHOT speed, run length) from the scenario the main Pico runs, read in one place (`tools/wingman_plan.py`); no pin, speed or fixture is written into the wingman's own code.** Equipment, not a test group. Not: anything about a real ESC, a motor or arming; a mock session is never evidence.

**One run is one session.** `scripts/run_session.py` (and the batch runner `scripts/run_scenarios.py`) names the session on the PC, records what was run in `session.json` (scenario, git commit, and the required `--load`: what the motors carry, `none` for bare shafts), has the **wingman** (a second board, found by ID, that records every wire edge) record the lines, pulls the Pico's SD record, and computes `summary.json`. **Owner's rule (2026-10-31): while the main Pico controls motors its I/O is the bare minimum (ESC I/O excepted); the spy records every wire edge.** A threshold that has too few samples to be judged is reported as not judged, never failed, and the spy's recording judges the line instead.

**Same mechanics as the scenario runner.** A bench test that runs motors uses the scenario runner's mechanics (`gc.collect()` before arming, `Core1Runner` at `group.UPDATE_INTERVAL_US`, the one paced loop, throttle set right after `ARMED`, Core 0 reading every reply every `poll_ms` and decoding every `decode_every`-th), so every result compares with the same baseline. A test that must differ (for example, no SD logging) says so in its header.

**Bench guards.** The scenario runner stops a run on two decoded-telemetry failsafes, both in `decode_tally.py`: `SpinWatch` (a motor commanded to spin that reports no real eRPM for 2 s) and `OverspeedWatch` (a motor that reports more than 300 eRPM per unit of its recently commanded throttle, 5 decodes in a row; throttle 60 reads about 8,400 eRPM, so its limit is 18,000). Both raise, so `fast_stop` zeroes the throttle. **The over-speed guard is switched off (owner, 2026-10-20)** by `OVERSPEED_GUARD_ENABLED = False` in `decode_tally.py` while the plateau reading on some channels is investigated; `run_scenario.py` prints "Over-speed guard: DISABLED" at the start of every run. Set it to `True` to bring it back; the no-spin failsafe and the throttle ceiling stay on. A guard only sees every `decode_every`-th reply, so a short disturbance can pass it: at one reply per 1045us period, 5 decodes 20 replies apart span about 100ms (arithmetic from the period, not measured).

Code shared by the bench test groups sits directly in `tests/bench/`: `scenario.py` and `throttle_profile.py` (load and validate a scenario's wiring and throttle curves), `decode_tally.py` (classifies decoded replies; holds the minimum-sample rule), `core1_runner.py` (the Core 1 `update()` loop and `fast_stop`, which every motor-spinning test imports; applications copy and adapt it too) and `session_recorder/` (the SD-card session recorder: `sdcard.py`, `capture_sink.py`, `bidir_capture_sink.py`, used by the scenario runner). It is not a test group: nothing in it has a verdict. Test groups may import this shared code and the wingman's `main_pico/start_line.py` (the start line the scenario runner drives), but never another group's files (a test that needs something from `dshot_protocol/` means that something belongs here instead). `deploy.py` flattens every file onto the Pico root and `run_test.py` finds a script by bare name, so keep file names unique across all of these folders.

`scripts/deploy.py` only uploads the driver/harness files and resets the board - it runs nothing on the Pico. `scripts/run_test.py` is the one that spins motors (it requires `--session <name>`: the PC names every session, `scripts/run_session.py` makes the name and calls it, and the batch runners make one per attempt): it calls `deploy.py`'s upload+reset, then runs the chosen script via `mpremote run` and streams its output live. `scripts/run_scenarios.py` wraps `run_test.py` for a batch of runs (`/run-scenarios`, mirroring `/deploy`) rather than a one-off shell loop. All three are on this project's Claude Code permission allow-list (`.claude/settings.local.json`), since none is interactive and `run_test.py`/`run_scenarios.py` are the ones that touch hardware.

### DShot Protocol

See `specification/DSHOT.md` for the wire protocol itself (packet structure, bit timing for all
variants, special commands 0-47, bidirectional DShot and eRPM telemetry, CRC), `specification/AM32.md` for how this project's target ESC firmware implements it, and `specification/BETAFLIGHT.md` for how a reference flight-controller host drives it.

## Usage

Code examples live in README.md (Quick Start: single motor, several motors, without a second core), not here. This file holds conventions and methodology.
