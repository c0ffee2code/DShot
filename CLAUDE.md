# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DShot driver for Raspberry Pi Pico, part of a flight control systems test bench. This repository focuses on the DShot protocol implementation. The bench itself (boards, ESCs, motors, props, wiring, ESC settings) is described in the README, Hardware section.

Original implementation from https://github.com/jrddupont/DShotPIO (GNU GPL v3.0 license).

**Supported ESC targets (design constraint):** a pet/exploration project; it does not aim to support the endless universe of ESCs. **AM32** is the sole firmware family in scope. It is open source (https://github.com/am32-firmware/AM32), so behavior is verified against its firmware source rather than guessed from generic protocol articles: when a generic spec and AM32's source disagree, the source wins. Do not add abstraction layers or configuration surface for other ESC families. **BLHeli_S** was an earlier bench target (its timing data is in ADR-001/ADR-004) and is no longer supported: add no code paths or timing accommodations for it.

## Project Goals

| Goal | Status | Details |
|------|--------|---------|
| **Improve arming sequence** | Done | Evidence-gated arming on a 2000ms floor, not a timer alone: BUG-002, BUG-003; what pacing has been bench-confirmed: README "Verified Parameters" |
| **Invert core assignment to client** | Done | ADR-004: library exposes `update()`, application owns the loop |
| **DShot commands** | TODO | No driver-side command API yet; `MotorGroup` rejects 1-47 (BUG-005), so a command API needs its own entry point. AM32 implements commands: `specification/AM32.md`#7 |
| **Bidirectional DShot** | Done | ADR-002: frame receiver, bench-validated on AM32, the sole production receiver |
| **ESC configuration access** | Read and write done | ADR-006, `tools/esc_buddy/README.md` and the `esc-buddy` skill. Firmware/bootloader update: a separate session, not built |
| **Smooth start** | In progress | Definition, scope and baseline: BUG-009 ("Current KPI") |

## Operational Envelope (what this driver guarantees)

| Guarantee | Value |
|-----------|-------|
| Arming success rate | **≥ 99%** of `arm()` attempts reach `ARMED` |
| Arming duration | **≤ 3.0 s** from `arm()` to `ARMED`, for every configured motor |
| ESC firmware | AM32 only |
| Start quality | **Pending - not yet guaranteed.** Target **≥ 99%** of runs succeeding, spin-up within 0.5 s |

An attempt that does not reach `ARMED` within 3.0 s counts as a failure, including one that arms later after an ESC reboot. The 99% arming rate is **not yet supported** on the current code. The evidence and the measurements behind these figures are in the README's Operational Envelope.

The arming bound is measured by `scripts/run_scenarios.py` (`CLEAN_ARM_THRESHOLD_MS`); keep that constant, README.md and this table in agreement. Not guaranteed: DShot commands, other ESC firmware, motor behaviour after arming.

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

**Pico-only runs need no permission** (owner, 2026-10-20). Code that relies on the Pico 2 but uses no real ESC and no motor - a test on unused GPIO with nothing attached, a PIO/DMA/timing experiment, a driver or harness change that can be exercised without an ESC - is written, deployed and run on the Pico without asking the owner first. Anything that can put a signal on an ESC's input or spin a motor (any pin with an ESC attached, any throttle above 0) still needs the owner's go (the ESC buddy's read-only commands are the exception: see its README). Say in the report which pins a run used.

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
comparing two batches, check `git log --date=iso-local -- driver tests/bench tools scripts` for every commit
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

## Where documents live

- **`specification/`** holds specifications and the external references used in the work (protocols, firmware behaviour, the vendor's facts about a part). It describes what exists outside this project.
- **What we set up on the bench** - the boards, motors, props, wiring and the settings we configured on the ESCs - is test bench setup. It lives in the top-level README's Hardware section, never in `specification/` and never in this file.
- This file holds conventions and pointers (see "Usage"), not facts about the setup.

## Architecture

Where things live is found with a file search, so this section lists no layout: it holds the rules the code must keep, each with a pointer to the document that says why.

### Driver rules (`driver/`)

How the driver behaves is in the ADRs and the README; these are the rules to keep.

1. **Speeds.** Only DSHOT300 and DSHOT600 are offered (the only speeds AM32 documents); do not add DSHOT150 or DSHOT1200 (see the class's own comment).
2. **Core-agnostic.** The library does not spawn threads or pick a core, and nothing under `driver/` imports `_thread`: the application owns the command loop and calls `update()` at a steady period (ADR-004).
3. **One paced loop only.** There is no unpaced mode and no per-test or per-scenario period. It was removed on 2026-10-19; do not reintroduce it.
4. **Bench code stops motors with `fast_stop`.** Any script that spins motors ends with `fast_stop(group, runner)` and treats a `False` result as a failed run (detail: `tests/README.md`).

Where the rest is: the receiver and its PIO cost, and the reply mailbox: ADR-002, ADR-005. Arming, `update()` while disarmed, and `disarm()` and the line state: ADR-004, ADR-002 and BUG-002/BUG-003. Lock-free throttle values: ADR-001.

### ESC buddy (`tools/esc_buddy/`)

Access to the ESCs' AM32 configuration through the main Pico. Not a test group and not part of a motor run. What it does, how to run it and which commands need no permission: `tools/esc_buddy/README.md` and the `esc-buddy` skill (use the skill; follow its rules). **Every write to an ESC needs the owner's explicit go, each time.** Why it is built this way: ADR-006; the bench ESC and its settings: `specification/KM55A2_DEFAULTS.md` (vendor) and the README's "ESC settings as configured" (now).

### Tests (`tests/`)

`tests/` is split first by whether it needs hardware, then by what it checks. Each folder has one rule; a file that
breaks its folder's rule goes elsewhere, not into a catch-all. How each script works, the scenario format, the session
folder contents and the spy's tools are in `tests/README.md`.

- **`tests/pc/`** - runs on a PC under plain Python with `fakes.py` (`python -m unittest discover -s tests/pc`). Nothing that touches the Pico.
- **`tests/bench/dshot_protocol/`** - does the DShot link and the arming/telemetry behaviour hold on this ESC across motor counts and configurations? The scenario runner, its JSON scenarios, the single-motor ESC probes. Not: physical RPM, code another group also needs, anything without an ESC.
- **`tests/bench/pio_and_cores/`** - how the driver uses the RP2350's PIO blocks and its two cores. Pico only, no ESC or motor power. Not: anything that arms an ESC.
- **`tests/bench/wingman/`** - the second Pico (the wingman) and everything it relies on, grouped by the machine the code runs on: the **spy** records every edge of the ESC lines, the **ESC mock** plays recorded ESC replies with the ESC disconnected. Equipment, not a test group. Not: anything about a real ESC, a motor or arming; a mock session is never evidence. **Design principle (owner, 2026-11-01): the wingman takes everything it needs to know about the main Pico (pins, which lines are bidirectional, DSHOT speed, run length) from the scenario the main Pico runs, read in one place (`tests/bench/wingman/tools/wingman_plan.py`); no pin, speed or fixture is written into the wingman's own code.**

**Rules for bench runs** (the mechanics are in `tests/README.md`, "Bench rules and mechanics"):

- One run is one session, named by the PC (`scripts/run_session.py`, or the batch runner `scripts/run_scenarios.py`).
- **While the main Pico controls motors its I/O is the bare minimum (ESC I/O excepted); the spy records every wire edge** (owner, 2026-10-31). A threshold with too few samples is reported as not judged, never failed.
- A bench test that runs motors uses the scenario runner's mechanics, so every result compares with the same baseline; a test that must differ says so in its header.
- Shared test code sits directly in `tests/bench/`; test groups never import another group's files. File names are unique across the repository, because `deploy.py` flattens them onto the Pico.
- Running hardware: `scripts/deploy.py` only uploads; `scripts/run_test.py` and `scripts/run_scenarios.py` spin motors (all three are on the Claude Code allow-list).

### DShot Protocol

See `specification/DSHOT.md` for the wire protocol itself (packet structure, bit timing for all
variants, special commands 0-47, bidirectional DShot and eRPM telemetry, CRC), `specification/AM32.md` for how this project's target ESC firmware implements it, and `specification/BETAFLIGHT.md` for how a reference flight-controller host drives it. The bench ESC itself (AM32 2.16, older bootloader): `specification/KM55A2_DEFAULTS.md` (vendor settings as read); what the ESCs hold now is in the README's "ESC settings as configured".

## Usage

Code examples live in README.md (Quick Start: single motor, several motors, without a second core), not here. This file holds conventions and methodology.
