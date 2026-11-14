# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DShot driver for Raspberry Pi Pico, part of a flight control systems test bench. This repository focuses on the DShot protocol implementation. The bench itself (boards, ESCs, motors, props, wiring, ESC settings) is described in the README, Hardware section.

Original implementation from https://github.com/jrddupont/DShotPIO (GNU GPL v3.0 license).

**Supported ESC targets (design constraint):** a pet/exploration project; it does not aim to support the endless universe of ESCs. **AM32** is the sole firmware family in scope. It is open source (https://github.com/am32-firmware/AM32), so behavior is verified against its firmware source rather than guessed from generic protocol articles: when a generic spec and AM32's source disagree, the source wins. Do not add abstraction layers or configuration surface for other ESC families. **BLHeli_S** was an earlier bench target (its timing data is in ADR-001/ADR-004) and is no longer supported: add no code paths or timing accommodations for it.

**Goals, status and guarantees** live in the README: "Roadmap" (every goal and its status) and "Operational Envelope" (what the driver guarantees, what it does not, and the evidence). The arming bound is the constant `CLEAN_ARM_THRESHOLD_MS` in `scripts/arming_bound.py`; keep that constant and the README's Operational Envelope in agreement.

## Development Environment

- **Target Platform**: Raspberry Pi Pico 2 (RP2350)
- **Runtime**: MicroPython (not standard Python)
- **Dependencies**: Built-in `machine` and `rp2` modules (no external packages)

Deploy code to Pico via USB mass storage or tools like Thonny, rshell, or mpremote.

## Hardware permissions

- **Pico-only runs need no permission** (owner, 2026-10-20). Code that relies on the Pico 2 but uses no real ESC and no motor (a test on unused GPIO with nothing attached, a PIO/DMA/timing experiment, a driver or harness change that can be exercised without an ESC) is written, deployed and run on the Pico without asking the owner first. Say in the report which pins a run used.
- **Anything that can put a signal on an ESC's input or spin a motor** (any pin with an ESC attached, any throttle above 0) **needs the owner's go.** The ESC buddy's read-only commands are the exception: see its README.
- Running hardware: `scripts/deploy.py` only uploads; `scripts/run_test.py` and `scripts/run_scenarios.py` spin motors (all three are on the Claude Code allow-list).
- **ESC configuration goes through the `esc-buddy` skill** (use the skill; follow its rules). What the buddy does, how to run it and which commands need no permission: `tools/esc_buddy/README.md` and the skill. **Every write to an ESC needs the owner's explicit go, each time.**

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

## Ways of working

- **Work on a feature branch, never on master.** Create the branch before the first change; agreed work is committed there without asking (owner, 2026-11-10).
- **Merging to master is a squash merge and needs the owner's go; so does a push.** Run the PC tests on master after the merge and before the push.
- **Delete the feature branch once its squash merge is on master** (owner, 2026-11-12). No go is needed for the deletion; git history and the squash commit are the record.

## Analysing bench results

- **A shift between two batches: check for a code change first.** Do not name "thermal drift", "power sag", "day-to-day factor" or any other unnamed environmental cause without evidence. Committed timestamps lag deployed code (code is bench-tested before it is committed), so check `git log --date=iso-local -- driver tests/bench tools scripts` for every commit between the batches and whether its mechanism can cause the shift.
- **No hardware speculation.** Offer no physical cause for a bench result (wiring, connectors, crosstalk, supply, grounds, solder joints, the lever, a "bad" ESC or motor) unless a measurement in this repository supports it. The Pico cannot see the power or the wiring.
- Report what the data shows (counts, timings, which motor on which channel and when) and what a test has ruled out, in those words.
- When asked "why", propose only hypotheses the repository can test in software, and say which result would confirm or reject each.
- Hardware actions (inspect, swap, measure the supply) are the user's to choose: list them as options without naming a suspected cause, and ask the user for hardware facts instead of guessing.
- A quantity from AM32's source may be quoted as a source fact; saying it explains a bench result is a hypothesis and is marked as one.

The reasoning and a worked example: `.claude/rules/bench-analysis.md` (loaded when a bug report or decision record is touched).

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
- **A tool's README (`tools/*/README.md`) describes the capability** (owner, 2026-11-10): what the tool is for, which firmware or versions it supports and how that is checked, what it does and does not do, and its complete set of properties or options. It is not a logbook or backlog: a dated bench run and what it showed ("poles 14 -> 12 on GPIO 6-9") has a lifetime of about one session, and when it is evidence for a decision it goes to the ADR or bug that owns the claim; open items go to the ADR or bug they belong to.
- This file holds conventions and pointers (see "Usage"), not facts about the setup.

## Folder rules

Where things live is found with a file search. The rules each folder's code must keep are in path-scoped rule files, loaded when a matching file is read or edited:

- **All MicroPython code** (`driver/`, `tests/bench/`, `tools/esc_buddy/`): `.claude/rules/pico-code-style.md` (naming and style; no `_` prefix for visibility).
- **Driver code** (`driver/`): `.claude/rules/driver-code.md` (speeds, core-agnostic, one paced loop, where the design is recorded, the protocol specifications).
- **Comments and docstrings in `driver/` say purpose and contract, never implementation or history.** The rules are in `.claude/rules/driver-comments.md` (loaded when a driver file is touched); the `driver-comment-review` agent applies them.
- **Tests and the bench harness** (`tests/`, `scripts/`): `.claude/rules/bench-harness.md` (what each test folder is for, the wingman, how a bench run is judged, `fast_stop`).
- **ESC buddy** (`tools/esc_buddy/`): `.claude/rules/esc-buddy.md` (it serves only the firmware on the bench; no backward compatibility).
- **Bug reports and decision records**: `.claude/rules/bench-analysis.md` (the reasoning behind "Analysing bench results").

## Usage

Code examples live in README.md (Quick Start: motors through `MotorGroup`, with telemetry, without a second core), not here. This file holds conventions and methodology.
