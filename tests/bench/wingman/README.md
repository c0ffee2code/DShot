# Wingman

The wingman is the **second Pico 2** of the bench and a permanent part of it (decision/ADR-008). It flies alongside the main Pico
(the one that drives the motors) and plays two roles, which can run together on the same board:

| Role | What it does | Details |
|------|--------------|---------|
| **Observer** | Records every edge of the ESC signal lines and the start line and streams it to the PC. It only watches; it never drives a pin. | `OBSERVER.md` (including why its timing can be trusted) |
| **ESC mock** | Answers the main Pico's frames with recorded real ESC replies, so the reply path runs with no ESC and no motor. The ESC must be disconnected. | `ESC_MOCK.md` |

Folder rule: **everything the wingman relies on lives here, grouped by the machine the code runs on.** It is equipment for the
test groups, not a test group: the scenario runner imports `main_pico/start_line.py` the way it imports
`core1_runner.py`.

| Folder | Runs on | Holds |
|--------|---------|-------|
| `board/` | the wingman Pico | `observer.py`, `esc_mock.py`, `esc_mock_replies_dshot600.bin` + `.json` (the recorded replies and where they came from; one pair per DSHOT speed) |
| `shared/` | the wingman and the PC | `esc_mock_format.py` (pure modules read by more than one machine's code) |
| `main_pico/` | the main Pico (uploaded by `scripts/deploy.py`) | `start_line.py`: the start/end line, GPIO 15 |
| `tools/` | the PC | `wingman_board.py` (find the wingman by ID, upload, reset), `wingman_plan.py` (reads the scenario), `wingman_run.py` (the launcher), the `observer_*` tools, `esc_mock_fixture.py` |

The session-level scripts that use them stay in `scripts/` and reach them through `scripts/bench_paths.py` (one `import` that puts the wingman tools and `driver/` on the path; the unit tests do the same): `run_session.py` (starts the wingman for a session),
`session_summary.py` (the session verdict: the only judge of a run, from the observer's recording; the main Pico reports facts only), `session_timeline.py` (the observer's record and the main Pico's on one time axis).
The frame decoder both the observer tools and the main Pico's pin test read is `driver/dshot_frame_decode.py`.

## What the wingman is told, and by what

**Design principle: the wingman takes everything it needs to know about the main Pico from the scenario file
the main Pico runs. No pin, speed or fixture is written into the wingman's code.** `tools/wingman_plan.py` is the one place that reads it:

| The wingman needs | Comes from |
|---|---|
| the lines to observer on | `motors[].pin`, in list order |
| which lines carry a bidirectional motor (the ESC mock answers only on them) | `motors[].bidirectional` |
| the DSHOT speed (selects the ESC mock's recorded replies; the mock refuses a speed it has none for) | `dshot_speed` |
| how long to wait for the run | `duration_ms` plus a margin for the deploy and arming (`wingman_plan.RUN_MARGIN_S`) |
| the start line | `start_line.START_PIN` in `main_pico/start_line.py`, the pin the main Pico drives (imported, not copied) |

A scenario the wingman cannot serve (no motors, a duplicate pin, a motor on the start line, no or unknown `dshot_speed`, `--esc-mock`
without a bidirectional motor or at a speed with no recorded replies) stops before any hardware is touched, with a message that says why.

## Wiring

One convention, written once in `wingman_plan.py`: **the wingman's GPIO number is the main Pico's GPIO number, on every line, plus a shared
ground.** For the current bench that is:

| Line | GPIO | Driven by |
|------|------|-----------|
| ESC signal lines 1-4 | 6, 7, 8, 9 (the scenario's motor pins) | the main Pico's DShot output (and the ESC's replies, or the ESC mock's) |
| Start/end line | 15 (`START_PIN`) | the main Pico (`main_pico/start_line.py`): low first, high from just before arming until the motors stop |

The wingman is found by its unique ID (`5d703f56522b50fd`), never by COM number; the main Pico is `31b4d922f5d5f436`.
`tools/wingman_board.py` refuses any other board, so the wingman's tools can never touch the main Pico.

## How to run

- A scenario run: `python scripts/run_session.py <scenario.json> --load "..."`. A scenario runs with an ESC, so it is always recorded and
  cannot be run without the wingman: with the wingman not connected, or not recording, the session stops before the main Pico runs
  anything. A session whose `signal_observation.bin` did not arrive has no verdict (`unknown`, with a note naming why).
- A device script (Pico alone, unused pins, no ESC; `tests/bench/pio_and_cores/`, probes): `python scripts/run_session.py <script>.py --load "..."`
  has nothing for the wingman to record and runs without it. `--observe` records it anyway (needed for `--esc-mock`, with a config).
- With the ESC disconnected, the wingman also plays the ESC: add `--esc-mock`. The session is marked `esc_mock` and is never evidence
  about an ESC, a motor or arming.
- By hand: `python tests/bench/wingman/tools/wingman_run.py --scenario <file> [--session <name>] [--esc-mock]` (the scenario is required).
- Read a recording: `tools/observer_analyze.py`, `tools/observer_check_am32.py`, `tools/observer_am32_table.py`, `tools/observer_start_summary.py` on
  `captures/<session>/signal_observation.bin`.

## Names

| Word | Means |
|------|-------|
| wingman | the second Pico 2 |
| observer | its recording role (`signal_observation.bin` is a recording, `observer_*` the tools that read it) |
| ESC mock | its stand-in role (`esc_mock*`) |
| start line | GPIO 15, tells the observer when to record |
