---
paths:
  - "tests/**"
  - "scripts/**"
---

# Test folders and the bench harness

What each test folder is for, and the design rules of the bench harness. How each script works, the scenario format, the session folder contents and the observer's tools: `tests/README.md` (the mechanics: its "Bench rules and mechanics").

## Test folders

`tests/` is split first by whether it needs hardware, then by what it checks. Each folder has one rule. A file that breaks its folder's rule goes elsewhere, not into a catch-all.

- **`tests/pc/`**: runs on a PC under plain Python with `fakes.py` (`python -m unittest discover -s tests/pc`). Nothing that touches the Pico.
- **`tests/bench/dshot_protocol/`**: does the DShot link and the arming/telemetry behaviour hold on this ESC across motor counts and configurations? The scenario runner, its JSON scenarios, the single-motor ESC probe. Not: physical RPM, code another group also needs, anything without an ESC.
- **`tests/bench/pio_and_cores/`**: how the driver uses the RP2350's PIO blocks and its two cores. Pico only, no ESC or motor power. Not: anything that arms an ESC.
- **`tests/bench/wingman/`**: the second Pico (the wingman) and everything it relies on, grouped by the machine the code runs on. The **observer** records every edge of the ESC lines; the **ESC mock** plays recorded ESC replies with the ESC disconnected. Equipment, not a test group, and a permanent part of the bench. Not: anything about a real ESC, a motor or arming; a mock session is never evidence.
- **Wingman design principle (owner, 2026-11-01):** the wingman takes everything it needs to know about the main Pico (pins, which lines are bidirectional, DSHOT speed, run length) from the scenario the main Pico runs, read in one place (`tests/bench/wingman/tools/wingman_plan.py`). No pin, speed or fixture is written into the wingman's own code.

## Harness rules

- **One run is one session**, named by the PC (`scripts/run_session.py`, or the batch runner `scripts/run_scenarios.py`).
- **The main Pico reports facts, the PC judges** (owner, 2026-11-11; ADR-008). The main Pico reports what happened and may bail out of a run early. A bail-out is reported as "aborted" with its reason, never as a verdict, and its console says completed or aborted, never pass or fail. The verdict of every session is made on the PC only (`scripts/session_summary.py`), from the wire recording. This is ADR-004's "the library reports facts" one level up: the library reports to the application, the main Pico reports to the PC.
- **The wingman is a permanent part of the bench** (owner, 2026-11-11). Every bench run with an ESC is recorded by it, and there is no way to run a scenario without it. A device script on the Pico alone, with no ESC, is recorded only when asked. A session whose recording is missing has not been judged (verdict "unknown").
- **While the main Pico controls motors its I/O is the bare minimum (ESC I/O excepted); the observer records every wire edge** (owner, 2026-10-31). A threshold with too few replies on the wire is reported as not judged, never failed.
- **A bench test that runs motors uses the scenario runner's mechanics**, so every result compares with the same baseline. A test that must differ says so in its header.
- **Bench code stops motors with `fast_stop`.** Any script that spins motors ends with `fast_stop(group, runner)` and treats a `False` result as a failed run (detail: `tests/README.md`).
- **Shared test code sits directly in `tests/bench/`**; test groups never import another group's files. File names are unique across the repository, because `deploy.py` flattens them onto the Pico.
