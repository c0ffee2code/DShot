---
paths:
  - "driver/**/*.py"
---

# Rules the driver code must keep

How the driver behaves is in the decision records and the README. These are the rules to keep, each with a pointer to the record that says why.

1. **Speeds.** Only DSHOT300 and DSHOT600 are offered, the only speeds AM32 documents. Do not add DSHOT150 or DSHOT1200 (see the class's own comment).
2. **Core-agnostic.** The library does not spawn threads or pick a core, and nothing under `driver/` imports `_thread`. The application owns the command loop and calls `update()` at a steady period (ADR-004).
3. **One paced loop only.** There is no unpaced mode and no per-test or per-scenario period. It was removed on 2026-10-19; do not reintroduce it.

## Where the rest is

- The receiver and its PIO cost, and the reply mailbox: ADR-002, ADR-005.
- Arming, `update()` while disarmed, and `disarm()` and the line state: ADR-004, ADR-002 and BUG-002/BUG-003.
- Lock-free throttle values: ADR-001.
- The DShot wire protocol (packet structure, bit timing for all variants, special commands 0-47, bidirectional DShot and eRPM telemetry, CRC): `specification/DSHOT.md`. How AM32, the target ESC firmware, implements it: `specification/AM32.md`. How a reference flight-controller host drives it: `specification/BETAFLIGHT.md`.
- The bench ESC itself (AM32 2.22 since 2026-11-10, shipped with 2.16; older bootloader): `specification/KM55A2_DEFAULTS.md` (vendor settings as read). What the ESCs hold now: the README's "ESC settings as configured".
