---
paths:
  - "driver/**/*.py"
---

# Rules the driver code must keep

How the driver behaves is in the decision records and the README. These are the rules to keep, each with a pointer to the record that says why.

1. **Speeds.** Only DSHOT300 and DSHOT600 are offered, the only speeds AM32 documents. Do not add DSHOT150 or DSHOT1200 (see the class's own comment).
2. **Core-agnostic.** The library does not spawn threads or pick a core, and nothing under `driver/` imports `_thread`. The application owns the command loop and calls `update()` at a steady period (ADR-004).
3. **One paced loop only.** There is no unpaced mode and no per-test or per-scenario period. It was removed on 2026-10-19; do not reintroduce it.
4. **Two layers, each named for and limited to what it does** (ADR-004, amendment of 2026-11-14). `MotorGroup` is the library's public API: arming, the throttle values and their valid range. `DShotPIO` is the low-level I/O core (PIO and pin): it sends the value it is given and checks none. No public method or state is named after stopping or disarming a motor (no `stop()`, `disarm()`, `DISARMED`): the library has no call that stops a motor, the application sends throttle 0.
5. **No Extended DShot Telemetry (EDT).** Out of scope: the driver does not enable or decode it (ADR-002, amendment of 2026-11-14). Do not add EDT frame handling to the decoder.

## Where the rest is

- The receiver and its PIO cost, and the reply mailbox: ADR-002, ADR-005.
- Arming, `update()` while inactive, and `deactivate()` and the line state: ADR-004, ADR-002 and BUG-002/BUG-003.
- Lock-free throttle values: ADR-001.
- Frame spacing: the bidirectional transmitter's wait for the ESC's reply (ADR-002, amendment 2026-11-14 (2)), and `update()` waiting until the transmitters have taken their previous frames, through `StateMachine.tx_fifo()` and not PIO registers (ADR-004, amendment 2026-11-14 (2)).
- The DShot wire protocol (packet structure, bit timing for all variants, special commands 0-47, bidirectional DShot and eRPM telemetry, CRC): `specification/DSHOT.md`. How AM32, the target ESC firmware, implements it: `specification/AM32.md`. How a reference flight-controller host drives it: `specification/BETAFLIGHT.md`.
- The bench ESC itself (AM32 2.22 since 2026-11-10, shipped with 2.16; older bootloader): `specification/KM55A2_DEFAULTS.md` (vendor settings as read). What the ESCs hold now: the README's "ESC settings as configured".
