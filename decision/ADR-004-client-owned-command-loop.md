# ADR-004: Client-Owned Command Loop

**Status:** Accepted
**Date:** 2026-08-12
**Supersedes in part:** [ADR-001](ADR-001-dual-core-motor-control.md) (core assignment and thread lifecycle only)

## Context

ADR-001 solved a real problem. Arming previously succeeded roughly half the time, with one motor left beeping, because Core 0 activity (display rendering, button polling, sleeps in the main loop) opened gaps in DShot transmission and the ESCs reset their arming counters. Dedicating Core 1 to a 1kHz command loop eliminated the failures completely, and that finding still holds.

But it placed the decision in the wrong layer. `MotorThrottleGroup` called `_thread.start_new_thread()` itself, so *the library* chose the threading topology of every application that imported it:

- An application that wants Core 1 for its own control algorithm, and the DShot loop on a timer IRQ, cannot have it.
- An application built around `uasyncio` gets a raw thread it did not ask for.
- A simple single-core sketch pays for a second core it does not need.
- Two libraries following this pattern in one project would both claim Core 1, and the second would fail.

The library is the wrong place to decide this, because the right answer differs per project. This repository is a DShot library; scheduling is not a DShot concern.

A second problem followed from the first: `arm()` blocked for 500ms in `utime.sleep_ms()`, which only works because *some other* context was transmitting. Any application driving the loop cooperatively from its own main loop would deadlock — it would be asleep inside the very call that needed it to keep pumping.

## Decision

**The library exposes `update()`. The application decides when and where it is called.**

`MotorThrottleGroup` remains a facade over the PIO state machines and throttle state. It knows *what* to transmit and *when it is due*; it does not know, and does not ask, which core it is running on.

```
┌─────────────────────────────────────────────────────────────┐
│                      APPLICATION                            │
│                                                             │
│  Owns the command loop and decides where it runs:           │
│    - a dedicated Core 1 thread (test bench does this)       │
│    - a machine.Timer IRQ                                    │
│    - its own cooperative main loop                          │
│                                                             │
│         group.update()   <- at least every 1ms              │
│         group.set_throttle(index, value)                    │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                  MOTOR GROUP FACADE (core-agnostic)         │
│                                                             │
│  arm()      - activate state machines, open arming window   │
│  disarm()   - transmit zeros, deactivate state machines     │
│  update()   - one frame per motor, advance arming           │
│  is_armed() - application polls for arming completion       │
│                                                             │
│        Shared throttle array (lock-free writes)             │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                     DRIVER (DShotPIO)                       │
│                                                             │
│  start() / stop() - PIO state machine activation            │
│  drain()          - wait for queued frames to go out        │
│  send_throttle_command(throttle) - encode and transmit      │
└─────────────────────────────────────────────────────────────┘
```

### No runner abstraction in the library

The library ships no `MotorRunner` interface for the application to implement. The shape of such an interface — callback versus poll, who owns the interval, restart semantics, how failures surface — is precisely what varies between projects, so defining it in `driver/` would re-create the same coupling one layer up.

`tests/harness/core1_runner.py` is a concrete Core 1 runner, and it is explicitly **application code**, kept next to the scripts that use it rather than in `driver/`. Projects copy and adapt it. If a genuinely common shape emerges across several projects, extract it then, from real usage rather than speculation.

### Sub-Decision: Non-blocking arming

`arm()` opens an arming window and returns immediately. `update()` advances it and flips the state to armed once the duration has elapsed. The application polls `is_armed()`.

This is what makes the library work under *any* scheduling arrangement, including the cooperative single-loop case that a blocking `arm()` could never support. It also lets the application do something useful while waiting — the test bench demo animates its ARMING screen, which the blocking version could not.

While arming, `update()` transmits literal zeros rather than the throttle array, so the arming window is genuinely at zero even if the application sets a throttle early.

Because the ESC resets its own arming counter when commands stop arriving, `update()` restarts the arming window if more than `ARM_GAP_TOLERANCE_MS` has passed since the previous call. The library's notion of "armed" then tracks what the ESC actually observed.

### Sub-Decision: `update()` is inert while disarmed

`update()` returns immediately unless the group is arming or armed.

This is a safety requirement, not an optimisation. `disarm()` deactivates the state machines; a loop that kept calling `update()` afterwards would fill each 4-word TX FIFO and then **block forever inside `StateMachine.put()`** — a silently hung core with no traceback, since Core 1 exceptions never reach the REPL.

Gating on state also makes call ordering unhazardous in general. The application may start its loop whenever convenient, and the loop only does real work between `arm()` and `disarm()`. This removes a latent race in the previous implementation, where Core 1 was launched *before* the state machines were activated and survived only because the heartbeat handshake won.

### Sub-Decision: The library reports facts, not health

`is_healthy()` is gone. It blocked for 5ms inside `utime.sleep_ms()`, and was called from within `arm()` — a hidden stall in a safety path. More fundamentally, a library that does not own the loop cannot judge whether that loop is healthy.

`update_age_ms()` reports milliseconds since the last transmission. The application sets its own threshold, because only the application knows what its scheduling arrangement should deliver. The arming state machine reuses the same timestamp, so there is one source of truth and no separate heartbeat counter to overflow.

### Sub-Decision: `arm()` / `disarm()` own the PIO lifecycle

The previous API had four overlapping methods — `start()`, `stop()`, `disarm()` and `emergencyStop()` — three of which ended at the same "write zeros". They collapse into one pair:

| Method | Does |
|---|---|
| `arm()` | zero throttles, `sm.active(1)`, open the arming window |
| `disarm()` | transmit zero-throttle frames, drain them, `sm.active(0)` |

`disarm()` *is* the emergency stop, and it is stronger than the old one: it both commands the stop **and** cuts the signal, whereas the old version relied on a loop still running to deliver its zeros.

Both halves are needed, and the order matters. Cutting the signal alone is not a stop — it only makes the ESC *eventually* time out, which on BLHeli_S is 100-250ms of a motor still spinning at its last commanded throttle. So `disarm()` transmits the zeros itself, waits for them to leave the shift register, and only then deactivates. That is a few hundred microseconds of blocking, in exchange for stopping the motor in about a millisecond rather than a quarter of a second. Cutting the signal afterwards is what makes the stop *stick* without any further `update()` calls.

Draining before deactivating also parks the signal line low, because an idle state machine stalls on the `side(0)` instruction at the top of the PIO program. Deactivating mid-frame would instead freeze the pin at whatever level that frame was driving.

State machines are deactivated, not destroyed. Constructing them per arm cycle would reload the PIO program into the 32-instruction store each time, requiring `PIO.remove_program()` bookkeeping to avoid exhausting it; `active(0)`/`active(1)` is cheap and cleanly re-armable.

`DShotPIO` gains the `stop()` that `start()` never had.

## What Carries Over From ADR-001

Unchanged and still in force:

- **Three-layer separation** — application, facade, driver. Only the core assignment moves.
- **Lock-free shared state.** `array('H')` throttles with atomic per-element writes, no mutex. This matters *more* now: the library no longer knows which core writes throttles versus which calls `update()`, so the guarantee has to hold unconditionally.
- **1kHz command rate**, 500ms arming duration, minimum usable throttle 70 — all hardware-verified in ADR-001 and unaffected.
- **`DShotPIO` stays scheduling-unaware.**

Reversed:

- Core 1 assignment inside the library
- Thread lifecycle (`start()` / `stop()`) inside the library
- Blocking `arm()`
- The heartbeat counter and `is_healthy()`

## Consequences

### Positive

- **The library composes.** It no longer conflicts with the application's own use of Core 1, timers, or async frameworks.
- **Works in arrangements the old design could not**, notably a single-core cooperative loop.
- **Smaller, more testable facade.** No thread lifecycle, no heartbeat, no startup handshake, no magic sleeps.
- **Stronger stop.** `disarm()` delivers the zeros itself and then cuts the signal, rather than depending on a live loop for either.
- **Two safety bugs removed** — the `start()` activation race and the blocking `is_healthy()` call inside `arm()`.

### Negative

- **More application code.** Every consumer must supply a loop; the test bench needs `tests/harness/core1_runner.py`. This is the cost of the inversion and is accepted deliberately.
- **The application can get it wrong.** Nothing prevents calling `update()` too slowly. `update_age_ms()` makes it detectable, but detection is now the application's job.
- **Arming requires a poll loop** rather than a single blocking call — slightly more verbose at the call site.

### Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Application forgets to call `update()` | Arming never completes; `is_armed()` stays false and `update_age_ms()` grows. Both are observable. |
| Application's loop dies mid-flight | `update_age_ms()` crosses the application's threshold; `disarm()` still works regardless. |
| `update()` called on a stopped group | Inert by design — this is why the state gate exists. |
| Core 1 exception invisible in REPL | `Core1Runner` captures the exception into `runner.error` instead of swallowing it. |

## Verification

**Status:** Pending hardware verification

The test hardware and pass criteria are those of ADR-001 — in particular, **both motors must arm reliably every time**, which is the regression this refactor must not introduce.

| Check | Purpose |
|---|---|
| `test_dshot_single_motor.py` | Low-level driver plus the new `stop()` |
| `test_motor_throttle_group.py` | Both motors arm reliably via an application-owned Core 1 loop |
| Signal cut on disarm | Motors stop and the ESC beeps its lost-signal tone |
| Stop latency from a spun-up motor | Confirms `disarm()`'s zeros land: the motor should wind down at once, not after the ESC's 100-250ms timeout |
| Signal line after `disarm()` | Should read low. Rests on side-set being applied when the `out` instruction stalls — inferred from the PIO program, not yet measured |
| `arm()` → `disarm()` → `arm()` | Deactivate-only release is genuinely re-armable |
| `update()` ×100 while disarmed | Returns immediately; confirms the TX FIFO hang guard |
| `disarm()` on a never-armed group | Returns immediately and does not block on an inactive state machine |
| Cooperative single-core loop | Arming completes without a second core — the case blocking `arm()` could not support |

## References

- [ADR-001: Dual-Core Architecture for Reliable Motor Control](ADR-001-dual-core-motor-control.md)
- [MicroPython _thread module](https://docs.micropython.org/en/latest/library/_thread.html)
- [MicroPython rp2.StateMachine](https://docs.micropython.org/en/latest/library/rp2.StateMachine.html)
- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
