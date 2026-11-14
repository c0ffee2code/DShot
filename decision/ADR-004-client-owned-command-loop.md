# ADR-004: Client-Owned Command Loop

**Status:** Accepted, amended by [ADR-005](ADR-005-bidirectional-telemetry-data-flow.md) (the facade takes motor objects and `update()` also drains replies)
**Date:** 2026-08-12
**Supersedes in part:** [ADR-001](ADR-001-dual-core-motor-control.md) (core assignment and thread lifecycle only)

## Context

ADR-001 solved a real problem. Arming previously succeeded roughly half the time, with one motor left beeping, because Core 0 activity (display rendering, button polling, sleeps in the main loop) opened gaps in DShot transmission and the ESCs reset their arming counters. Dedicating Core 1 to a 1kHz command loop eliminated the failures completely, and that finding still holds.

But it placed the decision in the wrong layer. `MotorGroup` called `_thread.start_new_thread()` itself, so *the library* chose the threading topology of every application that imported it:

- An application that wants Core 1 for its own control algorithm, and the DShot loop on a timer IRQ, cannot have it.
- An application built around `uasyncio` gets a raw thread it did not ask for.
- A simple single-core sketch pays for a second core it does not need.
- Two libraries following this pattern in one project would both claim Core 1, and the second would fail.

The library is the wrong place to decide this, because the right answer differs per project. This repository is a DShot library; scheduling is not a DShot concern.

A second problem followed from the first: `arm()` blocked for 500ms in `utime.sleep_ms()`, which only works because *some other* context was transmitting. Any application driving the loop cooperatively from its own main loop would deadlock — it would be asleep inside the very call that needed it to keep pumping.

## Decision

**The library exposes `update()`. The application decides when and where it is called.**

`MotorGroup` remains a facade over the motors it is given and their throttle state. It knows *what* to transmit and *when it is due*; it does not know, and does not ask, which core it is running on.

```
┌─────────────────────────────────────────────────────────────┐
│                      APPLICATION                            │
│                                                             │
│  Owns the command loop and decides where it runs:           │
│    - a dedicated Core 1 thread (test bench does this)       │
│    - a machine.Timer IRQ                                    │
│    - its own cooperative main loop                          │
│                                                             │
│         group.update()   <- every 1045us, steady            │
│         group.set_throttle(index, value)                    │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                  MOTOR GROUP FACADE (core-agnostic)         │
│                                                             │
│  arm()      - activate state machines, open arming window   │
│  disarm()   - deactivate and reset state machines, line low │
│  update()   - a frame per motor, drain replies, advance arm │
│  is_armed() - application polls for arming completion       │
│  raw_telemetry(i) - latest reply capture, once armed        │
│                                                             │
│        Shared throttle array (lock-free writes)             │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│     DRIVER (DShotPIO: Unidirectional / Bidirectional)       │
│                                                             │
│  start() / stop() - PIO state machine activation            │
│  drain()          - wait for queued frames to go out        │
│  send_throttle_command(throttle) - encode and transmit      │
│  drain_rx() / latest_capture() - bidirectional replies      │
└─────────────────────────────────────────────────────────────┘
```

*Update 2026-11-11:* `update()` no longer calls `send_throttle_command(throttle)`. Once armed it sends each motor's throttle through `send_unchecked(throttle)`, which builds and queues the packet with no range check; while arming it sends the precomputed `send_zero()`. The range check moved to the boundary: `MotorGroup.set_throttle()` and `set_all_throttles()` reject anything other than 0 or 48 to the bench ceiling (`validate_throttle()`). `send_throttle_command()` keeps its own check for code that drives a motor without a group. See ADR-002, "Command-loop and decode performance", update note of 2026-11-11 (the change ran on the Pico and the bench on 2026-11-11: in 15 runs (AM32 2.22, four bidirectional motors, DSHOT600, throttle 0 to 215 held 20 s, bare shafts; none of them KPI evidence) 14 armed and spun, 12 of them in 2112 to 2113 ms and 3 in 2630 to 2659 ms. This shows that the changed path arms and drives the motors. It establishes no arming rate (five runs per batch), and nothing was measured about per-tick timing.).

*Update 2026-11-14:* names in the diagram and in the note above are those of their dates. `send_throttle_command()` no longer exists, `send_unchecked()` is `send_frame()`, `start()` / `stop()` are `activate()` / `deactivate()` and `disarm()` is `deactivate()`: see "Amendment 2026-11-14" under "`arm()` / `disarm()` own the PIO lifecycle".

### No runner abstraction in the library

The library ships no `MotorRunner` interface for the application to implement. The shape of such an interface — callback versus poll, who owns the interval, restart semantics, how failures surface — is precisely what varies between projects, so defining it in `driver/` would re-create the same coupling one layer up.

`tests/harness/core1_runner.py` is a concrete Core 1 runner, and it is explicitly **application code**, kept next to the scripts that use it rather than in `driver/`. Projects copy and adapt it. If a genuinely common shape emerges across several projects, extract it then, from real usage rather than speculation.

*Paths (2026-11-11):* `tests/harness/` no longer exists (tests restructured in commit `2cbde0c`). The runner is now `tests/bench/core1_runner.py`, and the scenario harness named under Verification is `tests/bench/dshot_protocol/run_scenario.py`. `tests/device/test_pio_lifecycle.py` is now `tests/bench/pio_and_cores/test_pio_lifecycle.py`.

### Sub-Decision: Non-blocking arming

`arm()` opens an arming window and returns immediately. `update()` advances it and flips the state to armed once the duration has elapsed. The application polls `is_armed()`.

This is what makes the library work under *any* scheduling arrangement, including the cooperative single-loop case that a blocking `arm()` could never support. It also lets the application do something useful while waiting — the test bench demo animates its ARMING screen, which the blocking version could not.

While arming, `update()` transmits literal zeros rather than the throttle array, so the arming window is genuinely at zero even if the application sets a throttle early.

The arming window is plain elapsed time since `arm()`, unaffected by how unevenly `update()` gets called: AM32 does not reset its own arming counter on a gap in our transmissions, only on non-zero throttle or a gap long enough to be a reboot (source-verified). A group with bidirectional motors also gates on a separate reply-based check - every bidirectional ESC must have echoed its own "not running" frame steadily for a couple of seconds - which is what actually needs to notice a reboot, since a reboot is exactly the kind of gap that resets the ESC's own state.

### Sub-Decision: `update()` is inert while disarmed

`update()` returns immediately unless the group is arming or armed.

This is a safety requirement, not an optimisation. `disarm()` deactivates the state machines; a loop that kept calling `update()` afterwards would fill each 4-word TX FIFO and then **block forever inside `StateMachine.put()`** — a silently hung core with no traceback, since Core 1 exceptions never reach the REPL.

Gating on state also makes call ordering unhazardous in general. The application may start its loop whenever convenient, and the loop only does real work between `arm()` and `disarm()`. This removes a latent race in the previous implementation, where Core 1 was launched *before* the state machines were activated and survived only because the heartbeat handshake won.

### Sub-Decision: The library reports facts, not health

`is_healthy()` is gone. It blocked for 5ms inside `utime.sleep_ms()`, and was called from within `arm()` — a hidden stall in a safety path. More fundamentally, a library that does not own the loop cannot judge whether that loop is healthy.

`update_age_ms()` reports milliseconds since the last transmission. The application sets its own threshold, because only the application knows what its scheduling arrangement should deliver. The arming state machine reuses the same timestamp, so there is one source of truth and no separate heartbeat counter to overflow.

*Cross-reference 2026-11-11:* the same principle applied one level up, on the test bench (the main Pico reports facts and may bail out of a run; the PC judges every session from the wire recording), is [ADR-008](ADR-008-main-pico-reports-pc-judges.md).

#### Amendment 2026-11-11: failures outside the library are not handled by the library

**Decision.** The library does not detect and does not handle the two cases below. It gets no dead-hand mechanism, no health judgement, no automatic zero throttle and no accessor for the age of the last throttle write. The library reports facts; application code decides on them and provides the level of resilience it requires.

**Cases considered.**

1. **The application stops updating throttles while the command loop keeps running** (for example, the control code on one core fails while `update()` keeps running on the other). `update()` keeps sending the last throttle values, so the ESC keeps receiving valid frames and its signal-loss timeout never fires. The motors hold their last throttle until the application acts.
2. **An ESC goes silent or resets after `ARMED`.** The group stays `ARMED` and keeps sending. With the ESC silent, the receiver captures our own frames and publishes them (ADR-002, correction under "Hardware validation, part 5"), so captures keep arriving and some can pass the CRC at some throttles (ADR-005).

**Motivation.**

- A crash of the application without shutting the library down is a defect of the application, not of the library.
- Resilience and self-healing inside a library grow without a natural limit; they are kept to the minimum.
- The library cannot tell a stopped application from a quiet one: "a throttle was written" is not "the application is alive" (a bench script sets a throttle once and waits; a controller may be alive and skip a write). Only the application knows which holds.
- The application can detect both cases without help from the library.
- A timestamp in `set_throttle()` would add work on the core that sets throttles, which changes the command loop's frame spacing on this platform (BUG-011).

**Consequences.**

- Shutdown is the application's duty. An application that can fail calls `disarm()` on every exit path (for example in a `try`/`finally`). `disarm()` deactivates and re-initialises the PIO state machines and drives the lines low, nothing more; after it `update()` is inert, so a command loop that is still running sends nothing.
- Case 1 stays undetected by the library. `update_age_ms()` cannot show it, because `update()` itself refreshes that value. `set_throttle()` and `set_all_throttles()` store values only.
- Case 2 is application work: a per-motor check that the captures are real replies, and a flag for a reset after `ARMED`. BUG-008 records both for the test harness, which is the application on the bench.
- What the library reports (checked against `driver/motor_group.py` on the amendment date): `update_age_ms()`, the time since `update()` last ran; `state`, `is_armed()` and `is_arming()`; `arming_status()`, the reply evidence gathered while arming (not updated after `ARMED`); the latest capture per bidirectional motor through `raw_telemetry()`, and its decode (CRC, eRPM, `not_running`) through `decode_telemetry()`; the throttle values held, through `get_all_throttles()`.

### Sub-Decision: `arm()` / `disarm()` own the PIO lifecycle

The previous API had four overlapping methods — `start()`, `stop()`, `disarm()` and `emergencyStop()` — three of which ended at the same "write zeros". They collapse into one pair:

| Method | Does |
|---|---|
| `arm()` | zero throttles, `sm.active(1)`, open the arming window |
| `disarm()` | `sm.active(0)`, re-initialise, pin driven low - nothing transmitted |

**Revised 2026-10-13: `disarm()` no longer transmits zeros.** The original design below had `disarm()` command zero throttle, wait for it to leave the shift register, then deactivate, so the motor stopped in about a millisecond instead of waiting out the ESC's signal-loss timeout (AM32: 500ms while armed). The project accepts that timeout as the stop: cutting the signal makes AM32 switch every phase off and reset (specification/AM32.md#15), and the zeros did not change the reboot, only how soon the motor stopped. What the zeros cost was the only blocking call in the facade, a paced multi-round transmit that existed to keep a bidirectional motor's frames out of its ESC's reply, and a hazard when another core was still inside `update()`. An application that needs a fast stop sets throttle 0 while its loop runs, then calls `disarm()`.

What `disarm()` guarantees instead is a clean next start. Each motor's `stop()` deactivates its state machines and re-initialises them, so the TX FIFO is empty (no queued throttle can be the first frame of the next arm) and a bidirectional motor's RX FIFO and stored capture are dropped. The pin is then a plain output driven low rather than frozen at whatever level a cut frame left it. `start()` re-initialises once more and takes the pin back for PIO.

The text below is the original design, kept for the reasoning behind the state-machine lifecycle.

`disarm()` *is* the emergency stop, and it is stronger than the old one: it both commands the stop **and** cuts the signal, whereas the old version relied on a loop still running to deliver its zeros.

Both halves are needed, and the order matters. Cutting the signal alone is not a stop — it only makes the ESC *eventually* time out, which on AM32 is up to 500ms of a motor still spinning at its last commanded throttle (AM32's own signal-loss timeout is 500ms while armed, read from its firmware source; the design's original historical measurement was against BLHeli_S, which is no longer a supported target as of 2026-09-26). So `disarm()` transmits the zeros itself, waits for them to leave the shift register, and only then deactivates. That is a few hundred microseconds of blocking, in exchange for stopping the motor in about a millisecond rather than up to half a second. Cutting the signal afterwards is what makes the stop *stick* without any further `update()` calls.

Draining before deactivating also parks the signal line low, because an idle state machine stalls on the `side(0)` instruction at the top of the PIO program. Deactivating mid-frame would instead freeze the pin at whatever level that frame was driving.

State machines are deactivated, not destroyed. Constructing them per arm cycle would reload the PIO program into the 32-instruction store each time, requiring `PIO.remove_program()` bookkeeping to avoid exhausting it; `active(0)`/`active(1)` is cheap and cleanly re-armable. (2026-10-13: they are now also re-initialised with the same program object on every start and stop, as `BidirectionalDShot.start()` already did; this does not construct new state machines.)

`DShotPIO` gains the `stop()` that `start()` never had.

#### Amendment 2026-11-14: each layer is named for, and limited to, what it does

**Decision.**

1. *Layers.* `MotorGroup` is the library's public API. `DShotPIO` (`UnidirectionalDShot`, `BidirectionalDShot`) is the low-level I/O core beneath it: PIO state machines and the pin.
2. *Names.* No public method or state is named after stopping or disarming a motor. `MotorGroup` keeps the names of what it does: `arm()`, the arming states, and setting the throttles that `update()` sends. Its teardown `disarm()` becomes `deactivate()`, and the state `DISARMED` becomes `INACTIVE`. `DShotPIO` names its lifecycle in PIO terms: `start()` and `stop()` become `activate()` and `deactivate()`.
3. *Validation.* `DShotPIO` does not judge values. It sends the 11-bit value it is given through `send_frame(value)`; `send_throttle_command()` and `InvalidThrottleException` are removed. The valid throttle range, the temporary bench ceiling included (`THROTTLE_CEILING`), is defined and checked in `MotorGroup` only.

Behaviour does not change. No aliases for the old names are kept.

**Motivation.**

- The only way the library stops a motor is `update()` transmitting throttle 0. The renamed methods transmit nothing: the ESC holds the last throttle until its signal-loss timeout (AM32: 500 ms while armed, specification/AM32.md#15) and then resets.
- A method named `disarm()` or `stop()` invites an application to treat it as the stop. Stopping is the application's duty (amendment 2026-11-11).
- Arming keeps its name because the library performs and observes it: zero throttle for the arming window, and the reply gate. It has no counterpart: the library never commands an ESC to disarm.
- The low-level sender held a weaker copy of a rule the group owns in full: it refused values above the ceiling but sent 1 to 47, which AM32 executes as DShot commands. What a value means (stop, command, throttle) and which values are allowed belong to the layer that has that meaning.

**Consequences.**

- To stop motors, the application sets throttle 0 and keeps its command loop running until the ESCs have received it, then stops the loop and calls `deactivate()`. How long it keeps sending (a fixed time, or until telemetry shows the motor at rest) is its own decision.
- `deactivate()` at a non-zero throttle is a signal loss for the ESC. It remains the only action available when the command loop is dead.
- Code that uses a `DShotPIO` object directly answers for the values it sends: nothing below `MotorGroup` refuses a command value or a throttle above the bench ceiling. The README's single-motor example on the low-level class is removed; a single motor is a `MotorGroup` of one. The two bench probes that drove a motor without a group (`test_frame_arming_echo.py`, `test_frame_stalled_drain.py`) are deleted.
- This replaces the sentence in the update note of 2026-11-11 under the layer diagram ("`send_throttle_command()` keeps its own check for code that drives a motor without a group").
- Records written before this amendment use the old names: `disarm()` and `stop()` are `deactivate()`, `start()` is `activate()`, `DISARMED` is `INACTIVE`, `send_unchecked()` is `send_frame()`.

**Verification.** PC tests pass (675). On the Pico alone, with no ESC (`tests/bench/pio_and_cores/test_pio_lifecycle.py`, unused GPIO 10 and 11, 2026-11-14): 30 build/arm/deactivate cycles, both lines low after `deactivate()`, 15 re-arms each capturing again. With the ESCs (2026-11-14, one run, AM32 2.22, four bidirectional motors, DSHOT600, throttle 0 to 215 held 20 s, bare shafts): armed after 2111 ms, all four motors spun and held, every line's last frames carried throttle 0, session verdict pass from the wire recording. One run shows that the renamed path arms, drives and ends a session; it establishes no rate.

#### Amendment 2026-11-14 (2): `update()` sends only when the transmitters have taken the previous frames

**Decision.** At its entrance `update()` waits until every motor's transmit queue is empty, then sends one frame to each. The PIO transmit queues stay as they are (4 words); the wait keeps them at no more than one frame. The low-level senders (`send_frame()`, `send_zero()`) are unchanged and have no such wait.

**Motivation.** The bidirectional transmitter takes a frame at most every 650 PIO cycles (ADR-002, amendment 2026-11-14 (2)). Without the wait, a loop calling `update()` faster than that would stack up to five frames per motor before it blocked, and the newest throttle would go out behind them. With it a too-fast caller is slowed to the transmitter's rate from its third call (the first frame is taken at once, the second is queued behind it), no frame is dropped, and a command is at most one frame behind the wire. Blocking was chosen over skipping the tick: a command that must be delivered a set number of times is not lost.

**Consequences.**

- One wait for the whole group, before any frame of the tick is sent, so the frames of one tick still go out together, and a group that mixes speeds or motor kinds waits for its slowest transmitter. It asks each motor's state machine with `StateMachine.tx_fifo()`, MicroPython's own query of a transmit queue. A single read of each PIO block's status register was built and measured first: timed in isolation on the Pico it costs about 6us per check whatever the group size, against 4us, 7us and 12us with `tx_fifo()` for one, two and four motors. The documented API was chosen over it: the driver holds no register addresses and no compiled pointer code.
- "Empty" means the transmitter has taken the previous frame, not that it has finished it: at most one frame per motor is on the line and one is queued. That holds while `update()` is the only sender; code that also calls the low-level senders can queue more.
- The wait is a busy loop that reads the group's state on every pass: it ends when `deactivate()` sets the group inactive, and `update()` then returns without sending. `deactivate()` during a running loop is still not serialised (stop the loop first). If a frame is left queued on a transmitter that is not running (that race, or a motor deactivated directly while its group is live), `update()` waits until `deactivate()` is called. From MicroPython's source, not tried on the board: compiled code has no check for a pending Ctrl-C, so such a wait on the main thread is not ended by one, where the blocking `put()` it replaces was.
- Cost, measured on the Pico without an ESC (2026-11-14, GPIO 10-13, medians of 1,500 ticks paced at 1045us, the same build with the wait and with it switched off): four bidirectional motors 217us to 229us per tick while arming and 280us to 293us armed, the same at DSHOT600 and DSHOT300; one motor 94us to 99us and 106us to 110us. So about 12us per tick for four motors and 4 to 5us for one. These figures are this probe's own and are not comparable with tick figures in other records, which came from other probes.
- At the 1045us period the wait is never entered. This is derived (a tick of at most about 300us and a spacing of at most 271us against a 1045us period), not counted.
- Measured with `tests/bench/pio_and_cores/test_reply_holdoff.py`, one bidirectional motor, Pico only. *Behind a queue:* with two frames queued directly on the motor, one `update()` call took 362us (DSHOT600) and 639us (DSHOT300) and left one word queued; with the wait switched off it took 132us and 137us and left 2 and 3 queued. *A burst of six calls, each followed by a read of the queue:* at DSHOT300 the frames start 270.8 to 271.3us apart, the transmitter's spacing, with one word queued after a call; at DSHOT600 they start 142 to 155us apart with nothing queued, because one pass of the test's loop is longer than that speed's 135.4us spacing, so the wait is not entered there. A bare one-motor `update()` (99 to 110us above) is shorter than 135.4us; such a caller would be held at DSHOT600 as well, which the first check covers. All six throttles are on the wire, in order, in every burst.
- Bench, 2026-11-14, AM32 2.22, four motors on GPIO 6-9, bare shafts, one run of each of the four permanent scenarios (sessions 2026-11-14_20-53-25, 20-54-13, 20-55-01, 20-55-46): all four armed and passed the session verdict; the command loop never fell behind. Bidirectional DSHOT300 armed in 2112 ms, the unidirectional runs in 2002 ms. Bidirectional DSHOT600, the first run of the batch, armed in 2594 ms: during arming 107 to 461 of about 2,092 replies per line had a bad CRC and started less than 10us after our frame. The seven bidirectional runs recorded earlier that day on the code before this wait armed in 2111 to 2113 ms with no such replies. A second round of the four scenarios the same evening (sessions 2026-11-14_21-14-17, 21-15-05, 21-15-54, 21-16-39) passed with the usual arming times (2113 and 2112 ms bidirectional, 2001 and 2000 ms unidirectional) and no such replies; the owner watched it and saw all four motors turn in all four runs, the unidirectional ones included. So one late arming in four bidirectional runs on this code, in the first run after a pause in bench use. Four runs do not show whether the wait has a part in it; not investigated further.

## What Carries Over From ADR-001

Unchanged and still in force:

- **Three-layer separation** — application, facade, driver. Only the core assignment moves.
- **Lock-free shared state.** `array('H')` throttles with atomic per-element writes, no mutex. This matters *more* now: the library no longer knows which core writes throttles versus which calls `update()`, so the guarantee has to hold unconditionally.
- **The timing figures** - 1kHz command rate, 500ms arming duration, minimum usable throttle 70 - were verified in ADR-001 on the original BLHeli_S ESC, no longer a supported target (see CLAUDE.md). README's "Verified Parameters" has the corresponding AM32 figures, which the library's defaults already cover as the more demanding case. The library's current values are `UPDATE_INTERVAL_US` and `DEFAULT_ARM_DURATION_MS` in `driver/motor_group.py`.
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
- **Deterministic stop.** `disarm()` cuts the signal and resets every motor itself, rather than depending on a live loop. (Originally it also transmitted zeros first, for an immediate stop; revised 2026-10-13, see the note under the lifecycle table: the ESC's own timeout now stops the motor, and an application that wants it sooner sends throttle 0 first.)
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

*Update 2026-11-11:* the "loop dies mid-flight" row holds for the loop that calls `update()`. If the code that sets throttles dies while that loop keeps running, `update_age_ms()` stays fresh and the motors hold their last throttle (see the 2026-11-11 note under "The library reports facts, not health").

## Verification

**Status:** Partly verified. Arming, running and disarming through the facade with an application-owned Core 1 loop has been exercised repeatedly on the AM32 bench ESC, most recently through the scenario harness (`tests/harness/run_scenario.py`), which drives every scenario through `MotorGroup` with `update()` on an application-owned Core 1 loop. The other checks below have not been re-run individually, and the scripts named in the table were retired.

**PIO state machines are deactivated, never released.** `stop()` deactivates and restarts a state machine; nothing frees the state machines or the programs they loaded, and this MicroPython (1.28) has no way to free a state machine (`StateMachine` has no `deinit`), only `PIO.remove_program()` for a program. That is enough for how a `MotorGroup` is used: 30 builds of a group, each armed and disarmed, raised no error and left the heap unchanged, because a new state machine on the same id reuses the program already loaded. It would not be enough for code that puts a different program on the same state machine repeatedly: the replaced program stays loaded and the block's 32 slots run out (an ENOMEM, which is what swapping the receiver program on a constructed motor produced); `remove_program()` should free it, but has not been tried. `disarm()` leaves the receiver's FIFO full of unread words (8 with the joined FIFO); `start()` flushes them. *Correction (2026-11-11):* no longer true. Since 2026-10-13 each motor's `stop()` re-initialises its state machines, and `BidirectionalDShot.init_state_machines()` empties the RX FIFO and clears the published capture, so `disarm()` leaves the FIFO empty; `start()` does the same again. The receiver's FIFO is not joined any more: the frame receiver pushes one word per reply into the default 4-word FIFO (ADR-002, "Hardware validation, part 3").

Between runs: the next `mpremote run` clears the PIO state itself. After a run that exited armed, every state machine was inactive and every FIFO empty, and a Core 1 thread left running did not stop the next run from starting its own (both checked 2026-09-21). What does survive is pad configuration, such as a pull-up set by the previous run.

The original test hardware and pass criteria are those of ADR-001 — in particular, **both motors must arm reliably every time**, which is the regression this refactor must not introduce.

| Check | Purpose |
|---|---|
| `test_dshot_single_motor.py` | Low-level driver plus the new `stop()` |
| `test_motor_group.py` | Both motors arm reliably via an application-owned Core 1 loop |
| Signal cut on disarm | Motors stop and the ESC beeps its lost-signal tone |
| Stop latency from a spun-up motor | Originally: confirms `disarm()`'s zeros land and the motor winds down at once. Since 2026-10-13 `disarm()` transmits nothing: with the harness's `fast_stop()` (throttle 0 first) the motors were observed to stop at once; a bare `disarm()` leaves the motor at its last throttle until the ESC's signal-loss timeout (up to 500ms on AM32), which has not been measured on the bench |
| Signal line after `disarm()` | A unidirectional line reads low; a bidirectional line, at the time this was measured (2026-09-21, `tests/device/test_pio_lifecycle.py`), read high (released and pulled up) - that changed 2026-09-25: a bidirectional line is now also driven low, like the unidirectional case, because a released-high line left some ESC firmware unable to time out of its own post-disarm reboot cleanly (see ADR-002's "Implementation Update (2026-09-25)" and the test's own current assertion) |
| `arm()` → `disarm()` → `arm()` | The state machines and the receiver work again each time: 15 re-arms on one group, each capturing afresh with the published sequence starting over (same test, no ESC). Re-arming an actual ESC after a disarm is exercised by every scenario run, not separately |
| `update()` ×100 while disarmed | Returns immediately; confirms the TX FIFO hang guard |
| `disarm()` on a never-armed group | Returns immediately and does not block on an inactive state machine |
| Cooperative single-core loop | Arming completes without a second core — the case blocking `arm()` could not support |

## References

- [ADR-001: Dual-Core Architecture for Reliable Motor Control](ADR-001-dual-core-motor-control.md)
- [MicroPython _thread module](https://docs.micropython.org/en/latest/library/_thread.html)
- [MicroPython rp2.StateMachine](https://docs.micropython.org/en/latest/library/rp2.StateMachine.html)
- DShot Protocol Specification: `specification/DSHOT.md`
