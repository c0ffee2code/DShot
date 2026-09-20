# ADR-005: Bidirectional Telemetry Data Flow

**Status:** Accepted
**Date:** 2026-09-19
**Amends:** [ADR-004](ADR-004-client-owned-command-loop.md) (the facade's constructor and what `update()` does)
**Builds on:** [ADR-002](ADR-002-bidirectional-dshot.md) (the RX capture and decode), [ADR-001](ADR-001-dual-core-motor-control.md) (lock-free shared state)

## Context

ADR-002 got an AM32 ESC's telemetry reply captured and decoded on hardware. What it left open was how those captures should reach the application while the command loop keeps sending frames back-to-back. Three ways of keeping TX and RX in step were on the table: hold TX back until the previous reply has been consumed, tag each capture with a sequence number and discard the unassociable ones, or replace the two-state-machine design with a single state machine that stops and restarts around every frame.

The measurements taken on hardware changed what the decision has to solve:

- **Pairing a reply with its command is not the problem.** In an unpaced run, all 10,000 frames sent produced a structurally correct, correctly paired capture. In a run that deliberately left the RX FIFO undrained for 5ms, every one of 22 cycles recovered in exactly 4 frames, with no misaligned or partial capture.
- **The content is.** A capture can be complete, correctly framed and correctly paired, and still fail its CRC. This clustered around timing disruptions - most clearly right after the RX FIFO had been left undrained: with the confound removed, 0 of 4 captures were CRC-valid after each of 3 resumes, on a baseline that was otherwise 100% valid. A cheap structural check ("does the capture's first bit read 0") passed every one of those captures, so only a real CRC check can tell.
- **Decoding cannot run on the command loop.** One decode costs about 1.3ms on the Pico (it was about 10ms before the decoder was rewritten to work on integers). A DShot300 frame lasts about 53us, the command loop's tick is a few hundred microseconds and the loop runs with no delay between calls, so decoding per frame is out of the question; it has to be sampled, off the hot path.
- **The two cores really run in parallel.** MicroPython on the RP2350 has no global interpreter lock (`sys.implementation._thread` reports `'unsafe'`), so any state shared between the command loop and the application is genuinely concurrent.

## Decision

**The command loop drains every bidirectional motor's RX FIFO on every tick and keeps only the latest capture. The application decodes that capture when and how often it likes.**

1. **Motors are injected.** `DShotPIO` becomes a shared base with two subclasses, `UnidirectionalDShot` and `BidirectionalDShot`. `MotorGroup` takes 1 to 4 already-built motor objects instead of pins and a speed. The application picks each motor's state machines and pins, which removes the need for the library to allocate PIO resources; in exchange the group rejects a set of motors that collide, meaning two motors on one state machine (a bidirectional motor uses two) or on one pin, since either would fail silently on hardware. The telemetry methods live on the base class and raise `UnsupportedOperationException` unless a subclass implements them, so every motor presents the same interface. Bidirectional motors are built as such from the start, because the ESC only detects bidirectional DShot while it is disarmed.
2. **`update()` also drains, before it sends.** It calls `drain_rx()` on each bidirectional motor first, then sends every motor's command. While the group is ARMING the captures are discarded; once ARMED they are published. Draining lives in `update()`, rather than in a second call the application must remember, because an undrained FIFO is the one condition the data links to corrupted captures. A call takes whole 4-word captures only, at most `RX_DRAIN_LIMIT` (4) of them, so a burst cannot hold the loop - which also feeds TX - and a partial capture is left in the FIFO for the next call, so the word grouping cannot slip.
3. **One slot per bidirectional motor.** The latest completed 4-word capture, its `ticks_us` and a sequence number are held in a small `CaptureMailbox` owned by the motor object. The mailbox has no hardware dependency, which keeps the buffering protocol out of the PIO driver and testable on a PC. There is no history: the application samples telemetry, so a newer capture always replaces an older one. The slot is guarded by a sequence counter (odd while being rewritten) so a reader on the other core never sees half of one capture and half of the next, and its buffers are preallocated so the loop does not allocate.
4. **The group is a storage-free accessor.** `raw_telemetry(index)` returns the motor's latest capture only once the group is ARMED, and raises `UnsupportedOperationException` for a unidirectional motor. The group holds no telemetry data itself. `arm()` is rejected unless the group is disarmed: restarting the motors under a live command loop would flush their FIFOs and reset their published captures mid-write.
5. **Decoding is the application's.** `decode_telemetry(index, words)` on the group (or `decode_capture(words)` on the motor) runs the on-device decoder with that motor's RX profile. A capture that fails its CRC is discarded and the application asks again later; when to retry is its decision, and the library contains no retry logic.
6. **`poll_telemetry()` and its counters are removed.** Draining and decoding are now separate steps; the fail-streak counter needed the decode verdict, which the application now owns.

## Alternatives considered

- **Hold TX back until the previous reply is consumed.** Its useful half - never letting the RX FIFO stall - is what draining in `update()` already delivers, without blocking. The rest solves a pairing problem the data does not show.
- **Tag captures with a sequence number and discard the unassociable.** Pairing is not being lost. The slot keeps a sequence number for a different reason: so the application can tell a fresh capture from one it has already seen.
- **A single state machine that stops and restarts around every frame** (the Betaflight approach). It replaces the TX program that arms this ESC, and the per-frame stop/restart/refill is exactly the kind of timing disruption the data implicates in corrupted captures.
- **Decode inside `update()`.** A 1.3ms decode against a 53us frame period and a tick of a few hundred microseconds.
- **A queue of captures, or storage on the group.** The application does not need history, and a pass-through facade keeps the group free of telemetry state. The slot lives on the motor, which is the object that owns the FIFO.

## Consequences

- **`ARMED` is not "the ESC has armed".** It means the group's own arming window has elapsed. With a window shorter than the ESC needs, or an ESC without power, the captures handed out can be echoes of our own transmit or noise, and a small fraction of those pass a 4-bit CRC by chance. The CRC in `decode_capture()` is the validity gate; the ARMED gate only keeps out what arrives while the window is still open.
- **A valid CRC does not yet mean an eRPM value.** The decoder treats every CRC-valid reply as an eRPM frame. Extended telemetry frames and the stopped-motor value are not distinguished, and a CRC-valid reply decoding to an implausible 30,000,000 eRPM has been seen in some runs. Handling these is deferred.
- **The drain is on the hot path, and Python calls and allocations are its cost.** An early version of `CaptureMailbox` added a call layer between `update()` and the FIFO loop, and the group telemetry test then published about 19% fewer captures. `drain_rx` is therefore the mailbox's own flat `drain` method, bound on the motor. It also reads a whole capture with one bulk `get(array)` straight into the published slot: word-by-word reads made a heap integer for every word above 30 bits, about 74 bytes per capture and over 100KB/s of garbage, and a garbage collection on either core pauses both. Measured on real state machines (no ESC), that took a tick with one bidirectional motor from 388us to 175us and its allocation from 73 bytes to 0. Anything added to this path should be measured against that.
- **The slot relies on write ordering across cores** that no barrier enforces (single stores are atomic in practice, but ADR-001's guarantee covers single array elements, not ordering between variables). See Verification.
- **A telemetry reply is not evidence that the motor is spinning.** An armed ESC replies with a constant at-rest value when the motor has not started. Anything that claims a spinning motor has to check the reported eRPM.
- **The pole count is still unverified**, so this design returns eRPM, not mechanical RPM.
- **The drain must come before the send.** A reply capture is 4 words, exactly the RX FIFO's depth, and the next command starts the next capture within tens of microseconds. `update()` originally sent first and drained after; with one bidirectional motor that worked, but with two, the motor drained last (channel 3, after channel 1's slower drain) got no valid reply: its captures were a short burst of activity followed by idle words, which is the receiver stalling on a full FIFO and resuming after the reply had ended. Over 6 runs of an 8-second two-motor scenario at DSHOT300, channel 3 had 0 valid decodes in all 6 (channel 1 also failed in one), and with the order swapped both channels were 100% CRC-valid in 4 of 4 runs. The first harness that recorded two bidirectional motors read the FIFO word by word and never showed it.
- **Not yet confirmed on hardware:** that draining every tick removes the corruption seen after an undrained FIFO under the production loop. The runs below show no corruption in steady operation but do not deliberately stall the consumer.

## Verification

On the AM32 4-in-1 bench ESC, channel 1 (and channel 3 for the two-motor run), DSHOT300, with the arm window set to 3000ms:

| Check | Result |
|---|---|
| `tests/test_motor_group_telemetry.py` | No capture handed out for the whole arming window; 260 of 260 decoded captures CRC-valid; eRPM 21.2k-21.6k (median 21.4k) at throttle 100; no capture after disarm. |
| `tests/test_capture_slot_stress.py` (no motor) | Core 1 publishing at full speed against a tight-loop reader on Core 0: 26,472 valid reads, 0 inconsistent, 0 out of order, 4,479 reads landing mid-update. Evidence for the ordering assumption, not proof. |
| `two_channel_divergent.json` | Both bidirectional motors 28,706 of 28,706 CRC-valid, 0 dropped. |
| `single_channel_baseline.json` (186s) | 100,998 of 100,998 CRC-valid, 0 dropped. |

Still to do: a run that stalls the application's consumer for a few milliseconds, several times, and compares the CRC-valid rate of the captures after each stall against their neighbours.
