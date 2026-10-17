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
- **The two cores really run in parallel.** MicroPython on the RP2350 has no global interpreter lock (`sys.implementation._thread` reports `'unsafe'`; confirmed in source too - `MICROPY_PY_THREAD_GIL=0` in the RP2 port's own build config, both at the bench's v1.28.0 and at current `master`), so any state shared between the command loop and the application is genuinely concurrent.

## Decision

**The command loop drains every bidirectional motor's RX FIFO on every tick and keeps only the latest capture. The application decodes that capture when and how often it likes.**

1. **Motors are injected.** `DShotPIO` becomes a shared base with two subclasses, `UnidirectionalDShot` and `BidirectionalDShot`. `MotorGroup` takes 1 to 4 already-built motor objects instead of pins and a speed. The application picks each motor's state machines and pins, which removes the need for the library to allocate PIO resources; in exchange the group rejects a set of motors that collide, meaning two motors on one state machine (a bidirectional motor uses two) or on one pin, since either would fail silently on hardware. The telemetry methods live on the base class and raise `UnsupportedOperationException` unless a subclass implements them, so every motor presents the same interface. Bidirectional motors are built as such from the start, because the ESC only detects bidirectional DShot while it is disarmed.
2. **`update()` also drains, before it sends.** It calls `drain_rx()` on each bidirectional motor first, then sends every motor's command. While the group is ARMING the captures are discarded; once ARMED they are published. Draining lives in `update()`, rather than in a second call the application must remember, because an undrained FIFO is the one condition the data links to corrupted captures. A call takes whole 4-word captures only, at most `RX_DRAIN_LIMIT` (4) of them, so a burst cannot hold the loop - which also feeds TX - and a partial capture is left in the FIFO for the next call, so the word grouping cannot slip.
3. **One slot per bidirectional motor.** The latest completed 4-word capture, its `ticks_us` and a sequence number are held in a small `CaptureMailbox` owned by the motor object. The mailbox has no hardware dependency, which keeps the buffering protocol out of the PIO driver and testable on a PC. There is no history: the application samples telemetry, so a newer capture always replaces an older one. The slot is guarded by a sequence counter (odd while being rewritten) so a reader on the other core never sees half of one capture and half of the next, and its buffers are preallocated so the loop does not allocate. *(The guarded slot described here is replaced by a single word with a token; see "Update (2026-10-17)" below.)*
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
- **The slot relies on write ordering across cores** that no barrier enforces (single stores are atomic in practice, but ADR-001's guarantee covers single array elements, not ordering between variables). See Verification. *(The 2026-10-17 update removes this assumption: the slot becomes one array element.)*
- **A telemetry reply is not evidence that the motor is spinning.** An armed ESC replies with a constant at-rest value when the motor has not started. Anything that claims a spinning motor has to check the reported eRPM.
- **The pole count is still unverified**, so this design returns eRPM, not mechanical RPM.
- **The receiver's FIFO has to hold more than one capture, and the drain comes before the send.** A reply capture is 4 words, which was exactly the RX FIFO's depth, and the next command starts the next capture within tens of microseconds. `update()` originally sent first and drained after; with one bidirectional motor that worked, but with two, the motor drained last (channel 3, after channel 1's slower drain) got no valid reply: its captures were a short burst of activity followed by idle words, which is the receiver stalling on a full FIFO and resuming after the reply had ended. Over 6 runs of an 8-second two-motor scenario at DSHOT300, channel 3 had 0 valid decodes in all 6 (channel 1 also failed in one). Draining before sending was not enough on its own: 4 of 4 short runs were clean, but a run with faster Core 0 polling then lost one channel in 1 of 3 runs, and a 60-second run lost channel 3 entirely - once a capture is late the stall repeats on every following one, so which channel loses depends on the timing between the two cores. Joining the RX FIFO to 8 words (ADR-002) fixed it: 4 of 4 short runs with the old send-first order were clean, and with both changes the 60-second divergent scenario was 100% CRC-valid on both channels at DSHOT300 and DSHOT600. The drain order stays as a cheap safeguard. The first harness that recorded two bidirectional motors read the FIFO word by word and never showed it.
- **Not yet confirmed on hardware:** that draining every tick removes the corruption seen after an undrained FIFO under the production loop. The runs below show no corruption in steady operation but do not deliberately stall the consumer.

## Update (2026-10-17): the slot is one word, stamped with a token

**Status of this update:** implemented, unit-tested and bench-verified on 2026-10-17 (below: the Pico-only measurements, then the runs with an ESC: all six scenarios pass, the ESC-versus-optical RPM test passes at the default update rate, and an interleaved arming comparison against master shows no worse arming; the 99% arming claim still needs its long batch on this code). The evidence in "What went wrong" is from the slot as it was.

### What went wrong

Reading `raw_telemetry()` from Core 0 while Core 1 ran `update()` at the group's default maximum rate (no fixed pacing) returned nothing at all for long stretches, although the ESC was replying and the loop was publishing:

- A bench test stepped motor 1 through five throttles and polled `raw_telemetry()` in a loop. In four of the five steps every poll returned `None`: 2,024 to 2,344 polls over a 12-second window, none with a capture. This repeated in every run, and in both ascending and descending throttle order. In the same steps the slot's sequence number advanced by about 21,000 per 3 seconds, which is about 3,500 captures published per second, so the writer was working and the receiver's FIFO was draining.
- A direct probe of `latest_capture()` with the loop at the same rate found about half of all calls empty even with the motor idle (roughly 480 to 550 empty out of 760 to 1,150 calls per second, each call taking about 1ms).
- Pacing `update()` at 1ms removed it: 0 empty reads in about 560 polls per step, and 40 decodes arrived in under a second in every step.

**Cause (supported by those numbers, not proven):** the reader accepts a copy only if the counter was even and unchanged across it, and the copy (building a tuple from the slot's array, plus the timestamp) takes longer than the writer's period of about 0.3ms on a Core 0 that is slowed by contention with Core 1. A write then lands inside nearly every attempt. Retrying does not help when each attempt is longer than the period.

### Decision

**The slot holds one array element: the reply frame and a token packed together. A read is one load, so a reader can never see half of one capture and half of another, and nothing is retried.**

```
bit 29 ........ 21 | 20 ................. 0
  token (9 bits)   |  reply frame (21 bits)
```

- `value = token << 21 | frame`, where `token` counts the captures published since the slot was reset, from 1 to 511 and then 1 again. The token is never 0, so a published value is never 0 and 0 can mean "nothing published yet" (a frame can itself be 0, from a silent ESC).
- **Why 21 and 9 bits:** the receiver pushes the reply after exactly 21 bits, so the frame fills the low 21 bits of the word. MicroPython keeps an integer up to 30 bits unboxed; a larger value is a heap object, one allocation per publish, which is the garbage cost described under the drain above. That leaves 9 bits.
- **The token is not a clock.** It plays the part of an ETag: the caller treats a capture as new when the whole value differs from the one it saw last. A token alone does not need to be unique; it wraps after 511 publishes, about 146ms at the maximum rate. A wrapped token reads as "the same" only when the frame is also identical, so the data the caller holds is the data that was published; only the claim that it is new is lost. That range was accepted as enough. A caller that stalls for a multiple of 511 publishes and finds the same frame sees "nothing new", and the harness's published/missed figures undercount whenever a stall exceeds 511 publishes. Tokens start over at 1 on every `arm()` (`disarm()` clears the slot), so a caller forgets its last-seen value when it re-arms; otherwise the new run's first capture can equal the old last value and be skipped.
- **The capture's timestamp leaves the slot.** Telling new from old needs no clock. A caller that wants a time takes its own when it sees the value change, at the resolution of its own poll. An ESC that stops replying shows as a value that never changes, and the caller times that itself. The records the scenario runner writes are stamped this way, so its largest-gap figure and the offline timeline tools measure the poll cadence, not the ESC's reply cadence, and several captures published between two polls appear as one record.
- **The writer's cost is unchanged, not lower.** Per call on the Pico, with a fake receiver (median of three runs, the same timing loops on both versions): publishing one capture 82.50us before and 82.82us after; dropping one capture 71.48us before and 72.06us after; an empty FIFO 28.67us before and 29.33us after. So every path costs 0.3 to 0.7us (1 to 2%) more than it did, including the dropping path, whose loop is the same code as before; the cause is not known. The expected saving from no longer reading the clock was not measurable.
- **Publishing and discarding are two methods.** `drain_rx()` publishes, `discard_rx()` takes the replies off the FIFO and drops them. The group calls `discard_rx()` on every arming tick, when replies are not yet trusted and only the count of not-running replies matters to the arming gate, and `drain_rx()` once armed. The arming loop is therefore its own small function and carries no test for whether to publish. The group's diagnostic flag that published arming replies is removed; nothing used it.
- **The ordering assumption is gone.** The slot no longer depends on two cores observing separate stores in order (see the Consequences above).

### Alternatives considered

- **Shorten the read inside the counter check** (read the one word and the timestamp directly, build the tuple afterwards). It narrows the window but keeps a window, and the read still fails whenever a write lands in it.
- **More attempts.** No help when one attempt is longer than the writer's period.
- **A ring of slots,** so the writer must lap the reader to spoil a read. It works while a read is shorter than the ring's lead (16 slots at 0.3ms is about 4.8ms), but it adds a writer-side index and per-slot counters, and it still validates.
- **Publish an immutable `(ticks, sequence, word)` tuple by swapping one reference.** Atomic, but one allocation per publish: about 3,500 a second, over 100KB/s of garbage, which is what the drain above was rewritten to avoid.
- **Pack the frame with a short timestamp** (11 bits, 2ms at 1us resolution). Atomic, but a range too short to detect a silent ESC or to measure the gap between replies.
- **Require applications to pace `update()`.** It works (see above) but leaves the default loop, the one CLAUDE.md shows, unable to read telemetry reliably.

### Consequences

- The accessors change with the slot: the capture no longer carries a timestamp, so `raw_telemetry()`, `latest_capture()` and `decode_telemetry()` change shape, and the scenario runner stamps its records with its own read time. The "oldest capture at the moment it was read" figure the runner prints cannot be kept as it is.
- The stress test for the slot (`tests/bench/pio_and_cores/test_capture_slot_stress.py`) checked consistency of a multi-field slot; it needs reshaping for a slot with nothing to tear.
- Measured on the Pico with no ESC: the slot stress test (a writer thread publishing as fast as it can against a tight reader loop on the other core) ran 15 seconds with 110,517 reads, none refused and none inconsistent, and the token steps accounted for 126,847 of the 126,852 captures published (the difference is what was in flight when it stopped). The PIO lifecycle test (repeated build, arm and disarm, then 15 re-arms of one group) passes on the new driver.
- Confirmed with an ESC on 2026-10-17 (originally listed as still to do): all six scenarios pass with arming at 2000-2025ms; the RPM test passes at the default rate with every window closing on its sample targets, no empty polls, and ESC RPM within +0.04% to +0.58% of optical; 20 interleaved arming runs of `single_channel_bidirectional_300` on this code and 20 on master: branch 20 of 20 passed (18 at 2021-2022ms, two slow arms at 2450 and 2608ms, both inside the 3.0s bound), master 19 of 20 (one ESC reboot, 5796ms). Whether the slow arms come from this change is not settled by 2 in 20. What was to be confirmed: the ESC-versus-optical RPM test (`tests/bench/rpm_accuracy/test_esc_rpm_vs_optical_rpm.py`) at the default maximum update rate, repeated, with every step's window closing on its sample target and the share of polls that find a new value in line with the paced run it replaces; the scenario suite passing with unchanged arming times. Because a read can no longer be refused, "no empty polls" proves nothing about the fix; a problem would now show as values that stay the same, so that is what is compared.

## Verification

On the AM32 4-in-1 bench ESC (channels 1 and 3 mounted), with the arm window set to 3000ms. Every run below goes through `MotorGroup` (`tests/harness/run_scenario.py`): the group's `update()` on Core 1, Core 0 sampling `raw_telemetry()`, every 20th new capture decoded on the device and every logged capture decoded again on a PC. In each run the device's tally and the PC's replay of the same sampling were identical.

(Scenario names below are historical - `telemetry_settled_300/600`, `single_channel_baseline` and
`smoke_unidirectional` were later deleted in the harness's scenario-library restructure. These are
the names the runs actually used at the time; later scenarios with similar names and purposes
exist but did not produce these specific numbers, so they aren't swapped in here.)

| Scenario | Result |
|---|---|
| `telemetry_settled_300` / `_600` (channel 1, throttle 100, 8s) | 102 of 102 sampled decodes CRC-valid at each speed, median 21,490 eRPM; 2,058 (DSHOT300) and 2,049 (DSHOT600) logged captures, all CRC-valid on the PC. No capture was handed out for the whole arming window (checked on every poll). |
| `two_channel_divergent_300` / `_600` (channels 1 and 3, 60s, opposing ramps) | Both motors CRC-valid throughout: 654 of 654 (DSHOT300) and 676 of 676 (DSHOT600) sampled decodes each, and 13,094 / 13,522 logged captures each on the PC. Median eRPM 57.9k and 35.4k at DSHOT300, 58.1k and 34.4k at DSHOT600. |
| `single_channel_baseline` (channel 1, 186s, throttle 100-300) | 2,308 of 2,308 sampled decodes and 46,166 of 46,166 logged captures CRC-valid; median 76.3k eRPM. |
| `smoke_unidirectional` (4 unidirectional motors, 20s) | Arms, sustains the command stream and disarms through the group; nothing is read back. |
| `tests/device/test_capture_slot_stress.py` (no motor) | Core 1 publishing at full speed against a tight-loop reader on Core 0: 27,756 valid reads, 0 inconsistent, 0 out of order, 278 reads landing mid-update (rerun 2026-09-20 with the current driver). Evidence for the ordering assumption, not proof. |

The harness records the latest capture Core 0 sees, not every reply: about 255 records a second with one bidirectional motor and 225 with two, against well over a thousand captures a second published per motor. The first version of these runs (two bidirectional motors, send-then-drain, 4-word FIFO) is what exposed the FIFO problem in the Consequences above.

Still to do: a run that stalls the application's consumer for a few milliseconds, several times, and compares the CRC-valid rate of the captures after each stall against their neighbours.
