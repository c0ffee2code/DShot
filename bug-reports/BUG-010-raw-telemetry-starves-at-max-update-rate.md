# BUG-010: `raw_telemetry()` returns nothing for whole seconds when `update()` runs at its maximum rate

**Status:** OPEN. Found 2026-10-17. Worked around in one test (it paces `update()` at 1 ms); the driver is unchanged.
**Severity:** Medium. A caller polling `raw_telemetry()` can get no capture at all for seconds at a time while the ESC is replying normally and the driver is publishing. Nothing is lost on the wire, and the motor is unaffected.
**Component:** `driver/capture_mailbox.py` (`CaptureMailbox.latest()`), `driver/dshot_pio.py` (`BidirectionalDShot.latest_capture()`), reached through `MotorGroup.raw_telemetry()`
**Found:** while building `tests/bench/rpm_accuracy/test_esc_rpm_vs_optical_rpm.py`

## Symptom

The RPM accuracy test spins motor 1 through throttle 60-100 and reads the ESC's eRPM with a plain
`raw_telemetry(0)` poll on Core 0. With `Core1Runner(group.update, group.UPDATE_INTERVAL_US)` (interval 0, the
library's default: no fixed pacing) it got **no capture in four of five steps**, in every run:

- 12 s windows, 2,024-2,344 polls each, **every one returned `None`** (steps at throttle 60, 70, 90, 100).
- The one step that worked (the third, in both ascending and descending order) delivered 40 decodes in about 0.2 s.
- Earlier runs of the same test with 2 s steps showed the same shape (0, 5, 5, 37, 5 decodes per step).

## What was ruled out

- **The driver was publishing the whole time.** `motors[0].mailbox.slot_seq` advanced by about 21,000 per 3 s step (about 3,500 captures/s) in every step, including the silent ones.
- **The group state was fine** (`ARMED`) and the RX FIFO was draining (0-1 words waiting at the end of each step).
- **Not the RPM sensor.** The pattern was identical with the sensor interrupt switched off.
- **Not the throttle value.** Descending the throttle list gave the same pattern (third step works, others silent).
- **Not decode cost.** The counters showed zero captures *seen*, not captures skipped.
- **Not a dead ESC.** The disk kept turning at the commanded speed; with a paced loop the same ESC delivered CRC-valid replies in every step.

## Evidence for the mechanism (a hypothesis, not proven)

`CaptureMailbox.latest()` is a sequence lock with `LATEST_ATTEMPTS = 4`: it returns `None` when the sequence number is odd (a write in progress) or changed while it copied the words. A probe script (since deleted) polled `latest_capture()` for 1 s at a time with Core 1 at maximum rate:

| State | `latest()` calls | returned a capture | returned `None` | odd sequence seen |
|---|---|---|---|---|
| idle (throttle 0) | about 760-1,000 per second | 208-511 | 479-552 | 32-36 |
| throttle 60 / 80 | about 950-1,150 per second | 499-730 | 422-500 | 40-49 |

So even in the probe about half of the calls returned nothing, and each call took about 1 ms, while the writer published a new capture about every 0.3 ms. The reader's copy is slower than the writer's period, so a write usually lands inside it. The test's own loop did more work per poll and was slower still, and lost every time in four of five steps. Why the third step works is not understood.

Setting `Core1Runner`'s interval to 1000 us (the writer then publishes at most about 1,000 captures/s) removed the problem: **0 empty reads in about 560 polls per step**, 40 decodes in under a second in every step, and the full test passed (commit `4991d1a`).

## Who is affected

- **Any application that polls `raw_telemetry()` while running `update()` at the library's default unpaced rate.** CLAUDE.md's usage example does exactly that (`motors.UPDATE_INTERVAL_US`, which is 0).
- **The scenario runner** also runs unpaced. It still delivers about 276 records/s (`largest_gap` 17 ms in the 2026-10-17 run), but it samples only a small share of what is published (15,881 of 18,091 captures went unseen in that run). Its loop does extra work between polls, which changes the timing against the writer; that is why it does not starve completely. Its record-rate thresholds hide this.
- **Not affected:** arming (the group's own gate reads the mailbox's counters, not `latest()`), the motor commands, and the ESC.

## Not yet checked

- Whether raising `LATEST_ATTEMPTS` removes it, or only moves the problem.
- Why one step in five worked, in both throttle orders.
- Whether a fixed 1 ms pace is safe for arming reliability in general. The arming evidence in CLAUDE.md (104/104) was gathered with maximum-rate pacing; the RPM test armed once per run at 1 ms pacing (2,108 ms) but this is not a batch.
- Whether the reader being slow comes from both cores fetching code from flash at the same time (the 2026-09 perf findings measured the interpreter about 10x slow).

## Options

1. **Document and pace.** State in CLAUDE.md that a telemetry-reading application should pace `update()` (about 1 ms), and change the usage example. No driver change; needs an arming batch at that pace before it becomes the recommendation.
2. **Make the read cheaper or more patient.** More attempts, or a copy that cannot be overtaken (for example two slots the reader alternates between). Changes `driver/`, so it needs the unit tests and a bench pass.
3. **Make the scenario runner measure it.** Add a threshold on the share of polls that returned `None`, so a regression shows.
