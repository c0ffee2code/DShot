# BUG-007: `Core1Runner.stop()` can return while Core 1 is still inside `update()`

**Status:** FIXED 2026-10-13 (bench-verified, see the update below)
**Severity:** Medium. It reopens the `disarm()`/`update()` race that CLAUDE.md tells callers to
avoid by stopping the loop first.
**Component:** `tests/harness/core1_runner.py` (example application code that projects copy),
`driver/dshot_pio.py` (`UnidirectionalDShot` start/stop)
**Found:** 2026-09-27, code review

## Summary

**Update 2026-10-13:** the stale-frame half is closed in the driver: every `stop()` and `start()` re-initialises the state machines, so a word queued behind `disarm()` is discarded at the next start for both motor types (unit-tested; on-device check: `tests/device/test_stop_clears_queues.py`). The `Core1Runner.stop()` early return is fixed too: `stop()` waits up to 200 ms for the loop to exit, returns True/False, and sets `error` on a timeout; the scenario runner marks the run failed and the device tests raise on False. Unit-tested with a loop that takes longer than the old 4 ms grace.

`stop()` clears `running`, then polls `stopped` for `(interval_us + STOP_GRACE_US) // POLL_US`
iterations (`Core1Runner.stop()`'s poll loop). That is **4 ms** at the library's `UPDATE_INTERVAL_US = 0`.
It then **returns silently whether or not the loop has exited.** ADR-002 measured garbage-collection
stalls of the command loop of up to 14 ms, longer than that grace.

When that happens, `run_scenario.py` and the device tests call `group.disarm()` while Core 1 is
still inside `update()`:

- **A late throttle frame after the zeros.** An `update()` already past its state check can queue
  a non-zero frame after `disarm()`'s zeros. That frame is the last one the ESC sees, so the motor
  keeps spinning until AM32's 0.5 s signal-loss reset.
- **A stale frame carried to the next run.** A frame put after `disarm()`'s `drain()` stays in the
  TX FIFO. `UnidirectionalDShot.start()` only reactivates the state machine; `stop()`'s
  `restart()` does not clear the FIFO. So that stale frame, possibly a non-zero throttle, is the
  first one sent at the next `arm()`. `BidirectionalDShot.start()` re-inits and is not affected.

## Fix plan

1. **`Core1Runner.stop(timeout_ms=200)`**:
   - poll `stopped` until the loop has actually exited, or the timeout passes;
   - return `True`/`False`, and set `self.error` to a descriptive exception on timeout.
   - Keep the poll-not-sleep approach.
2. **Callers** (`run_scenario.py`, `tests/device/*.py`) check the result. They still call
   `disarm()` (stopping the motors comes first), but report the race loudly so the run is marked.
3. **`UnidirectionalDShot.start()`** re-initialises its state machine, as `BidirectionalDShot.start()`
   does, so no queued word survives a `stop()`.
4. **CLAUDE.md usage example:** show checking `runner.stop()`'s result.

## Verification

- Unit: a fake runner whose `update()` blocks longer than the old 4 ms grace; `stop()` must wait,
  or return `False` after the timeout.
- Unit: `UnidirectionalDShot` start after stop with a queued word, using fakes that model the FIFO.
  The word must not be transmitted.
- Device: `tests/device/test_pio_lifecycle.py`-style check with a forced `gc.collect()` on Core 1
  during `stop()`.

## Related

- BUG-006
- ADR-004 (client-owned loop, shutdown order)
