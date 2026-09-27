# BUG-008: Harness health checks cannot see a silent ESC, a Core 1 stall, or "commanded but not running"

**Status:** OPEN - fix proposed, not implemented
**Severity:** Medium (diagnostics). BUG-002-type failures pass the harness's own checks and are
found only by reading logs afterwards.
**Component:** `tests/harness/run_scenario.py`, `tests/harness/scenarios/*.json`, `tests/harness/scenario.py`
**Found:** 2026-09-27, code review

## Summary

1. **The reply failsafe cannot trip with the frame receiver.**
   - `check_reply_failsafe()` (`run_scenario.py:69-88`) trips only if every captured record is
     all-zero words.
   - When the ESC does not reply, `dshot_bidir_rx_frame` falls through to our own next frame and
     pushes it, and that capture is non-zero. So a completely silent ESC keeps
     `total_nonzero_records` climbing and the failsafe never fires.
   - It is also any-motor, not per-motor: one live motor masks a silent one.
2. **Core 1 stalls are measured but never judged.**
   - `measured()` records the longest gap between `update()` calls, and the run prints it and
     stores it as `max_loop_gap_us` in the session summary.
   - But **no scenario sets `expect.max_loop_gap_ms`**, so `check_expect()` never checks it.
   - A gap of ≥0.5 s makes an armed AM32 reset itself
     ([`main.c#L1992-L2017`][am32-timeouts]), and the result looks exactly like BUG-002.
3. **"Commanded but not running" is only caught indirectly**, through `min_median_erpm`. A motor
   whose replies are all `not_running` (`0xFFF`) while its commanded throttle is ≥48 is BUG-002's
   exact signature. It deserves its own failure message, and it should fail as soon as it happens
   rather than at the end.

## Fix plan

1. **Per-motor, CRC-based failsafe.** Replace the non-zero-words test with "each bidirectional
   motor has at least one CRC-valid reply by `REPLY_FAILSAFE_GRACE_MS`". Use the sampled decodes
   the tallies already make. With BUG-003 in place, use its `ready_first_ms` instead.
   CRC alone is not enough. At some throttles (227 of 2,048 at DSHOT600, e.g. 67, 72, 93, 98),
   the receiver's capture of our *own* frame passes CRC when the ESC is silent. Count a reply only
   if it is also not one of those echo words (see `scripts/classify_reply_timeline.py` and
   BUG-002's classification section). On the device, a per-throttle lookup of the few echo words
   is cheap.
2. **`max_loop_gap_ms` in every scenario** (e.g. 50 ms). Make `scenario.py` require it, like the
   other fail-fast fields. Also check the stored `max_loop_gap_us` of the existing BUG-002
   sessions; that data is already on disk.
3. **Not-running check.** For each bidirectional motor, fail when its sampled decodes have been
   `not_running` continuously for more than N seconds while its profile throttle is ≥48. Suggested
   N = 3 s, because a start can take a moment. The message should name BUG-002.
4. **Keep arming-phase captures in the log**, flagged and not handed to the application
   (BUG-003 step 2). They let the offline analyser show what the ESC did before `ARMED`.

## Verification

- Unit (`tests/unit/test_scenario.py`, `test_decode_tally.py`):
  - a silent ESC, modelled as non-zero CRC-failing words only, trips the failsafe per motor;
  - a scenario without `max_loop_gap_ms` fails to load;
  - a tally of `not_running` under throttle ≥48 fails after N seconds.
- Bench: a BUG-002 run now ends with the not-running failure, instead of passing its live checks.

## Related

- BUG-002
- BUG-003

[am32-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017
