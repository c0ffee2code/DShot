# BUG-008: Harness health checks cannot see a silent ESC, a Core 1 stall, or "commanded but not running"

**Status:** PARTLY FIXED 2026-10-13 (items 2 and 3 done; healthy path bench-verified on two scenarios, failing path unit-tested only; items 1 and 5 still open - see "Update 2026-10-13")
**Severity:** Medium (diagnostics). BUG-002-type failures pass the harness's own checks and are
found only by reading logs afterwards.
**Component:** `tests/harness/run_scenario.py`, `tests/harness/scenarios/*.json`, `tests/harness/scenario.py`
**Found:** 2026-09-27, code review

## Summary

1. **The reply failsafe cannot trip with the frame receiver.**
   - `check_reply_failsafe()` (`run_scenario.py`) trips only if every captured record is
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

## Update 2026-10-13

- **Item 3 (not-running check): done.** `decode_tally.SpinWatch` replaces the permanent
  `real_spin_seen` latch. A bidirectional motor commanded at throttle 48 or above must decode one
  real eRPM every `NOT_RUNNING_LIMIT_MS` (2000 ms; the existing, bench-justified spin-up grace, not
  the 3 s first suggested below). It is a continuous streak, so a single glitch frame (BUG-009)
  restarts the clock once and cannot defeat the check for the rest of the run.
- **Item 2 (loop gap): done in a different form.** The per-tick loop-gap measurement this item
  assumed was retired with the rest of the per-tick instrumentation (BUG-002, 2026-10-06), so there
  is no `max_loop_gap_us` to judge. The check that exists is `expect.max_gap_ms`, the largest gap
  between records; `scenario.py` now requires it for every scenario with a bidirectional motor
  (the two unidirectional scenarios produce no records and are exempt).
- **Items 1 and 5 remain open:** the per-motor, echo-aware silent-ESC failsafe, and the
  reboot-after-`ARMED` flag.

## Fix plan

1. **Per-motor, CRC-based failsafe.** Replace the non-zero-words test with "each bidirectional
   motor has at least one CRC-valid reply by `REPLY_FAILSAFE_GRACE_MS`". Use the sampled decodes
   the tallies already make. With BUG-003 in place, use its `ready_last_us` instead.
   CRC alone is not enough. At some throttles (227 of 2,048 at DSHOT600, e.g. 67, 72, 93, 98),
   the receiver's capture of our *own* frame passes CRC when the ESC is silent. Count a reply only
   if it is also not one of those echo words (see `scripts/classify_reply_timeline.py`'s
   `EchoModel`, which computes them directly - no need to go via BUG-002's history for this). On
   the device, a per-throttle lookup of the few echo words is cheap.
2. **`max_loop_gap_ms` in every scenario** (e.g. 50 ms). Make `scenario.py` require it, like the
   other fail-fast fields. Also check the stored `max_loop_gap_us` of the existing BUG-002
   sessions; that data is already on disk.
3. **Not-running check.** For each bidirectional motor, fail when its sampled decodes have been
   `not_running` continuously for more than N seconds while its profile throttle is ≥48. Suggested
   N = 3 s, because a start can take a moment. The message should name BUG-002.
4. **Keep arming-phase captures in the log.** Done 2026-09-27: `MotorGroup.publish_while_arming`
   plus the harness's `arming.bin`, shown at negative times by `classify_reply_timeline.py`
   (BUG-002, R6).
5. **Flag a reboot after `ARMED`.** Fail when a bidirectional motor's words are all-zero (the
   line held low, AM32's startup tune) for more than ~300 ms. That is a reset, and one
   BUG-003's gate cannot catch once `ARMED` has passed.

## Confirmed instance, 2026-10-10 (BUG-009): a single glitch frame defeats the current check

*As of 2026-10-13 the permanent latch described here has been replaced (see "Update 2026-10-13" above); this section records the instance that motivated it.*

`run_scenario.py`'s `check_stuck_at_rest_failsafe` (the check that exists today, ahead of this
file's fix plan) sets `real_spin_seen[index] = True` the first time a decoded capture has `crc_ok`
true and `not_running` false, and **never resets it**. BUG-009 found a reproducible, CRC-valid,
non-sentinel glitch value (`data12=2`, decoding to a physically absurd `erpm=30,000,000`) - one
occurrence permanently marks that motor "confirmed spinning" for the rest of the run, even if it
then genuinely never spins (or stops) again. This is a confirmed, concrete instance of exactly the
gap this file already describes in general terms.

Note for whoever implements item 3's fix: a *continuous*-`not_running`-for-N-seconds check, unlike
the current permanent latch, is not defeated the same way - one glitch frame resets that streak, but
it starts counting again immediately and still fires N seconds later if the motor is truly stuck.
The latch's bug is permanence, not the one-glitch trigger itself; item 3's design already avoids it.

## Verification

- Unit (`tests/unit/test_scenario.py`, `test_decode_tally.py`):
  - a silent ESC, modelled as non-zero CRC-failing words only, trips the failsafe per motor;
  - a scenario without `max_loop_gap_ms` fails to load;
  - a tally of `not_running` under throttle ≥48 fails after N seconds.
- Bench: a BUG-002 run now ends with the not-running failure, instead of passing its live checks.

## Related

- BUG-002
- BUG-003
- BUG-009 - confirmed instance of the gap this file's item 3 (and its proposed fix) has.

[am32-timeouts]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/main.c#L1992-L2017
