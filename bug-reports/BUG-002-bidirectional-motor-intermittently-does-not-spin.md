# BUG-002: A bidirectional motor sometimes doesn't spin despite valid telemetry

**Status: RESOLVED.** (BUG-009, opened afterward, is a different, still-open problem - see Related.)

## The symptom and its fix

A bidirectional motor would occasionally stay silent - never spinning - while its ESC kept
replying with valid, CRC-good telemetry. Root cause: arming was purely timer-based. An ESC that
reset mid-arm-window (AM32 reboots roughly every 2.6s while its line is held low) could come back
*after* the timer had already fired, under throttle, and never satisfy its own zero-throttle
arming gate afterward. Fixed by BUG-003: `ARMED` is now evidence-gated (every bidirectional ESC
must reply "not running" steadily for `READY_SPAN_MS` with no gap over `READY_GAP_MS`), not just a
timer. See BUG-003.md for the fix itself - not retold here. Verified on 28 post-fix runs (09-27):
every reset seen came from an ESC that had never replied once.

**Historical record:** across the first 284 bench runs (through 2026-10-01), refusals (arming
timing out) totaled 10 (3.5%). None of those 284 runs ever armed a motor that then failed to spin.
The same symptom did reappear once, 2026-10-10, under BUG-009 - confounded by a hardware change on
the same channel, not re-examined here.

## The remaining problem: arming itself was unreliable

Even with BUG-003 in place, arming itself was unreliable for weeks - ESCs rebooting mid-arm,
sometimes several times, sometimes never completing - driving a long investigation into Core 1's
command-loop pacing (a sleep-after-work pad vs. a deadline-anchored scheduler). That pacing work
was real, and the deadline scheduler (`Core1Runner`, `interval_us > 0`) is still what ships - kept
on its own merits (no magic constant, matches Betaflight's documented behavior - see
`Core1Runner`'s own docstring for that citation) - but its effect on *arming reliability*
specifically was never cleanly measured, because of what follows. The pad-based mode was removed
from the codebase entirely 2026-10-09, on principle: it only ever worked by retuning a magic
constant to one specific measurement of `update()`'s own cost.

## Root cause, found 2026-10-10

**`tests/harness/run_scenario.py`'s `arm_group()` had a per-tick safety check** - calling
`raw_telemetry()` on every bidirectional motor, every ~1ms during arming, to re-verify "no capture
is ever handed out before ARMED." The chain, by confidence:

1. *(code)* That check ran on Core 0, concurrently with Core 1's `update()` loop.
2. *(code, corrected 2026-10-11)* Not an interpreter lock - MicroPython's RP2 port has none
   (`MICROPY_PY_THREAD_GIL=0`, confirmed at the bench's own v1.28.0 and at `master`); the two
   cores genuinely run Python bytecode in parallel (see ADR-005). Exactly how Core 0's extra
   per-tick polling raised Core 1's own `update()` cost was not identified - candidates, not
   confirmed, and not mutually exclusive:
   - Contention on MicroPython's cross-core GC/allocation mutex (`py/gc.c`'s `gc_mutex`).
     `raw_telemetry()` returns a fresh tuple on every poll, so this mutex is plausible even
     though `update()`'s own hot path was separately tuned to near-zero allocation.
   - A full garbage-collection pause: ADR-002 measured that a collection triggered on either
     core pauses both, for up to several milliseconds. This would need to fire often enough,
     and for long enough each time, to shift a ~530us median by the observed 30-85us - not
     shown either way.
   - RP2's shared-bus access contention between the two cores.

   Steps 3 and 6 below - the measured cost and the causal isolation test - don't depend on which
   mechanism it is.
3. *(measured)* `update()`'s cost: ~530us baseline vs. ~560-615us with the check active (9 runs).
4. *(measured, correlational)* Cost increase correlated with worse arming outcomes (r≈0.67).
5. *(inferred, not independently verified at the firmware level)* that cost landing inside AM32's
   narrow one-shot frame-timing calibration window is what disrupted it often enough to look like a
   driver bug.
6. *(confirmed, causal - the strongest evidence)* **Isolation test:** a standalone tool with
   `arm_group()`'s own SD/RTC setup but a bare `is_armed()` poll scored 9/9 clean. Copying in only
   `arm_group()`'s exact polling loop (nothing else changed) dragged that same tool to 4/10 - the
   harness's own bad rate at the time (3/10, same session). That is direct causal evidence, not
   just a correlation.

**Fix:** removed the check entirely, not slowed down. It added no safety value the code didn't
already guarantee elsewhere - `MotorGroup.raw_telemetry()` itself returns `None` unless `ARMED`,
already covered by zero-cost PC unit tests, and the documented production usage
(`arm()` then poll `is_armed()`) never called `raw_telemetry()` during arming at all.

**Evidence:** the affected scenario, which had scored 7/50 (14%) clean across this investigation's
full log, went to 20/20 clean across two independent post-fix batches (p≈5.5e-12), and generalized
cleanly across three more configurations (a different pacing mode, a different DShot speed, a
frame-spacing diagnostic) - 50/50 clean, no exceptions. Later the same day, after unrelated
performance changes (W33, `@micropython.native` - see `bidirectional_dshot_review.md`), two more
spot checks still armed clean: the same scenario 13/13, and a two-motor 30-second scenario 30/30
(arming classification only - not separately judged on spin by full decode).

## Telemetry overhead and the timed window

Two more things were found and fixed the same week, in the same spirit as the root cause above,
though neither has a *measured* reliability effect the way the fix does:

- **Per-tick arming instrumentation** (`measured.py`, `arming.bin` logging) cost Core 1 itself
  ~87-97us per `update()` call, even once idle - real, unnecessary overhead on the exact loop this
  investigation measures. Removed entirely 2026-10-06 rather than kept "just in case."
- **The harness's periodic status print** used to run live, during the scenario's own timed
  window; moved to in-memory buffering, formatted and printed only after the run ends. This cut
  `schedule_behind_count`'s fall-behind rate (91 -> 40 per run), but the clean-arming rate itself
  did not move measurably (p≈0.21) - a real cost reduction, not a confirmed reliability fix.

The forward-looking rule, stated once: nothing extra should run on either core during a timed
window whose own results are about to be judged. Only the first case above was shown to matter for
arming reliability specifically.

## Caveat for anything measured before the fix

The `arm_group()` check predates the whole investigation - present in every batch this file ever
reported, on *both* sides of every comparison made (pad vs. deadline, threshold scans, the burst-bug
retest). The cutoff is functional, not a timestamp: any run made while the check was still
present in the code actually on the bench. Per this project's own rule that commit timestamps lag
bench-verified code, that is *not* simply "before `1d23629`'s commit time" - that same commit
includes its own post-fix confirmation sessions, which necessarily predate it, since the fix was
bench-tested first. Absolute reliability figures from before the check was
actually removed should be read as potentially harness-inflated - including this file's own
"periodic status print" clean-arming-rate figure above, from a batch committed 08:59 that same
day, well before the fix; relative rankings between mechanisms may still hold, but weren't as
clean as they looked at the time. None of that historical work was re-run. The full day-by-day log, including
everything pruned from this distillation, was deliberately not kept - by explicit user decision, in
preparation for merging this branch (whose history is squashed at merge anyway). This summary is
the permanent record.

## Related

- BUG-001 - ruled out as cause or fix.
- BUG-003 - the fix for the original silent-non-spin symptom.
- BUG-004 - implemented, bench-verified, does not independently clear this bug.
- BUG-008 - harness safety-net gaps found during this and the follow-up investigation.
- BUG-009 - a separate, still-open investigation into intermittent "rough" motor starts, opened
  2026-10-10 after this bug's root cause was fixed. Not the same problem; do not conflate the two.
