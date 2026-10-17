# Runs a JSON-defined scenario (see tests/bench/dshot_protocol/scenarios/) against all 4
# DShot channels through MotorGroup, exactly the way an application
# uses the library: the group's update() runs on Core 1 (core1_runner.py), and
# Core 0 arms the group, follows each motor's throttle profile with
# set_throttle(), and reads telemetry with raw_telemetry(). Raw captures go to
# the PicoBell's SD card - see tests/bench/scenario.py (loading/validation),
# tests/bench/throttle_profile.py (per-motor throttle curves), and
# tests/bench/session_recorder/bidir_capture_sink.py (SD writes).
#
# scripts/deploy.py --scenario <path> uploads the chosen scenario file to this
# fixed device-side name, since mpremote's `run` has no way to pass an extra
# file/argument into the running script.
#
# What a record is: the group keeps ONE capture per bidirectional motor (the
# latest), so this cannot log every reply. Core 0 polls raw_telemetry() and
# writes a record whenever a motor has a capture whose value differs from the last one seen.
# Captures published but never seen (the token stepped by more than one) are counted as missed;
# they are not an error, because reading the latest is the contract - the
# scenario's `expect` thresholds say how much sampling a run must achieve.
# Nothing is logged while the group is still arming (see arm_group()) - no
# per-capture diagnostics run during that window - so capture.bin, and
# everything judged on it, starts at ARMED; meta.txt's armed_ticks_us marks
# the boundary.
#
# Fail-fast, deliberately: a Core 1 error, a gap/rate violation of the
# scenario's own "expect" thresholds, a tripped reply failsafe (see
# check_reply_failsafe: a capture only says the receiver ran, not that the ESC
# answered, so a sustained run of all-zero captures is itself the anomaly), a
# capture handed out while the group is still arming, or Ctrl-C all still run the
# disarm/close sequence in `finally`, then propagate - this never swallows a
# failure into a "Test Complete" banner. A run that fails exits non-zero and marks
# outcome=failed in meta.txt, so a truncated capture.bin is never mistaken for a
# complete one by the PC-side analyzer.
#
# Decoding: every Nth new capture per motor (scenario `decode_every`) is decoded
# on the device with MotorGroup.decode_telemetry(), the way an application would
# on Core 0, and tallied (decode_tally.py) as a real reply, a CRC failure or not
# a reply at all. That sample is what the scenario's `min_crc_valid_pct` and
# `min_median_erpm` are checked against at the end of the run, so a run gives its
# own verdict without pulling the SD card. Decoding costs milliseconds, so it is
# sampled, never done for every capture. scripts/analyze_bidir_capture_log.py
# decodes everything logged on a PC, checks the same thresholds on all of it, and
# repeats the device's sampling to check the two decoders agree.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot
from motor_group import MotorGroup
from capture_mailbox import FRAME_BITS, FRAME_MASK, TOKEN_MAX
from core1_runner import Core1Runner, fast_stop
from scenario import load_scenario
from bidir_capture_sink import BidirCaptureSink
from decode_tally import DecodeTally, SpinWatch, is_sampled
from array import array
import gc
import utime

SCENARIO_PATH = "scenario.json"

# Grace window before the reply failsafe is armed. Deliberately short (not
# the 20s settling window check_expect's min_record_rate_hz needs, which
# is a CUMULATIVE-average threshold and genuinely needs one) - this checks
# for the mere existence of one real reply, and arming already finished
# before this function is ever called, so a healthy ESC's first real reply
# lands within milliseconds of run_start.
REPLY_FAILSAFE_GRACE_MS = 2000

# How long past the scenario's own arming window arming may take before it is
# an error: arming completes inside update() on Core 1, so a dead loop shows
# here, and MotorGroup also waits for 2s of replies from every bidirectional
# ESC - ~4.5s after arm() when an ESC reboots once while arming, ~7s twice,
# ~9.5s three times (bug-reports/BUG-003; BUG-002's "10-run arming-reliability
# sample" measured a ~2.46s reboot-cycle period and saw up to 4 reboots on one
# motor before it settled). Sized for 6 reboot cycles with margin, not the
# floor itself - MotorGroup.arm()'s duration_ms is left alone, since raising it
# only delays every healthy arm and was tried and found not to help (see the
# bug report's "bigger waiting time?" note).
ARM_TIMEOUT_MARGIN_MS = 16000

# How long a bidirectional motor commanded to spin (throttle >= 48) may go
# without one real eRPM decode before the run is stopped (decode_tally.SpinWatch).
# With the reply-gated ARMED transition (bug-reports/BUG-003), reaching ARMED
# already means the ESC was replying steadily - this catches the remaining
# case, where it replies but the motor never turns, or stops turning (BUG-002's
# signature; BUG-008). Every healthy spin-up on this bench starts well inside
# it. A continuous streak, not an "ever spun" flag: a single glitch frame does
# not defeat it for the rest of the run.
NOT_RUNNING_LIMIT_MS = 2000

def check_reply_failsafe(has_bidir, nonzero_records, elapsed_ms):
    # Bidirectional DShot's contract is a continuous eRPM reply every
    # command - unlike a rate/gap threshold, "the ESC has never once
    # replied" is not a tunable performance bar, it's a protocol violation,
    # so this fires unconditionally once armed rather than via the
    # scenario's own "expect" block.
    #
    # Words are non-zero only when the receiver saw the line move, so a run of
    # captures that are literally all-zero words is a dead bench/ESC that would
    # otherwise only show in a post-hoc tally, well after the fact.
    #
    # any-MOTOR, not per-motor: a live motor would mask a silent one.
    if not has_bidir:
        return
    if elapsed_ms >= REPLY_FAILSAFE_GRACE_MS and nonzero_records == 0:
        raise RuntimeError(
            "reply failsafe tripped: no ESC reply seen by " + str(elapsed_ms) +
            "ms elapsed (every captured record's words are still all-zero) "
            "- check ESC power/arming/bidir mode before continuing"
        )


def check_stuck_at_rest_failsafe(bidir_indices, spin_watches, throttles, elapsed_ms):
    """
    A bidirectional motor commanded to spin that has decoded no real eRPM for
    NOT_RUNNING_LIMIT_MS is BUG-002's own signature: CRC-valid replies, no
    error state, and no spin. This stops the run instead of waiting out its
    full duration for a result already decided.

    A motor deliberately held below throttle 48 (an idle bidirectional motor
    used as a second-line control) legitimately replies the same sentinel
    forever and cannot trip this: SpinWatch only counts while commanded.
    """
    for index in bidir_indices:
        spin_watches[index].note_throttle(throttles[index], elapsed_ms)
        if spin_watches[index].tripped(throttles[index], elapsed_ms):
            raise RuntimeError(
                "not-running failsafe tripped: motor " + str(index) + " has been commanded "
                "throttle " + str(throttles[index]) + " for " +
                str(spin_watches[index].stuck_ms(throttles[index], elapsed_ms)) +
                "ms with no real eRPM decoded - it is not going to start on its own this run "
                "(see bug-reports/BUG-002-bidirectional-motor-intermittently-does-not-spin.md)"
            )


def check_expect(expect, largest_gap_us, records, elapsed_ms):
    if not expect:
        return

    max_gap_ms = expect.get("max_gap_ms")
    if max_gap_ms is not None and largest_gap_us > max_gap_ms * 1000:
        raise RuntimeError(
            "largest_gap=" + str(largest_gap_us / 1000) + "ms > max_gap_ms=" + str(max_gap_ms)
        )

    min_rate = expect.get("min_record_rate_hz")
    # Give the rate a real settling window before enforcing it: this is a
    # CUMULATIVE average since run start, and early on that average is
    # dominated by startup jitter (arming's tail, the first few telemetry
    # replies syncing up) rather than the steady-state rate the threshold is
    # actually meant to police. 20s comfortably clears that noise while still
    # catching a genuine sustained rate problem well before a long hold ends.
    if min_rate is not None and elapsed_ms >= 20000:
        rate = records / (elapsed_ms / 1000)
        if rate < min_rate:
            raise RuntimeError(
                "record rate=" + str(rate) + "/s < min_record_rate_hz=" + str(min_rate)
            )


def check_decode_expect(expect, tallies):
    """The scenario's decode thresholds the sampled tallies miss, as messages."""
    failures = []
    crc = expect.get("min_crc_valid_pct", {})
    erpm = expect.get("min_median_erpm", {})
    for index in tallies:
        for message in tallies[index].check(crc.get(str(index)), erpm.get(str(index))):
            failures.append("motor " + str(index) + ": " + message)
    return failures


def build_motor(spec, dshot_speed):
    if spec.bidirectional:
        return BidirectionalDShot(spec.sm_id, Pin(spec.pin), dshot_speed,
                                  rx_state_machine_id=spec.rx_sm_id)
    return UnidirectionalDShot(spec.sm_id, Pin(spec.pin), dshot_speed)


def arm_group(group, scenario, runner, arm_times):
    """
    Arm and wait for the group to reach ARMED.

    No per-capture logging or diagnostics run during this window. BUG-002's
    arming.bin/measured() instrumentation, and MotorGroup's own reboot_log,
    were both retired 2026-10-06: real overhead on the exact loop whose timing
    this investigation cares about, measured at ~90us/call even once idle -
    see BUG-002.md's "Telemetry overhead" section. A real application calling
    update() directly never paid this cost, so this harness shouldn't either.

    This loop used to also re-verify, every tick, that raw_telemetry() never
    hands out a capture before ARMED. BUG-002, 2026-10-10: that check was
    itself found to be the dominant cause of this harness's own unreliable
    arming - its extra per-tick Core 0 work during arming measurably raised
    Core 1's own update() cost (not a GIL - MicroPython's RP2 port has none -
    the exact mechanism was never identified, see BUG-002.md's root-cause
    section), and removing it alone took a scenario that had been arming
    cleanly only 0-40% of the time back to 10/10. The invariant it checked is
    already guaranteed by
    MotorGroup.raw_telemetry() itself (`if self.state != ARMED: return None`)
    and already covered, with no hardware timing cost, by
    tests/pc/test_motor_group.py's `test_no_capture_is_handed_out_while_disarmed_or_arming`
    and `test_the_group_itself_refuses_captures_while_arming` - so this bench
    check added no safety value a real application's documented usage
    (CLAUDE.md's `while not is_armed(): sleep_ms(10)`, which never calls
    raw_telemetry() during arming at all) doesn't already have for free.

    `arm_times` still gets the ticks_us of arm() as soon as it is called and
    of ARMED once reached - "Armed after Nms" and the clean/reboot KPI
    classification need nothing else.
    """
    print("Arming for {}ms...".format(scenario.arm_duration_ms))
    gc.collect()  # start the window with a clean heap: fewer collections pausing Core 1 while arming
    arm_times[0] = utime.ticks_us()
    group.arm(scenario.arm_duration_ms)
    arm_start = utime.ticks_ms()
    arm_timeout_ms = scenario.arm_duration_ms + ARM_TIMEOUT_MARGIN_MS
    while not group.is_armed():
        if runner.error is not None:
            raise runner.error
        if utime.ticks_diff(utime.ticks_ms(), arm_start) > arm_timeout_ms:
            raise RuntimeError("arming did not complete within " + str(arm_timeout_ms) +
                               "ms; per motor (replying_for_ms, last_reply_ms_ago), None = no reply: " +
                               str(group.arming_status()))
        utime.sleep_ms(1)
    arm_times[1] = utime.ticks_us()
    print("Armed after {}ms.".format(utime.ticks_diff(arm_times[1], arm_times[0]) // 1000))
    print()


def run_scenario():
    print("=== Scenario Capture ===")

    scenario = load_scenario(SCENARIO_PATH)
    print("Loaded scenario: duration={}ms dshot_speed={} bidir_motors={} core1_interval_us={}".format(
              scenario.duration_ms, scenario.dshot_speed, scenario.bidir_indices,
              scenario.core1_interval_us))

    sink = BidirCaptureSink()
    sink.init_session(scenario, SCENARIO_PATH)
    print("Session:", sink.path)
    print()

    group = None
    runner = None
    bidir_indices = scenario.bidir_indices
    has_bidir = bool(bidir_indices)
    total_records = 0
    total_nonzero_records = 0
    last_value = [0, 0, 0, 0]  # per motor: the capture value last seen (0 = none yet)
    published = [0, 0, 0, 0]   # per motor: captures published since ARMED, from the token's steps
    missed = 0                 # captures published but never seen
    last_record_us = None
    largest_gap_us = 0
    outcome = "failed"
    arm_times = [None, None]  # ticks_us of arm() and of ARMED, filled in by arm_group()
    tallies = {i: DecodeTally() for i in bidir_indices}
    seen = [0, 0, 0, 0]      # per motor: non-empty captures seen, for the sampling rule
    spin_watches = [SpinWatch(NOT_RUNNING_LIMIT_MS) for _ in range(4)]  # per motor, see SpinWatch
    failures = []
    # Set for real once the running loop starts (below); 0 here covers a run
    # that fails before reaching it, so finally's status-printing loop is a
    # no-op rather than a NameError that would mask the real failure
    status_count = 0

    try:
        group = MotorGroup([build_motor(spec, scenario.dshot_speed) for spec in scenario.motors])
        interval_us = scenario.core1_interval_us
        if interval_us is None:
            interval_us = group.UPDATE_INTERVAL_US
        runner = Core1Runner(group.update, interval_us)
        runner.start()

        arm_group(group, scenario, runner, arm_times)

        print("Running scenario for {}ms...".format(scenario.duration_ms))
        run_start = utime.ticks_ms()
        last_status_ms = run_start
        # Reused every iteration below instead of allocating a fresh list each
        # pass - this loop runs for the whole scenario at poll_ms cadence
        words = [0, 0, 0, 0]

        # Status snapshots are stored as raw numbers in preallocated arrays
        # while the loop runs - no string formatting, no print(), no file
        # I/O - and only formatted/printed once the loop has ended (see
        # below). Any COM-port or SD-card write during this loop is a Core 0
        # cost that can stall Core 1's deadline schedule (advisor's
        # 2026-10-09 performance review found status_interval_ms's own old
        # print() lining up almost exactly with schedule_behind_count's
        # fall-behind timestamps - see bidirectional_dshot_review.md's W32/
        # W34); the fix is not WHERE the write goes, it's that there is no
        # write at all until after the window closes. Sized from the
        # scenario's own already-known duration/interval, not a tuned
        # constant, with one slot of headroom.
        status_capacity = scenario.duration_ms // scenario.status_interval_ms + 1
        status_count = 0
        status_elapsed_ms = array('I', [0] * status_capacity)
        status_records = array('I', [0] * status_capacity)
        status_nonzero = array('I', [0] * status_capacity)
        status_missed = array('I', [0] * status_capacity)
        status_gap_us = array('I', [0] * status_capacity)

        while True:
            elapsed_ms = utime.ticks_diff(utime.ticks_ms(), run_start)
            if elapsed_ms >= scenario.duration_ms:
                break

            if runner.error is not None:
                raise runner.error

            for index, spec in enumerate(scenario.motors):
                group.set_throttle(index, spec.profile.throttle_at(elapsed_ms))

            throttles = group.get_all_throttles()

            # One record per pass in which any motor has a capture it has not
            # shown before; the other motors' word slots stay zero
            for i in range(4):
                words[i] = 0
            new_capture = False
            for index in bidir_indices:
                capture = group.raw_telemetry(index)
                if capture is None or capture == last_value[index]:
                    continue
                # The token steps by one per publish and wraps at TOKEN_MAX, so the
                # step since the last capture seen counts the captures published in
                # between (exact while fewer than TOKEN_MAX arrive between two polls,
                # about 146ms at the maximum rate - a longer gap also fails the
                # scenario's own max_gap_ms). The same token with a new frame is a
                # whole lap.
                step = ((capture >> FRAME_BITS) - (last_value[index] >> FRAME_BITS)) % TOKEN_MAX
                if step == 0:
                    step = TOKEN_MAX
                published[index] += step
                missed += step - 1
                last_value[index] = capture
                frame = capture & FRAME_MASK
                words[index] = frame
                new_capture = True
                if frame:
                    seen[index] += 1
                    if is_sampled(seen[index], scenario.decode_every):
                        result = group.decode_telemetry(index, capture)
                        tallies[index].add(result)
                        if result is not None and result["crc_ok"] and not result["not_running"]:
                            spin_watches[index].note_spin(elapsed_ms)

            if new_capture:
                # No timestamp travels with a capture: the record is stamped when
                # Core 0 sees the value change, at the resolution of its own poll
                record_us = utime.ticks_us()
                total_records += 1
                if any(words):
                    total_nonzero_records += 1
                sink.write_record(record_us, throttles, words)
                if last_record_us is not None:
                    gap = utime.ticks_diff(record_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = record_us

            check_reply_failsafe(has_bidir, total_nonzero_records, elapsed_ms)
            check_stuck_at_rest_failsafe(bidir_indices, spin_watches, throttles, elapsed_ms)
            check_expect(scenario.expect, largest_gap_us, total_records, elapsed_ms)

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_status_ms) >= scenario.status_interval_ms:
                last_status_ms = now
                # Numbers only - no .format(), no print(), no file write.
                # See this loop's own setup comment above for why.
                if status_count < status_capacity:
                    status_elapsed_ms[status_count] = utime.ticks_diff(now, run_start)
                    status_records[status_count] = total_records
                    status_nonzero[status_count] = total_nonzero_records
                    status_missed[status_count] = missed
                    status_gap_us[status_count] = largest_gap_us
                    status_count += 1

            utime.sleep_ms(scenario.poll_ms)

        outcome = "completed"
        failures = check_decode_expect(scenario.expect, tallies)
        print()
        print("Scenario duration complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")
        raise

    finally:
        print("Stopping...")
        # fast_stop(): throttle 0 goes out while the loop still runs (disarm()
        # transmits nothing, so without this a motor keeps its last throttle
        # until the ESC's own timeout), then the loop is stopped before
        # disarm() touches the state machines, so the two cannot race.
        loop_exited = True
        if runner is not None and group is not None:
            loop_exited = fast_stop(group, runner)
        else:
            if runner is not None:
                loop_exited = runner.stop()
            if group is not None:
                group.disarm()
        if not loop_exited:
            # disarm() ran while Core 1 may still have been inside update():
            # mark the run so it cannot pass as clean (BUG-007).
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")
            outcome = "failed: Core 1 loop did not stop"
        print("Motors stopped and disarmed.")

        # Formatted and printed only now - runner.stop() above already
        # returned, so Core 1 is idle and nothing here can affect its
        # pacing. The loop above only ever stored raw numbers.
        for i in range(status_count):
            elapsed_s = status_elapsed_ms[i] / 1000
            rate = status_records[i] / elapsed_s if elapsed_s else 0.0
            print("  [{:7.1f}s] records={} nonzero={} missed={} rate={:.1f}/s largest_gap={:.1f}ms".format(
                elapsed_s, status_records[i], status_nonzero[i], status_missed[i],
                rate, status_gap_us[i] / 1000))

        published_by_motor = {i: published[i] for i in bidir_indices}
        # The thresholds are judged on a run that reached its end; one that was
        # cut short (a gap or rate threshold, a Core 1 error) has no verdict
        if outcome != "completed":
            verdict = "not evaluated: the run did not complete"
        elif failures:
            verdict = "fail: " + "; ".join(failures)
        else:
            verdict = "pass"
        extra = {}
        if runner is not None:
            # >0 means the deadline schedule fell behind and had to skip
            # forward at least once this run - the loop's own cost exceeded
            # core1_interval_us for at least one tick - see Core1Runner's
            # own note on this counter.
            extra["schedule_behind_count"] = runner.schedule_behind_count
        if arm_times[0] is not None:
            # Time 0 for classify_reply_timeline.py --from-arm, on a run that
            # never armed as well: that is the run whose arming log matters most
            extra["arm_ticks_us"] = arm_times[0]
            if runner is not None and runner.schedule_behind_count:
                # When (ms since arm()) each of the first few fall-behind
                # events happened - BUG-002, 2026-10-08: testing whether
                # these land disproportionately inside AM32's narrow
                # early-boot calibration window, not just how often they
                # happen. Same clock as arm_ticks_us.
                filled = min(runner.schedule_behind_count, runner.BEHIND_AT_CAPACITY)
                extra["schedule_behind_at_ms_since_arm"] = ",".join(
                    str(utime.ticks_diff(runner.behind_at_us[i], arm_times[0]) // 1000)
                    for i in range(filled))
        if arm_times[1] is not None:
            # Time 0 for scripts/classify_reply_timeline.py
            extra["armed_ticks_us"] = arm_times[1]
        sink.finalize(outcome, total_records, missed, largest_gap_us, published_by_motor, tallies, verdict, extra)
        sink.close()
        print("SD card flushed and unmounted.")

        # Printed here, inside finally, so it shows up on a failed run too -
        # not just after a clean completion (in which case control also
        # reaches here, then continues normally once finally exits).
        print()
        print("=== Summary ===")
        print("Session:", sink.path)
        print("Outcome:", outcome)
        print("Records (captures seen for the first time):", total_records)
        print("Records with a real (non-all-zero) reply:", total_nonzero_records)
        print("Captures published (counted from the token steps, per motor):", published_by_motor)
        print("Captures published but never seen:", missed)
        print("Largest gap between records: {:.1f}ms".format(largest_gap_us / 1000))
        if runner is not None and runner.schedule_behind_count:
            print("WARNING: the deadline schedule fell behind {} time(s) - update() cost "
                  "exceeded core1_interval_us for at least that many ticks".format(
                      runner.schedule_behind_count))
        for index in tallies:
            print("Motor {} decoded on the device (every {}th capture): {}".format(
                index, scenario.decode_every, tallies[index].summary()))
        for message in failures:
            print("  expectation missed - " + message)
        print("=== Test Complete ===" if outcome == "completed" and not failures else "=== Test FAILED ===")

    if failures:
        raise RuntimeError("decode expectations missed: " + "; ".join(failures))


run_scenario()
