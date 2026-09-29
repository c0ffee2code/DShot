# Runs a JSON-defined scenario (see tests/harness/scenarios/) against all 4
# DShot channels through MotorGroup, exactly the way an application
# uses the library: the group's update() runs on Core 1 (core1_runner.py), and
# Core 0 arms the group, follows each motor's throttle profile with
# set_throttle(), and reads telemetry with raw_telemetry(). Raw captures go to
# the PicoBell's SD card - see tests/harness/scenario.py (loading/validation),
# tests/harness/throttle_profile.py (per-motor throttle curves), and
# tests/harness/bidir_capture_sink.py (SD writes).
#
# scripts/deploy.py --scenario <path> uploads the chosen scenario file to this
# fixed device-side name, since mpremote's `run` has no way to pass an extra
# file/argument into the running script.
#
# What a record is: the group keeps ONE capture per bidirectional motor (the
# latest), so this cannot log every reply. Core 0 polls raw_telemetry() and
# writes a record whenever a motor has a capture with a new sequence number.
# Captures published but never seen (the sequence jumped) are counted as missed;
# they are not an error, because reading the latest is the contract - the
# scenario's `expect` thresholds say how much sampling a run must achieve.
# Captures taken while the group is still arming go to a separate arming.bin in
# the same format (see arm_group()), so capture.bin, and everything judged on
# it, still starts at ARMED; meta.txt's armed_ticks_us marks the boundary.
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
from core1_runner import Core1Runner
from scenario import load_scenario
from bidir_capture_sink import BidirCaptureSink
from decode_tally import DecodeTally, is_sampled
import gc
import utime
from array import array

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

# Grace window, after ARMED, before a bidirectional motor commanded to nonzero
# throttle that has never once decoded a real (non-"not running") eRPM is
# treated as stuck rather than still starting up. With the reply-gated ARMED
# transition (bug-reports/BUG-003), reaching ARMED already means the ESC was
# replying steadily - this catches the remaining case, where it replies but
# the motor still never turns (per the user: "if it continuously returns this
# eRPM constant - no point to run the test scenario, motors won't spin").
# Matches REPLY_FAILSAFE_GRACE_MS - every healthy spin-up on this bench starts
# well inside it.
STUCK_AT_REST_GRACE_MS = 2000

RECORD_ZERO_WORDS = (0,)


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


def check_stuck_at_rest_failsafe(bidir_indices, real_spin_seen, throttles, elapsed_ms):
    """
    A bidirectional motor commanded to nonzero throttle that has never once
    decoded a real eRPM is BUG-002's own signature: CRC-valid replies, no
    error state, and no spin, ever. Past the grace window this stops the run
    instead of waiting out its full duration for a result already decided.

    A motor deliberately held at 0 throttle (an idle bidirectional motor used
    as a second-line control - see two_channel_bidir_one_idle_600) legitimately
    replies the same sentinel forever and must not trip this - only checked
    for motors with nonzero commanded throttle right now.
    """
    if elapsed_ms < STUCK_AT_REST_GRACE_MS:
        return
    for index in bidir_indices:
        if throttles[index] > 0 and not real_spin_seen[index]:
            raise RuntimeError(
                "stuck-at-rest failsafe tripped: motor " + str(index) + " has been commanded a "
                "nonzero throttle for " + str(elapsed_ms) + "ms since ARMED and every decoded "
                "reply is still AM32's not-running sentinel (917 eRPM) - it is not going to start "
                "on its own this run (see bug-reports/BUG-002-bidirectional-motor-intermittently-does-not-spin.md)"
            )


def check_expect(expect, largest_gap_us, records, elapsed_ms, loop_gap_us):
    if not expect:
        return

    # The longest gap between two update() calls, measured on Core 1 itself -
    # checked against how long the ESC tolerates a stalled command loop before
    # disarming on its own. See CLAUDE.md's disarm() notes for that timeout figure.
    max_loop_gap_ms = expect.get("max_loop_gap_ms")
    if max_loop_gap_ms is not None and loop_gap_us > max_loop_gap_ms * 1000:
        raise RuntimeError(
            "command loop gap " + str(loop_gap_us / 1000) + "ms > max_loop_gap_ms=" + str(max_loop_gap_ms)
        )

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


def measured(update, times, group, arming_stats):
    """
    Wrap `update` so Core 1 records:
    - times[1]: the longest gap between any two calls, for the whole run
      (times[0] is the last call's time).
    - arming_stats = [count, min_us, max_us, sum_us]: how densely update()
      actually ran while ARMING - one call sends one frame to every motor, so
      this is directly how often we sent frames during the window BUG-002's
      bisection runs care about, not an average inferred from replies (see
      the doc's "what are the delays between DShot packets" answer - the
      capture log samples too coarsely for this).

    Both share one ticks_us() call and one gap computation; the arming branch
    only adds a state check and a few integer operations when it is arming.
    Allocates nothing. Measuring on Core 1 is the point: a garbage collection
    pauses both cores, so Core 0 only ever sees the loop after it has caught up.
    """
    ticks_us = utime.ticks_us
    ticks_diff = utime.ticks_diff
    is_arming = group.is_arming

    def wrapper():
        now = ticks_us()
        last = times[0]
        if last:
            gap = ticks_diff(now, last)
            if gap > times[1]:
                times[1] = gap
            if is_arming():
                arming_stats[0] += 1
                if arming_stats[1] == 0 or gap < arming_stats[1]:
                    arming_stats[1] = gap
                if gap > arming_stats[2]:
                    arming_stats[2] = gap
                arming_stats[3] += gap
        times[0] = now
        update()

    return wrapper


def build_motor(spec, dshot_speed):
    if spec.bidirectional:
        return BidirectionalDShot(spec.sm_id, Pin(spec.pin), dshot_speed,
                                  rx_state_machine_id=spec.rx_sm_id)
    return UnidirectionalDShot(spec.sm_id, Pin(spec.pin), dshot_speed)


def arm_group(group, scenario, runner, bidir_indices, sink, last_seq, arm_times):
    """
    Arm, and check that no capture is handed out before the group is ARMED.

    Every capture taken while arming is logged to arming.bin (BUG-002: what the
    ESC did before ARMED - replying, silent, or rebooting - is otherwise
    invisible). The group publishes them to each motor's own slot because
    publish_while_arming is set; they are read from the motor directly, since
    raw_telemetry() withholds them. `last_seq` is left at the last sequence seen
    per motor, so the main loop starts after them. `arm_times` gets the ticks_us
    of arm() as soon as it is called and of ARMED once reached, so a run that
    fails while arming still records when arming began.
    """
    print("Arming for {}ms...".format(scenario.arm_duration_ms))
    group.publish_while_arming = bool(bidir_indices)
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
        for index in bidir_indices:
            # Re-check the state: the group may have become ARMED since the
            # loop test, and then the slot legitimately holds a capture
            if group.raw_telemetry(index) is not None and not group.is_armed():
                raise RuntimeError("raw_telemetry(" + str(index) + ") returned a capture while arming")
        log_new_captures(group, bidir_indices, last_seq, sink.arming_file, sink)
        utime.sleep_ms(1)
    arm_times[1] = utime.ticks_us()
    sink.close_arming()
    print("Armed after {}ms.".format(utime.ticks_diff(arm_times[1], arm_times[0]) // 1000))
    print()


def log_new_captures(group, bidir_indices, last_seq, file, sink):
    """Write one record of the captures each motor published since `last_seq`,
    read straight from the motors (the group withholds them while arming)."""
    words = [RECORD_ZERO_WORDS] * 4
    record_us = None
    for index in bidir_indices:
        capture = group.motors[index].latest_capture()
        if capture is None:
            continue
        ticks_us, seq, capture_words = capture
        if seq == last_seq[index]:
            continue
        last_seq[index] = seq
        words[index] = capture_words
        if record_us is None or utime.ticks_diff(ticks_us, record_us) > 0:
            record_us = ticks_us
    if record_us is not None:
        sink.write_record(record_us, group.get_all_throttles(), words, file)


def run_scenario():
    print("=== Scenario Capture ===")

    scenario = load_scenario(SCENARIO_PATH)
    print("Loaded scenario: duration={}ms dshot_speed={} bidir_motors={} core1_interval_us={} "
          "arming_frame_gap_us={}".format(
              scenario.duration_ms, scenario.dshot_speed, scenario.bidir_indices,
              scenario.core1_interval_us, scenario.arming_frame_gap_us))

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
    last_seq = [0, 0, 0, 0]  # per motor: highest capture sequence seen
    missed = 0               # captures published but never seen
    last_record_us = None
    largest_gap_us = 0
    max_age_us = 0
    loop_times = array('I', [0, 0])  # Core 1's last update() time, and the longest gap between calls
    arming_call_stats = array('I', [0, 0, 0, 0])  # while ARMING: count, min_us, max_us, sum_us
    last_gc_ms = 0
    gc_runs = 0
    gc_max_us = 0
    outcome = "failed"
    arm_times = [None, None]  # ticks_us of arm() and of ARMED, filled in by arm_group()
    arming_seq = {}
    tallies = {i: DecodeTally() for i in bidir_indices}
    seen = [0, 0, 0, 0]      # per motor: non-empty captures seen, for the sampling rule
    real_spin_seen = [False, False, False, False]  # per motor: ever decoded a non-"not running" eRPM
    failures = []

    try:
        group = MotorGroup([build_motor(spec, scenario.dshot_speed) for spec in scenario.motors])
        group.arming_frame_gap_us = scenario.arming_frame_gap_us
        if bidir_indices:
            # BUG-002: ground-truth classification bins, immune to how often
            # this loop happens to poll latest_capture() - see
            # CaptureMailbox.enable_class_bins(). Scenario-controlled (not
            # just on/off) since the diagnostic's own per-tick cost is itself
            # under investigation as a confound - see the bug report.
            group.arming_class_bin_width_us = scenario.arming_class_bin_width_us
        interval_us = scenario.core1_interval_us
        if interval_us is None:
            interval_us = group.UPDATE_INTERVAL_US
        runner = Core1Runner(measured(group.update, loop_times, group, arming_call_stats), interval_us)
        runner.start()

        arm_group(group, scenario, runner, bidir_indices, sink, last_seq, arm_times)
        arming_seq = {i: last_seq[i] for i in bidir_indices}

        print("Running scenario for {}ms...".format(scenario.duration_ms))
        run_start = utime.ticks_ms()
        last_status_ms = run_start
        last_gc_ms = run_start

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
            words = [RECORD_ZERO_WORDS] * 4
            record_us = None
            for index in bidir_indices:
                capture = group.raw_telemetry(index)
                if capture is None:
                    continue
                ticks_us, seq, capture_words = capture
                if seq == last_seq[index]:
                    continue
                if seq < last_seq[index]:
                    raise RuntimeError("capture sequence went backwards on motor " + str(index))
                missed += seq - last_seq[index] - 1
                last_seq[index] = seq
                words[index] = capture_words
                if any(capture_words):
                    seen[index] += 1
                    if is_sampled(seen[index], scenario.decode_every):
                        result = group.decode_telemetry(index, capture_words)
                        tallies[index].add(result)
                        if result is not None and result["crc_ok"] and not result["not_running"]:
                            real_spin_seen[index] = True
                if record_us is None or utime.ticks_diff(ticks_us, record_us) > 0:
                    record_us = ticks_us
                age_us = utime.ticks_diff(utime.ticks_us(), ticks_us)
                if age_us > max_age_us:
                    max_age_us = age_us

            if record_us is not None:
                total_records += 1
                if any(any(w) for w in words):
                    total_nonzero_records += 1
                sink.write_record(record_us, throttles, words)
                if last_record_us is not None:
                    gap = utime.ticks_diff(record_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = record_us

            check_reply_failsafe(has_bidir, total_nonzero_records, elapsed_ms)
            check_stuck_at_rest_failsafe(bidir_indices, real_spin_seen, throttles, elapsed_ms)
            check_expect(scenario.expect, largest_gap_us, total_records, elapsed_ms, loop_times[1])

            if scenario.gc_every_ms and utime.ticks_diff(utime.ticks_ms(), last_gc_ms) >= scenario.gc_every_ms:
                gc_start_us = utime.ticks_us()
                gc.collect()
                gc_us = utime.ticks_diff(utime.ticks_us(), gc_start_us)
                gc_runs += 1
                if gc_us > gc_max_us:
                    gc_max_us = gc_us
                last_gc_ms = utime.ticks_ms()

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_status_ms) >= scenario.status_interval_ms:
                last_status_ms = now
                elapsed_s = utime.ticks_diff(now, run_start) / 1000
                rate = total_records / elapsed_s if elapsed_s else 0.0
                print("  [{:7.1f}s] records={} nonzero={} missed={} rate={:.1f}/s largest_gap={:.1f}ms".format(
                    elapsed_s, total_records, total_nonzero_records, missed, rate, largest_gap_us / 1000))

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
        # Stop the loop before disarming: while it's still running, Core 1 can
        # call update() concurrently with disarm()'s own send/drain/stop calls
        # on the same state machines, from the other core, with nothing
        # serialising the two beyond a single state check disarm() makes at its
        # start. Halting the loop first removes that race entirely, rather than
        # relying on disarm() to tolerate it.
        if runner is not None:
            runner.stop()
        if group is not None:
            group.disarm()
        print("Motors stopped and disarmed.")
        published = {i: last_seq[i] for i in bidir_indices}
        # The thresholds are judged on a run that reached its end; one that was
        # cut short (a gap or rate threshold, a Core 1 error) has no verdict
        if outcome != "completed":
            verdict = "not evaluated: the run did not complete"
        elif failures:
            verdict = "fail: " + "; ".join(failures)
        else:
            verdict = "pass"
        extra = {"max_loop_gap_us": loop_times[1], "gc_runs": gc_runs, "gc_max_us": gc_max_us}
        if arming_call_stats[0]:
            extra["arming_call_count"] = arming_call_stats[0]
            extra["arming_call_min_us"] = arming_call_stats[1]
            extra["arming_call_max_us"] = arming_call_stats[2]
            extra["arming_call_avg_us"] = arming_call_stats[3] // arming_call_stats[0]
        if group is not None:
            for index in bidir_indices:
                log = group.reboot_log(index)
                if log:
                    extra["motor" + str(index) + "_reboot_log"] = ";".join(
                        "{}:{}:{}".format(ms, gap, source) for ms, gap, source in log)
                mailbox = group.motors[index].mailbox
                if mailbox.class_bins is not None:
                    totals = [0, 0, 0]
                    for b in range(mailbox.class_bin_count):
                        totals[0] += mailbox.class_bins[b * 3]
                        totals[1] += mailbox.class_bins[b * 3 + 1]
                        totals[2] += mailbox.class_bins[b * 3 + 2]
                    extra["motor" + str(index) + "_class_totals"] = "{}/{}/{}".format(*totals)
                    width_ms = mailbox.class_bin_width_us // 1000
                    entries = []
                    for b in range(mailbox.class_bin_count):
                        nr, zero, other = (mailbox.class_bins[b * 3], mailbox.class_bins[b * 3 + 1],
                                            mailbox.class_bins[b * 3 + 2])
                        if nr or zero or other:
                            entries.append("{}:{}/{}/{}".format(b * width_ms, nr, zero, other))
                    extra["motor" + str(index) + "_class_bins_ms"] = ",".join(entries)
        if arm_times[0] is not None:
            # Time 0 for classify_reply_timeline.py --from-arm, on a run that
            # never armed as well: that is the run whose arming log matters most
            extra["arm_ticks_us"] = arm_times[0]
        if arm_times[1] is not None:
            # Time 0 for scripts/classify_reply_timeline.py; arming.bin's records are before it.
            # captures_published counts from arm(), the arming ones included
            extra["armed_ticks_us"] = arm_times[1]
            for index in arming_seq:
                extra["motor" + str(index) + "_captures_while_arming"] = arming_seq[index]
        sink.finalize(outcome, total_records, missed, largest_gap_us, published, tallies, verdict, extra)
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
        print("Captures published (last sequence per motor):", published)
        print("Captures published but never seen:", missed)
        print("Largest gap between records: {:.1f}ms".format(largest_gap_us / 1000))
        print("Oldest capture at the moment it was read: {:.1f}ms".format(max_age_us / 1000))
        print("Longest gap between update() calls (measured on Core 1): {:.1f}ms".format(loop_times[1] / 1000))
        if arming_call_stats[0]:
            print("Packets sent while arming: {} (min {}us, max {}us, avg {}us)".format(
                arming_call_stats[0], arming_call_stats[1], arming_call_stats[2],
                arming_call_stats[3] // arming_call_stats[0]))
        if group is not None:
            for index in bidir_indices:
                log = group.reboot_log(index)
                if log:
                    print("Motor {} reboot log (ms since arm(), duration ms, source): {}".format(index, log))
                mailbox = group.motors[index].mailbox
                if mailbox.class_bins is not None:
                    total_key = "motor" + str(index) + "_class_totals"
                    if total_key in extra:
                        print("Motor {} ground-truth captures while arming (not_running/low/other): {}".format(
                            index, extra[total_key]))
        if gc_runs:
            print("Forced garbage collections: {} (longest {:.1f}ms)".format(gc_runs, gc_max_us / 1000))
        for index in tallies:
            print("Motor {} decoded on the device (every {}th capture): {}".format(
                index, scenario.decode_every, tallies[index].summary()))
        for message in failures:
            print("  expectation missed - " + message)
        print("=== Test Complete ===" if outcome == "completed" and not failures else "=== Test FAILED ===")

    if failures:
        raise RuntimeError("decode expectations missed: " + "; ".join(failures))


run_scenario()
