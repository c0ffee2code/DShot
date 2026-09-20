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
# an error: arming completes inside update() on Core 1, so a dead loop shows here
ARM_TIMEOUT_MARGIN_MS = 1000

RECORD_ZERO_WORDS = (0, 0, 0, 0)


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


def arm_group(group, scenario, runner, bidir_indices):
    """Arm, and check that no capture is handed out before the group is ARMED."""
    print("Arming for {}ms...".format(scenario.arm_duration_ms))
    group.arm(scenario.arm_duration_ms)
    arm_start = utime.ticks_ms()
    arm_timeout_ms = scenario.arm_duration_ms + ARM_TIMEOUT_MARGIN_MS
    while not group.is_armed():
        if runner.error is not None:
            raise runner.error
        if utime.ticks_diff(utime.ticks_ms(), arm_start) > arm_timeout_ms:
            raise RuntimeError("arming did not complete within " + str(arm_timeout_ms) + "ms")
        for index in bidir_indices:
            if group.raw_telemetry(index) is not None:
                raise RuntimeError("raw_telemetry(" + str(index) + ") returned a capture while arming")
        utime.sleep_ms(1)
    print("Armed.")
    print()


def test_scenario_capture():
    print("=== Scenario Capture ===")

    scenario = load_scenario(SCENARIO_PATH)
    print("Loaded scenario: duration={}ms dshot_speed={} bidir_motors={}".format(
        scenario.duration_ms, scenario.dshot_speed, scenario.bidir_indices))

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
    outcome = "failed"
    tallies = {i: DecodeTally() for i in bidir_indices}
    seen = [0, 0, 0, 0]      # per motor: non-empty captures seen, for the sampling rule
    failures = []

    try:
        group = MotorGroup([build_motor(spec, scenario.dshot_speed) for spec in scenario.motors])
        runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)
        runner.start()

        arm_group(group, scenario, runner, bidir_indices)

        print("Running scenario for {}ms...".format(scenario.duration_ms))
        run_start = utime.ticks_ms()
        last_status_ms = run_start

        while True:
            elapsed_ms = utime.ticks_diff(utime.ticks_ms(), run_start)
            if elapsed_ms >= scenario.duration_ms:
                break

            if runner.error is not None:
                raise runner.error

            for index, spec in enumerate(scenario.motors):
                group.set_throttle(index, spec.profile.throttle_at(elapsed_ms))

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
                        tallies[index].add(group.decode_telemetry(index, capture_words))
                if record_us is None or utime.ticks_diff(ticks_us, record_us) > 0:
                    record_us = ticks_us
                age_us = utime.ticks_diff(utime.ticks_us(), ticks_us)
                if age_us > max_age_us:
                    max_age_us = age_us

            if record_us is not None:
                total_records += 1
                if any(any(w) for w in words):
                    total_nonzero_records += 1
                throttles = group.get_all_throttles()
                sink.write_record(record_us, throttles, words)
                if last_record_us is not None:
                    gap = utime.ticks_diff(record_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = record_us

            check_reply_failsafe(has_bidir, total_nonzero_records, elapsed_ms)
            check_expect(scenario.expect, largest_gap_us, total_records, elapsed_ms)

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
        if group is not None:
            group.disarm()
        if runner is not None:
            runner.stop()
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
        sink.finalize(outcome, total_records, missed, largest_gap_us, published, tallies, verdict)
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
        for index in tallies:
            print("Motor {} decoded on the device (every {}th capture): {}".format(
                index, scenario.decode_every, tallies[index].summary()))
        for message in failures:
            print("  expectation missed - " + message)
        print("=== Test Complete ===" if outcome == "completed" and not failures else "=== Test FAILED ===")

    if failures:
        raise RuntimeError("decode expectations missed: " + "; ".join(failures))


test_scenario_capture()
