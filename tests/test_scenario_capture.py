# Runs a JSON-defined scenario (see tests/harness/scenarios/) against all 4
# DShot channels, capturing raw telemetry to the PicoBell's SD card - see
# tests/harness/scenario.py (loading/validation), tests/harness/
# throttle_profile.py (per-motor throttle curves), tests/harness/
# scenario_runner.py (Core 1 send+drain loop), and tests/harness/
# bidir_capture_sink.py (SD writes). Replaces the old single-scenario
# test_bidir_rx_capture.py; that script's exact behavior is now
# tests/harness/scenarios/single_channel_baseline.json.
#
# scripts/deploy.py --scenario <path> uploads the chosen scenario file to
# this fixed device-side name, since mpremote's `run` has no way to pass an
# extra file/argument into the running script.
#
# Fail-fast, deliberately: a Core 1 error, a dropped-record/gap/rate
# violation of the scenario's own "expect" thresholds, a tripped reply
# failsafe (see _check_reply_failsafe - a record is written on the RX
# FIFO's own fixed capture cadence, NOT only when the ESC actually replies,
# so record *count* alone cannot catch a non-replying ESC; bidirectional
# DShot's contract is a continuous eRPM reply, so a sustained run of
# all-zero records is itself the anomaly - this checks for at least one
# non-all-zero reply within a grace window), or Ctrl-C all still run the
# stop/disarm/close sequence in `finally`, then propagate - this never
# swallows a failure into a "Test Complete" banner. A run that fails exits
# non-zero and marks outcome=failed in meta.txt, so a truncated capture.bin
# is never mistaken for a complete one by the PC-side analyzer.
#
# No GCR decoding happens here - that's scripts/dshot_bidir_decode.py's job,
# run on the PC against whatever gets logged.

from machine import Pin
from dshot_pio import DShotPIO
from scenario import load_scenario
from scenario_runner import ScenarioRunner
from bidir_capture_sink import BidirCaptureSink
import utime

SCENARIO_PATH = "scenario.json"



# Grace window before the reply failsafe is armed. Deliberately short (not
# the 20s settling window _check_expect's min_record_rate_hz needs, which
# is a CUMULATIVE-average threshold and genuinely needs one) - this checks
# for the mere existence of one real reply, and arming already finished
# scenario.arm_duration_ms before this function is ever called, so a
# healthy ESC's first real reply lands within milliseconds of run_start. A
# short window also matters practically: short scenarios like
# period_tally_short.json (duration_ms=8000) need the failsafe to be able
# to trip at all before the run just ends on its own.
_REPLY_FAILSAFE_GRACE_MS = 2000


def _check_reply_failsafe(has_bidir, nonzero_records, elapsed_ms):
    # Bidirectional DShot's contract is a continuous eRPM reply every
    # command - unlike a rate/gap threshold, "the ESC has never once
    # replied" is not a tunable performance bar, it's a protocol violation,
    # so this fires unconditionally once armed rather than via the
    # scenario's own "expect" block.
    #
    # A record is written whenever a bidir motor's RX FIFO delivers a full
    # 4-word group - that happens on the PIO program's own fixed capture
    # cadence regardless of whether the ESC is actually replying, so
    # `records` alone can't distinguish "replying" from "silent" (confirmed
    # 2026-09-10: three different rx_speed candidates plus two reruns of
    # the previously-always-reliable rate all produced ~600/s of records
    # that were literally all-zero words - a dead bench/ESC, invisible
    # until a post-hoc tally caught it well after the fact, by which point
    # it wasn't clear whether the motors had even been spinning). Checking
    # for at least one non-all-zero record catches that within one short
    # grace window instead of only in hindsight.
    #
    # any(record[5:]) below is any-MOTOR, not per-motor: with today's
    # single bidirectional channel that's exactly right, but once a second
    # bidir channel is in the same scenario (see project backlog's W18) a
    # live motor would mask a silent one. Revisit per-motor tracking then.
    if not has_bidir:
        return
    if elapsed_ms >= _REPLY_FAILSAFE_GRACE_MS and nonzero_records == 0:
        raise RuntimeError(
            "reply failsafe tripped: no ESC reply seen by " + str(elapsed_ms) +
            "ms elapsed (every captured record's words are still all-zero) "
            "- check ESC power/arming/bidir mode before continuing"
        )


def _check_expect(expect, dropped, largest_gap_us, records, elapsed_ms):
    if not expect:
        return

    max_dropped = expect.get("max_dropped")
    if max_dropped is not None and dropped > max_dropped:
        raise RuntimeError("dropped=" + str(dropped) + " > max_dropped=" + str(max_dropped))

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
    # actually meant to police - confirmed on hardware, where a real
    # ~517/s cumulative average at just past 2s (perfectly normal early
    # noise) tripped a naive 2s grace and aborted a run that was otherwise
    # healthy. 20s comfortably clears that noise while still catching a
    # genuine sustained rate problem well before a multi-minute hold ends.
    if min_rate is not None and elapsed_ms >= 20000:
        rate = records / (elapsed_ms / 1000)
        if rate < min_rate:
            raise RuntimeError(
                "record rate=" + str(rate) + "/s < min_record_rate_hz=" + str(min_rate)
            )


def test_scenario_capture():
    print("=== Scenario Capture ===")

    scenario = load_scenario(SCENARIO_PATH)
    print("Loaded scenario: duration={}ms dshot_speed={} bidir_motors={}".format(
        scenario.duration_ms, scenario.dshot_speed, scenario.bidir_indices))

    sink = BidirCaptureSink()
    sink.init_session(scenario, SCENARIO_PATH)
    print("Session:", sink.path)
    print()

    motors = []
    runner = None
    total_records = 0
    total_nonzero_records = 0
    total_dropped = 0
    last_record_us = None
    largest_gap_us = 0
    outcome = "failed"
    has_bidir = bool(scenario.bidir_indices)

    try:
        for spec in scenario.motors:
            motor = DShotPIO(spec.sm_id, Pin(spec.pin), scenario.dshot_speed,
                              bidirectional=spec.bidirectional, rx_state_machine_id=spec.rx_sm_id)
            motors.append(motor)
            motor.start()

        runner = ScenarioRunner(motors)
        runner.start()

        print("Arming for {}ms...".format(scenario.arm_duration_ms))
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < scenario.arm_duration_ms:
            if runner.error is not None:
                raise runner.error
            runner.drain()  # discard - just keeping the ring buffer from filling
            utime.sleep_ms(scenario.poll_ms)
        print("Armed.")
        print()

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
                runner.set_throttle(index, spec.profile.throttle_at(elapsed_ms))

            for record in runner.drain():
                total_records += 1
                if any(record[5:]):
                    total_nonzero_records += 1
                sink.write_record(*record)
                ticks_us = record[0]
                if last_record_us is not None:
                    gap = utime.ticks_diff(ticks_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = ticks_us

            total_dropped = runner.dropped
            _check_reply_failsafe(has_bidir, total_nonzero_records, elapsed_ms)
            _check_expect(scenario.expect, total_dropped, largest_gap_us, total_records, elapsed_ms)

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_status_ms) >= scenario.status_interval_ms:
                last_status_ms = now
                elapsed_s = utime.ticks_diff(now, run_start) / 1000
                rate = total_records / elapsed_s if elapsed_s else 0.0
                print("  [{:7.1f}s] records={} nonzero={} dropped={} rate={:.1f}/s largest_gap={:.1f}ms".format(
                    elapsed_s, total_records, total_nonzero_records, total_dropped, rate, largest_gap_us / 1000))

            utime.sleep_ms(scenario.poll_ms)

        outcome = "completed"
        print()
        print("Scenario duration complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")
        raise

    finally:
        print("Stopping...")
        if runner is not None:
            for index in range(len(motors)):
                runner.set_throttle(index, 0)
            stop_start = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), stop_start) < scenario.stop_duration_ms:
                for record in runner.drain():
                    total_records += 1
                    if any(record[5:]):
                        total_nonzero_records += 1
                    sink.write_record(*record)
                utime.sleep_ms(scenario.poll_ms)
            runner.stop()
        for motor in motors:
            motor.drain()
            motor.stop()
        print("Motors stopped and deactivated.")
        sink.finalize(outcome, total_records, total_dropped, largest_gap_us)
        sink.close()
        print("SD card flushed and unmounted.")

        # Printed here, inside finally, so it shows up on a failed run too -
        # not just after a clean completion (in which case control also
        # reaches here, then continues normally once finally exits).
        print()
        print("=== Summary ===")
        print("Session:", sink.path)
        print("Outcome:", outcome)
        print("Total records captured:", total_records)
        print("Records with a real (non-all-zero) reply:", total_nonzero_records)
        print("Records dropped (ring buffer full):", total_dropped)
        print("Largest gap between records: {:.1f}ms".format(largest_gap_us / 1000))
        print("=== Test Complete ===" if outcome == "completed" else "=== Test FAILED ===")


test_scenario_capture()
