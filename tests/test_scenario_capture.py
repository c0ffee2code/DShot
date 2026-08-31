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
# violation of the scenario's own "expect" thresholds, or Ctrl-C all still
# run the stop/disarm/close sequence in `finally`, then propagate - this
# never swallows a failure into a "Test Complete" banner. A run that fails
# exits non-zero and marks outcome=failed in meta.txt, so a truncated
# capture.bin is never mistaken for a complete one by the PC-side analyzer.
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
    total_dropped = 0
    last_record_us = None
    largest_gap_us = 0
    outcome = "failed"

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
                sink.write_record(*record)
                ticks_us = record[0]
                if last_record_us is not None:
                    gap = utime.ticks_diff(ticks_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = ticks_us

            total_dropped = runner.dropped
            _check_expect(scenario.expect, total_dropped, largest_gap_us, total_records, elapsed_ms)

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_status_ms) >= scenario.status_interval_ms:
                last_status_ms = now
                elapsed_s = utime.ticks_diff(now, run_start) / 1000
                rate = total_records / elapsed_s if elapsed_s else 0.0
                print("  [{:7.1f}s] records={} dropped={} rate={:.1f}/s largest_gap={:.1f}ms".format(
                    elapsed_s, total_records, total_dropped, rate, largest_gap_us / 1000))

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
        print("Records dropped (ring buffer full):", total_dropped)
        print("Largest gap between records: {:.1f}ms".format(largest_gap_us / 1000))
        print("=== Test Complete ===" if outcome == "completed" else "=== Test FAILED ===")


test_scenario_capture()
