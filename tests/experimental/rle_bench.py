# The frame receiver (dshot_bidir_rx_rle) against a real ESC, driven through
# the same BidirectionalDShot/MotorGroup facade every other motor uses
# (BidirectionalDShot(..., receiver=BidirectionalDShot.FRAME_RECEIVER)).
# Shared by test_rle_receiver_300.py and test_rle_receiver_600.py, which pick
# the speed.
#
# Purpose: dshot_bidir_rx_rle rebuilds the ESC's reply in the state machine and
# hands the CPU one already-reconstructed 21-bit frame per reply
# (gcr_decode.analyze_frame()), instead of 128 raw samples the CPU turns into
# a frame (gcr_decode.analyze_capture()). This runs it on the bench, on the
# same wiring and at the same settled throttle as the telemetry_settled
# scenarios, which are the baseline for the sample receiver (>=98% CRC-valid,
# eRPM around 21k at throttle 100), so the two can be compared.
#
# CaptureMailbox keeps only the latest capture, so per-frame counting comes
# from its own sequence number (raw_telemetry()'s second element), the same
# way tests/harness/run_scenario.py counts missed captures.
#
# Reports: frames received (and how many the sequence numbers say were missed
# between samples), the DecodeTally tests/harness/run_scenario.py's own
# scenarios are judged on (crc_ok / crc_fail / invalid, median eRPM), the
# marker-bit check DecodeTally does not track, and the CPU cost of
# decode_telemetry() - all that is left for the CPU, versus about 1.3ms for
# the sample path's analyze_capture(). Any failure raises.
#
# Hardware: as tests/harness scenarios - 4-in-1 AM32 ESC, channel 1 -> GPIO 6
# (motor + prop mounted, the only bidirectional channel), channels 2-4 ->
# GPIO 7/8/9 (idle, but the ESC only completes its arm handshake with valid
# signal on all 4). The frame receiver and dshot_bidir_tx fill their PIO
# block's 32 instruction slots between them (see BidirectionalDShot's
# constructor docstring), so the idle channels' state machines sit on the
# next block.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot
from motor_group import MotorGroup
from core1_runner import Core1Runner
from decode_tally import DecodeTally
import utime

THROTTLE = 100
ARM_DURATION_MS = 3000
ARM_TIMEOUT_MS = ARM_DURATION_MS + 1000
SETTLE_MS = 500
SAMPLE_MS = 5000
MIN_FRAMES = 50
MIN_CRC_VALID_PCT = 98.0
MIN_MEDIAN_ERPM = 10000


def run(dshot_speed):
    """Run the bench check at `dshot_speed` (a DSHOT_SPEEDS value); any failure raises."""
    print("=== Frame receiver test ===")

    bidir = BidirectionalDShot(0, Pin(6), dshot_speed, rx_state_machine_id=1,
                                receiver=BidirectionalDShot.FRAME_RECEIVER)
    motors = MotorGroup([
        bidir,
        UnidirectionalDShot(4, Pin(7), dshot_speed),
        UnidirectionalDShot(5, Pin(8), dshot_speed),
        UnidirectionalDShot(6, Pin(9), dshot_speed),
    ])
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)

    try:
        runner.start()

        print("Arming...")
        motors.arm(ARM_DURATION_MS)
        arm_start = utime.ticks_ms()
        while not motors.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_TIMEOUT_MS:
                raise Exception("Arming timed out")
            utime.sleep_ms(1)
        print("  armed")

        motors.set_throttle(0, THROTTLE)
        utime.sleep_ms(SETTLE_MS)

        print("Sampling for " + str(SAMPLE_MS) + "ms at throttle " + str(THROTTLE) + "...")
        tally = DecodeTally()
        last_seq = 0
        total = 0
        lost = 0
        marker_ok = 0
        decode_us_sum = 0
        decode_us_max = 0
        ticks_us = utime.ticks_us
        ticks_diff = utime.ticks_diff
        sample_start = utime.ticks_ms()
        while ticks_diff(utime.ticks_ms(), sample_start) < SAMPLE_MS:
            if runner.error is not None:
                raise runner.error
            capture = motors.raw_telemetry(0)
            if capture is None:
                continue
            _, seq, words = capture
            if seq == last_seq:
                continue
            lost += seq - last_seq - 1
            last_seq = seq
            total += 1

            t0 = ticks_us()
            result = motors.decode_telemetry(0, words)
            dt = ticks_diff(ticks_us(), t0)
            decode_us_sum += dt
            if dt > decode_us_max:
                decode_us_max = dt

            tally.add(result)
            if result is not None and result["marker_ok"]:
                marker_ok += 1

        print("  frames received: " + str(total) + " (" + str(total * 1000 // SAMPLE_MS) +
              "/s), lost to mailbox overwrite: " + str(lost))
        print("  marker bit 0: " + str(marker_ok) + ", " + tally.summary())
        if total:
            print("  decode cost per frame: mean " + str(decode_us_sum // total) +
                  "us, max " + str(decode_us_max) + "us")

        if total < MIN_FRAMES:
            raise Exception("FAIL only " + str(total) + " frames (need " + str(MIN_FRAMES) + ")")
        failures = tally.check(MIN_CRC_VALID_PCT, MIN_MEDIAN_ERPM)
        for failure in failures:
            print("  FAIL " + failure)
        if failures:
            raise Exception("FAIL " + "; ".join(failures))
        print("  OK   motor spinning")

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        # Stop the loop before disarming: while it's still running, Core 1 can
        # call update() concurrently with disarm()'s own send/drain/stop calls
        # on the same state machines, from the other core, with nothing
        # serialising the two beyond a single state check disarm() makes at
        # its start.
        runner.stop()
        motors.disarm()
        print("Motors stopped and disarmed.")

    print()
    print("=== Test Complete ===")
