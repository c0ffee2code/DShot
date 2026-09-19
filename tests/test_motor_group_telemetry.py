# Test: MotorThrottleGroup's arm-gated raw telemetry accessor
#
# Purpose: exercises the whole bidirectional data flow through the public
# facade - the group's update() drains each bidirectional motor's RX FIFO into
# its one-slot capture, raw_telemetry() hands the latest capture out only once
# the group is ARMED, and decoding happens here on Core 0, at this test's own
# pace (not on the Core 1 command loop).
#
# Checks (any failure raises):
#   1. Group construction rejects 0 and 5 motors, two motors on one state
#      machine, and two motors on one pin; arm() is rejected while armed.
#   2. raw_telemetry() on a unidirectional motor raises
#      UnsupportedOperationException, on a bidirectional one returns None
#      while disarmed, on a bad index raises MotorThrottleGroupException.
#   3. raw_telemetry() returns None for the whole ARMING window - captures
#      taken before the ESC arms are echoes of our own transmit.
#   4. After arming, at a settled throttle, sequence numbers only advance,
#      the CRC-valid rate of decoded captures matches earlier baselines, and
#      the reported eRPM shows the motor actually spinning. CRC-valid alone
#      proves only the telemetry link: an armed ESC keeps replying with a
#      constant at-rest eRPM (917) even when the motor never starts.
#   5. raw_telemetry() returns None again after disarm().
#
# Hardware: 4-in-1 AM32 ESC. Channel 1 -> GPIO 6 (motor + prop mounted, the
# only bidirectional channel here), channels 2-4 -> GPIO 7/8/9 (idle, but the
# ESC only completes its arm handshake with valid signal on all 4 - see
# tests/test_slow_spin.py).

from machine import Pin
from dshot_pio import (BidirectionalDShot, UnidirectionalDShot,
                       UnsupportedOperationException, DSHOT_SPEEDS)
from motor_throttle_group import MotorThrottleGroup, MotorThrottleGroupException
from core1_runner import Core1Runner
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
THROTTLE = 100
# Armed-and-replying is not spinning, so this test checks eRPM as well. The
# arm window is the 3000ms the bench tests have been run with when checking
# that the motor spins, rather than the library default.
ARM_DURATION_MS = 3000
SETTLE_MS = 500
SAMPLE_MS = 5000
MIN_DECODED = 50
MIN_CRC_VALID_PCT = 98.0
# Throttle 100 sits around 20k eRPM on this bench; at rest the ESC reports 917
MIN_MEDIAN_ERPM = 10000
ARM_TIMEOUT_MS = ARM_DURATION_MS + 1000


def expect_raises(exception_type, fn, label):
    try:
        fn()
    except exception_type:
        print("  OK   " + label)
        return
    raise Exception("FAIL " + label + ": expected " + exception_type.__name__)


def test_motor_group_telemetry():
    print("=== MotorThrottleGroup Telemetry Test ===")

    print("Construction checks...")
    expect_raises(MotorThrottleGroupException, lambda: MotorThrottleGroup([]), "0 motors rejected")
    expect_raises(MotorThrottleGroupException, lambda: MotorThrottleGroup([object()] * 5), "5 motors rejected")
    expect_raises(MotorThrottleGroupException, lambda: MotorThrottleGroup([
        UnidirectionalDShot(0, Pin(6), DSHOT_SPEED),
        UnidirectionalDShot(0, Pin(7), DSHOT_SPEED),
    ]), "two motors on one state machine rejected")
    expect_raises(MotorThrottleGroupException, lambda: MotorThrottleGroup([
        UnidirectionalDShot(0, Pin(6), DSHOT_SPEED),
        BidirectionalDShot(2, Pin(7), DSHOT_SPEED, rx_state_machine_id=3),
        UnidirectionalDShot(3, Pin(8), DSHOT_SPEED),
    ]), "a state machine used as another motor's RX rejected")
    expect_raises(MotorThrottleGroupException, lambda: MotorThrottleGroup([
        UnidirectionalDShot(0, Pin(6), DSHOT_SPEED),
        UnidirectionalDShot(2, Pin(6), DSHOT_SPEED),
    ]), "two motors on one pin rejected")

    bidir = BidirectionalDShot(0, Pin(6), DSHOT_SPEED, rx_state_machine_id=1)
    motors = MotorThrottleGroup([
        bidir,
        UnidirectionalDShot(2, Pin(7), DSHOT_SPEED),
        UnidirectionalDShot(4, Pin(8), DSHOT_SPEED),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEED),
    ])
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)

    print("Accessor checks while disarmed...")
    expect_raises(UnsupportedOperationException, lambda: motors.raw_telemetry(1), "unidirectional motor raises")
    expect_raises(MotorThrottleGroupException, lambda: motors.raw_telemetry(9), "bad index raises")
    if motors.raw_telemetry(0) is not None:
        raise Exception("FAIL raw_telemetry(0) should be None while disarmed")
    print("  OK   bidirectional motor returns None while disarmed")

    try:
        runner.start()

        print("Arming (raw_telemetry must stay None throughout)...")
        motors.arm(ARM_DURATION_MS)
        arm_start = utime.ticks_ms()
        arm_polls = 0
        while not motors.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_TIMEOUT_MS:
                raise Exception("Arming timed out")
            if motors.raw_telemetry(0) is not None:
                raise Exception("FAIL raw_telemetry(0) returned a capture while ARMING")
            arm_polls += 1
            utime.sleep_ms(1)
        print("  OK   armed; None on all " + str(arm_polls) + " polls during arming")
        expect_raises(MotorThrottleGroupException, lambda: motors.arm(), "arm() while armed rejected")

        motors.set_throttle(0, THROTTLE)
        utime.sleep_ms(SETTLE_MS)

        print("Sampling for " + str(SAMPLE_MS) + "ms at throttle " + str(THROTTLE) + "...")
        last_seq = 0
        decoded = 0
        crc_ok = 0
        erpms = []
        no_capture = 0
        stale = 0
        max_age_us = 0
        sample_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), sample_start) < SAMPLE_MS:
            if runner.error is not None:
                raise runner.error

            capture = motors.raw_telemetry(0)
            if capture is None:
                no_capture += 1
                utime.sleep_ms(1)
                continue

            ticks_us, seq, words = capture
            if seq == last_seq:
                stale += 1
                utime.sleep_ms(1)
                continue
            if seq < last_seq:
                raise Exception("FAIL sequence went backwards: " + str(last_seq) + " -> " + str(seq))
            last_seq = seq

            age_us = utime.ticks_diff(utime.ticks_us(), ticks_us)
            if age_us > max_age_us:
                max_age_us = age_us

            result = motors.decode_telemetry(0, words)
            decoded += 1
            if result is not None and result["crc_ok"]:
                crc_ok += 1
                if result["erpm"] is not None:
                    erpms.append(result["erpm"])

        print("  captures published by Core 1 (last sequence): " + str(last_seq))
        print("  decoded on Core 0: " + str(decoded) + ", CRC-valid: " + str(crc_ok))
        print("  polls with no capture: " + str(no_capture) + ", already-seen: " + str(stale))
        print("  max age at read: " + str(max_age_us) + "us")

        if decoded < MIN_DECODED:
            raise Exception("FAIL only " + str(decoded) + " captures decoded (need " + str(MIN_DECODED) + ")")
        pct = 100.0 * crc_ok / decoded
        if pct < MIN_CRC_VALID_PCT:
            raise Exception("FAIL CRC-valid " + str(pct) + "% below " + str(MIN_CRC_VALID_PCT) + "%")
        print("  OK   CRC-valid " + str(pct) + "%")

        if not erpms:
            raise Exception("FAIL no CRC-valid capture carried an eRPM value")
        erpms.sort()
        median_erpm = erpms[len(erpms) // 2]
        print("  eRPM min/median/max: " + str(erpms[0]) + " / " + str(median_erpm) + " / " + str(erpms[-1]))
        if median_erpm < MIN_MEDIAN_ERPM:
            raise Exception("FAIL median eRPM " + str(median_erpm) + " below " + str(MIN_MEDIAN_ERPM) + " - motor is not spinning")
        print("  OK   motor spinning")

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        motors.disarm()
        print("Motors stopped and disarmed.")
        runner.stop()
        print("Core 1 stopped.")

    if motors.raw_telemetry(0) is not None:
        raise Exception("FAIL raw_telemetry(0) should be None after disarm")
    print("  OK   raw_telemetry(0) is None after disarm")

    print()
    print("=== Test Complete ===")


test_motor_group_telemetry()
