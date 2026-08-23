# Test: All 4 ESC channels driven together, throttle only on channel 1
#
# Purpose: This is a 4-in-1 AM32 ESC. Only channel 1's motor is physically
# mounted, but AM32 firmware won't complete its arm handshake ("3 short
# beeps" power-check -> "2 longer beeps" armed) until all 4 channels see
# valid DShot signal, not just one - so all 4 get armed together via
# MotorThrottleGroup; only channel 1 (motor index 0) gets nonzero throttle.
#
# This ESC also would not complete arming at the library's old 500ms
# default, even at max frame rate - needs the full DEFAULT_ARM_DURATION_MS
# (see driver/motor_throttle_group.py and README.md "Verified Parameters").
#
# Hardware: 4-in-1 ESC, channel 1 -> GPIO 2 (motor + prop mounted),
# channels 2-4 -> GPIO 3/4/5 (wired but no motor mounted - idle only).

from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner
import utime

# Configuration
MOTOR_PINS = [2, 3, 4, 5]
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
THROTTLE = 100
RUN_DURATION_SEC = 10
ARM_DURATION_MS = MotorThrottleGroup.DEFAULT_ARM_DURATION_MS

# Longest acceptable gap between command transmissions before we call the
# command loop unhealthy. Well under the ESC's own disarm timeout.
MAX_UPDATE_AGE_MS = 50

# Arming must complete within its own duration plus slack for loop startup
ARM_TIMEOUT_MS = ARM_DURATION_MS + 500


def test_slow_spin():
    print("=== Slow Spin Test (4-in-1 AM32 ESC, all channels armed, throttle on ch1 only) ===")
    print(f"Motor pins: {MOTOR_PINS}")
    print(f"Speed: {DSHOT_SPEED}")
    print()

    motors = MotorThrottleGroup([Pin(p) for p in MOTOR_PINS], DSHOT_SPEED)
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)

    try:
        print("Starting Core 1 command loop...")
        runner.start()
        print(f"Core 1 running (interval: {motors.UPDATE_INTERVAL_US}us).")
        print()

        print("Arming all 4 channels...")
        motors.arm(ARM_DURATION_MS)

        arm_start = utime.ticks_ms()
        while not motors.is_armed():
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_TIMEOUT_MS:
                raise Exception(f"Arming timed out (runner error: {runner.error})")
            utime.sleep_ms(10)

        print("Armed.")
        print()

        print(f"Running channel 1 at throttle {THROTTLE} for {RUN_DURATION_SEC} seconds (channels 2-4 stay at zero)...")
        motors.set_throttle(0, THROTTLE)

        run_start = utime.ticks_ms()
        run_duration_ms = RUN_DURATION_SEC * 1000

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            if motors.update_age_ms() > MAX_UPDATE_AGE_MS:
                print(f"WARNING: no commands sent for {motors.update_age_ms()}ms!")
                print(f"         runner error: {runner.error}")
                break

            elapsed_sec = utime.ticks_diff(utime.ticks_ms(), run_start) // 1000
            remaining = RUN_DURATION_SEC - elapsed_sec
            print(f"  {remaining}s remaining... (throttles: {motors.get_all_throttles()})")
            utime.sleep_ms(2000)

        print("Run complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        motors.disarm()
        print("Motors stopped and disarmed.")

        runner.stop()
        print("Core 1 stopped.")

    print()
    print("=== Test Complete ===")


test_slow_spin()
