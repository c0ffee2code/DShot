# Test: Two motors using MotorThrottleGroup facade
#
# Purpose: Validate the facade with an application-owned command loop
# - Runs the 1kHz loop on Core 1 via Core1Runner (application code, not library)
# - Arms both motors, polling for completion
# - Runs at minimal throttle (70) for 10 seconds
# - Disarms
#
# Hardware: Two ESCs + motors on GPIO 4 and GPIO 5

from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner
import utime

# Configuration
MOTOR1_PIN = 4
MOTOR2_PIN = 5
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT600
THROTTLE_MIN = 70
RUN_DURATION_SEC = 10
ARM_DURATION_MS = 500

# Longest acceptable gap between command transmissions before we call the
# command loop unhealthy. Well under the ESC's own disarm timeout.
MAX_UPDATE_AGE_MS = 50

# Arming must complete within its own duration plus slack for loop startup
ARM_TIMEOUT_MS = ARM_DURATION_MS + 500


def test_motor_group():
    print("=== Motor Throttle Group Test ===")
    print(f"Motor 1: GPIO {MOTOR1_PIN}")
    print(f"Motor 2: GPIO {MOTOR2_PIN}")
    print(f"Speed: {DSHOT_SPEED}")
    print()

    # Create motor throttle group facade (accepts Pin objects, creates DShotPIO internally)
    motors = MotorThrottleGroup([Pin(MOTOR1_PIN), Pin(MOTOR2_PIN)], DSHOT_SPEED)

    # The application decides where the command loop runs - here, Core 1
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)

    try:
        print("Starting Core 1 command loop...")
        runner.start()
        print("Core 1 running at 1kHz.")
        print()

        # Phase 1: Arming (non-blocking - Core 1 advances it via update())
        print("Arming ESCs...")
        motors.arm(ARM_DURATION_MS)

        arm_start = utime.ticks_ms()
        while not motors.is_armed():
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_TIMEOUT_MS:
                raise Exception(f"Arming timed out (runner error: {runner.error})")
            utime.sleep_ms(10)

        print("Armed.")
        print()

        # Phase 2: Run at minimal throttle
        print(f"Running both motors at throttle {THROTTLE_MIN} for {RUN_DURATION_SEC} seconds...")
        motors.set_all_throttles([THROTTLE_MIN, THROTTLE_MIN])

        run_start = utime.ticks_ms()
        run_duration_ms = RUN_DURATION_SEC * 1000

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            # Core 1 handles command transmission
            # Core 0 (this loop) just monitors

            # Health check - the library reports the age, we set the threshold
            if motors.update_age_ms() > MAX_UPDATE_AGE_MS:
                print(f"WARNING: no commands sent for {motors.update_age_ms()}ms!")
                print(f"         runner error: {runner.error}")
                break

            # Progress indicator every 5 seconds
            elapsed_sec = utime.ticks_diff(utime.ticks_ms(), run_start) // 1000
            remaining = RUN_DURATION_SEC - elapsed_sec
            print(f"  {remaining}s remaining... (throttles: {motors.get_all_throttles()})")
            utime.sleep_ms(5000)

        print("Run complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        # disarm() stops the motors immediately and deactivates the state
        # machines, so it is safe even if the Core 1 loop has died
        motors.disarm()
        print("Motors stopped and disarmed.")

        runner.stop()
        print("Core 1 stopped.")

    print()
    print("=== Test Complete ===")


test_motor_group()
