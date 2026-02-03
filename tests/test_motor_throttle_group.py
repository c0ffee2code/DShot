# Test: Two motors using MotorThrottleGroup facade
#
# Purpose: Validate MotorThrottleGroup dual-core architecture works
# - Arms both motors via facade
# - Runs at minimal throttle (70) for 10 seconds
# - Stops and disarms
#
# Hardware: Two ESCs + motors on GPIO 4 and GPIO 5

from machine import Pin
from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
import utime

# Configuration
MOTOR1_PIN = 4
MOTOR2_PIN = 5
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT600
THROTTLE_MIN = 70
RUN_DURATION_SEC = 10
ARM_DURATION_MS = 500


def test_motor_group():
    print("=== Motor Throttle Group Test ===")
    print(f"Motor 1: GPIO {MOTOR1_PIN}")
    print(f"Motor 2: GPIO {MOTOR2_PIN}")
    print(f"Speed: {DSHOT_SPEED}")
    print()

    # Create motor throttle group facade (accepts Pin objects, creates DShotPIO internally)
    motors = MotorThrottleGroup([Pin(MOTOR1_PIN), Pin(MOTOR2_PIN)], DSHOT_SPEED)

    try:
        # Start Core 1 command loop
        print("Starting Core 1 command loop...")
        motors.start()
        print("Core 1 running at 1kHz.")
        print()

        # Phase 1: Arming (handled by facade)
        print("Arming ESCs...")
        motors.arm(ARM_DURATION_MS)
        print("Armed.")
        print()

        # Phase 2: Run at minimal throttle
        print(f"Running both motors at throttle {THROTTLE_MIN} for {RUN_DURATION_SEC} seconds...")
        motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])

        run_start = utime.ticks_ms()
        run_duration_ms = RUN_DURATION_SEC * 1000

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            # Core 1 handles command transmission
            # Core 0 (this loop) just monitors

            # Health check
            if not motors.isHealthy():
                print("WARNING: Core 1 stopped unexpectedly!")
                break

            # Progress indicator every 5 seconds
            elapsed_sec = utime.ticks_diff(utime.ticks_ms(), run_start) // 1000
            remaining = RUN_DURATION_SEC - elapsed_sec
            print(f"  {remaining}s remaining... (throttles: {motors.getAllThrottles()})")
            utime.sleep_ms(5000)

        print("Run complete.")
        print()

        # Phase 3: Stop and disarm
        print("Stopping motors...")
        motors.disarm()
        print("Motors stopped and disarmed.")

    except KeyboardInterrupt:
        print("\nInterrupted! Emergency stop...")
        motors.emergencyStop()
        print("Motors stopped.")

    finally:
        # Always stop Core 1 on exit
        motors.stop()
        print("Core 1 stopped.")

    print()
    print("=== Test Complete ===")


test_motor_group()
