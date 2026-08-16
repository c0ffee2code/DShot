# Test: Single motor using low-level DShotPIO driver directly
#
# Purpose: Validate DShotPIO works in isolation
# - Arms one motor
# - Runs at minimal throttle (70) for 10 seconds
# - Stops and disarms
#
# Hardware: Single ESC + motor on GPIO 4

from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

# Configuration
MOTOR_PIN = 4
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT600
THROTTLE_MIN = 70
RUN_DURATION_SEC = 10
ARM_DURATION_MS = 500
COMMAND_INTERVAL_MS = 1  # Send commands every X ms


def test_single_motor():
    print("=== Single Motor Test (Low-level DShotPIO) ===")
    print(f"Pin: GPIO {MOTOR_PIN}")
    print(f"Speed: {DSHOT_SPEED}")
    print()

    # Create motor driver
    motor = DShotPIO(0, MOTOR_PIN, DSHOT_SPEED)
    motor.start()

    try:
        # Phase 1: Arming
        print("Arming ESC...")
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            motor.send_throttle_command(0)
            utime.sleep_ms(COMMAND_INTERVAL_MS)
        print("Armed.")
        print()

        # Phase 2: Run at minimal throttle
        print(f"Running at throttle {THROTTLE_MIN} for {RUN_DURATION_SEC} seconds...")
        run_start = utime.ticks_ms()
        run_duration_ms = RUN_DURATION_SEC * 1000

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            motor.send_throttle_command(THROTTLE_MIN)
            utime.sleep_ms(COMMAND_INTERVAL_MS)

        print("Run complete.")
        print()

        # Phase 3: Stop and disarm
        print("Stopping motor...")
        stop_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), stop_start) < 500:
            motor.send_throttle_command(0)
            utime.sleep_ms(COMMAND_INTERVAL_MS)
        print("Motor stopped and disarmed.")

    except KeyboardInterrupt:
        print("\nInterrupted! Emergency stop...")
        for _ in range(100):
            motor.send_throttle_command(0)
            utime.sleep_ms(COMMAND_INTERVAL_MS)
        print("Motor stopped.")

    finally:
        # Deactivate the state machine - the line stops carrying DShot
        # transitions, so the ESC times out and the motor cannot spin
        motor.stop()
        print("State machine deactivated.")

    print()
    print("=== Test Complete ===")


test_single_motor()
