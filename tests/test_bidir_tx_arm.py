# Test: Channel 1 sends the inverted (bidirectional) DShot waveform for the
# whole arm + run sequence. No RX yet - this is a Phase 2 checkpoint for
# bidirectional DShot (see decision/ADR-002).
#
# Why the whole sequence, not just the run phase: AM32 only samples the idle
# line polarity to detect bidirectional mode while its own internal state is
# still disarmed (confirmed from AM32 firmware source - see driver/dshot_pio.py's
# `bidirectional` docstring and ADR-002). There is no "arm normally, then
# switch to inverted" option, so this test uses DShotPIO directly rather than
# MotorThrottleGroup (which doesn't yet know about per-motor bidirectional
# signaling - that facade integration is a later phase) and inverts channel 1
# from the first frame of arming.
#
# Channels 2-4 stay plain, unmodified DShotPIO - this 4-in-1 AM32 ESC won't
# complete its arm handshake unless all 4 channels see valid signal, but only
# channel 1 needs to go bidirectional for this phase (see tests/test_slow_spin.py).
#
# Pass criteria: arming still completes (listen for AM32's "3 short beeps,
# then 2 longer beeps" tone - same as the unidirectional case) and channel 1
# spins normally afterward. No telemetry is read or printed here; this test
# only proves the inverted waveform doesn't regress arming/spin-up.
#
# Hardware: 4-in-1 AM32 ESC, channel 1 -> GPIO 2 (motor+prop mounted),
# channels 2-4 -> GPIO 3/4/5 (wired but idle only).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

MOTOR_PINS = [2, 3, 4, 5]
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
ARM_DURATION_MS = 3000  # matches MotorThrottleGroup.DEFAULT_ARM_DURATION_MS
THROTTLE = 100
RUN_DURATION_SEC = 5
STOP_DURATION_MS = 300


def test_bidir_tx_arm():
    print("=== Bidirectional TX Arm Test (channel 1 inverted waveform, no RX) ===")
    print(f"Motor pins: {MOTOR_PINS}")
    print(f"Speed: {DSHOT_SPEED}")
    print()

    motors = [
        DShotPIO(i, Pin(pin), DSHOT_SPEED, bidirectional=(i == 0))
        for i, pin in enumerate(MOTOR_PINS)
    ]
    for motor in motors:
        motor.start()

    try:
        print(f"Arming for {ARM_DURATION_MS}ms (channel 1 inverted from frame 1)...")
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
        print("Arm window complete - listen for AM32's confirmation tone.")
        print()

        print(f"Running channel 1 at throttle {THROTTLE} for {RUN_DURATION_SEC}s (channels 2-4 stay at zero)...")
        run_start = utime.ticks_ms()
        run_duration_ms = RUN_DURATION_SEC * 1000
        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            motors[0].send_throttle_command(THROTTLE)
            for motor in motors[1:]:
                motor.send_throttle_command(0)
        print("Run complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        print("Stopping...")
        stop_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), stop_start) < STOP_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)

        for motor in motors:
            motor.drain()
            motor.stop()
        print("Motors stopped and deactivated.")

    print()
    print("=== Test Complete ===")


test_bidir_tx_arm()
