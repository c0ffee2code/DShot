# Validation sweep for the verified bidirectional DShot RX/decode pipeline
# (see decision/ADR-002-bidirectional-dshot.md's "RX redesign: unslotted
# dense oversampling" section - Phase 3 verified 17/17 CRC-valid on a short
# run, then 59/59 on a 50-600 gradual up-ramp). This run adds an up/down
# profile with longer holds:
#   - 60 immediately post-arm as a brief settle throttle (50 produced
#     audibly noisy/unstable spin - too low to use as a base).
#   - Up-ramp: 100 (first real measurement) in 50-unit steps to 600.
#   - Down-ramp: 600 back down to 100 in 100-unit steps - larger steps are
#     fine on the way down, since this ESC's power protection reacts to
#     sharp *increases*, not decreases.
#   - Every measured step (100 and up) holds 10s+ for stabilization.
#
# Same PIO-level mechanics as tests/test_bidir_rx_raw.py (dense unslotted
# 128-sample/4-word RX capture, IRQ-synced to dshot_bidir_tx) - this script
# only changes the throttle profile and snapshot cadence. Decoding is still
# done offline via scripts/decode_bidir_capture.py, not here.

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
ARM_DURATION_MS = 3000

# (throttle, seconds to hold): 60 settle -> 100..600 step 50 -> 500..100 step 100
THROTTLE_STEPS = (
    [(60, 3)]
    + [(t, 10) for t in range(100, 601, 50)]
    + [(t, 10) for t in range(500, 99, -100)]
)

STOP_DURATION_MS = 300
SNAPSHOT_INTERVAL_MS = 1000
MAX_SNAPSHOT_WORDS = 8  # 2 full 4-word frames


def current_throttle(elapsed_ms):
    remaining = elapsed_ms
    for throttle, seconds in THROTTLE_STEPS:
        step_ms = seconds * 1000
        if remaining < step_ms:
            return throttle
        remaining -= step_ms
    return THROTTLE_STEPS[-1][0]


def test_bidir_rx_sweep():
    print("=== Bidirectional RX Sweep (channel 1, verified decode pipeline) ===")
    print(f"Speed: {DSHOT_SPEED}")
    print(f"Throttle steps: {THROTTLE_STEPS}")
    print()

    ch1 = DShotPIO(0, Pin(2), DSHOT_SPEED, bidirectional=True, rx_state_machine_id=1)
    others = [
        DShotPIO(sm_id, Pin(pin), DSHOT_SPEED)
        for sm_id, pin in zip((4, 5, 6), (3, 4, 5))
    ]
    motors = [ch1] + others

    for motor in motors:
        motor.start()

    try:
        print(f"Arming for {ARM_DURATION_MS}ms (channel 1 inverted from frame 1)...")
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
            while ch1.rx_read() is not None:
                pass
        print("Arm window complete - listen for AM32's confirmation tone.")
        print()

        run_duration_ms = sum(seconds for _, seconds in THROTTLE_STEPS) * 1000
        print(f"Running channel 1 through {len(THROTTLE_STEPS)} throttle steps "
              f"({run_duration_ms/1000:.0f}s total), snapshotting RX once every "
              f"{SNAPSHOT_INTERVAL_MS}ms...")
        run_start = utime.ticks_ms()
        last_snapshot_ms = run_start
        total_words = 0
        snapshots_taken = 0

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            now = utime.ticks_ms()
            elapsed_ms = utime.ticks_diff(now, run_start)
            throttle = current_throttle(elapsed_ms)

            for motor in motors:
                motor.send_throttle_command(throttle if motor is ch1 else 0)

            take_snapshot = utime.ticks_diff(now, last_snapshot_ms) >= SNAPSHOT_INTERVAL_MS
            if take_snapshot:
                last_snapshot_ms = now

            snapshot = []
            while True:
                word = ch1.rx_read()
                if word is None:
                    break
                total_words += 1
                if take_snapshot and len(snapshot) < MAX_SNAPSHOT_WORDS:
                    snapshot.append(word)

            if take_snapshot:
                snapshots_taken += 1
                formatted = ", ".join(f"0x{w:08x}" for w in snapshot)
                print(f"  [{snapshots_taken}] throttle={throttle} words=[{formatted}]")

        print()
        print(f"{total_words} total words drained over the run "
              f"({snapshots_taken} snapshots printed above).")
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
    print("=== Sweep Complete ===")


test_bidir_rx_sweep()
