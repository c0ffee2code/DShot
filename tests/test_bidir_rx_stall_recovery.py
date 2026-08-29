# Test: RX-FIFO backpressure -> stale IRQ-4 recovery (R1/R2, see
# bidirectional_dshot_review.md's W8 item and decision/ADR-002-bidirectional-dshot.md).
#
# dshot_bidir_rx uses autopush - if Python doesn't drain the RX FIFO fast
# enough, the state machine stalls mid-capture on a full FIFO. While
# stalled, dshot_bidir_tx keeps firing its per-frame irq(4) release signal
# (non-blocking - costs TX nothing), but IRQ 4 is a single sticky flag, not
# a queue, so those signals collapse into one stale "already set" flag. When
# the stall clears and RX wraps back to wait(1, irq, 4), it can match
# immediately on that stale flag instead of a fresh one, re-phasing the
# predelay + marker search against the wrong point in time - potentially
# capturing TX's own waveform and handing it back as a fake "reply".
#
# This test deliberately withholds draining rx_read() for a run of
# iterations at settled throttle (long enough to fill the 4-word RX FIFO
# and stall the SM), then resumes draining and checks how quickly captures
# recover to a decodable state. Run against unmodified driver/dshot_pio.py
# first to record the pre-fix failure, then again after adding
# irq(clear, 4) to dshot_bidir_rx to confirm recovery.
#
# Decoding is intentionally not done on-device - see
# scripts/decode_bidir_capture.py, which runs offline against the printed
# captures, same as test_bidir_rx_raw.py.
#
# Hardware: 4-in-1 AM32 ESC, channel 1 -> GPIO 2 (motor+prop mounted),
# channels 2-4 -> GPIO 3/4/5 (wired but idle only).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
ARM_DURATION_MS = 3000
SETTLE_MS = 4000        # let the motor settle at SETTLE_THROTTLE before withholding drains
SETTLE_THROTTLE = 200
WITHHOLD_FRAMES = 10    # frames to send without draining rx_read() - enough to fill the 4-word FIFO and stall
RECOVERY_FRAMES = 20    # frames to capture (draining every time) immediately after resuming
STOP_DURATION_MS = 300


def test_bidir_rx_stall_recovery():
    print("=== Bidirectional RX Stall-Recovery Test (channel 1, R1/R2 repro) ===")
    print(f"Speed: {DSHOT_SPEED}")
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
        print("Arm window complete.")
        print()

        print(f"Settling at throttle={SETTLE_THROTTLE} for {SETTLE_MS}ms, draining normally...")
        settle_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), settle_start) < SETTLE_MS:
            for motor in motors:
                motor.send_throttle_command(SETTLE_THROTTLE if motor is ch1 else 0)
            while ch1.rx_read() is not None:
                pass
        print("Settled.")
        print()

        print(f"Withholding rx_read() for {WITHHOLD_FRAMES} frames (expect the RX FIFO "
              "to fill and the SM to stall mid-capture)...")
        for i in range(WITHHOLD_FRAMES):
            for motor in motors:
                motor.send_throttle_command(SETTLE_THROTTLE if motor is ch1 else 0)
            utime.sleep_us(ch1.frame_us)
        print("Withhold window complete - not reading rx_read() at all during it.")
        print()

        print(f"Resuming draining for {RECOVERY_FRAMES} frames, printing every capture group "
              "of 4 words as it drains...")
        recovered_words = []
        for i in range(RECOVERY_FRAMES):
            for motor in motors:
                motor.send_throttle_command(SETTLE_THROTTLE if motor is ch1 else 0)
            while True:
                word = ch1.rx_read()
                if word is None:
                    break
                recovered_words.append(word)
            utime.sleep_us(ch1.frame_us)

        print(f"{len(recovered_words)} words drained during recovery window.")
        for i in range(0, len(recovered_words) - 3, 4):
            group = recovered_words[i:i + 4]
            formatted = ", ".join(f"0x{w:08x}" for w in group)
            print(f"  capture[{i // 4}] = [{formatted}]")
        print()
        print("Feed the printed groups above into scripts/decode_bidir_capture.py to check "
              "CRC-valid rate: near-zero / TX-waveform-shaped on unmodified driver code, "
              "recovering to the normal baseline within a frame or two after the irq(clear, 4) fix.")
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


test_bidir_rx_stall_recovery()
