# Test: Channel 1 raw GCR capture - Phase 3 checkpoint for bidirectional
# DShot, Option A' design (see decision/ADR-002-bidirectional-dshot.md's
# "Implementation Update" section for the full design history/rationale).
#
# Unlike the first RX attempt (removed - see ADR-002), synchronisation is
# entirely PIO-side: dshot_bidir_tx raises a PIO IRQ the instant it releases
# the pin each frame, and dshot_bidir_rx waits on that IRQ, holds a fixed
# ~4.7us delay (AM32's actual reply turnaround on this ESC - see ADR-002),
# then samples the pin uniformly and densely (128 samples, no assumed bit
# period) rather than trying to land pre-timed samples within an assumed bit
# "slot" - see dshot_bidir_rx's comments for why that slotted design was
# replaced. No per-capture setup call is needed from Python any more - the
# RX state machine free-runs, one attempt per TX frame, entirely in hardware.
#
# Each real reply produces FOUR raw 32-bit words back to back (128 samples
# total, covering the marker bit, the 20 real GCR data bits, and idle tail).
# Decoding (finding the real bit period/phase, majority vote, GCR table
# reverse-lookup, CRC check) is deliberately NOT done here - see
# scripts/decode_bidir_capture.py, which runs offline against printed
# captures. Iterating on decode logic in plain Python is much faster than
# round-tripping through hardware.
#
# Draining matters here in a way it didn't before: AM32 replies to every
# frame once bidirectional mode is detected, and autopush stalls in_()
# itself if the RX FIFO (4 words) fills - so this loop drains ch1's RX FIFO
# on every iteration, not just when taking a sample. Only a small snapshot
# is kept for printing each second; everything else is drained and discarded
# to keep output readable.
#
# PIO placement: channel 1's TX and RX state machines share GPIO 2 and must
# be on the same PIO block (ids 0 and 1, both PIO0) - both for the pin's
# function-select routing (see DShotPIO's docstring) and because the IRQ
# handshake between them only reaches state machines on the same block.
# Channels 2-4 (TX-only, different GPIOs) use ids 4, 5, 6 on PIO1.
#
# Hardware: 4-in-1 AM32 ESC, channel 1 -> GPIO 2 (motor+prop mounted),
# channels 2-4 -> GPIO 3/4/5 (wired but idle only).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
ARM_DURATION_MS = 3000
# (throttle, seconds to hold it) - each step should be long enough for the
# motor to settle at the new speed before we trust captures against it
THROTTLE_STEPS = [(100, 6), (200, 6), (300, 6)]
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


def test_bidir_rx_raw():
    print("=== Bidirectional RX Raw Capture Test (channel 1, Option A', no decode) ===")
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
            # Drain and discard - RX free-runs from the first frame, whether
            # or not AM32 has detected bidirectional mode yet
            while ch1.rx_read() is not None:
                pass
        print("Arm window complete - listen for AM32's confirmation tone.")
        print()

        run_duration_ms = sum(seconds for _, seconds in THROTTLE_STEPS) * 1000
        print(f"Running channel 1 through throttle steps {THROTTLE_STEPS}, "
              f"snapshotting RX once every {SNAPSHOT_INTERVAL_MS}ms...")
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
    print("=== Test Complete ===")


test_bidir_rx_raw()
