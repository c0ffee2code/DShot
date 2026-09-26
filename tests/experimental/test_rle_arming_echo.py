# Test: what does the frame receiver capture before the ESC starts replying?
#
# Purpose: W25 (bidirectional_dshot_review.md). dshot_bidir_rx_rle waits for a
# falling edge exactly like dshot_bidir_rx (see its own comment), so early in
# the arming window - before the ESC has locked onto bidirectional DShot and
# started sending real GCR replies - a capture can just as easily be TX's own
# waveform as a genuine reply. MotorGroup.arm()/update() already discards
# every capture taken during ARMING (drain_rx(False)), so this never reaches
# an application; what hasn't been checked is what these discarded captures
# actually look like, and whether they could pass CRC by chance.
#
# Method: single-threaded, no MotorGroup/Core1Runner - this drives the motors
# directly with DShotPIO's own low-level API (see CLAUDE.md's usage example)
# so the same core that sends commands also drains rx_read() every iteration,
# with no second core/thread reading the same FIFO concurrently. Commands are
# paced at BURST_INTERVAL_US, well above a full reply's duration (see
# test_rle_stalled_drain.py's header on why this matters - too fast corrupts
# replies by TX interference, which looks identical to a receiver bug).
#
# Sends zero throttle for ARM_MS (mirroring MotorGroup's own arming window),
# bucketing every capture decoded in that window by which BUCKET_MS-wide slice
# of the window it landed in, then keeps sending a real THROTTLE for SPIN_MS
# as a clean baseline for comparison. Reports marker_ok/CRC-valid rates per
# bucket. Not a pass/fail check - a wrong/echoed frame during arming is
# already known to be discarded by the group; this is characterization.
#
# Wiring: as rle_bench.py - channel 1 (GPIO 6) frame receiver, channels 2-4
# (GPIO 7/8/9) idle unidirectional on the next PIO block.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
import gcr_decode
import utime

ARM_MS = 3000
BUCKET_MS = 500
SPIN_MS = 2000
THROTTLE = 100
BURST_INTERVAL_US = 200  # see test_rle_stalled_drain.py's header - must exceed a full reply
DISARM_FRAMES = 4


def drive_and_tally(motor, others, duration_ms, throttle, buckets, start):
    end = utime.ticks_add(utime.ticks_ms(), duration_ms)
    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        motor.send_throttle_command(throttle)
        for other in others:
            other.send_throttle_command(throttle)
        while motor.rx_sm.rx_fifo() >= 1:
            word = motor.rx_read()
            result = gcr_decode.analyze_frame(word)
            elapsed = utime.ticks_diff(utime.ticks_ms(), start)
            bucket = elapsed // BUCKET_MS
            b = buckets.setdefault(bucket, {"total": 0, "marker_ok": 0, "crc_ok": 0})
            b["total"] += 1
            if result["marker_ok"]:
                b["marker_ok"] += 1
            if result["crc_ok"]:
                b["crc_ok"] += 1
        utime.sleep_us(BURST_INTERVAL_US)


def report(label, buckets, bucket_count):
    print("  " + label + ":")
    for i in range(bucket_count):
        b = buckets.get(i, {"total": 0, "marker_ok": 0, "crc_ok": 0})
        if b["total"] == 0:
            print("    [%5dms] no captures" % (i * BUCKET_MS))
            continue
        print("    [%5dms] total=%-4d marker_ok=%3d%% crc_ok=%3d%%" % (
            i * BUCKET_MS, b["total"],
            100 * b["marker_ok"] // b["total"],
            100 * b["crc_ok"] // b["total"]))


def main():
    motor = BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1,
                                receiver=BidirectionalDShot.FRAME_RECEIVER)
    others = [
        UnidirectionalDShot(4, Pin(7), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(5, Pin(8), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEEDS.DSHOT300),
    ]
    all_motors = [motor] + others

    try:
        for m in all_motors:
            m.start()

        print("=== Arming window: %dms of zero throttle, bucketed every %dms ===" %
              (ARM_MS, BUCKET_MS))
        arm_buckets = {}
        arm_start = utime.ticks_ms()
        drive_and_tally(motor, others, ARM_MS, 0, arm_buckets, arm_start)
        report("during arming (throttle 0)", arm_buckets, (ARM_MS + BUCKET_MS - 1) // BUCKET_MS)

        print("=== Baseline: %dms at throttle %d, for comparison ===" % (SPIN_MS, THROTTLE))
        spin_buckets = {}
        spin_start = utime.ticks_ms()
        drive_and_tally(motor, others, SPIN_MS, THROTTLE, spin_buckets, spin_start)
        report("while spinning (throttle %d)" % THROTTLE, spin_buckets,
                (SPIN_MS + BUCKET_MS - 1) // BUCKET_MS)

    finally:
        for _ in range(DISARM_FRAMES):
            for m in all_motors:
                m.send_throttle_command(0)
            utime.sleep_us(BURST_INTERVAL_US)
        for m in all_motors:
            m.drain()
        for m in all_motors:
            m.stop()
        print("Motors stopped.")

    print()
    print("=== Test Complete ===")


main()
