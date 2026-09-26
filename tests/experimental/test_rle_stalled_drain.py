# Test: an undrained frame receiver holds a correct capture, it does not
# corrupt one.
#
# Purpose: W25 (bidirectional_dshot_review.md). dshot_bidir_rx_rle's own
# comment argues a full RX FIFO cannot corrupt a capture, only delay
# delivering an already-complete one: the 21st of its 21 reads is the one
# autopush fires on, and every bit is already shifted into the ISR by then, so
# the stall holds a finished value rather than an in-progress one - unlike
# dshot_bidir_rx, which autopushes mid-capture and can push a torn one.
#
# An earlier version of this test (and this comment) concluded the opposite:
# a burst with no drains came back mostly corrupted. That was a test bug, not
# a receiver bug - the burst paced commands at motor.frame_us (~54us at
# DSHOT300), which is only the TX bit-shift time. It does not include the
# ESC's own reply (another ~54us at this profile's bit period, after a ~4us
# predelay), so re-arming TX that fast drove the line again before the ESC's
# reply had finished, corrupting it by interference - a failure that looks
# identical to a genuine RX-FIFO-stall corruption from the drained result
# alone. Re-run with a wide enough interval to let a full reply complete
# (BURST_INTERVAL_US below), the corruption disappeared - including with the
# FIFO at its unmodified default depth, and at bursts up to 60 frames with
# zero drains. Lesson for any future version of this test: pace a burst
# against the reply's own duration, not the command's.
#
# Method: arm and settle normally, then take manual control of the command
# loop (stop Core1Runner) and send a burst of frames with no drain at all in
# between. Drain everything with rx_read() (the diagnostic raw read, not
# drain_rx()/the mailbox - this wants every word that piled up, not just the
# latest) and decode each one. Resync briefly between the two bursts by
# resuming normal draining for a short period.
#
# Pass: every drained capture, at both burst sizes, is marker_ok and
# CRC-valid. The FIFO's default depth (4 one-word captures) means only
# capacity+1 captures are ever drained regardless of burst size - the state
# machine holds the (capacity+1)th's complete value stalled at its own push,
# and every reply after that is silently never captured until draining
# resumes. That capture loss is expected and not itself a failure here; only
# a corrupted (not merely missing) capture is.
#
# Wiring: as rle_bench.py - channel 1 (GPIO 6) frame receiver, channels 2-4
# (GPIO 7/8/9) idle unidirectional on the next PIO block.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner
import gcr_decode
import utime

THROTTLE = 100
ARM_MS = 3000
SETTLE_MS = 500
RESYNC_MS = 300  # normal draining between bursts, so each starts from a clean sync
SMALL_BURST = 4    # the RX FIFO's default one-word-capture depth
LARGE_BURST = 20   # several times the depth, to confirm the property holds at scale

# motor.frame_us is only the TX bit-shift time - it does NOT include the
# ESC's own reply. See this file's header comment: pacing a burst at
# frame_us alone re-arms TX before the reply finishes, corrupting it by
# interference - a test artifact that looks identical to genuine RX-FIFO-stall
# corruption unless the interval is wide enough for a full reply to complete.
BURST_INTERVAL_US = 200


def run_burst(motor, count):
    """Send `count` frames with zero drains, then drain everything with
    rx_read() and decode it. Returns (frames, bad)."""
    print("  sending a burst of %d frames with no drain at all..." % count)
    for _ in range(count):
        motor.send_throttle_command(THROTTLE)
        utime.sleep_us(BURST_INTERVAL_US)

    pending = motor.rx_sm.rx_fifo()
    print("  words waiting in the RX FIFO after the burst: %d" % pending)

    frames = []
    while motor.rx_sm.rx_fifo() >= 1:
        frames.append(motor.rx_read())
    print("  captures drained: %d" % len(frames))

    bad = []
    for frame in frames:
        result = gcr_decode.analyze_frame(frame)
        if not (result["marker_ok"] and result["crc_ok"]):
            bad.append((frame, result))
    for frame, result in bad:
        print("  BAD frame=0x%06x marker_ok=%s crc_ok=%s" %
              (frame, result["marker_ok"], result["crc_ok"]))

    return frames, bad


def main():
    motors = MotorGroup([
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1,
                            receiver=BidirectionalDShot.FRAME_RECEIVER),
        UnidirectionalDShot(4, Pin(7), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(5, Pin(8), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEEDS.DSHOT300),
    ])
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)
    motor = motors.motors[0]
    all_bad = []

    try:
        runner.start()

        print("Arming...")
        motors.arm(ARM_MS)
        arm_start = utime.ticks_ms()
        while not motors.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_MS + 1000:
                raise Exception("Arming timed out")
            utime.sleep_ms(1)
        print("  armed")

        motors.set_throttle(0, THROTTLE)
        utime.sleep_ms(SETTLE_MS)

        for label, count in (("SMALL_BURST", SMALL_BURST), ("LARGE_BURST", LARGE_BURST)):
            runner.stop()
            print("=== %s (%d frames) ===" % (label, count))
            frames, bad = run_burst(motor, count)
            all_bad.extend(bad)
            print("  %d of %d drained captures were marker_ok and CRC-valid" %
                  (len(frames) - len(bad), len(frames)))

            print("Resyncing for %dms..." % RESYNC_MS)
            runner.start()
            utime.sleep_ms(RESYNC_MS)

        if all_bad:
            raise Exception("FAIL %d drained captures were corrupted (see above)" % len(all_bad))
        print("  OK   every drained capture, at both burst sizes, was marker_ok and CRC-valid")

    finally:
        runner.stop()
        motors.disarm()
        print("Motors stopped and disarmed.")

    print()
    print("=== Test Complete ===")


main()
