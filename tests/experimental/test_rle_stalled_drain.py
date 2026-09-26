# Test: does a stalled drain corrupt the frame receiver's captures?
#
# Purpose: W25 (bidirectional_dshot_review.md). ADR-002/ADR-005 document the
# sample receiver's own stalled-drain failure: it autopushes 4 times across
# one 128-sample capture, so a full RX FIFO stalls it mid-capture, and the
# capture that comes back is a short burst followed by idle-level words - a
# corrupted reply, not a missing one. dshot_bidir_rx_rle's own comment already
# argues this cannot happen to it, because it only pushes once, after all 21
# bits are already assembled in the ISR: a stall at that push holds a complete,
# correct value, not a partial one. This test checks that argument on
# hardware rather than trusting the comment.
#
# Method: arm and settle normally, then take manual control of the command
# loop (stop Core1Runner) and send a burst of frames with NO drain at all in
# between - enough to overflow the RX FIFO's default depth (4 one-word
# captures) several times over. Then drain everything with rx_read() (the
# diagnostic raw read, not drain_rx()/the mailbox - this wants every word that
# piled up, not just the latest) and decode each one.
#
# Pass: every capture that comes back decodes to marker_ok and CRC-valid -
# some replies are expected to be silently skipped (RX was stalled and
# couldn't start a new capture while blocked pushing the previous one), but
# none should come back garbled.
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
BURST_FRAMES = 20  # several times the RX FIFO's 4-word (4-capture) depth


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

        # Take manual control: Core1Runner stops calling update(), so nothing
        # drains the RX FIFO or sends throttle commands until this loop does.
        runner.stop()
        print("Core 1 stopped - sending a burst of %d frames with no drain at all..." % BURST_FRAMES)
        for _ in range(BURST_FRAMES):
            motor.send_throttle_command(THROTTLE)
            utime.sleep_us(motor.frame_us)

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

        if not frames:
            raise Exception("FAIL no captures drained after the burst - nothing to check")
        if bad:
            raise Exception("FAIL %d of %d drained captures were corrupted (see above)" %
                             (len(bad), len(frames)))
        print("  OK   all %d drained captures were marker_ok and CRC-valid" % len(frames))

    finally:
        # runner.stop() already called above in the success path; harmless to
        # call again if an exception hit before that point.
        runner.stop()
        motors.disarm()
        print("Motors stopped and disarmed.")

    print()
    print("=== Test Complete ===")


main()
