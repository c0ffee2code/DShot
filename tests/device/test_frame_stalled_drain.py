# Test: an undrained frame receiver holds a correct capture, it does not
# corrupt one.
#
# Purpose: W25 (bidirectional_dshot_review.md). dshot_bidir_rx_frame's own
# comment argues a full RX FIFO cannot corrupt a capture, only delay
# delivering an already-complete one: the 21st of its 21 reads is the one
# autopush fires on, and every bit is already shifted into the ISR by then, so
# the stall holds a finished value rather than an in-progress one - unlike
# dshot_bidir_rx, which autopushes mid-capture and can push a torn one.
#
# Commands are paced at BURST_INTERVAL_US, wide enough for a full ESC reply to
# complete before the next command re-arms TX - a tighter pacing (e.g.
# motor.frame_us alone, which is only the TX bit-shift time) corrupts replies
# by TX interference on the wire, indistinguishable from a genuine RX-FIFO-
# stall corruption in the drained result alone (see bidirectional_dshot_review.md's
# W25a).
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
# Wiring: channel 1 (GPIO 6) frame receiver, channels 2-4 (GPIO 7/8/9) idle
# unidirectional on the next PIO block.

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

# See this file's header comment for why this must exceed a full reply's duration.
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
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1),
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
            # + the reply gate's 2s, several seconds more after an ESC reboot (BUG-003)
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_MS + 8000:
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
