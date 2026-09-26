# Standalone calibration/diagnostic tool: captures the sample receiver's raw
# oversampled replies (dshot_bidir_rx) for measuring a real
# driver/dshot_profiles.py BIDIR_PROFILES expected_ratio for a specific ESC
# unit. Not wired into MotorGroup or BidirectionalDShot (which only has the
# frame receiver, dshot_bidir_rx_rle, with no period search of its own to
# fall back on - see decision/ADR-002-bidirectional-dshot.md's "run-length
# capture" section) - this builds a bare TX/raw-RX pair directly, the way the
# low-level usage example in CLAUDE.md does for a single motor.
#
# rx_speed is reused from the existing BIDIR_PROFILES entry for this DShot
# speed: it sets the receiver's oversampling density, a fixed design choice
# that works across ESC units (see dshot_bidir_rx's own comment - the program
# assumes nothing about the real bit period), not something to calibrate
# itself. expected_ratio - the actual measured bit period, which DOES vary
# per unit's oscillator - is what this tool exists to help measure.
#
# Prints each capture as one line: "CAPTURE " followed by 4 hex words,
# space-separated. Redirect stdout to a file to keep them:
#   python scripts/deploy.py calibrate_bidir_rx.py > calibration.log
# Then paste the printed words into scripts/dshot_bidir_decode.py's
# analyze_capture()/estimate_bit_period() (expected_ratio=None runs its
# brute-force sweep) to measure this ESC's real bit period, or write a small
# parser if calibrating routinely - this tool's job stops at getting clean
# raw captures off the device.
#
# Wiring: channel 1 (GPIO 6) is the motor being calibrated; channels 2-4
# (GPIO 7/8/9) are idle unidirectional, since the ESC only completes its arm
# handshake with valid signal on all 4 (as in tests/harness scenarios).
#
# UNTESTED ON HARDWARE as of 2026-09-26 - written with the bench powered off.
# Verify it end to end (arms, captures look sane, a pasted batch measures a
# plausible expected_ratio) before relying on it to calibrate a new ESC unit.

from array import array
from machine import Pin
from rp2 import StateMachine
from dshot_pio import DShotPIO, UnidirectionalDShot, dshot_bidir_tx, dshot_bidir_rx, DSHOT_SPEEDS
from dshot_profiles import BIDIR_PROFILES
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
THROTTLE = 100
ARM_MS = 3000
CAPTURES_WANTED = 200

# Comfortably longer than one command+reply cycle at either supported speed -
# see tests/experimental/test_rle_stalled_drain.py's header comment on why an
# interval this generous matters: too fast re-arms TX before the ESC's reply
# has finished, corrupting it by interference in a way indistinguishable from
# a receiver bug once only the captured words are inspected.
SEND_INTERVAL_US = 700


class _CalibrationTx(DShotPIO):
    """A bidirectional TX, only for send_throttle_command()'s CRC inversion
    (see DShotPIO.bidirectional). Not BidirectionalDShot itself, which only
    builds the frame receiver - this needs the raw sample receiver instead."""
    bidirectional = True


def main():
    profile = BIDIR_PROFILES[DSHOT_SPEED]

    pin = Pin(6)
    # See BidirectionalDShot's constructor comment: a weak pull-up holds the
    # line at idle-HIGH while neither side drives it, between frames.
    pin.init(Pin.IN, Pin.PULL_UP)
    tx = _CalibrationTx(0, pin, DSHOT_SPEED, dshot_bidir_tx)
    rx = StateMachine(1, dshot_bidir_rx, freq=profile["rx_speed"], in_base=pin)

    others = [
        UnidirectionalDShot(4, Pin(7), DSHOT_SPEED),
        UnidirectionalDShot(5, Pin(8), DSHOT_SPEED),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEED),
    ]

    tx.start()
    rx.active(1)
    for motor in others:
        motor.start()

    try:
        print("Arming for %dms..." % ARM_MS)
        arm_end = utime.ticks_add(utime.ticks_ms(), ARM_MS)
        while utime.ticks_diff(arm_end, utime.ticks_ms()) > 0:
            tx.send_throttle_command(0)
            for motor in others:
                motor.send_throttle_command(0)
            utime.sleep_us(SEND_INTERVAL_US)
        print("Armed - spinning at throttle %d, capturing %d replies..." %
              (THROTTLE, CAPTURES_WANTED))

        words = array('I', [0, 0, 0, 0])
        captured = 0
        while captured < CAPTURES_WANTED:
            tx.send_throttle_command(THROTTLE)
            for motor in others:
                motor.send_throttle_command(THROTTLE)
            utime.sleep_us(SEND_INTERVAL_US)
            if rx.rx_fifo() >= 4:
                rx.get(words)
                print("CAPTURE " + " ".join(hex(w) for w in words))
                captured += 1

        print()
        print("Done - %d captures printed above. Paste them into a Python list and run "
              "scripts/dshot_bidir_decode.py's analyze_capture()/estimate_bit_period() "
              "to measure this ESC's real expected_ratio." % captured)

    finally:
        for _ in range(4):
            tx.send_throttle_command(0)
            for motor in others:
                motor.send_throttle_command(0)
            utime.sleep_us(SEND_INTERVAL_US)
        tx.drain()
        for motor in others:
            motor.drain()
        tx.stop()
        rx.active(0)
        for motor in others:
            motor.stop()
        print("Motors stopped.")


main()
