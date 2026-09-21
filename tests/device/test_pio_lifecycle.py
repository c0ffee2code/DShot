# Test: what the PIO state machines are left in across arm/disarm cycles
#
# Purpose: the driver never releases its state machines or the programs it loaded;
# stop() only deactivates and restarts a state machine. This shows that is enough
# for the ways an application uses a MotorGroup:
#   1. Building a new group, arming and disarming it, over and over (as a test
#      script or an application that re-creates its motors does) must not use up
#      the block's instruction slots or its state machines.
#   2. Disarming must leave everything quiet: state machines inactive, the TX FIFO
#      empty, and each signal line at the level the ESC should see - released and
#      pulled up for a bidirectional motor, low for a unidirectional one.
#   3. Arming the same group again after a disarm must work as the first time did:
#      the receiver captures again, and the published capture starts over.
#
# No ESC and no motor: the motors sit on unused GPIO 10 (bidirectional, state
# machines 0 and 1) and 11 (unidirectional, state machine 2), and nothing is wired
# to them. With no ESC replying, the receiver captures the transmitter's own
# waveform, which is enough to show it is running.

import gc
from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
BIDIR_PIN = 10
UNI_PIN = 11
ARM_MS = 30
REBUILD_CYCLES = 30
REARM_CYCLES = 15
RUN_MS = 60


def build():
    return MotorGroup([
        BidirectionalDShot(0, Pin(BIDIR_PIN), DSHOT_SPEED, rx_state_machine_id=1),
        UnidirectionalDShot(2, Pin(UNI_PIN), DSHOT_SPEED),
    ])


def run_for(group, milliseconds):
    end = utime.ticks_add(utime.ticks_ms(), milliseconds)
    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        group.update()
        utime.sleep_us(300)


def arm_fully(group):
    group.arm(ARM_MS)
    end = utime.ticks_add(utime.ticks_ms(), ARM_MS + 500)
    while not group.is_armed():
        if utime.ticks_diff(end, utime.ticks_ms()) < 0:
            raise Exception("FAIL arming did not complete")
        group.update()
        utime.sleep_us(300)


def check(condition, label):
    if not condition:
        raise Exception("FAIL " + label)
    print("  OK   " + label)


def test_pio_lifecycle():
    print("=== PIO lifecycle test (no ESC, GPIO 10/11, nothing attached) ===")

    print("Rebuilding the group %d times..." % REBUILD_CYCLES)
    gc.collect()
    free_before = gc.mem_free()
    for cycle in range(REBUILD_CYCLES):
        group = build()
        arm_fully(group)
        run_for(group, 10)
        group.disarm()
        group = None
    gc.collect()
    print("  heap free before/after: %d / %d bytes" % (free_before, gc.mem_free()))
    check(True, "%d build/arm/disarm cycles raised nothing (no ENOMEM from programs or state machines)" % REBUILD_CYCLES)

    print("State left behind by disarm()...")
    group = build()
    bidir, uni = group.motors
    arm_fully(group)
    run_for(group, RUN_MS)
    group.disarm()
    check(not bidir.sm.active() and not bidir.rx_sm.active() and not uni.sm.active(),
          "all three state machines are inactive")
    check(bidir.sm.tx_fifo() == 0 and uni.sm.tx_fifo() == 0, "both TX FIFOs are empty")
    print("  RX FIFO words left after disarm: %d" % bidir.rx_sm.rx_fifo())
    print("  pin levels: bidirectional GPIO%d = %d, unidirectional GPIO%d = %d" % (
        BIDIR_PIN, Pin(BIDIR_PIN).value(), UNI_PIN, Pin(UNI_PIN).value()))
    check(Pin(BIDIR_PIN).value() == 1, "the bidirectional line is released and pulled up (high)")
    check(Pin(UNI_PIN).value() == 0, "the unidirectional line is parked low")

    print("Arming the same group %d more times..." % REARM_CYCLES)
    first_sequence = None
    for cycle in range(REARM_CYCLES):
        arm_fully(group)
        run_for(group, RUN_MS)
        capture = group.raw_telemetry(0)
        if capture is None:
            raise Exception("FAIL cycle %d: no capture after %dms armed - the receiver is not running" % (cycle, RUN_MS))
        if cycle == 0 or cycle == REARM_CYCLES - 1:
            print("  cycle %d: sequence %d after %dms" % (cycle, capture[1], RUN_MS))
        if first_sequence is None:
            first_sequence = capture[1]
        elif capture[1] > 2 * first_sequence:
            raise Exception("FAIL cycle %d: sequence %d against %d in the first cycle - the published capture did not start over" % (
                cycle, capture[1], first_sequence))
        group.disarm()
        if group.raw_telemetry(0) is not None:
            raise Exception("FAIL cycle %d: a capture was handed out after disarm" % cycle)
    check(True, "every re-arm captured again, and the sequence restarted each time")

    print()
    print("=== Test Complete ===")


test_pio_lifecycle()
