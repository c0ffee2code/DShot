# Test: does the bidirectional transmitter space frames that are queued early as designed?
#
# Purpose: dshot_bidir_tx holds off for 512 PIO cycles after it releases the line before it takes the next
# word, so that a frame queued while the ESC is still replying is sent after the reply, not into it. This
# queues four frames back to back (far faster than any command loop) on one bidirectional motor, records the
# pin at 10 MHz (0.1us) with the pin recorder, and measures from the waveform alone where each frame starts.
# It shows the spacing, not that the spacing is long enough for an ESC, and not that the line is released
# during the wait (the pull-up holds a released line high as well). Done at DSHOT600 and at DSHOT300, with
# words that end in a 1 bit and in a 0 bit (the two paths into the program's tail).
#
# Second part, through the public API. MotorGroup.update() waits at its entrance until the transmitter has
# taken every queued frame, so that update() never queues more than one. Two checks on a one-motor group:
#   - behind a queue: three frames are queued directly on the motor (one goes onto the line, two wait), then
#     update() is called once. It must wait until both queued frames are taken, so at most one word (its
#     own) is queued when it returns. The same is done with the wait switched off, as a control: then at
#     least two are queued. This does not depend on how fast the caller is;
#   - a burst: update() six times back to back, each call with another throttle. Every throttle must come
#     out, in order, none dropped, no two frames closer than the transmitter's spacing.
#
# No ESC and no motor: GPIO 10, nothing attached. The values are only numbers on a loose pin.
#
# Pass, at both speeds:
#   - dshot_bidir_tx is 12 instructions (with the receiver's 19 it must fit the block's 32);
#   - all four frames are complete, in the order queued, each with a valid inverted CRC;
#   - each frame starts 650 PIO cycles after the one before it (651 when that one ended in a 0 bit),
#     within 0.15us (one cycle is 0.21us at DSHOT600): 16 bits of 8 cycles, the tail, the 512-cycle hold-off
#     and the next frame's lead-in. That is 135.4us at DSHOT600 and 270.8us at DSHOT300, start to start;
#   - behind a queue: at most one word queued when update() returns, and two or more in the control;
#   - the burst: six complete frames carrying the six throttles in order, no two closer than that spacing,
#     and never more than one word queued after a call.
#
# Run with: python scripts/run_session.py test_reply_holdoff.py

import gc
import utime
from array import array
from machine import Pin
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS, dshot_bidir_tx
from group_without_reply_gate import GroupWithoutReplyGate
from pin_sampler import PinSampler
import dshot_frame_decode

PIN = 10
SAMPLE_HZ = 10_000_000
UNIT_NS = 100
BUF_BYTES = 10 * 1024           # 2ms of samples at 10 MHz
VALUES = (0, 56, 128, 215)      # 11-bit values, sent as given; the word of 0 ends in a 1 bit, of 56 and 128 in a 0 bit
EXPECTED_INSTRUCTIONS = 12
PERIOD_CYCLES = 650             # frame start to frame start, after a frame that ended in a 1 bit
TOLERANCE_UNITS = 1.5           # 0.15us: less than one PIO cycle at either speed
THROTTLES = (48, 56, 128, 215, 100, 60)     # through update(): one per call
ARM_MS = 20


def burst(speed, name, buf, scratch, failures):
    motor = BidirectionalDShot(0, Pin(PIN), speed, rx_state_machine_id=1)
    sampler = PinSampler(Pin(PIN), sample_hz=SAMPLE_HZ)
    try:
        motor.activate()
        utime.sleep_ms(2)
        sampler.start(buf)
        for value in VALUES:
            motor.send_frame(value)
        while not sampler.done():
            pass
        if not sampler.finish():
            failures.append(name + ": the recording did not complete")
            return
        frames = dshot_frame_decode.decode_frames(sampler.edges(buf, 0, scratch, UNIT_NS))
    finally:
        motor.deactivate()
        sampler.close()

    complete = [f for f in frames if f["ok"]]
    print("%s: %d frames (%d complete)" % (name, len(frames), len(complete)))
    if len(frames) != len(VALUES) or len(complete) != len(VALUES):
        failures.append("%s: expected %d complete frames, got %d of %d" % (
            name, len(VALUES), len(complete), len(frames)))
        return
    for frame, value in zip(complete, VALUES):
        got = frame["word"] >> 5
        if got != value or not frame["crc_ok"]:
            failures.append("%s: frame carries %d (CRC ok: %s), queued %d" % (name, got, frame["crc_ok"], value))

    units_per_cycle = 10_000_000 / speed
    for before, after in zip(complete, complete[1:]):
        last_bit = before["word"] & 1
        expected = (PERIOD_CYCLES + (0 if last_bit else 1)) * units_per_cycle
        measured = after["start"] - before["start"]
        print("  after a frame ending in %d: next frame starts %.1fus later (expected %.1fus)" % (
            last_bit, measured / 10, expected / 10))
        if abs(measured - expected) > TOLERANCE_UNITS:
            failures.append("%s: frame spacing %.1fus, expected %.1fus" % (name, measured / 10, expected / 10))


def armed_group(speed):
    """A one-motor group on the loose pin, armed, with nothing queued. None when arming did not complete."""
    group = GroupWithoutReplyGate([BidirectionalDShot(0, Pin(PIN), speed, rx_state_machine_id=1)])   # nothing replies here
    group.arm(ARM_MS)
    deadline = utime.ticks_add(utime.ticks_ms(), ARM_MS + 500)
    while not group.is_armed():
        if utime.ticks_diff(deadline, utime.ticks_ms()) < 0:
            group.deactivate()
            return None
        group.update()
        utime.sleep_us(1000)
    utime.sleep_ms(2)
    return group


def update_behind_queue(speed, name, failures, wait):
    group = armed_group(speed)
    if group is None:
        failures.append(name + ": arming did not complete")
        return
    motor = group.motors[0]
    if not wait:
        group.tx_state_machines = []    # control: update() finds nothing to wait for
    try:
        motor.send_frame(0)             # taken at once: on the line
        motor.send_frame(0)             # queued behind it
        motor.send_frame(0)             # and a second one
        began = utime.ticks_us()
        group.update()
        queued = motor.sm.tx_fifo()
        took = utime.ticks_diff(utime.ticks_us(), began)
    finally:
        group.deactivate()
    kind = "waiting" if wait else "control without the wait"
    print("%s update() behind two queued frames, %s: took %dus, %d word(s) queued when it returned" % (
        name, kind, took, queued))
    if wait and queued > 1:
        failures.append("%s update() behind two queued frames: %d words queued, at most 1 expected" % (name, queued))
    if not wait and queued < 2:
        failures.append("%s control without the wait: %d word queued, so the check of the wait proves nothing" % (
            name, queued))


def update_burst(speed, name, buf, scratch, failures):
    group = armed_group(speed)
    if group is None:
        failures.append(name + ": arming did not complete")
        return
    motor = group.motors[0]
    sampler = PinSampler(Pin(PIN), sample_hz=SAMPLE_HZ)
    throttles = group.throttles
    try:
        sampler.start(buf)
        began = utime.ticks_us()
        queued = 0
        for throttle in THROTTLES:
            throttles[0] = throttle
            group.update()
            queued = max(queued, motor.sm.tx_fifo())
        elapsed = utime.ticks_diff(utime.ticks_us(), began)
        while not sampler.done():
            pass
        if not sampler.finish():
            failures.append(name + ": the recording did not complete")
            return
        frames = dshot_frame_decode.decode_frames(sampler.edges(buf, 0, scratch, UNIT_NS))
    finally:
        group.set_throttle(0, 0)
        group.deactivate()
        sampler.close()

    complete = [f for f in frames if f["ok"]]
    sent = [f["word"] >> 5 for f in complete]
    print("%s through update(): %d calls took %dus, at most %d word(s) queued after a call; on the wire: %s" % (
        name, len(THROTTLES), elapsed, queued, sent))
    if sent != list(THROTTLES) or len(frames) != len(THROTTLES) or not all(f["crc_ok"] for f in complete):
        failures.append("%s through update(): expected %s with valid CRCs, got %s (%d frames)" % (
            name, list(THROTTLES), sent, len(frames)))
        return
    if queued > 1:
        failures.append("%s through update(): %d words queued after a call, at most 1 expected" % (name, queued))
    units_per_cycle = 10_000_000 / speed
    gaps = []
    for before, after in zip(complete, complete[1:]):
        least = (PERIOD_CYCLES + (0 if before["word"] & 1 else 1)) * units_per_cycle
        measured = after["start"] - before["start"]
        gaps.append("%.1f" % (measured / 10))
        if measured < least - TOLERANCE_UNITS:
            failures.append("%s through update(): frames %.1fus apart, at least %.1fus expected" % (
                name, measured / 10, least / 10))
    print("  frame starts apart, in us: " + " ".join(gaps))


def main():
    print("=== Reply hold-off of dshot_bidir_tx (no ESC, GPIO %d, nothing attached), 10 MHz recording ===" % PIN)
    failures = []

    instructions = len(dshot_bidir_tx[0])
    print("dshot_bidir_tx: %d instructions" % instructions)
    if instructions != EXPECTED_INSTRUCTIONS:
        failures.append("dshot_bidir_tx is %d instructions, expected %d" % (instructions, EXPECTED_INSTRUCTIONS))

    gc.collect()
    buf = bytearray(BUF_BYTES)
    scratch = array('I', [0] * 512)
    burst(DSHOT_SPEEDS.DSHOT600, "DSHOT600", buf, scratch, failures)
    burst(DSHOT_SPEEDS.DSHOT300, "DSHOT300", buf, scratch, failures)
    for speed, name in ((DSHOT_SPEEDS.DSHOT600, "DSHOT600"), (DSHOT_SPEEDS.DSHOT300, "DSHOT300")):
        gc.collect()
        update_behind_queue(speed, name, failures, True)
        update_behind_queue(speed, name, failures, False)
        gc.collect()
        update_burst(speed, name, buf, scratch, failures)

    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
