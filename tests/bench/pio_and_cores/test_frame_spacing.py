# Test: do the transmitters space frames that are queued early as designed?
#
# Purpose: both transmitters hold off for 512 PIO cycles before they take the next word, so that a frame queued
# early does not reach the ESC while it is still replying (bidirectional: dshot_bidir_tx, after it releases the
# line) or still processing the previous frame (unidirectional: dshot, after the 16th bit). Per transmitter this
# queues four frames back to back (far faster than any command loop) on one motor, records the pin at 10 MHz
# (0.1us) with the pin recorder, and measures from the waveform alone where each frame starts, at DSHOT600 and
# DSHOT300, with words that end in a 1 bit and in a 0 bit. It shows the spacing, not that it is long enough for
# an ESC. The unidirectional part also checks the bit timing, and runs the previous 4-instruction program
# (autopull, no gap; test-local) as a control: its frames must come one frame length apart.
#
# Through the public API, per transmitter: MotorGroup.update() waits at its entrance until the transmitter has
# taken every queued frame, so that update() never queues more than one. Behind a queue: three frames are queued
# on the motor, then update() is called once; at most one word may be queued when it returns, and with the wait
# switched off (control) at least two. A burst: update() six times back to back with another throttle each; every
# throttle must come out in order, none dropped, no two frames closer than the spacing.
#
# No ESC and no motor: GPIO 10, nothing attached.
#
# Pass, at both speeds:
#   - dshot_bidir_tx is 12 instructions (with the receiver's 19 it must fit the block's 32), dshot is 8;
#   - all four frames complete, in the order queued, each with a valid CRC (inverted for bidirectional);
#   - frame start to frame start: bidirectional 650 PIO cycles (651 after a frame ending in a 0 bit), unidirectional
#     642 (643), within 0.15us (one cycle is 0.21us at DSHOT600). At DSHOT600 / DSHOT300: 135.4us / 270.8us
#     and 133.75us / 267.50us;
#   - unidirectional bit timing: mean bit period, high time of a 1 bit (6 of 8 cycles) and of a 0 bit (3 of 8)
#     within 0.08us at DSHOT600 and 0.16us at DSHOT300;
#   - unidirectional control: frames one frame length (128 PIO cycles) apart, at most 3us more;
#   - behind a queue: at most one word queued when update() returns, two or more in the control;
#   - the burst: six complete frames carrying the throttles in order, no two closer than the spacing, and never
#     more than one word queued after a call.
#
# Run with: python scripts/run_session.py test_frame_spacing.py

import gc
import utime
from array import array
from machine import Pin
from rp2 import PIO, asm_pio
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS, dshot, dshot_bidir_tx
from group_without_reply_gate import GroupWithoutReplyGate
from motor_group import MotorGroup
from pin_sampler import PinSampler
import dshot_frame_decode

PIN = 10
SAMPLE_HZ = 10_000_000
UNIT_NS = 100
BUF_BYTES = 10 * 1024           # 2ms of samples at 10 MHz
TOLERANCE_UNITS = 1.5           # 0.15us: less than one PIO cycle at either speed
THROTTLES = (48, 56, 128, 215, 100, 60)     # through update(): one per call
ARM_MS = 20
CONTROL_CYCLES = 128            # the control program: one frame length
CONTROL_LATE_UNITS = 30         # 3us: how much later than one frame length a control frame may start
SPEEDS = ((DSHOT_SPEEDS.DSHOT600, "DSHOT600"), (DSHOT_SPEEDS.DSHOT300, "DSHOT300"))


def make_bidirectional(speed):
    return BidirectionalDShot(0, Pin(PIN), speed, rx_state_machine_id=1)


def make_unidirectional(speed):
    return UnidirectionalDShot(4, Pin(PIN), speed)      # the other PIO block: the bidirectional programs fill the first


def bidirectional_group(speed):
    return GroupWithoutReplyGate([make_bidirectional(speed)])      # nothing replies here


def unidirectional_group(speed):
    return MotorGroup([make_unidirectional(speed)])


# values: 11-bit values, sent as given; their words end in 1 and 0 bits. period: cycles from frame start to
# frame start after a frame that ended in a 1 bit. inverted: bidirectional wire (idle high, inverted CRC).
TRANSMITTERS = (
    {"name": "bidirectional", "program": dshot_bidir_tx, "instructions": 12, "values": (0, 56, 128, 215),
     "period": 650, "inverted": True, "make": make_bidirectional, "group": bidirectional_group},
    {"name": "unidirectional", "program": dshot, "instructions": 8, "values": (56, 0, 128, 48),
     "period": 642, "inverted": False, "make": make_unidirectional, "group": unidirectional_group},
)


# The unidirectional program as it was before the hold-off: autopull, the next word follows the last bit with no gap
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def dshot_no_gap():
    wrap_target()
    label("start")
    out(x, 1)            .side(0)    [1]
    jmp(not_x, "zero")   .side(1)    [2]
    jmp("start")         .side(1)    [2]
    label("zero")
    jmp("start")         .side(0)    [2]
    wrap()


def split_every_16_bits(edges):
    """Frames cut by counting: frames with no gap between them cannot be cut by the decoder's gap rule."""
    frames = []
    falls = 0
    for t, level in edges:
        if level == 0:
            if falls % dshot_frame_decode.FRAME_BITS == 0:
                frames.append([])
            falls += 1
        if frames:
            frames[-1].append((t, level))
    return frames


def decode(sampler, buf, scratch, tx, gapless=False):
    edges = sampler.edges(buf, 0, scratch, UNIT_NS, idle_high=tx["inverted"])
    if gapless:
        return [dshot_frame_decode.decode_frame(f, inverted_crc=False) for f in split_every_16_bits(edges)]
    return dshot_frame_decode.decode_frames(edges, inverted_crc=tx["inverted"])


def spacing(tx, speed, before):
    """Frame start to frame start after the frame `before`, in 0.1us units."""
    return (tx["period"] + (0 if before["word"] & 1 else 1)) * 10_000_000 / speed


def record_frames(tx, motor, buf, scratch, send, gapless=False):
    """Activate motor, record while send() queues the frames, return decoded frames, or None when the
    recording did not complete. gapless: the frames follow each other with no gap, cut by counting bits."""
    sampler = PinSampler(Pin(PIN), sample_hz=SAMPLE_HZ)
    try:
        motor.activate()
        utime.sleep_ms(2)
        sampler.start(buf)
        send()
        while not sampler.done():
            pass
        if not sampler.finish():
            return None
        return decode(sampler, buf, scratch, tx, gapless)
    finally:
        motor.deactivate()
        sampler.close()


def check_timing(name, speed, complete, failures):
    tolerance = (0.8 if speed == DSHOT_SPEEDS.DSHOT600 else 1.6)       # 0.08us / 0.16us, in 0.1us units
    cycle = 10_000_000 / speed
    s = dshot_frame_decode.timing_summary(complete)
    print("  bit timing: period %.3fus (expected %.3f), high of a 1 bit %.3fus (%.3f), of a 0 bit %.3fus (%.3f)" % (
        s["period_mean"] / 10, 8 * cycle / 10, s["one_low"] / 10, 6 * cycle / 10, s["zero_low"] / 10, 3 * cycle / 10))
    for label, got, want in (("bit period", s["period_mean"], 8 * cycle), ("high of a 1 bit", s["one_low"], 6 * cycle),
                             ("high of a 0 bit", s["zero_low"], 3 * cycle)):
        if abs(got - want) > tolerance:
            failures.append("%s: %s %.3fus, expected %.3fus" % (name, label, got / 10, want / 10))


def burst(tx, speed, name, buf, scratch, failures):
    motor = tx["make"](speed)
    values = tx["values"]

    def send():
        for value in values:
            motor.send_frame(value)

    frames = record_frames(tx, motor, buf, scratch, send)
    if frames is None:
        failures.append(name + ": the recording did not complete")
        return
    complete = [f for f in frames if f["ok"]]
    print("%s: %d frames (%d complete)" % (name, len(frames), len(complete)))
    if len(frames) != len(values) or len(complete) != len(values):
        failures.append("%s: expected %d complete frames, got %d of %d" % (
            name, len(values), len(complete), len(frames)))
        return
    for frame, value in zip(complete, values):
        got = frame["word"] >> 5
        if got != value or not frame["crc_ok"]:
            failures.append("%s: frame carries %d (CRC ok: %s), queued %d" % (name, got, frame["crc_ok"], value))
    if not tx["inverted"]:
        check_timing(name, speed, complete, failures)
    for before, after in zip(complete, complete[1:]):
        expected = spacing(tx, speed, before)
        measured = after["start"] - before["start"]
        print("  after a frame ending in %d: next frame starts %.1fus later (expected %.1fus)" % (
            before["word"] & 1, measured / 10, expected / 10))
        if abs(measured - expected) > TOLERANCE_UNITS:
            failures.append("%s: frame spacing %.1fus, expected %.1fus" % (name, measured / 10, expected / 10))


def control(tx, speed, name, buf, scratch, failures):
    motor = tx["make"](speed)
    motor.program = dshot_no_gap        # activate() loads self.program

    def send():
        for value in tx["values"]:
            motor.send_frame(value)

    frames = record_frames(tx, motor, buf, scratch, send, True)
    if frames is None:
        failures.append(name + " control: the recording did not complete")
        return
    complete = [f for f in frames if f["ok"]]
    starts = [f["start"] for f in complete]
    gaps = ["%.1f" % ((b - a) / 10) for a, b in zip(starts, starts[1:])]
    print("%s control (no hold-off): %d frames (%d complete), frame starts apart, in us: %s" % (
        name, len(frames), len(complete), " ".join(gaps)))
    if len(complete) != len(tx["values"]):
        failures.append("%s control: expected %d complete frames, got %d" % (name, len(tx["values"]), len(complete)))
        return
    length = CONTROL_CYCLES * 10_000_000 / speed
    for a, b in zip(starts, starts[1:]):
        # one frame length; the second word may reach the FIFO a little late (the first frame is already running)
        if not length - TOLERANCE_UNITS <= (b - a) <= length + CONTROL_LATE_UNITS:
            failures.append("%s control: frames %.1fus apart, expected %.1fus" % (name, (b - a) / 10, length / 10))


def armed_group(tx, speed):
    """A one-motor group on the loose pin, armed, with nothing queued. None when arming did not complete."""
    group = tx["group"](speed)
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


def update_behind_queue(tx, speed, name, failures, wait):
    group = armed_group(tx, speed)
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


def update_burst(tx, speed, name, buf, scratch, failures):
    group = armed_group(tx, speed)
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
        frames = decode(sampler, buf, scratch, tx)
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
    gaps = []
    for before, after in zip(complete, complete[1:]):
        least = spacing(tx, speed, before)
        measured = after["start"] - before["start"]
        gaps.append("%.1f" % (measured / 10))
        if measured < least - TOLERANCE_UNITS:
            failures.append("%s through update(): frames %.1fus apart, at least %.1fus expected" % (
                name, measured / 10, least / 10))
    print("  frame starts apart, in us: " + " ".join(gaps))


def main():
    print("=== Frame spacing of both transmitters (no ESC, GPIO %d, nothing attached), 10 MHz recording ===" % PIN)
    failures = []
    gc.collect()
    buf = bytearray(BUF_BYTES)
    scratch = array('I', [0] * 512)
    for tx in TRANSMITTERS:
        label = tx["name"]
        instructions = len(tx["program"][0])
        print("%s: %d instructions" % (label, instructions))
        if instructions != tx["instructions"]:
            failures.append("%s program is %d instructions, expected %d" % (label, instructions, tx["instructions"]))
        for speed, speed_name in SPEEDS:
            name = label + " " + speed_name
            burst(tx, speed, name, buf, scratch, failures)
            if not tx["inverted"]:
                control(tx, speed, name, buf, scratch, failures)
        for speed, speed_name in SPEEDS:
            name = label + " " + speed_name
            gc.collect()
            update_behind_queue(tx, speed, name, failures, True)
            update_behind_queue(tx, speed, name, failures, False)
            gc.collect()
            update_burst(tx, speed, name, buf, scratch, failures)

    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
