# Test: cross-core behaviour of BidirectionalDShot's one-slot capture
#
# Purpose: MotorGroup.update() (Core 1) publishes each completed telemetry
# capture into a single slot (CaptureMailbox) via drain_rx(), and the application
# (Core 0) reads it with latest_capture(). The slot is one array element holding
# the reply frame and a token that steps on every publish, so a read is one load:
# it cannot return half of one capture and half of the next, and it is never
# refused. MicroPython on the RP2350 runs the two cores truly in parallel with no
# global interpreter lock, so this test hammers that against a writer far faster
# than a real ESC.
#
# No motor and no ESC power are needed: the RX state machine is replaced by a
# fake that supplies synthetic words, so the real drain_rx() and
# latest_capture() run against a FIFO that never runs dry. The state machines
# are never started and nothing is transmitted.
#
# The k-th published capture carries token ((k - 1) mod TOKEN_MAX) + 1, and its
# frame is derived from the token alone, so the reader can check that the frame
# it holds belongs to the token it holds. The token also gives the number of
# captures published between two reads (the step modulo TOKEN_MAX), which is
# summed and compared with what the writer published.
#
# Pass: every value read is consistent (frame matches token), no read returns
# None once the first capture is published, and the reader ran long enough to
# have collided with the writer many times. Reported, not judged: how many
# captures the reader missed between reads, and whether the token count agrees
# with the writer's (it can only fall short when the reader is paused for more
# than TOKEN_MAX publishes).

import _thread
from machine import Pin
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
from capture_mailbox import FRAME_BITS, FRAME_MASK, TOKEN_MAX
import utime

RUN_MS = 15000
MIN_READS = 10000

# Pin and state machines are only claimed, never started. GPIO 10 is not
# wired to an ESC channel.
PIN = 10
SM_ID = 0
RX_SM_ID = 1


def frame_for(token):
    # A distinct, non-zero 21-bit frame per token value
    return ((token * 0x1003F + 0x5A5A5) & FRAME_MASK) | 1


class FakeRxStateMachine:
    """A receive FIFO that always holds a capture, handing out capture 0, 1, 2, ... in order."""

    def __init__(self):
        self.n = 0
        self.next_frame = 0

    def rx_fifo(self):
        # The frame is worked out here, outside the writer's publish, so get()
        # below is as cheap as the real bulk read
        self.next_frame = frame_for(self.n % TOKEN_MAX + 1)
        return 1

    def get(self, buf):
        buf[0] = self.next_frame
        self.n += 1


class Writer:
    def __init__(self, motor):
        self.motor = motor
        self.running = False
        self.stopped = True
        self.error = None
        self.calls = 0

    def loop(self):
        motor = self.motor
        try:
            while self.running:
                motor.drain_rx()
                self.calls += 1
        except Exception as e:
            self.error = e
        finally:
            self.stopped = True


def test_capture_slot_stress():
    print("=== Capture Slot Stress Test (no motor, no ESC power) ===")

    motor = BidirectionalDShot(SM_ID, Pin(PIN), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=RX_SM_ID)
    fake = FakeRxStateMachine()
    motor.mailbox.source = fake
    writer = Writer(motor)

    reads = 0
    new_values = 0
    none_after_start = 0
    bad_records = 0
    steps_total = 0
    biggest_step = 0
    first_bad = None
    started = False
    last_token = 0
    last_value = 0

    writer.running = True
    writer.stopped = False
    _thread.start_new_thread(writer.loop, ())

    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < RUN_MS:
        if writer.error is not None:
            raise writer.error

        value = motor.latest_capture()
        if value is None:
            if started:
                none_after_start += 1
            continue
        started = True
        reads += 1

        token = value >> FRAME_BITS
        if (value & FRAME_MASK) != frame_for(token):
            bad_records += 1
            if first_bad is None:
                first_bad = value

        if value != last_value:
            new_values += 1
            step = (token - last_token) % TOKEN_MAX
            if step == 0:
                step = TOKEN_MAX
            steps_total += step
            if step > biggest_step:
                biggest_step = step
            last_token = token
            last_value = value

    writer.running = False
    for _ in range(100):
        if writer.stopped:
            break
        utime.sleep_ms(10)

    published = fake.n
    print("  writer drain_rx calls:        " + str(writer.calls))
    print("  captures published:           " + str(published))
    print("  reader reads (valid):         " + str(reads))
    print("  of which a new value:         " + str(new_values))
    print("  reader None after the first:  " + str(none_after_start))
    print("  captures counted by token steps: " + str(steps_total) + " (writer published " + str(published) + ")")
    print("  biggest step between two new values: " + str(biggest_step))
    print("  INCONSISTENT values (frame does not match token): " + str(bad_records))
    if first_bad is not None:
        print("  first inconsistent value: " + str(first_bad))

    if writer.error is not None:
        raise writer.error
    if bad_records:
        raise Exception("FAIL slot returned a frame that does not belong to its token")
    if none_after_start:
        raise Exception("FAIL " + str(none_after_start) + " reads returned nothing after the first capture - a read must never be refused")
    if reads < MIN_READS:
        raise Exception("FAIL only " + str(reads) + " reads - not enough to mean anything")
    print("  OK   no inconsistent values, no refused reads")

    print()
    print("=== Test Complete ===")


test_capture_slot_stress()
