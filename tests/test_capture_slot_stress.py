# Test: cross-core consistency of BidirectionalDShot's one-slot capture
#
# Purpose: MotorThrottleGroup.update() (Core 1) publishes each completed
# telemetry capture into a single slot via drain_rx(), and the application
# (Core 0) reads it with latest_capture(). The slot is guarded by a seqlock made
# of plain attribute and array writes, and MicroPython on the RP2350 runs the
# two cores truly in parallel with no global interpreter lock, so nothing but
# the protocol itself keeps a reader from seeing half of one capture and half of
# the next. This test hammers that protocol far harder than a real ESC can.
#
# No motor and no ESC power are needed: the RX state machine is replaced by a
# fake that supplies synthetic words, so the real drain_rx() and
# latest_capture() run against a FIFO that never runs dry. The state machines
# are never started and nothing is transmitted.
#
# Capture n consists of four different words derived from n, and captures are
# published in order, so the k-th published capture (sequence k) must be exactly
# capture k-1. The reader checks every capture it reads against that, which
# catches a torn record, a stale sequence number and mixed captures alike.
#
# Pass: no inconsistent record, sequence never goes backwards, and the writer
# and reader both ran long enough to have collided many times.

import _thread
from machine import Pin
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
import utime

RUN_MS = 15000
MIN_READS = 10000

# Pin and state machines are only claimed, never started. GPIO 10 is not
# wired to an ESC channel.
PIN = 10
SM_ID = 0
RX_SM_ID = 1


def word(n, j):
    # Four distinct 32-bit words per capture, all derived from the capture number
    return ((n * 0x01000193 + 0x811C9DC5) ^ (j * 0x11111111)) & 0xFFFFFFFF


class FakeRxStateMachine:
    """A receive FIFO that is never empty, handing out capture 0, 1, 2, ... in order."""

    def __init__(self):
        self.n = 0
        self.j = 0

    def rx_fifo(self):
        return 4

    def get(self):
        w = word(self.n, self.j)
        self.j += 1
        if self.j == 4:
            self.j = 0
            self.n += 1
        return w


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
                motor.drain_rx(True)
                self.calls += 1
        except Exception as e:
            self.error = e
        finally:
            self.stopped = True


def test_capture_slot_stress():
    print("=== Capture Slot Stress Test (no motor, no ESC power) ===")

    motor = BidirectionalDShot(SM_ID, Pin(PIN), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=RX_SM_ID)
    motor.rx_sm = FakeRxStateMachine()
    writer = Writer(motor)

    reads = 0
    none_reads = 0
    bad_records = 0
    backwards = 0
    odd_seen = 0
    last_seq = 0
    first_bad = None

    writer.running = True
    writer.stopped = False
    _thread.start_new_thread(writer.loop, ())

    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < RUN_MS:
        if writer.error is not None:
            raise writer.error

        # Sampling the raw counter shows how often a read lands inside the
        # writer's update window - the case the seqlock exists for
        if motor.slot_seq & 1:
            odd_seen += 1

        capture = motor.latest_capture()
        if capture is None:
            none_reads += 1
            continue

        ticks_us, seq, words = capture
        reads += 1

        if seq < last_seq:
            backwards += 1
        last_seq = seq

        n = seq - 1
        if words != (word(n, 0), word(n, 1), word(n, 2), word(n, 3)):
            bad_records += 1
            if first_bad is None:
                first_bad = (seq, words)

    writer.running = False
    for _ in range(100):
        if writer.stopped:
            break
        utime.sleep_ms(10)

    published = motor.slot_seq >> 1
    print("  writer drain_rx calls:        " + str(writer.calls))
    print("  captures published:           " + str(published))
    print("  reader reads (valid):         " + str(reads))
    print("  reader None (gave up/empty):  " + str(none_reads))
    print("  reads seeing writer mid-update (odd seq): " + str(odd_seen))
    print("  sequence went backwards:      " + str(backwards))
    print("  INCONSISTENT records:         " + str(bad_records))
    if first_bad is not None:
        print("  first inconsistent record: seq=" + str(first_bad[0]) + " words=" + str(first_bad[1]))

    if writer.error is not None:
        raise writer.error
    if bad_records or backwards:
        raise Exception("FAIL slot returned an inconsistent record")
    if reads < MIN_READS:
        raise Exception("FAIL only " + str(reads) + " reads - not enough to mean anything")
    if odd_seen == 0:
        raise Exception("FAIL reader never saw the writer mid-update - the race was not exercised")
    print("  OK   no torn or out-of-order records")

    print()
    print("=== Test Complete ===")


test_capture_slot_stress()
