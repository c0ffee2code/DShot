# Benchmark: what the Python-level pieces of the command loop cost on this board
#
# Purpose: find where the command loop spends its time. Runs with no motor and no
# ESC signal: the state machines are replaced by fakes that accept and discard, so
# only CPU cost is measured (no waiting on the wire). Nothing is transmitted.
#
# Part A measures the cost of individual Python operations on this board, so the
# part B numbers can be read as a count of those operations. Part B times the
# real driver code: send_throttle_command(), MotorThrottleGroup.update() for 1-4
# motors, and the drain path with and without a reply waiting.
#
# Each figure is microseconds per operation, with the cost of the measuring loop
# itself subtracted.

import gc
import utime
from array import array
from machine import Pin

from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup, ARMED, ARMING

N = 4000
PINS = (10, 11, 12, 13)


def per_op(label, fn, n=N, overhead=0.0):
    gc.collect()
    t0 = utime.ticks_us()
    fn(n)
    dt = utime.ticks_diff(utime.ticks_us(), t0)
    us = dt / n - overhead
    print("  %-46s %7.2f us" % (label, us))
    return us


# ------------------------------------------------------------------ part A

class Obj:
    cls_attr = 5

    def __init__(self):
        self.inst_attr = 5

    def method(self):
        return None


GLOBAL_VALUE = 5


def empty_function():
    return None


def loop_empty(n):
    for _ in range(n):
        pass


def part_a():
    print("A. Cost of individual operations (loop overhead subtracted)")
    base = per_op("empty loop iteration", loop_empty)
    o = Obj()
    arr = array('I', [1, 2, 3, 4])
    lst = [1, 2, 3, 4]

    def t_function(n):
        f = empty_function
        for _ in range(n):
            f()

    def t_method(n):
        for _ in range(n):
            o.method()

    def t_builtin(n):
        for _ in range(n):
            abs(5)

    def t_inst_attr(n):
        for _ in range(n):
            o.inst_attr

    def t_cls_attr(n):
        for _ in range(n):
            o.cls_attr

    def t_global(n):
        for _ in range(n):
            GLOBAL_VALUE

    def t_local(n):
        v = 5
        for _ in range(n):
            v

    def t_arr_read(n):
        for _ in range(n):
            arr[1]

    def t_arr_write(n):
        for _ in range(n):
            arr[1] = 7

    def t_list_read(n):
        for _ in range(n):
            lst[1]

    def t_ticks_us(n):
        for _ in range(n):
            utime.ticks_us()

    def t_ticks_ms(n):
        for _ in range(n):
            utime.ticks_ms()

    def t_ticks_diff(n):
        a = utime.ticks_ms()
        for _ in range(n):
            utime.ticks_diff(a, 3)

    def t_int_ops(n):
        v = 12345
        for _ in range(n):
            v ^ (v >> 4) ^ (v >> 8)

    def t_alloc_tuple(n):
        for _ in range(n):
            (1, 2, 3, 4)

    def t_alloc_list(n):
        for _ in range(n):
            [1, 2, 3, 4]

    def t_bigint(n):
        v = 0x7FFFFFFF
        for _ in range(n):
            v + 1

    for label, fn in (
        ("call to a Python function", t_function),
        ("call to a Python method", t_method),
        ("call to a builtin (abs)", t_builtin),
        ("instance attribute load", t_inst_attr),
        ("class attribute load via instance", t_cls_attr),
        ("global load", t_global),
        ("local load", t_local),
        ("array('I') read", t_arr_read),
        ("array('I') write", t_arr_write),
        ("list read", t_list_read),
        ("utime.ticks_us()", t_ticks_us),
        ("utime.ticks_ms()", t_ticks_ms),
        ("utime.ticks_diff()", t_ticks_diff),
        ("v ^ (v>>4) ^ (v>>8)", t_int_ops),
        ("allocate a 4-tuple (constant, no alloc?)", t_alloc_tuple),
        ("allocate a 4-list", t_alloc_list),
        ("int add that overflows 30 bits (bignum)", t_bigint),
    ):
        per_op(label, fn, overhead=base)
    print()


# ------------------------------------------------------------------ part B

class FakeSM:
    """Accepts and discards; abs() is a cheap builtin, so put() costs little."""
    put = staticmethod(abs)

    def __init__(self):
        self.fifo = []

    def active(self, v):
        pass

    def restart(self):
        pass

    def rx_fifo(self):
        return len(self.fifo)

    def get(self):
        return self.fifo.pop(0)


class FakeRx:
    """RX FIFO that is empty until words are assigned to it."""

    def __init__(self):
        self.words = []

    def rx_fifo(self):
        return len(self.words)

    def get(self, buf):
        for i in range(len(buf)):
            buf[i] = self.words[i]
        self.words = self.words[len(buf):]


def make_unidirectional(index):
    m = UnidirectionalDShot(index * 2, Pin(PINS[index]), DSHOT_SPEEDS.DSHOT300)
    m.sm = FakeSM()
    return m


def make_bidirectional(index):
    m = BidirectionalDShot(index * 2, Pin(PINS[index]), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=index * 2 + 1)
    m.sm = FakeSM()
    m.mailbox.source = FakeRx()
    return m


def part_b():
    print("B. The real driver code, with fake state machines (CPU cost only)")

    m = make_unidirectional(0)
    b = make_bidirectional(1)

    def t_send(n):
        send = m.send_throttle_command
        for _ in range(n):
            send(300)

    def t_send_bidir(n):
        send = b.send_throttle_command
        for _ in range(n):
            send(300)

    per_op("send_throttle_command (unidirectional)", t_send)
    per_op("send_throttle_command (bidirectional, inverted CRC)", t_send_bidir)

    # what the CRC and packet build alone would cost, to see how much of the
    # call is the maths and how much is call overhead and validation
    def t_send_math_only(n):
        for _ in range(n):
            throttle = 300
            packet = throttle << 1
            crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
            packet = ((packet << 4) | crc) << 16

    per_op("  packet maths only, inline (no call, no checks)", t_send_math_only)

    # packet cache: how cheap the send could be if the packet were precomputed
    cache = {}

    def t_send_cached(n):
        put = m.sm.put
        for _ in range(n):
            packet = cache.get(300)
            if packet is None:
                cache[300] = packet = 1
            put(packet)

    per_op("  cached-packet send, inline (dict lookup + put)", t_send_cached)

    print()
    for count in (1, 2, 4):
        motors = [make_unidirectional(i) for i in range(count)]
        group = MotorThrottleGroup(motors)
        group.state = ARMED

        def t_update(n):
            update = group.update
            for _ in range(n):
                update()

        per_op("update(), %d unidirectional, ARMED" % count, t_update, n=2000)

    for count in (1, 2, 4):
        motors = [make_bidirectional(i) for i in range(count)]
        group = MotorThrottleGroup(motors)
        group.state = ARMED

        def t_update_bidir(n):
            update = group.update
            for _ in range(n):
                update()

        per_op("update(), %d bidirectional, ARMED, no reply waiting" % count, t_update_bidir, n=2000)

    motors = [make_bidirectional(0)]
    group = MotorThrottleGroup(motors)
    group.state = ARMED
    src = motors[0].mailbox.source
    words = [0x12345678, 0x9ABCDEF0, 0x0FEDCBA9, 0x87654321]

    def t_update_reply(n):
        update = group.update
        for _ in range(n):
            src.words = words[:]
            update()

    per_op("update(), 1 bidirectional, ARMED, 4-word reply waiting", t_update_reply, n=2000)

    group.state = ARMING
    group.arm_duration_ms = 1000000000
    group.arm_started_ms = utime.ticks_ms()

    def t_update_arming(n):
        update = group.update
        for _ in range(n):
            group.last_update_ms = utime.ticks_ms()
            update()

    per_op("update(), 1 bidirectional, ARMING", t_update_arming, n=2000)
    print()

    print("  raw_telemetry() / latest_capture() on the application side")
    group.state = ARMED
    motors[0].mailbox.slot_seq = 2

    def t_raw(n):
        for _ in range(n):
            group.raw_telemetry(0)

    def t_latest(n):
        for _ in range(n):
            motors[0].latest_capture()

    per_op("raw_telemetry(0)", t_raw)
    per_op("latest_capture()", t_latest)
    print()


def main():
    print("=== Command-loop CPU cost benchmark (no motor, no ESC signal) ===")
    print("free heap at start: %d bytes" % gc.mem_free())
    part_a()
    part_b()
    print("=== Benchmark Complete ===")


main()
