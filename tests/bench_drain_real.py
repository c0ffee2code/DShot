# Benchmark: what draining the RX FIFO costs on real state machines, and where
# the command loop's heap churn comes from
#
# Purpose: bench_loop_gaps.py showed the command loop allocating ~113KB/s on its
# own and slowing sharply when a bidirectional motor is present. This breaks a
# tick down on real hardware (state machines on unused GPIO 10-13, no ESC): what
# each call costs and how much heap it allocates, for the send path and the
# drain path, and what a bulk read - StateMachine.get(buffer), which fills an
# array without creating a Python integer per word - would change.
#
# No ESC replies, so the RX state machine captures the transmitter's own
# waveform: realistic words arriving at a realistic rate. Nothing reaches a motor.

import gc
import utime
from array import array
from machine import Pin

from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup

TICKS = 3000


def measure(label, fn, n=TICKS, quiet=False):
    """Run fn(n); report microseconds per call and heap bytes per call."""
    gc.collect()
    gc.disable()
    m0 = gc.mem_alloc()
    t0 = utime.ticks_us()
    fn(n)
    dt = utime.ticks_diff(utime.ticks_us(), t0)
    allocated = gc.mem_alloc() - m0
    gc.enable()
    if not quiet:
        print("  %-52s %8.1f us/call  %8.1f bytes/call" % (label, dt / n, allocated / n))
    return dt / n, allocated / n


def build(bidir_count, uni_count):
    motors = []
    sm = 0
    pin = 0
    for _ in range(bidir_count):
        motors.append(BidirectionalDShot(sm, Pin(10 + pin), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=sm + 1))
        sm += 2
        pin += 1
    for _ in range(uni_count):
        motors.append(UnidirectionalDShot(sm, Pin(10 + pin), DSHOT_SPEEDS.DSHOT300))
        sm += 2
        pin += 1
    group = MotorThrottleGroup(motors)
    group.arm(1)
    for _ in range(200):
        group.update()
        if group.is_armed():
            break
    return group


def main():
    print("=== Drain and allocation benchmark, real state machines, no ESC ===")

    print("Send path (unidirectional motor, real state machine)")
    group = build(0, 1)
    uni = group.motors[0]
    for throttle in (0, 100, 300, 1500):
        def send(n, t=throttle):
            s = uni.send_throttle_command
            for _ in range(n):
                s(t)
        measure("send_throttle_command(%d)" % throttle, send)

    def update_uni(n):
        u = group.update
        for _ in range(n):
            u()
    measure("update(), 1 unidirectional (throttle 0)", update_uni)
    group.disarm()
    print()

    print("Drain path (1 bidirectional motor, TX echo captured by RX)")
    group = build(1, 0)
    motor = group.motors[0]
    rx = motor.rx_sm

    def update_bidir(n):
        u = group.update
        for _ in range(n):
            u()
    measure("update(), 1 bidirectional, echo captures arriving", update_bidir)
    published = motor.mailbox.slot_seq >> 1
    print("    captures published during that run: %d" % published)

    # Stop the loop from draining, let the FIFO fill, then time the pieces
    group.disarm()
    group.arm(1)
    for _ in range(200):
        group.update()
        if group.is_armed():
            break
    # fill: send a few frames without draining so 4 words are waiting
    for _ in range(6):
        motor.send_throttle_command(0)
        utime.sleep_ms(1)
    print("    words waiting in the RX FIFO now: %d" % rx.rx_fifo())

    def call_rx_fifo(n):
        f = rx.rx_fifo
        for _ in range(n):
            f()
    measure("rx_sm.rx_fifo()  (one C method call)", call_rx_fifo)

    def call_rx_fifo_attr(n):
        for _ in range(n):
            rx.rx_fifo()
    measure("rx_sm.rx_fifo()  (attribute lookup each time)", call_rx_fifo_attr)
    print()

    # words per get(): refill between measurements, so time single get() calls
    words = []
    total_us = 0
    total_bytes = 0
    reps = 200
    for _ in range(reps):
        while rx.rx_fifo() < 4:
            motor.send_throttle_command(0)
            utime.sleep_us(300)
        gc.collect()
        gc.disable()
        m0 = gc.mem_alloc()
        t0 = utime.ticks_us()
        w0 = rx.get()
        w1 = rx.get()
        w2 = rx.get()
        w3 = rx.get()
        dt = utime.ticks_diff(utime.ticks_us(), t0)
        total_bytes += gc.mem_alloc() - m0
        gc.enable()
        total_us += dt
        if len(words) < 8:
            words.append((w0, w1, w2, w3))
        while rx.rx_fifo():
            rx.get()
    print("  %-52s %8.1f us/call  %8.1f bytes/call" % ("4 x rx_sm.get()  (one capture, word by word)", total_us / reps, total_bytes / reps))
    print("    sample captures: %s" % [["%08x" % w for w in c] for c in words[:2]])

    buf = array('I', [0, 0, 0, 0])
    total_us = 0
    total_bytes = 0
    for _ in range(reps):
        while rx.rx_fifo() < 4:
            motor.send_throttle_command(0)
            utime.sleep_us(300)
        gc.collect()
        gc.disable()
        m0 = gc.mem_alloc()
        t0 = utime.ticks_us()
        rx.get(buf)
        dt = utime.ticks_diff(utime.ticks_us(), t0)
        total_bytes += gc.mem_alloc() - m0
        gc.enable()
        total_us += dt
        while rx.rx_fifo():
            rx.get()
    print("  %-52s %8.1f us/call  %8.1f bytes/call" % ("rx_sm.get(array)  (bulk read of one capture)", total_us / reps, total_bytes / reps))
    print("    buffer after bulk read: %s" % ["%08x" % v for v in buf])

    # Does bulk read block or misbehave with fewer than 4 words waiting? Only
    # ever call it with a full capture waiting, so just record the guard cost.
    def guard(n):
        f = rx.rx_fifo
        for _ in range(n):
            if f() >= 4:
                pass
    measure("guard: rx_fifo() >= 4", guard)
    print()

    print("Whole-tick comparison, drain via the current mailbox vs a bulk-read prototype")
    group.disarm()
    group.arm(1)
    for _ in range(200):
        group.update()
        if group.is_armed():
            break

    slot = array('I', [0, 0, 0, 0])
    state = [0]

    def proto_update(n):
        send = motor.send_throttle_command
        fifo = rx.rx_fifo
        get = rx.get
        ticks = utime.ticks_us
        for _ in range(n):
            send(0)
            if fifo() >= 4:
                get(slot)
                state[0] += 1

    def current_update(n):
        send = motor.send_throttle_command
        drain = motor.drain_rx
        for _ in range(n):
            send(0)
            drain(True)

    for _ in range(2):
        m0 = motor.mailbox.slot_seq >> 1
        us, by = measure("send + drain_rx (current)", current_update, quiet=True)
        got = (motor.mailbox.slot_seq >> 1) - m0
        print("  %-52s %8.1f us/tick  %8.1f bytes/tick  (%d captures)" % ("send + drain_rx (current)", us, by, got))
        state[0] = 0
        us, by = measure("send + bulk read (prototype)", proto_update, quiet=True)
        print("  %-52s %8.1f us/tick  %8.1f bytes/tick  (%d captures)" % ("send + bulk read (prototype)", us, by, state[0]))
    group.disarm()
    print("=== Benchmark Complete ===")


main()
