# Benchmark: how fast the real command loop runs, and what stalls it
#
# Purpose: bench_cpu_costs.py times the Python code with the hardware faked out.
# This runs the real thing - real state machines transmitting on unused GPIO 10-13
# (no ESC, no motor) - to answer two questions:
#
#   Part C. How many command-loop ticks per second does MotorThrottleGroup
#           achieve for 1, 2 and 4 motors at DSHOT300/600, and is that limited by
#           the CPU or by the wire? (Wire time per frame: 53us at DSHOT300, 27us
#           at DSHOT600.)
#   Part D. Does anything Core 0 does stall Core 1's loop? Core 1 runs update()
#           while Core 0 idles, polls telemetry, decodes, forces garbage
#           collection or allocates. The loop's worst gap and the number of long
#           gaps show whether an ESC waiting for the next frame would notice.
#
# One bidirectional motor is used so the RX state machine sees real words: with
# no ESC replying, it captures the transmitter's own waveform, which exercises
# the drain and decode paths with realistic data.

import gc
import _thread
import utime
from machine import Pin

from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup, ARMED
import gcr_decode

PINS = (10, 11, 12, 13)
RUN_MS = 3000


class Loop:
    """Runs group.update() on Core 1 and records how long each tick took."""

    def __init__(self, group):
        self.group = group
        self.running = False
        self.stopped = True
        self.error = None
        self.ticks = 0
        self.total_us = 0
        self.max_us = 0
        self.over_1ms = 0
        self.over_2ms = 0
        self.over_5ms = 0
        self.over_10ms = 0

    def loop(self):
        update = self.group.update
        ticks_us = utime.ticks_us
        ticks_diff = utime.ticks_diff
        n = 0
        total = 0
        worst = 0
        c1 = c2 = c5 = c10 = 0
        try:
            last = ticks_us()
            while self.running:
                update()
                now = ticks_us()
                gap = ticks_diff(now, last)
                last = now
                n += 1
                total += gap
                if gap > worst:
                    worst = gap
                if gap > 1000:
                    c1 += 1
                    if gap > 2000:
                        c2 += 1
                        if gap > 5000:
                            c5 += 1
                            if gap > 10000:
                                c10 += 1
        except Exception as e:
            self.error = e
        finally:
            self.ticks = n
            self.total_us = total
            self.max_us = worst
            self.over_1ms = c1
            self.over_2ms = c2
            self.over_5ms = c5
            self.over_10ms = c10
            self.stopped = True

    def start(self):
        self.running = True
        self.stopped = False
        _thread.start_new_thread(self.loop, ())

    def stop(self):
        self.running = False
        for _ in range(500):
            if self.stopped:
                return
            utime.sleep_ms(2)


def build(uni_count, bidir_count, speed):
    motors = []
    sm = 0
    pin_index = 0
    for _ in range(bidir_count):
        motors.append(BidirectionalDShot(sm, Pin(PINS[pin_index]), speed, rx_state_machine_id=sm + 1))
        sm += 2
        pin_index += 1
    for _ in range(uni_count):
        motors.append(UnidirectionalDShot(sm, Pin(PINS[pin_index]), speed))
        sm += 2
        pin_index += 1
    return MotorThrottleGroup(motors)


def arm(group):
    group.arm(1)
    for _ in range(200):
        group.update()
        if group.is_armed():
            return
    raise Exception("did not arm")


def report(label, loop, ms):
    if loop.error is not None:
        raise loop.error
    per_s = loop.ticks * 1000 / ms
    mean = loop.total_us / loop.ticks if loop.ticks else 0
    print("  %-34s %7.0f ticks/s  mean %6.0f us  max %6d us  >1ms:%d >2ms:%d >5ms:%d >10ms:%d" % (
        label, per_s, mean, loop.max_us, loop.over_1ms, loop.over_2ms, loop.over_5ms, loop.over_10ms))


def part_c():
    print("C. Command-loop rate, real state machines, no ESC")
    print("  (wire time for all motors' frames per tick: DSHOT300 53us/motor, DSHOT600 27us/motor)")
    for speed_name, speed, frame_us in (("DSHOT300", DSHOT_SPEEDS.DSHOT300, 53.3), ("DSHOT600", DSHOT_SPEEDS.DSHOT600, 26.7)):
        for uni, bidir in ((1, 0), (2, 0), (4, 0), (3, 1)):
            group = build(uni, bidir, speed)
            arm(group)
            loop = Loop(group)
            gc.collect()
            loop.start()
            utime.sleep_ms(RUN_MS)
            loop.stop()
            motors = uni + bidir
            group.disarm()
            report("%s %d uni + %d bidir" % (speed_name, uni, bidir), loop, RUN_MS)
            print("      wire-limited floor: %.0f us per tick" % (motors * frame_us))
    print()


def part_d():
    print("D. What Core 0 activity does to the Core 1 loop (DSHOT300, 1 bidir + 3 uni)")
    group = build(3, 1, DSHOT_SPEEDS.DSHOT300)
    bidir = group.motors[0]
    words_sample = [0x0F0F0F0F, 0x33333333, 0x55555555, 0x00FF00FF]
    ratio = bidir.expected_ratio

    def idle():
        utime.sleep_ms(RUN_MS)

    def poll():
        end = utime.ticks_add(utime.ticks_ms(), RUN_MS)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            group.raw_telemetry(0)

    def decode_loop():
        end = utime.ticks_add(utime.ticks_ms(), RUN_MS)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            capture = group.raw_telemetry(0)
            if capture is not None:
                gcr_decode.analyze_capture(capture[2], bidir.rx_clock_hz, ratio)
            else:
                utime.sleep_ms(1)

    def force_gc_small():
        end = utime.ticks_add(utime.ticks_ms(), RUN_MS)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            gc.collect()
            utime.sleep_ms(100)

    ballast = []

    def force_gc_big_heap():
        end = utime.ticks_add(utime.ticks_ms(), RUN_MS)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            gc.collect()
            utime.sleep_ms(100)

    def allocate():
        end = utime.ticks_add(utime.ticks_ms(), RUN_MS)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            junk = [(i, i + 1, i + 2, i + 3) for i in range(50)]
            junk = None
            utime.sleep_ms(1)

    for label, fn, big in (
        ("Core 0 idle (sleeping)", idle, False),
        ("Core 0 polls raw_telemetry() flat out", poll, False),
        ("Core 0 decodes every capture", decode_loop, False),
        ("Core 0 gc.collect() every 100ms", force_gc_small, False),
        ("Core 0 gc.collect() every 100ms, 150KB live", force_gc_big_heap, True),
        ("Core 0 allocates small objects", allocate, False),
    ):
        if big:
            ballast[:] = [(i, i + 1) for i in range(4500)]
        else:
            ballast[:] = []
        gc.collect()
        arm(group)
        loop = Loop(group)
        m0 = gc.mem_alloc()
        loop.start()
        fn()
        loop.stop()
        alloc = gc.mem_alloc() - m0
        group.disarm()
        report(label, loop, RUN_MS)
        if fn is idle:
            print("      heap allocated while Core 0 idle: %d bytes over %d ms (%d bytes/s, Core 1's own churn)" % (alloc, RUN_MS, alloc * 1000 // RUN_MS))
        published = bidir.mailbox.slot_seq >> 1
        print("      captures published: %d (%.0f/s)" % (published, published * 1000 / RUN_MS))
    ballast[:] = []
    print()


def main():
    print("=== Command-loop rate and stall benchmark (no motor, no ESC) ===")
    print("free heap at start: %d bytes" % gc.mem_free())
    part_c()
    part_d()
    print("=== Benchmark Complete ===")


main()
