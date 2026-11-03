# EXAMPLE APPLICATION CODE - not part of the DShot library, and a diagnostic.
#
# Measures how steadily the command loop sends frames while Core 0 does a given piece of work. It builds
# the production set-up on unused pins - four bidirectional motors in a MotorGroup, Core1Runner at
# MotorGroup.UPDATE_INTERVAL_US, armed at zero throttle, nothing attached - starts the pin recorder
# (pin_sampler.py) and, for each piece of Core 0 work handed to measure(), records a number of
# windows of the four transmit pins and keeps the spread of the frame spacing.
#
# Used by tests/bench/pio_and_cores/test_core0_effect_on_pacing.py and test_core0_io_options.py.
# Needs the Pico only: GPIO 10-13, no ESC, no motor.

import gc
import utime
from machine import Pin
from array import array
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner, fast_stop
from pin_sampler import PinSampler, SAMPLE_HZ
import pin_timing

FIRST_PIN = 10
CHANNELS = 4
BUF_BYTES = 100 * 1024                 # 204.8 ms of samples
HIST_US = 4096
ARM_MS = 2000


def hist_percentile(hist, total, fraction):
    target = total * fraction
    seen = 0
    for value in range(len(hist)):
        seen += hist[value]
        if seen > target:
            return value
    return len(hist) - 1


class PacingProbe:
    def __init__(self, sample_hz=SAMPLE_HZ):
        gc.collect()
        self.period_us = MotorGroup.UPDATE_INTERVAL_US
        self.buf = bytearray(BUF_BYTES)
        self.scratch = array('I', [0] * 16384)
        motors = [BidirectionalDShot(2 * i, Pin(FIRST_PIN + i), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=2 * i + 1)
                  for i in range(CHANNELS)]
        self.group = MotorGroup(motors)
        self.group.wait_for_replies = False          # nothing replies on these pins
        self.runner = Core1Runner(self.group.update, self.group.UPDATE_INTERVAL_US)
        self.sampler = PinSampler(Pin(FIRST_PIN), sample_hz=sample_hz)
        self.incomplete = 0

    def arm(self):
        self.runner.start()
        self.group.arm(ARM_MS)
        start = utime.ticks_ms()
        while not self.group.is_armed():
            if self.runner.error is not None:
                raise self.runner.error
            if utime.ticks_diff(utime.ticks_ms(), start) > ARM_MS + 4000:
                raise Exception("arming timed out")
            utime.sleep_ms(1)

    def measure(self, work, windows=8, pause_ms=100):
        """Record `windows` windows while work() runs over and over on Core 0 (in the pauses as well as during
        every recording). Returns the spread of the frame spacing over all four channels as a dict."""
        hist = array('H', [0] * HIST_US)
        count = 0
        total = 0
        low = 10 ** 9
        high = 0
        behind_before = self.runner.schedule_behind_count
        for _ in range(windows):
            until = utime.ticks_add(utime.ticks_ms(), pause_ms)
            while utime.ticks_diff(until, utime.ticks_ms()) > 0:
                work()
            self.sampler.start(self.buf)
            while not self.sampler.done():
                work()
            if not self.sampler.finish():
                self.incomplete += 1
                continue
            for bit in range(CHANNELS):
                starts, _, _ = pin_timing.analyze_edges(self.sampler.edges(self.buf, bit, self.scratch))
                for gap in pin_timing.intervals(starts):
                    hist[min(gap, HIST_US - 1)] += 1
                    count += 1
                    total += gap
                    if gap < low:
                        low = gap
                    if gap > high:
                        high = gap
            gc.collect()
        if not count:
            return None
        mean = total / count
        return {
            "gaps": count, "mean": mean, "min": low, "max": high,
            "p1": hist_percentile(hist, count, 0.01), "p99": hist_percentile(hist, count, 0.99),
            "off25": sum(hist[g] for g in range(HIST_US) if abs(g - mean) > mean * 0.25),
            "off50": sum(hist[g] for g in range(HIST_US) if abs(g - mean) > mean * 0.5),
            "behind": self.runner.schedule_behind_count - behind_before,
        }

    def close(self):
        self.sampler.close()
        return fast_stop(self.group, self.runner)


HEADER = "%-16s %6s %9s %6s %6s %6s %6s %7s %9s %9s %6s" % (
    "mode", "gaps", "mean us", "min", "p1", "p99", "max", "spread", ">25% off", ">50% off", "behind")


def row(name, stats):
    if stats is None:
        return "%-16s recorded nothing" % name
    return "%-16s %6d %9.1f %6d %6d %6d %6d %7d %9d %9d %6d" % (
        name, stats["gaps"], stats["mean"], stats["min"], stats["p1"], stats["p99"], stats["max"],
        stats["p99"] - stats["p1"], stats["off25"], stats["off50"], stats["behind"])
