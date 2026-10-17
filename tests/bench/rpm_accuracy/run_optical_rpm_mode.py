# One DShot mode, one run: arms motor 1 in the mode named by the uploaded scenario.json
# (dshot_speed, bidirectional, throttle - see modes/*.json), spins it at that throttle,
# and prints the shaft RPM the optical sensor measures as
#   OPTICAL_RPM rpm=<n> pulses=<n>
# scripts/run_optical_rpm_modes.py runs this once per mode, each after a Pico reset (as
# run_test.py always does), and judges whether the modes agree. A run that cannot
# arm, or sees too few sensor pulses, raises instead of printing a figure.
#
# Sensor: Adafruit STEMMA reflective photo interrupt sensor (TCRT1000) on GPIO 14
# looking at the 2-hole RPM disk on motor 1's shaft (level 0 = reflection; one disk
# pulse = the start of a reflection); see test_esc_rpm_vs_optical_rpm.py for the details.
# Wiring and state machines are the single-channel scenarios': motor 1 on Pin6,
# channels 2-4 (Pin7-9) idle at throttle 0. Spins motor 1; make sure the disk is
# secure and nothing can touch it first. Run it through the PC-side runner:
#   python scripts/run_optical_rpm_modes.py

import json
from machine import Pin
from array import array
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner, fast_stop
import utime

SENSOR_PIN = 14
PULSES_PER_REV = 2       # holes in the disk
MIN_PULSES = 40          # sensor pulses a run needs before its RPM counts
TARGET_PULSES = 100      # the window closes at this many pulses ...
MAX_WINDOW_MS = 10000    # ... or after this long (then it raises if short of MIN_PULSES)
SETTLE_MS = 1500         # the motor speeds up before the window opens
ARM_MS = 2000
ARM_TIMEOUT_MS = ARM_MS + 16000
MAX_EDGES = 2048

edge_us = array('I', [0] * MAX_EDGES)
edge_level = array('B', [0] * MAX_EDGES)
edge_count = array('I', [0])


def on_edge(pin):
    n = edge_count[0]
    if n < MAX_EDGES:
        edge_us[n] = utime.ticks_us()
        edge_level[n] = pin.value()
        edge_count[0] = n + 1


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def build_group(speed, bidirectional):
    if bidirectional:
        motors = [
            BidirectionalDShot(0, Pin(6), speed, rx_state_machine_id=1),
            UnidirectionalDShot(8, Pin(7), speed),
            UnidirectionalDShot(9, Pin(8), speed),
            UnidirectionalDShot(10, Pin(9), speed),
        ]
    else:
        motors = [
            UnidirectionalDShot(0, Pin(6), speed),
            UnidirectionalDShot(2, Pin(7), speed),
            UnidirectionalDShot(4, Pin(8), speed),
            UnidirectionalDShot(6, Pin(9), speed),
        ]
    return MotorGroup(motors)


def main():
    with open("scenario.json") as f:
        mode = json.load(f)
    speed = getattr(DSHOT_SPEEDS, mode["dshot_speed"])
    bidirectional = mode["bidirectional"]
    throttle = mode["throttle"]
    print("Mode: %s, %s, throttle %d" % (
        mode["dshot_speed"], "bidirectional" if bidirectional else "regular", throttle))

    sensor = Pin(SENSOR_PIN, Pin.IN)
    group = build_group(speed, bidirectional)
    runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)
    runner.start()
    window_first = None
    window_end = None
    try:
        t_arm = utime.ticks_ms()
        group.arm(ARM_MS)
        while not group.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), t_arm) > ARM_TIMEOUT_MS:
                raise RuntimeError("arming did not complete: " + str(group.arming_status()))
            utime.sleep_ms(1)
        print("Armed after {}ms.".format(utime.ticks_diff(utime.ticks_ms(), t_arm)))
        utime.sleep_ms(500)

        group.set_throttle(0, throttle)
        t_spin = utime.ticks_ms()
        sensor.irq(handler=on_edge, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, hard=True)
        while window_end is None:
            if runner.error is not None:
                raise runner.error
            elapsed = utime.ticks_diff(utime.ticks_ms(), t_spin)
            if window_first is None and elapsed >= SETTLE_MS:
                window_first = edge_count[0]
            if window_first is not None:
                if (edge_count[0] - window_first) // 2 >= TARGET_PULSES or elapsed - SETTLE_MS >= MAX_WINDOW_MS:
                    window_end = edge_count[0]
            utime.sleep_ms(5)
    finally:
        sensor.irq(handler=None)
        if not fast_stop(group, runner):
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")

    falls = [edge_us[i] for i in range(window_first, window_end) if edge_level[i] == 0]
    if len(falls) < MIN_PULSES:
        raise RuntimeError("not enough data: %d sensor pulses (need %d)" % (len(falls), MIN_PULSES))
    periods = [utime.ticks_diff(falls[i + 1], falls[i]) for i in range(len(falls) - 1)]
    print("OPTICAL_RPM rpm=%.1f pulses=%d" % (60000000 / (median(periods) * PULSES_PER_REV), len(falls)))


main()
