# Does the ESC's eRPM agree with the physical shaft speed? Spins motor 1 at a series
# of throttles and, at each, compares the shaft RPM the ESC reports (eRPM / pole
# pairs) with the RPM the optical sensor measures. Pass: every step is within
# TOLERANCE_PCT, on at least MIN_SAMPLES decodes and MIN_SENSOR_PULSES sensor pulses
# (decode_tally.py's own minimum: a median over fewer cannot support a percentage).
# The window of a step stays open until it holds TARGET_SAMPLES decodes and sensor pulses (capped at
# MAX_STEP_MS), so a slow step cannot end the run early with too little data.
# A step that falls short on samples fails as "not enough data", not as inaccurate.
# Prints "PASSED" or "FAILED: n" like the other bench tests.
#
# The ESC reports ELECTRICAL RPM (shaft RPM x pole
# pairs, pole pairs = magnets / 2), so the ESC's shaft RPM is eRPM / pole pairs
# (6 for 12 magnets, specification/AM32.md#9). A large gap means the magnet count, the disk's pulses per
# revolution, or the sensor is wrong, and every RPM figure derived from eRPM is
# then wrong with it.
#
# Sensor: Adafruit STEMMA reflective photo interrupt sensor (TCRT1000) on GPIO 14,
# 3.3V powered, looking at the 3D-printed RPM disk on motor 1's shaft. The disk has
# 2 holes. Sensor level 0 = reflection (solid disk), level 1 = none (a hole); one
# sensor pulse is the start of a reflection (falling edge). Edges are timestamped in
# a hard interrupt, so a spinning disk is not missed by polling.
#
# Wiring matches single_channel_bidirectional_600.json: channel 1 bidirectional
# (Pin6/sm0/rx1), channels 2-4 idle unidirectional placeholders at throttle 0.
# Spins motor 1 at STEPS throttles; make sure the disk is secure and nothing can
# touch it first. Run with:
#   python scripts/run_test.py test_esc_rpm_vs_optical_rpm.py
# Edit the constants below for a different motor or throttle list - mpremote run
# has no way to pass an argument in.

from machine import Pin
from array import array
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner, fast_stop
from decode_tally import MIN_SAMPLES
from capture_mailbox import FRAME_MASK
import utime

SENSOR_PIN = 14
PULSES_PER_REV = 2       # holes in the disk
MOTOR_MAGNETS = 12       # same as motor 1's "magnets" in tests/bench/dshot_protocol/scenarios/*.json
TOLERANCE_PCT = 3.0      # ESC RPM further than this from the optical RPM fails the step
MIN_SENSOR_PULSES = 20     # sensor pulses a step needs before its RPM counts
STEPS = (60, 70, 80, 90, 100)   # throttle per step; 48 is the lowest motor command
TARGET_SAMPLES = 2 * MIN_SAMPLES   # a step's window closes once it holds this many eRPM decodes and sensor pulses
MAX_STEP_MS = 12000     # ... or when the step has lasted this long (then it fails if short of MIN_SAMPLES)
SETTLE_MS = 1000        # the first part of each step is ignored while the motor speeds up
ARM_MS = 2000
ARM_TIMEOUT_MS = ARM_MS + 16000
DECODE_EVERY = 5         # decoded on Core 0 (about 10ms each), so every Nth new capture
MAX_EDGES = 8192

POLE_PAIRS = MOTOR_MAGNETS // 2

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


def analyze(first, last, throttle, stats):
    """Optical RPM over edge indices [first, last) against the ESC's median eRPM.
    Returns None when the step passes, else why it failed."""
    esc_erpms = stats["erpms"]
    holes = []    # time spent at level 1 (no reflection)
    solids = []   # time spent at level 0 (reflection)
    falls = []
    for i in range(first, last - 1):
        width = utime.ticks_diff(edge_us[i + 1], edge_us[i])
        if edge_level[i] == 1:
            holes.append(width)
        else:
            solids.append(width)
            falls.append(edge_us[i])
    print("--- throttle %d ---" % throttle)
    print("  telemetry in the window: %d captures seen, %d sampled, %d decoded OK, %d CRC failures, "
          "%d not-running, %d without eRPM" % (stats["seen"], stats["sampled"], len(esc_erpms),
                                                stats["crc_fail"], stats["not_running"], stats["no_erpm"]))
    print("  polls in the window: %d, no capture %d, same capture as last poll %d, all-zero reply %d" % (
        stats["polls"], stats["none"], stats["stale"], stats["zero"]))
    print("  window: %s ms, closed on %s; new captures seen per second: %s" % (
        stats["window_ms"], "its sample targets" if stats["full"] else "the time cap",
        "--" if not stats["window_ms"] else "%.0f" % (stats["seen"] * 1000 / stats["window_ms"])))
    if len(falls) < MIN_SENSOR_PULSES or not holes:
        print("  sensor pulses in the window: %d (edges %d)" % (len(falls), last - first))
        return "throttle %d: not enough data, %d sensor pulses (need %d)" % (throttle, len(falls), MIN_SENSOR_PULSES)
    periods = [utime.ticks_diff(falls[i + 1], falls[i]) for i in range(len(falls) - 1)]
    rpm_median = 60000000 / (median(periods) * PULSES_PER_REV)
    hole_med = median(holes)
    solid_med = median(solids)
    short_holes = sum(1 for h in holes if h < hole_med // 4)
    short_solids = sum(1 for s in solids if s < solid_med // 4)
    print("  optical: %.0f RPM by median pulse period (%d pulses), suspected glitches holes=%d solids=%d" % (
        rpm_median, len(falls), short_holes, short_solids))
    if len(esc_erpms) < MIN_SAMPLES:
        return "throttle %d: not enough data, %d eRPM decodes (need %d)" % (throttle, len(esc_erpms), MIN_SAMPLES)
    erpm = median(esc_erpms)
    esc_rpm = erpm / POLE_PAIRS
    off_rpm = esc_rpm - rpm_median
    off_pct = 100 * off_rpm / rpm_median
    print("  ESC: eRPM median %.0f / %d pole pairs = %.0f RPM" % (erpm, POLE_PAIRS, esc_rpm))
    print("  ESC minus optical = %+.0f RPM (%+.2f%%), tolerance %.1f%%" % (off_rpm, off_pct, TOLERANCE_PCT))
    if abs(off_pct) > TOLERANCE_PCT:
        return "throttle %d: ESC RPM %+.2f%% from the optical RPM (limit %.1f%%)" % (throttle, off_pct, TOLERANCE_PCT)
    return None


def window_full(window_first, stats):
    """True once the step holds TARGET_SAMPLES eRPM decodes and TARGET_SAMPLES sensor pulses
    (two edges each) since its window opened."""
    if window_first is None or len(stats["erpms"]) < TARGET_SAMPLES:
        return False
    return (edge_count[0] - window_first) // 2 >= TARGET_SAMPLES


def main():
    sensor = Pin(SENSOR_PIN, Pin.IN)
    motors = [
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=1),
        UnidirectionalDShot(8, Pin(7), DSHOT_SPEEDS.DSHOT600),
        UnidirectionalDShot(9, Pin(8), DSHOT_SPEEDS.DSHOT600),
        UnidirectionalDShot(10, Pin(9), DSHOT_SPEEDS.DSHOT600),
    ]
    group = MotorGroup(motors)
    runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)
    runner.start()

    windows = []   # (throttle, first_edge, last_edge, stats)
    seen = 0
    last_value = 0
    try:
        print("Arming for {}ms...".format(ARM_MS))
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

        sensor.irq(handler=on_edge, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, hard=True)

        for throttle in STEPS:
            group.set_throttle(0, throttle)
            print("throttle %d: window opens after %dms, closes at %d decodes and %d sensor pulses, or after %dms" % (
                throttle, SETTLE_MS, TARGET_SAMPLES, TARGET_SAMPLES, MAX_STEP_MS))
            step_start = utime.ticks_ms()
            window_first = None
            window_opened_ms = None
            stats = {"erpms": [], "seen": 0, "sampled": 0, "crc_fail": 0, "not_running": 0, "no_erpm": 0,
             "polls": 0, "none": 0, "stale": 0, "zero": 0}
            while (utime.ticks_diff(utime.ticks_ms(), step_start) < MAX_STEP_MS
                   and not window_full(window_first, stats)):
                if runner.error is not None:
                    raise runner.error
                elapsed = utime.ticks_diff(utime.ticks_ms(), step_start)
                if window_first is None and elapsed >= SETTLE_MS:
                    window_first = edge_count[0]
                    window_opened_ms = elapsed
                capture = group.raw_telemetry(0)
                if window_first is not None:
                    stats["polls"] += 1
                    if capture is None:
                        stats["none"] += 1
                if capture is not None:
                    if capture == last_value:
                        if window_first is not None:
                            stats["stale"] += 1
                    else:
                        last_value = capture
                        if not capture & FRAME_MASK and window_first is not None:
                            stats["zero"] += 1
                        if capture & FRAME_MASK:
                            seen += 1
                            in_window = window_first is not None
                            if in_window:
                                stats["seen"] += 1
                            if seen % DECODE_EVERY == 0 and in_window:
                                stats["sampled"] += 1
                                result = group.decode_telemetry(0, capture)
                                if result is None or not result["crc_ok"]:
                                    stats["crc_fail"] += 1
                                elif result["not_running"]:
                                    stats["not_running"] += 1
                                elif result["erpm"] is None:
                                    stats["no_erpm"] += 1
                                else:
                                    stats["erpms"].append(result["erpm"])
            stats["window_ms"] = None if window_opened_ms is None else \
                utime.ticks_diff(utime.ticks_ms(), step_start) - window_opened_ms
            stats["full"] = window_full(window_first, stats)
            windows.append((throttle, window_first, edge_count[0], stats))
    finally:
        sensor.irq(handler=None)
        if not fast_stop(group, runner):
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")

    print()
    print("total edges recorded: %d (buffer %d)" % (edge_count[0], MAX_EDGES))
    failures = []
    for throttle, first, last, stats in windows:
        if first is None:
            failures.append("throttle %d: no measurement window" % throttle)
            continue
        problem = analyze(first, last, throttle, stats)
        if problem is not None:
            failures.append(problem)
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
