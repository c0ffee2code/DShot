# Diagnostic, not a test: no pass/fail assertion, just data for a person to
# read (see CLAUDE.md's tools/ vs tests/device/ distinction).
#
# One arm/disarm cycle, self-contained and standalone (mpremote run transfers
# only this one file - no sibling imports). Measures update()'s own per-tick
# cost (arming phase and post-armed phase separately) with minimal Core 0
# overhead, so it can be used as a clean before/after harness for driver
# changes - see bidirectional_dshot_review.md's W33 for how it was used.
#
# Prints "Armed after Nms." and "Outcome: completed" (or, on a timeout,
# "arming did not complete within ...") in the exact form
# scripts/run_scenarios.py's own classifier already parses from
# run_scenario.py's output - no changes needed there for this file to be
# counted the same way.
#
# Wiring matches tests/harness/scenarios/two_channel_arming_check_600_deadline.json
# (the current bench: channel 1 = Pin6/sm0/rx1, channel 3 = Pin8/sm4/rx5,
# both bidirectional; channels 2/4 idle unidirectional) and needs the ESC
# powered, motors mounted. Run standalone with:
#   python scripts/run_test.py measure_arming_tick_timing_once.py
# or interleaved against the harness scenario with:
#   python scripts/run_scenarios.py --scenario tools/measure_arming_tick_timing_once.py \
#       --scenario tests/harness/scenarios/two_channel_arming_check_600_deadline.json --rounds 10

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from array import array
import _thread
import utime

ARM_MS = 2000
POST_ARM_HOLD_MS = 3000
ARM_TIMEOUT_MARGIN_MS = 16000  # matches tests/harness/run_scenario.py's own constant
INTERVAL_US = 1045             # matches two_channel_arming_check_600_deadline.json
CAPACITY = 18000                # see measure_arming_tick_timing.py's own note on sizing


class InstrumentedRunner:
    """Same as measure_arming_tick_timing.py's class of the same name - kept
    in sync by hand, not shared code (see this file's own header)."""

    def __init__(self, update, interval_us):
        self.update = update
        self.interval_us = interval_us
        self.running = False
        self.stopped = True
        self.error = None
        self.count = 0
        self.tick_start_us = array('I', [0] * CAPACITY)
        self.update_done_us = array('I', [0] * CAPACITY)
        self.sleep_done_us = array('I', [0] * CAPACITY)

    def start(self):
        self.running = True
        self.stopped = False
        _thread.start_new_thread(self.loop, ())

    def stop(self):
        if not self.running:
            return
        self.running = False
        while not self.stopped:
            utime.sleep_us(200)

    def loop(self):
        update = self.update
        interval_us = self.interval_us
        ticks_us = utime.ticks_us
        ticks_add = utime.ticks_add
        ticks_diff = utime.ticks_diff
        sleep_us = utime.sleep_us
        tick_start_us = self.tick_start_us
        update_done_us = self.update_done_us
        sleep_done_us = self.sleep_done_us
        capacity = CAPACITY
        next_due = ticks_us()
        try:
            while self.running:
                t0 = ticks_us()
                update()
                t1 = ticks_us()
                next_due = ticks_add(next_due, interval_us)
                remaining = ticks_diff(next_due, t1)
                if remaining <= 0:
                    while remaining <= 0:
                        next_due = ticks_add(next_due, interval_us)
                        remaining = ticks_diff(next_due, ticks_us())
                sleep_us(remaining)
                t2 = ticks_us()
                count = self.count
                if count < capacity:
                    tick_start_us[count] = t0
                    update_done_us[count] = t1
                    sleep_done_us[count] = t2
                    self.count = count + 1
        except Exception as e:
            self.error = e
            self.running = False
        finally:
            self.stopped = True


def percentile(values, p):
    if not values:
        return 0
    index = int(len(values) * p)
    if index >= len(values):
        index = len(values) - 1
    return values[index]


def report_phase(name, tick_start, update_done, start_index, end_index):
    n = end_index - start_index
    if n <= 0:
        print("  {}: no ticks recorded".format(name))
        return
    update_us = sorted(update_done[i] - tick_start[i] for i in range(start_index, end_index))
    print("  {}: n={} update_us(min={} median={} p95={} max={})".format(
        name, n, update_us[0], percentile(update_us, 0.5), percentile(update_us, 0.95), update_us[-1]))


def main():
    motors = [
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=1),
        UnidirectionalDShot(8, Pin(7), DSHOT_SPEEDS.DSHOT600),
        BidirectionalDShot(4, Pin(8), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=5),
        UnidirectionalDShot(9, Pin(9), DSHOT_SPEEDS.DSHOT600),
    ]
    group = MotorGroup(motors)
    runner = InstrumentedRunner(group.update, INTERVAL_US)

    runner.start()
    try:
        group.arm(ARM_MS)
        arm_start_ms = utime.ticks_ms()
        deadline_ms = utime.ticks_add(arm_start_ms, ARM_MS + ARM_TIMEOUT_MARGIN_MS)
        timed_out = False
        while not group.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(deadline_ms, utime.ticks_ms()) < 0:
                timed_out = True
                break
            utime.sleep_ms(1)

        armed_tick_index = runner.count

        if timed_out:
            print("arming did not complete within {}ms".format(ARM_MS + ARM_TIMEOUT_MARGIN_MS))
            report_phase("arming phase (never armed)", runner.tick_start_us, runner.update_done_us,
                         0, armed_tick_index)
            return

        armed_after_ms = utime.ticks_diff(utime.ticks_ms(), arm_start_ms)
        utime.sleep_ms(POST_ARM_HOLD_MS)
    finally:
        runner.stop()
        group.disarm()

    print("Armed after {}ms.".format(armed_after_ms))
    report_phase("arming phase", runner.tick_start_us, runner.update_done_us, 0, armed_tick_index)
    report_phase("post-armed phase", runner.tick_start_us, runner.update_done_us,
                 armed_tick_index, runner.count)
    print("Outcome: completed")


main()
