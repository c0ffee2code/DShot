# Test: the pin recorder (pin_sampler.py) - does a logic analyzer inside the Pico work, and what
# does the production command loop's frame spacing look like on the pins?
#
# Purpose: the project needs to know at which pace the command loop really sends frames and whether
# anything interrupts or unevens it, without external instruments. This builds the real thing - four
# bidirectional motors in a MotorGroup, Core1Runner at MotorGroup.UPDATE_INTERVAL_US, the same
# mechanics as everywhere - and records the four transmit pins with a PIO state machine and DMA
# while the group is ARMED (zero throttle only).
#
# No ESC and no motor: the four transmitters sit on unused GPIO 10-13 and nothing is wired to them
# (the SD card uses 16-19, the RPM sensor 14, the motors 6-9). With no ESC replying,
# the only edges on a line are our own frames.
#
# Pass:
#   - the DMA transfer completes and every channel recorded frames (the recorder neither stole a pin
#     from its transmitter nor missed the window);
#   - each channel's mean frame spacing equals the loop's period within 2us (the paced loop anchors
#     each deadline to the previous one, so the mean holds even when single frames wobble);
#   - no spacing is below half or above one and a half periods (no burst, no missing frame);
#   - the transmitter's own frame lasts 18-28us, which is a DSHOT600 frame of 16 bits (26.7us) seen
#     through a 1us sampling clock - this proves the recorder's time base.
# It also PRINTS the jitter of the spacing and the phase of each channel inside a loop pass, which
# are the findings, not pass criteria.
#
# Run with: python scripts/run_session.py test_pin_sampler.py

import gc
import utime
from machine import Pin
from array import array
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner, fast_stop
from pin_sampler import PinSampler, SAMPLE_HZ
import pin_timing

SAMPLE_US = 1_000_000 // SAMPLE_HZ
FIRST_PIN = 10
CHANNELS = 4
BUF_BYTES = 100 * 1024              # 25,600 words x 8 samples x 1us = 204.8 ms
EDGE_CAPACITY = 16384
ARM_MS = 2000
MEAN_TOLERANCE_US = 2
DURATION_RANGE_US = (18, 28)


def main():
    period_us = MotorGroup.UPDATE_INTERVAL_US
    print("=== Pin recorder self-test (no ESC, GPIO %d-%d, nothing attached) ===" % (
        FIRST_PIN, FIRST_PIN + CHANNELS - 1))
    print("loop period %dus, %d channels, recording %dms at %dus per sample" % (
        period_us, CHANNELS, BUF_BYTES // 4 * 8 * SAMPLE_US // 1000, SAMPLE_US))

    gc.collect()
    buf = bytearray(BUF_BYTES)
    scratch = array('I', [0] * EDGE_CAPACITY)
    motors = [BidirectionalDShot(2 * i, Pin(FIRST_PIN + i), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=2 * i + 1)
              for i in range(CHANNELS)]
    group = MotorGroup(motors)
    group.wait_for_replies = False          # nothing replies on these pins
    runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)
    sampler = PinSampler(Pin(FIRST_PIN))
    completed = False
    try:
        runner.start()
        group.arm(ARM_MS)
        start = utime.ticks_ms()
        while not group.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), start) > ARM_MS + 3000:
                raise Exception("arming timed out")
            utime.sleep_ms(1)
        utime.sleep_ms(100)
        completed = sampler.capture(buf)
    finally:
        sampler.close()
        if not fast_stop(group, runner):
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")
    behind = runner.schedule_behind_count

    failures = []
    if not completed:
        failures.append("the DMA transfer did not complete")
    all_starts = []
    for bit in range(CHANNELS if completed else 0):
        starts, odd, durations = pin_timing.analyze_edges(sampler.edges(buf, bit, scratch))
        all_starts.append(starts)
        stats = pin_timing.spacing_stats(starts, period_us)
        durations.sort()
        median_duration = durations[len(durations) // 2] if durations else None
        print("GPIO %d: %d edges, %d frames, spacing mean %s us (min %s, max %s, 99%% within %s us of the period), "
              "short %d long %d, first edge rising %d, frame length median %s us" % (
                  FIRST_PIN + bit, sampler.last_count, stats["frames"],
                  "%.2f" % stats["mean_us"] if stats["mean_us"] is not None else "-", stats["min_us"],
                  stats["max_us"], stats["p99_dev_us"], stats["short"], stats["long"], odd, median_duration))
        if stats["frames"] < 10:
            failures.append("GPIO %d recorded only %d frames" % (FIRST_PIN + bit, stats["frames"]))
            continue
        if abs(stats["mean_us"] - period_us) > MEAN_TOLERANCE_US:
            failures.append("GPIO %d mean spacing %.2fus, expected %dus" % (FIRST_PIN + bit, stats["mean_us"], period_us))
        if stats["short"] or stats["long"]:
            failures.append("GPIO %d has %d short and %d long gaps" % (FIRST_PIN + bit, stats["short"], stats["long"]))
        if odd:
            failures.append("GPIO %d has %d bursts that did not start with a falling edge" % (FIRST_PIN + bit, odd))
        if not (DURATION_RANGE_US[0] <= median_duration <= DURATION_RANGE_US[1]):
            failures.append("GPIO %d frame length %sus is outside %d-%dus (is the sampling clock %dus?)" % (
                FIRST_PIN + bit, median_duration, DURATION_RANGE_US[0], DURATION_RANGE_US[1], SAMPLE_US))

    if len(all_starts) == CHANNELS and all(len(s) > 10 for s in all_starts):
        for bit in range(1, CHANNELS):
            offsets = pin_timing.phase_offsets(all_starts[0], all_starts[bit])
            if offsets:
                print("GPIO %d starts %d to %d us after GPIO %d within a pass (median %d)" % (
                    FIRST_PIN + bit, offsets[0], offsets[-1], FIRST_PIN, offsets[len(offsets) // 2]))
    print("loop fell behind its period %d time(s) during the whole run" % behind)

    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
