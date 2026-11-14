# Test: which of Core 0's activities disturb the frame spacing of the command loop on Core 1?
#
# Purpose: with Core 0 logging to the SD card or printing over USB the loop's frame spacing on the pins
# gets uneven (BUG-011). pacing_probe.py keeps one command loop running (four bidirectional motors in a
# MotorGroup, Core1Runner at MotorGroup.UPDATE_INTERVAL_US, armed at zero throttle, GPIO 10-13, nothing
# attached) and records the transmit pins with the pin recorder while Core 0 does, mode after mode:
#   idle    sleeps 1ms per pass (the baseline)
#   busy    spins in a tight loop and never sleeps
#   alloc   makes a small list every millisecond (garbage for the collector)
#   gc      calls gc.collect() every 10ms
#   print   prints a line over USB every 10ms
#   sd      writes a capture record to the SD card every millisecond (the scenario runner's sink, in a
#           throwaway session folder that is deleted at the end)
#
# Pass: every recording completes, the baseline mode has no gap more than 50% off the mean, and its mean
# spacing is within 2us of MotorGroup.UPDATE_INTERVAL_US. The other modes are the findings: the spread of
# the spacing per mode.
#
# Run with: python scripts/run_session.py test_core0_effect_on_pacing.py

import gc
import os
import utime
from motor_group import MotorGroup
from bidir_capture_sink import BidirCaptureSink
import pacing_probe


def main():
    print("=== Core 0 effect on the loop's frame spacing (no ESC, GPIO %d-%d, nothing attached) === period %dus" % (
        pacing_probe.FIRST_PIN, pacing_probe.FIRST_PIN + pacing_probe.CHANNELS - 1, MotorGroup.UPDATE_INTERVAL_US))
    probe = pacing_probe.PacingProbe()
    sink = BidirCaptureSink()
    sink.create_session_dir({"outcome": "core 0 pacing test, delete"})
    sink.open_capture()
    state = [None, 0]
    zeros = (0, 0, 0, 0)

    def idle():
        utime.sleep_ms(1)

    def busy():
        pass

    def alloc():
        state[0] = [state[1], state[1] + 1, state[1] + 2, state[1] + 3]
        state[1] += 1
        utime.sleep_ms(1)

    def collect():
        state[1] += 1
        if state[1] % 10 == 0:
            gc.collect()
        utime.sleep_ms(1)

    def usb_print():
        state[1] += 1
        if state[1] % 10 == 0:
            print("tick", state[1])
        utime.sleep_ms(1)

    def sd_write():
        sink.write_sample(utime.ticks_us(), 0, False, zeros, zeros)
        utime.sleep_ms(1)

    modes = (("idle", idle), ("busy", busy), ("alloc", alloc), ("gc", collect), ("print", usb_print), ("sd", sd_write))
    results = []
    try:
        probe.arm()
        for name, work in modes:
            results.append((name, probe.measure(work)))
    finally:
        clean = probe.close()
        try:
            sink.file.close()
            for name in ("capture.bin", "meta.txt"):
                os.remove(sink.run_dir + "/" + name)
            os.rmdir(sink.run_dir)
        except OSError as error:
            print("WARNING: could not delete the throwaway session folder", sink.run_dir, error)
        if not clean:
            print("WARNING: Core 1 loop did not stop - deactivate() raced update()")

    print(pacing_probe.HEADER)
    failures = []
    for name, stats in results:
        print(pacing_probe.row(name, stats))
    baseline = results[0][1]
    if probe.incomplete:
        failures.append("%d recordings did not complete" % probe.incomplete)
    if baseline is None:
        failures.append("the baseline recorded nothing")
    else:
        if baseline["off50"]:
            failures.append("the baseline has %d gaps more than 50%% off the mean" % baseline["off50"])
        if abs(baseline["mean"] - probe.period_us) > 2:
            failures.append("the baseline mean spacing is %.1fus, expected %dus" % (baseline["mean"], probe.period_us))
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
