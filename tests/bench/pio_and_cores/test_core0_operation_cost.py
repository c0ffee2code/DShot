# Test: which plain Core 0 operation, done once per pass, widens the command loop's frame spacing (BUG-011)?
#
# Purpose: test_core0_io_options.py found that merely packing a capture record into a RAM buffer every
# millisecond widens the spacing as much as most of the SD-card path. This narrows it down on the same loop
# with the pin recorder watching GPIO 10-13 (nothing attached, no ESC, no motor): each mode does one small
# thing per pass, then sleeps, so the modes differ only by that thing:
#   idle_1ms        sleep 1ms (baseline)
#   idle_5ms        sleep 5ms (fewer wake-ups)
#   idle_200us      sleep 200us (more wake-ups)
#   ticks           utime.ticks_us()
#   store           two bytearray element stores
#   arith           20 integer additions in a loop
#   pack_const      struct.pack_into() of nine constant integers
#   pack_ticks      struct.pack_into() with utime.ticks_us() as its first value
#
# Pass: every recording completes and the baseline is clean. The other modes are the findings.
#
# Run with: python scripts/run_session.py test_core0_operation_cost.py

import struct
import utime
from motor_group import MotorGroup
import pacing_probe

RECORD_FMT = "<I4H4I"


def main():
    print("=== Core 0 operation cost (no ESC, GPIO %d-%d, nothing attached) === period %dus" % (
        pacing_probe.FIRST_PIN, pacing_probe.FIRST_PIN + pacing_probe.CHANNELS - 1, MotorGroup.UPDATE_INTERVAL_US))
    probe = pacing_probe.PacingProbe()
    ram = bytearray(64)
    state = [0]

    def idle_1ms():
        utime.sleep_ms(1)

    def idle_5ms():
        utime.sleep_ms(5)

    def idle_200us():
        utime.sleep_us(200)

    def ticks():
        state[0] = utime.ticks_us()
        utime.sleep_ms(1)

    def store():
        ram[0] = 1
        ram[1] = 2
        utime.sleep_ms(1)

    def arith():
        total = 0
        for i in range(20):
            total += i
        state[0] = total
        utime.sleep_ms(1)

    def pack_const():
        struct.pack_into(RECORD_FMT, ram, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)
        utime.sleep_ms(1)

    def pack_ticks():
        struct.pack_into(RECORD_FMT, ram, 0, utime.ticks_us(), 0, 0, 0, 0, 0, 0, 0, 0)
        utime.sleep_ms(1)

    modes = (("idle_1ms", idle_1ms), ("idle_5ms", idle_5ms), ("idle_200us", idle_200us), ("ticks", ticks),
             ("store", store), ("arith", arith), ("pack_const", pack_const), ("pack_ticks", pack_ticks))
    results = []
    try:
        probe.arm()
        for name, work in modes:
            results.append((name, probe.measure(work)))
    finally:
        if not probe.close():
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")

    print(pacing_probe.HEADER)
    for name, stats in results:
        print(pacing_probe.row(name, stats))
    failures = []
    if probe.incomplete:
        failures.append("%d recordings did not complete" % probe.incomplete)
    if results[0][1] is None or results[0][1]["off50"]:
        failures.append("the baseline is not clean")
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
