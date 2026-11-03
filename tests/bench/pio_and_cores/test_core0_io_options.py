# Test: how can Core 0 do its SD-card logging and USB printing without disturbing the command loop's
# frame spacing (BUG-011)?
#
# Purpose: test_core0_effect_on_pacing.py shows that an SD record per millisecond and a USB line per 10ms
# widen the frame spacing on Core 1. This tries the ways out, on the same loop and the same minute, with the
# pin recorder watching GPIO 10-13 (nothing attached, no ESC, no motor):
#   idle              sleeps 1ms per pass (baseline)
#   sd_each_ms        sink.write_sample() every millisecond (what the scenario runner did per pass before 2026-10-31)
#   pack_only         the same record packed into a RAM buffer every millisecond, SD not touched
#   batch_4k          packed into RAM, the filled 4 KB written to the SD file when it is full
#   batch_16k         the same with 16 KB
#   batch_16k_flush   16 KB, and the file flushed after each write
#   print_10ms        one USB line every 10ms
#   print_100ms       one USB line every 100ms
#   print_1s          one USB line every second
#
# Pass: every recording completes and the baseline mode is clean (see test_core0_effect_on_pacing.py).
# The other modes are the findings.
#
# Run with: python scripts/run_session.py test_core0_io_options.py

import os
import struct
import utime
from motor_group import MotorGroup
from bidir_capture_sink import BidirCaptureSink
import pacing_probe

RECORD_FMT = BidirCaptureSink.RECORD_FMT


def main():
    print("=== Core 0 I/O options (no ESC, GPIO %d-%d, nothing attached) === period %dus" % (
        pacing_probe.FIRST_PIN, pacing_probe.FIRST_PIN + pacing_probe.CHANNELS - 1, MotorGroup.UPDATE_INTERVAL_US))
    probe = pacing_probe.PacingProbe()
    sink = BidirCaptureSink()
    sink.create_session_dir({"outcome": "core 0 io options test, delete"})
    sink.open_capture()
    record_size = struct.calcsize(RECORD_FMT)
    ram = bytearray(16 * 1024)
    zeros = (0, 0, 0, 0)
    state = [0, 0]          # bytes waiting in ram, a counter

    def idle():
        utime.sleep_ms(1)

    def sd_each_ms():
        sink.write_sample(utime.ticks_us(), 0, False, zeros, zeros)
        utime.sleep_ms(1)

    def pack_into_ram(limit, flush):
        if state[0] + record_size > limit:
            if limit < len(ram) + 1 and flush is not None:
                sink.file.write(memoryview(ram)[:state[0]])
                if flush:
                    sink.file.flush()
            state[0] = 0
        struct.pack_into(RECORD_FMT, ram, state[0], utime.ticks_us(), 0, 0, 0, 0, 0, 0, 0, 0, 0)
        state[0] += record_size
        utime.sleep_ms(1)

    def pack_only():
        pack_into_ram(len(ram), None)

    def batch_4k():
        pack_into_ram(4096, False)

    def batch_16k():
        pack_into_ram(16384, False)

    def batch_16k_flush():
        pack_into_ram(16384, True)

    def printer(every_ms):
        def work():
            state[1] += 1
            if state[1] % every_ms == 0:
                print("tick", state[1])
            utime.sleep_ms(1)
        return work

    modes = (("idle", idle), ("sd_each_ms", sd_each_ms), ("pack_only", pack_only), ("batch_4k", batch_4k),
             ("batch_16k", batch_16k), ("batch_16k_flush", batch_16k_flush), ("print_10ms", printer(10)),
             ("print_100ms", printer(100)), ("print_1s", printer(1000)))
    results = []
    try:
        probe.arm()
        for name, work in modes:
            state[0] = 0
            state[1] = 0
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
            print("WARNING: Core 1 loop did not stop - disarm() raced update()")

    print(pacing_probe.HEADER)
    for name, stats in results:
        print(pacing_probe.row(name, stats))
    failures = []
    baseline = results[0][1]
    if probe.incomplete:
        failures.append("%d recordings did not complete" % probe.incomplete)
    if baseline is None or baseline["off50"]:
        failures.append("the baseline is not clean")
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
