# Diagnostic, not a test: no pass/fail assertion, just data for a person to
# read (see CLAUDE.md's tools/ vs tests/device/ distinction).
#
# BUG-009 (bug-reports/BUG-009-motors-sometimes-start-rough.md), 2026-10-10:
# the user observed motors starting rough (present before any of that day's
# code changes), and a full-resolution replay of two_channel_divergent_600.json
# captures found motor 2/channel 3 occasionally spiking to physically
# implausible eRPM readings (up to 159,151) in the first ~2s after a throttle
# change, on an otherwise perfectly CRC-valid telemetry link.
#
# This tool isolates ONE variable at a time, the same way the arming
# investigation's measure_arming_tick_timing_once*.py family did: instead of
# divergent_600's own ramp profile (hold 60, ramp to 200, diverge), it jumps
# both motor-mounted channels straight to one fixed BASE_THROTTLE right after
# ARMED and holds there - so a comparison across BASE_THROTTLE values isn't
# also comparing different ramp shapes. The question it's built to answer:
# does a different base/floor throttle change how often the rough-start
# spike shows up.
#
# Logs every capture to the SD card in the exact format
# tests/harness/bidir_capture_sink.py uses (ticks_us, 4 throttles, 4 one-word
# groups) - no on-device decoding, no decode_every sampling, no expect
# thresholds: full-resolution analysis is the only check that has caught this
# phenomenon so far (BUG-009.md's own caveat on this), so this tool doesn't
# bother with a sampled on-device verdict at all. Read the result with
# scripts/pull_captures.py then the same ad hoc analysis BUG-009.md's finding
# used (every capture, not a sample).
#
# Wiring matches two_channel_divergent_600.json: Pin6/sm0/rx1 and
# Pin8/sm4/rx5 bidirectional (channels 1 and 3, the two motor-mounted ones),
# Pin7/sm8 and Pin9/sm9 idle unidirectional placeholders (AM32 needs a valid
# signal on all four channels to complete its arm handshake).
#
# Run with:
#   python scripts/run_test.py measure_rough_start.py
# Edit BASE_THROTTLE below for a comparison run - no CLI argument (mpremote
# run has no mechanism to pass one in, same limitation run_scenario.py's own
# scenario.json workaround exists for - not worth it for a one-constant knob).

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner
from bidir_capture_sink import BidirCaptureSink
import struct
import utime

BASE_THROTTLE = 65
ARM_MS = 2000
HOLD_MS = 8000
ARM_TIMEOUT_MARGIN_MS = 16000  # matches tests/harness/run_scenario.py's own constant
POLL_MS = 1
BIDIR_INDICES = (0, 2)  # matches the wiring below

RECORD_FMT = "<I4H4I"
ZERO_GROUP = (0,)


def arm_group(group, runner, arm_times):
    """Exact shape of tests/harness/run_scenario.py's own (fixed) arm_group() -
    no per-tick raw_telemetry() check; see that function's own docstring for
    why one used to be here and isn't any more."""
    print("Arming for {}ms...".format(ARM_MS))
    arm_times[0] = utime.ticks_us()
    group.arm(ARM_MS)
    arm_start = utime.ticks_ms()
    arm_timeout_ms = ARM_MS + ARM_TIMEOUT_MARGIN_MS
    while not group.is_armed():
        if runner.error is not None:
            raise runner.error
        if utime.ticks_diff(utime.ticks_ms(), arm_start) > arm_timeout_ms:
            raise RuntimeError("arming did not complete within {}ms".format(arm_timeout_ms))
        utime.sleep_ms(1)
    arm_times[1] = utime.ticks_us()
    print("Armed after {}ms.".format(utime.ticks_diff(arm_times[1], arm_times[0]) // 1000))


def main():
    sink = BidirCaptureSink()
    sink.create_session_dir({
        "dshot_speed": str(DSHOT_SPEEDS.DSHOT600),
        "rx_clock_hz": "0",  # filled in below once known
        "record_fmt": RECORD_FMT,
        "bidir_motor_indices": "0,2",
        "base_throttle": str(BASE_THROTTLE),
        "outcome": "running",
    })
    sink.open_capture()

    motors = [
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=1),
        UnidirectionalDShot(8, Pin(7), DSHOT_SPEEDS.DSHOT600),
        BidirectionalDShot(4, Pin(8), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=5),
        UnidirectionalDShot(9, Pin(9), DSHOT_SPEEDS.DSHOT600),
    ]
    sink.meta["rx_clock_hz"] = str(motors[0].rx_clock_hz)
    sink.write_meta()

    group = MotorGroup(motors)
    runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)
    runner.start()

    arm_times = [None, None]
    last_seq = [0, 0, 0, 0]
    total_records = 0
    missed = 0
    largest_gap_us = 0
    last_record_us = None
    pack_buf = bytearray(struct.calcsize(RECORD_FMT))
    outcome = "failed"

    try:
        arm_group(group, runner, arm_times)

        # Holds at forced zero throttle for 500ms before the first real
        # command - isolates whether an anomaly is tied to the ARMED moment
        # itself versus the first-nonzero-throttle moment (see BUG-009 for
        # what this distinguished and why it matters).
        utime.sleep_ms(500)

        group.set_throttle(0, BASE_THROTTLE)
        group.set_throttle(2, BASE_THROTTLE)

        print("Holding at throttle {} for {}ms...".format(BASE_THROTTLE, HOLD_MS))
        hold_start = utime.ticks_ms()
        words = [ZERO_GROUP] * 4
        while utime.ticks_diff(utime.ticks_ms(), hold_start) < HOLD_MS:
            if runner.error is not None:
                raise runner.error

            throttles = group.get_all_throttles()
            for i in range(4):
                words[i] = ZERO_GROUP
            record_us = None
            for index in BIDIR_INDICES:
                capture = group.raw_telemetry(index)
                if capture is None:
                    continue
                ticks_us, seq, capture_words = capture
                if seq == last_seq[index]:
                    continue
                missed += seq - last_seq[index] - 1
                last_seq[index] = seq
                words[index] = capture_words
                if record_us is None or utime.ticks_diff(ticks_us, record_us) > 0:
                    record_us = ticks_us

            if record_us is not None:
                total_records += 1
                struct.pack_into(RECORD_FMT, pack_buf, 0, record_us, *throttles,
                                  *words[0], *words[1], *words[2], *words[3])
                sink.file.write(pack_buf)
                if last_record_us is not None:
                    gap = utime.ticks_diff(record_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = record_us

            utime.sleep_ms(POLL_MS)

        outcome = "completed"
    finally:
        runner.stop()
        group.disarm()
        sink.finalize_meta(outcome, {
            "total_records": str(total_records),
            "captures_missed": str(missed),
            "largest_gap_us": str(largest_gap_us),
            "arm_ticks_us": str(arm_times[0]) if arm_times[0] is not None else "",
            "armed_ticks_us": str(arm_times[1]) if arm_times[1] is not None else "",
        })
        sink.close()
        print("SD card flushed and unmounted.")

    print("Records: {} missed: {} largest_gap: {:.1f}ms".format(
        total_records, missed, largest_gap_us / 1000))
    print("Outcome: {}".format(outcome))


main()
