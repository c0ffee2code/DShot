# Dual-core raw telemetry capture test - replaces the on-device live-decode
# soak tests (test_bidir_rx_soak.py, test_bidir_rx_soak_dual.py, both
# retired). Core 1 (via BidirCaptureRunner, see
# tests/harness/bidir_capture_runner.py) owns ESC communication; this script
# runs on Core 0 as the orchestrator, draining raw 4-word captures and
# writing them to the PicoBell's SD card via BidirCaptureSink (see
# tests/harness/bidir_capture_sink.py).
#
# No GCR decoding happens here - that's scripts/dshot_bidir_decode.py's job,
# run on the PC against whatever gets logged.
#
# All four motor channels moved to a contiguous GPIO6-9 block (was
# GPIO2/3/4/5) so the signal wiring bundle stays together: the PicoBell
# Adalogger's RTC needs GPIO4 (SDA)/GPIO5 (SCL) for I2C, and its SD card
# needs GPIO16-19 for SPI0 - see bidir_capture_sink.py's header.
#
# Throwaway diagnostic, not a permanent regression test - matches this
# project's convention (see the retired test_bidir_rx_raw.py's header).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS, BIDIR_PROFILES
from bidir_capture_runner import BidirCaptureRunner
from bidir_capture_sink import BidirCaptureSink
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
RX_CLOCK_HZ = BIDIR_PROFILES[DSHOT_SPEED]

ARM_DURATION_MS = 3000
RAMP_STEPS = [(100, 3), (200, 3)]  # (throttle, seconds) - spin up gradually before holding
HOLD_THROTTLE = 300
HOLD_DURATION_MS = 180_000  # 3 minutes
STOP_DURATION_MS = 300
STATUS_INTERVAL_MS = 15_000
POLL_MS = 10  # how often Core 0 drains the ring buffer


def test_bidir_rx_capture():
    print("=== Dual-Core Bidirectional RX Raw Capture Test (channel 1, DSHOT300) ===")
    print(f"Hold throttle={HOLD_THROTTLE} for {HOLD_DURATION_MS / 1000:.0f}s, "
          f"Core 1 captures raw words, Core 0 writes them to SD")
    print()

    print("Mounting SD card and opening session (before arming, so a bad "
          "card fails fast)...")
    sink = BidirCaptureSink()
    sink.init_session(DSHOT_SPEED, RX_CLOCK_HZ)
    print(f"Session: {sink.path}")
    print()

    ch1 = DShotPIO(0, Pin(6), DSHOT_SPEED, bidirectional=True, rx_state_machine_id=1)
    others = [
        DShotPIO(sm_id, Pin(pin), DSHOT_SPEED)
        for sm_id, pin in zip((4, 5, 6), (7, 8, 9))
    ]
    all_motors = [ch1] + others

    for motor in all_motors:
        motor.start()

    runner = BidirCaptureRunner(ch1, others)

    total_records = 0
    last_record_us = None
    largest_gap_us = 0

    try:
        runner.start()

        print(f"Arming for {ARM_DURATION_MS}ms...")
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            runner.drain()  # discard - just keeping the ring buffer from filling
            utime.sleep_ms(POLL_MS)
        print("Armed.")
        print()

        print(f"Ramping through {RAMP_STEPS} before holding at {HOLD_THROTTLE}...")
        for throttle, seconds in RAMP_STEPS:
            runner.set_throttle(throttle)
            step_start = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), step_start) < seconds * 1000:
                runner.drain()
                utime.sleep_ms(POLL_MS)

        runner.set_throttle(HOLD_THROTTLE)
        print(f"Holding throttle={HOLD_THROTTLE} for {HOLD_DURATION_MS / 1000:.0f}s...")
        print()

        hold_start = utime.ticks_ms()
        last_status_ms = hold_start

        while utime.ticks_diff(utime.ticks_ms(), hold_start) < HOLD_DURATION_MS:
            for record in runner.drain():
                total_records += 1
                sink.write_record(*record)
                ticks_us = record[0]
                if last_record_us is not None:
                    gap = utime.ticks_diff(ticks_us, last_record_us)
                    if gap > largest_gap_us:
                        largest_gap_us = gap
                last_record_us = ticks_us

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_status_ms) >= STATUS_INTERVAL_MS:
                last_status_ms = now
                elapsed_s = utime.ticks_diff(now, hold_start) / 1000
                rate = total_records / elapsed_s if elapsed_s else 0.0
                print(f"  [{elapsed_s:6.1f}s] records={total_records} "
                      f"dropped={runner.dropped} rate={rate:.1f}/s "
                      f"largest_gap={largest_gap_us / 1000:.1f}ms")

            utime.sleep_ms(POLL_MS)

        print()
        print("Hold window complete.")
        print()

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        print("Stopping...")
        runner.set_throttle(0)
        stop_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), stop_start) < STOP_DURATION_MS:
            for record in runner.drain():
                total_records += 1
                sink.write_record(*record)
            utime.sleep_ms(POLL_MS)
        runner.stop()
        for motor in all_motors:
            motor.drain()
            motor.stop()
        print("Motors stopped and deactivated.")
        sink.close()
        print("SD card flushed and unmounted.")

    print()
    print("=== Summary ===")
    print(f"Session: {sink.path}")
    print(f"Total records captured: {total_records}")
    print(f"Records dropped (ring buffer full): {runner.dropped}")
    print(f"Largest gap between records: {largest_gap_us / 1000:.1f}ms")
    if runner.error is not None:
        print(f"Core 1 loop raised: {runner.error}")
    print("=== Test Complete ===")


test_bidir_rx_capture()
