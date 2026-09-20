# Test: the run-length PIO receiver (dshot_bidir_rx_rle) against a real ESC
#
# Purpose: dshot_bidir_rx_rle rebuilds the ESC's reply in the state machine and
# hands the CPU one 21-bit frame per reply, instead of 128 raw samples that the
# CPU turns into a frame. This runs it on the bench, on the same wiring and at
# the same settled throttle as tests/test_motor_group_telemetry.py, which is the
# baseline for the raw receiver (>=98% CRC-valid, eRPM around 21k at throttle
# 100), so the two can be compared.
#
# It replaces the bidirectional motor's receiver with the run-length one and
# supplies its own drain in place of the CaptureMailbox: the mailbox takes
# 4-word captures, this receiver produces one word per reply. The drain keeps
# every frame in a ring for Core 0 to decode at its own pace, which also gives
# a per-frame count (the mailbox keeps only the latest).
#
# Reports: frames received, how many decode to a valid GCR frame and pass the
# CRC, the median eRPM, and the CPU cost of decoding a frame (decode() plus
# check_crc(); the raw path's whole decode is about 1.3ms). Any failure raises.
#
# Hardware: as tests/test_motor_group_telemetry.py - 4-in-1 AM32 ESC, channel 1
# -> GPIO 6 (motor + prop mounted, the only bidirectional channel), channels 2-4
# -> GPIO 7/8/9 (idle, but the ESC only completes its arm handshake with valid
# signal on all 4).

from array import array
from machine import Pin
from rp2 import StateMachine
from dshot_pio import (BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS,
                       dshot_bidir_rx_rle, rle_rx_speed)
from motor_throttle_group import MotorThrottleGroup
from core1_runner import Core1Runner
import gcr_decode
import utime

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
THROTTLE = 100
ARM_DURATION_MS = 3000
ARM_TIMEOUT_MS = ARM_DURATION_MS + 1000
SETTLE_MS = 500
SAMPLE_MS = 5000
MIN_FRAMES = 50
MIN_CRC_VALID_PCT = 98.0
MIN_MEDIAN_ERPM = 10000
RING = 64          # frames the drain keeps for Core 0; must be a power of two
MAX_ERPM_SAMPLES = 512
SHOW_BAD = 6


def test_rle_receiver():
    print("=== Run-length receiver test ===")
    print("receiver clock: " + str(rle_rx_speed(DSHOT_SPEED)) + " Hz")

    pin = Pin(6)
    bidir = BidirectionalDShot(0, pin, DSHOT_SPEED, rx_state_machine_id=1)
    motors = MotorThrottleGroup([
        bidir,
        UnidirectionalDShot(2, Pin(7), DSHOT_SPEED),
        UnidirectionalDShot(4, Pin(8), DSHOT_SPEED),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEED),
    ])

    # Swap the receiver: the same state machine id and pin, the run-length program
    bidir.rx_sm.active(0)
    rx = StateMachine(1, dshot_bidir_rx_rle, freq=rle_rx_speed(DSHOT_SPEED),
                      in_base=pin, jmp_pin=pin)
    bidir.rx_sm = rx

    frames = array('I', [0] * RING)
    written = array('I', [0])

    def drain(publish):
        while rx.rx_fifo():
            word = rx.get()
            if publish:
                frames[written[0] & (RING - 1)] = word
                written[0] += 1

    bidir.drain_rx = drain
    runner = Core1Runner(motors.update, motors.UPDATE_INTERVAL_US)

    try:
        runner.start()

        print("Arming...")
        motors.arm(ARM_DURATION_MS)
        arm_start = utime.ticks_ms()
        while not motors.is_armed():
            if runner.error is not None:
                raise runner.error
            if utime.ticks_diff(utime.ticks_ms(), arm_start) > ARM_TIMEOUT_MS:
                raise Exception("Arming timed out")
            utime.sleep_ms(1)
        if written[0] != 0:
            raise Exception("FAIL frames were kept while arming: " + str(written[0]))
        print("  OK   armed; no frames kept during arming")

        motors.set_throttle(0, THROTTLE)
        utime.sleep_ms(SETTLE_MS)

        print("Sampling for " + str(SAMPLE_MS) + "ms at throttle " + str(THROTTLE) + "...")
        seen = written[0]
        total = 0
        lost = 0
        marker_ok = 0
        symbols_ok = 0
        crc_ok = 0
        erpms = array('f', [0.0] * MAX_ERPM_SAMPLES)
        erpm_count = 0
        decode_us_sum = 0
        decode_us_max = 0
        bad = []
        decode = gcr_decode.decode
        check_crc = gcr_decode.check_crc
        ticks_us = utime.ticks_us
        ticks_diff = utime.ticks_diff
        sample_start = utime.ticks_ms()
        while ticks_diff(utime.ticks_ms(), sample_start) < SAMPLE_MS:
            if runner.error is not None:
                raise runner.error
            end = written[0]
            if end == seen:
                utime.sleep_ms(1)
                continue
            if end - seen > RING:
                lost += end - seen - RING
                seen = end - RING
            while seen != end:
                frame = frames[seen & (RING - 1)]
                seen += 1
                total += 1
                t0 = ticks_us()
                number = decode(frame)
                kind = None
                data12 = 0
                if number is not None:
                    kind, data12 = check_crc(number)
                dt = ticks_diff(ticks_us(), t0)
                decode_us_sum += dt
                if dt > decode_us_max:
                    decode_us_max = dt
                if frame >> 20 == 0:
                    marker_ok += 1
                if number is not None:
                    symbols_ok += 1
                if kind is not None:
                    crc_ok += 1
                    eperiod = (data12 & 0x1FF) << ((data12 >> 9) & 0x7)
                    if eperiod and erpm_count < MAX_ERPM_SAMPLES:
                        erpms[erpm_count] = 60000000 / eperiod
                        erpm_count += 1
                elif len(bad) < SHOW_BAD:
                    bad.append(frame)

        print("  frames received: " + str(total) + " (" + str(total * 1000 // SAMPLE_MS) + "/s), lost to ring overflow: " + str(lost))
        print("  marker bit 0: " + str(marker_ok) + ", valid GCR symbols: " + str(symbols_ok) + ", CRC-valid: " + str(crc_ok))
        if total:
            print("  decode cost per frame: mean " + str(decode_us_sum // total) + "us, max " + str(decode_us_max) + "us")
        for frame in bad:
            print("  bad frame: " + "{:021b}".format(frame))

        if total < MIN_FRAMES:
            raise Exception("FAIL only " + str(total) + " frames (need " + str(MIN_FRAMES) + ")")
        pct = 100.0 * crc_ok / total
        print("  CRC-valid " + str(pct) + "%")
        if pct < MIN_CRC_VALID_PCT:
            raise Exception("FAIL CRC-valid " + str(pct) + "% below " + str(MIN_CRC_VALID_PCT))
        if erpm_count == 0:
            raise Exception("FAIL no CRC-valid frame carried an eRPM value")
        samples = sorted(erpms[:erpm_count])
        median_erpm = samples[erpm_count // 2]
        print("  eRPM min/median/max (first " + str(erpm_count) + " values): " + str(samples[0]) + " / " + str(median_erpm) + " / " + str(samples[-1]))
        if median_erpm < MIN_MEDIAN_ERPM:
            raise Exception("FAIL median eRPM " + str(median_erpm) + " below " + str(MIN_MEDIAN_ERPM) + " - motor is not spinning")
        print("  OK   motor spinning")

    except KeyboardInterrupt:
        print("\nInterrupted!")

    finally:
        motors.disarm()
        print("Motors stopped and disarmed.")
        runner.stop()
        print("Core 1 stopped.")

    print()
    print("=== Test Complete ===")


test_rle_receiver()
