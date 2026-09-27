# Test: BidirectionalDShot survives repeated arm/disarm cycles in one session
#
# Purpose: every scenario in tests/harness/ builds its motors once, arms once,
# and disarms once - deploy.py resets the Pico before each run, so nothing has
# ever exercised BidirectionalDShot.start() reclaiming the pin from PIO after
# stop() hands it to SIO (see stop()'s own comment). A scenario passing proves
# the FIRST start() works; it says nothing about the second.
#
# This arms, spins and disarms the same motor three times in a row, on one
# Pico session, with a gap between cycles long enough for the disarmed ESC to
# time out, reboot and settle - the same condition every real flight would hit
# between arming sequences. Each cycle's own command loop runs on a fresh
# Core1Runner start/stop, in the documented order (stop the loop, then
# disarm) so a race between them can't be mistaken for a reclaim failure.
#
# Wiring matches tests/harness/scenarios/single_channel_bidirectional_300.json exactly
# (channel 1 bidirectional, channels 2-4 unidirectional at zero) so channel
# 3's line stays driven, not floating - the same reason F3 used that scenario
# rather than building channel 1 alone (see bidirectional_dshot_review.md's
# ESC bootloader hang finding). During the gaps, channel 1 and channel 3
# should sound the same: this is worth listening to, not just reading the
# printed numbers.
#
# Needs the ESC powered and motors on channels 1 and 3, matching the current
# bench wiring (GPIO6-9).
#
# Pass: every cycle decodes at least one capture, at least 99% of decoded
# captures are CRC-valid, and the median eRPM is at least 10,000 (the motor
# genuinely spun, not just replied at-rest) - not only on cycle 1.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner
import utime

CYCLES = 3
ARM_MS = 3000
SPIN_MS = 3000
THROTTLE = 100
GAP_MS = 3000  # >= the ESC's own unarmed signal-loss timeout, so it truly reboots between cycles
MIN_CRC_VALID_PCT = 99.0
MIN_MEDIAN_ERPM = 10000


def run_one_cycle(group, runner, cycle):
    print("=== Cycle %d/%d: arming ===" % (cycle, CYCLES))
    runner.start()
    group.arm(ARM_MS)
    deadline = utime.ticks_add(utime.ticks_ms(), ARM_MS + 2000)
    while not group.is_armed():
        if utime.ticks_diff(deadline, utime.ticks_ms()) < 0:
            raise Exception("cycle %d: arming timed out" % cycle)
        utime.sleep_us(300)

    group.set_throttle(0, THROTTLE)

    decoded = 0
    crc_ok = 0
    erpms = []
    last_seq = -1
    end = utime.ticks_add(utime.ticks_ms(), SPIN_MS)
    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        capture = group.raw_telemetry(0)
        if capture is not None:
            _, seq, words = capture
            if seq != last_seq:
                last_seq = seq
                result = group.decode_telemetry(0, words)
                decoded += 1
                if result["crc_ok"]:
                    crc_ok += 1
                    if result["erpm"] is not None:
                        erpms.append(result["erpm"])
        utime.sleep_ms(20)

    # Documented shutdown order (see CLAUDE.md's disarm() bullet and the
    # usage example): stop the command loop before disarm() touches any
    # state machine, so a reclaim failure can't be confused with this race.
    runner.stop()
    group.disarm()

    erpms.sort()
    median_erpm = erpms[len(erpms) // 2] if erpms else 0
    crc_valid_pct = (100.0 * crc_ok / decoded) if decoded else 0.0
    print("  decoded=%d crc_ok=%d crc_valid_pct=%.1f median_erpm=%d" %
          (decoded, crc_ok, crc_valid_pct, median_erpm))

    ok = decoded > 0 and crc_valid_pct >= MIN_CRC_VALID_PCT and median_erpm >= MIN_MEDIAN_ERPM
    if not ok:
        print("  FAIL on cycle %d" % cycle)
    return ok


def main():
    motors = [
        BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1),
        UnidirectionalDShot(8, Pin(7), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(9, Pin(8), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(10, Pin(9), DSHOT_SPEEDS.DSHOT300),
    ]
    group = MotorGroup(motors)
    runner = Core1Runner(group.update, group.UPDATE_INTERVAL_US)

    results = []
    try:
        for cycle in range(1, CYCLES + 1):
            results.append(run_one_cycle(group, runner, cycle))
            if cycle < CYCLES:
                print("  disarmed - waiting %dms for the ESC to time out and reboot" % GAP_MS)
                utime.sleep_ms(GAP_MS)
    finally:
        runner.stop()
        group.disarm()

    print("=== Summary: %d/%d cycles passed ===" % (sum(results), CYCLES))
    if not all(results):
        raise Exception("at least one cycle failed - see FAIL lines above")


main()
