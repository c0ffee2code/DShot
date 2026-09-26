# Test: the frame receiver (dshot_bidir_rx_rle) survives an arm/disarm/arm
# restart in one session, on a PIO block it fills alone.
#
# Purpose: W23 (bidirectional_dshot_review.md). tests/device/test_bidir_restart_cycles.py
# already proved this for the sample receiver, on a block with room to spare
# (13 + 10 of 32 slots). The frame receiver's block is exactly full (13 + 19),
# and BidirectionalDShot.start() calls rx_sm.init() with this program on every
# arm(), restart included - a path nothing has run on hardware before this.
# If MicroPython cannot re-init an already-loaded program on an exactly-full
# block, cycle 2's arm() fails with ENOMEM; cycle 1 alone would not show that.
#
# Wiring matches rle_bench.py: channel 1 (GPIO 6) is the frame receiver, on
# its own PIO block (sm0 TX + sm1 RX); channels 2-4 (GPIO 7/8/9) are
# unidirectional and sit on the next block (sm4/5/6) since a frame-receiver
# block has no room left for them.
#
# Pass: every cycle decodes at least one CRC-valid frame with a real (spinning)
# eRPM, not only cycle 1.

from machine import Pin
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS
from motor_group import MotorGroup
from core1_runner import Core1Runner
import utime

CYCLES = 2
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

    # Documented shutdown order (see CLAUDE.md's disarm() bullet): stop the
    # command loop before disarm() touches any state machine.
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
        UnidirectionalDShot(4, Pin(7), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(5, Pin(8), DSHOT_SPEEDS.DSHOT300),
        UnidirectionalDShot(6, Pin(9), DSHOT_SPEEDS.DSHOT300),
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
