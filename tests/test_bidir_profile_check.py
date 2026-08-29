# One-off sanity check for BIDIR_PROFILES (W1): confirm DShotPIO's public
# constructor now looks up rx_speed per dshot_speed instead of the old
# hardcoded 4MHz, by exercising DSHOT600 through the real API (not the
# spike's hand-built StateMachine bypass), and confirm the ValueError guard
# fires for dshot_speeds with no verified profile. DSHOT150 and DSHOT1200
# are no longer named DSHOT_SPEEDS constants at all (AM32 only documents
# DShot300/600 support - see DSHOT_SPEEDS's comment in driver/dshot_pio.py),
# so their old frequencies are passed as raw literals here purely to confirm
# the guard still rejects them.
#
# Not a permanent regression test - throwaway, matches this project's
# convention for one-off verification scripts (see test_bidir_rx_raw.py's
# header for the pattern this borrows).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
import utime

ARM_DURATION_MS = 3000
SETTLE_MS = 3000  # let the motor spin up and settle before sampling, matching the properly
                  # throttle-stepped W1 sweep rather than jumping straight from arm to sampling
RUN_DURATION_MS = 3000
SETTLE_THROTTLE = 200
STOP_DURATION_MS = 300


def test_speed_rejected(speed, label):
    print(f"--- {label} + bidirectional=True should raise ValueError ---")
    try:
        DShotPIO(0, Pin(2), speed, bidirectional=True, rx_state_machine_id=1)
    except ValueError as e:
        print(f"  OK - raised ValueError: {e}")
    else:
        print("  FAIL - no exception raised")
    print()


def test_dshot600_via_public_api():
    print("--- DSHOT600 bidirectional through the public DShotPIO API ---")
    pin = Pin(2)
    ch1 = DShotPIO(0, pin, DSHOT_SPEEDS.DSHOT600, bidirectional=True, rx_state_machine_id=1)
    others = [
        DShotPIO(sm_id, Pin(p), DSHOT_SPEEDS.DSHOT600)
        for sm_id, p in zip((4, 5, 6), (3, 4, 5))
    ]
    motors = [ch1] + others

    for motor in motors:
        motor.start()

    try:
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
            while ch1.rx_read() is not None:
                pass

        settle_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), settle_start) < SETTLE_MS:
            for motor in motors:
                motor.send_throttle_command(SETTLE_THROTTLE if motor is ch1 else 0)
            while ch1.rx_read() is not None:
                pass

        run_start = utime.ticks_ms()
        total_words = 0
        snapshot = []
        while utime.ticks_diff(utime.ticks_ms(), run_start) < RUN_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(SETTLE_THROTTLE if motor is ch1 else 0)
            while True:
                word = ch1.rx_read()
                if word is None:
                    break
                total_words += 1
                if len(snapshot) < 8:
                    snapshot.append(word)

        formatted = ", ".join(f"0x{w:08x}" for w in snapshot)
        print(f"  {total_words} words drained. Sample: [{formatted}]")
        print("  Feed the sample above into scripts/decode_bidir_capture.py with "
              "RX_CLOCK_HZ=8_000_000 to confirm CRC-valid.")

    finally:
        stop_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), stop_start) < STOP_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
        for motor in motors:
            motor.drain()
            motor.stop()
    print()


test_speed_rejected(1_200_000, "old DSHOT150 rate")   # 150,000 bit/s * 8 cycle/bit - no longer a named speed
test_speed_rejected(9_600_000, "old DSHOT1200 rate")  # 1,200,000 bit/s * 8 cycle/bit - no longer a named speed
test_dshot600_via_public_api()
print("=== Test Complete ===")
