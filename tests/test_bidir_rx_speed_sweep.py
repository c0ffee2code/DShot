# Standalone spike (W1, see bidirectional_dshot_review.md): does the current
# dense-oversampling RX design (dshot_bidir_rx, tuned for DSHOT300's ~2.5-2.6us
# real GCR bit period at rx_speed=4MHz) still decode at faster DShot request
# rates, or does it need a faster rx_speed / different predelay?
#
# Deliberately does NOT touch driver/dshot_pio.py or DShotPIO's public shape -
# this is throwaway characterization, not a driver change. Channel 1's TX
# comes from a real DShotPIO(bidirectional=True) instance (reuses its
# verified packet/CRC encoding); channel 1's actual RX under test is a
# separate, hand-built StateMachine reusing dshot_bidir_rx UNCHANGED, at
# whatever rx_speed this run is sweeping - DShotPIO's own internal RX (fixed
# 4MHz, required by its constructor) is created but never read here.
#
# rx_speed candidates: current 4MHz as the control, plus one scaled up to
# restore ~5 samples/plausible-bit at the faster wire rate (dshot_bidir_rx's
# own stated minimum - see its module comment). DSHOT600's real bit period is
# nominally ~half DSHOT300's -> try 8MHz. DSHOT1200's is nominally ~half
# DSHOT600's again -> try 8MHz (matches DSHOT600's density) and 16MHz (tries
# to restore it further). Per AM32's Src/signal.c checkDshot(): the ESC only
# bins detected input rate into two bands (roughly slow=150/300,
# fast=600/1200), each with its own output_timer_prescaler/buffer_padding for
# the reply - so 600 and 1200 likely share AM32-side reply timing, which this
# sweep's data will confirm or refute.
#
# No decoding here - see scripts/decode_bidir_capture.py (RX_CLOCK_HZ needs
# to be set to whichever rx_speed a given block below used before decoding
# it).
#
# Hardware: 4-in-1 AM32 ESC, channel 1 -> GPIO 2 (motor+prop mounted),
# channels 2-4 -> GPIO 3/4/5 (wired but idle only).

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS, dshot_bidir_rx
from rp2 import StateMachine
import utime

# (dshot_speed, label, [candidate rx_speeds to try])
CANDIDATES = [
    (DSHOT_SPEEDS.DSHOT600, "DSHOT600", [4_000_000, 8_000_000]),
    (DSHOT_SPEEDS.DSHOT1200, "DSHOT1200", [4_000_000, 8_000_000, 16_000_000]),
]

ARM_DURATION_MS = 3000
THROTTLE_STEPS = [(100, 3), (200, 3), (300, 3)]  # (throttle, seconds) - shorter than
                                                  # test_bidir_rx_raw.py's 6s/step to keep
                                                  # this multi-config sweep's total runtime down
STOP_DURATION_MS = 300
SNAPSHOT_INTERVAL_MS = 1000
MAX_SNAPSHOT_WORDS = 8


def current_throttle(elapsed_ms):
    remaining = elapsed_ms
    for throttle, seconds in THROTTLE_STEPS:
        step_ms = seconds * 1000
        if remaining < step_ms:
            return throttle
        remaining -= step_ms
    return THROTTLE_STEPS[-1][0]


def run_one(dshot_speed, label, rx_speed):
    print(f"--- {label} (dshot_speed={dshot_speed}) @ candidate rx_speed={rx_speed} ---")
    pin = Pin(2)
    # TX + DShotPIO's own required-but-unused internal RX (fixed 4MHz, ignored below)
    ch1 = DShotPIO(0, pin, dshot_speed, bidirectional=True, rx_state_machine_id=1)
    # The actual RX under test: same PIO block (id 2, still PIO0), dshot_bidir_rx
    # unchanged, this sweep's candidate rx_speed
    candidate_rx = StateMachine(2, dshot_bidir_rx, freq=rx_speed, in_base=pin)

    others = [
        DShotPIO(sm_id, Pin(p), dshot_speed)
        for sm_id, p in zip((4, 5, 6), (3, 4, 5))
    ]
    motors = [ch1] + others

    for motor in motors:
        motor.start()
    candidate_rx.active(1)

    try:
        arm_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), arm_start) < ARM_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
            while ch1.rx_read() is not None:
                pass
            while candidate_rx.rx_fifo():
                candidate_rx.get()

        run_duration_ms = sum(seconds for _, seconds in THROTTLE_STEPS) * 1000
        run_start = utime.ticks_ms()
        last_snapshot_ms = run_start
        snapshots_taken = 0
        total_words = 0

        while utime.ticks_diff(utime.ticks_ms(), run_start) < run_duration_ms:
            now = utime.ticks_ms()
            elapsed_ms = utime.ticks_diff(now, run_start)
            throttle = current_throttle(elapsed_ms)

            for motor in motors:
                motor.send_throttle_command(throttle if motor is ch1 else 0)

            while ch1.rx_read() is not None:
                pass  # DShotPIO's own internal 4MHz RX - unused in this spike

            take_snapshot = utime.ticks_diff(now, last_snapshot_ms) >= SNAPSHOT_INTERVAL_MS
            if take_snapshot:
                last_snapshot_ms = now

            snapshot = []
            while candidate_rx.rx_fifo():
                word = candidate_rx.get()
                total_words += 1
                if take_snapshot and len(snapshot) < MAX_SNAPSHOT_WORDS:
                    snapshot.append(word)

            if take_snapshot:
                snapshots_taken += 1
                formatted = ", ".join(f"0x{w:08x}" for w in snapshot)
                print(f"  [{snapshots_taken}] throttle={throttle} words=[{formatted}]")

        print(f"  {total_words} words drained ({snapshots_taken} snapshots printed above).")

    finally:
        stop_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), stop_start) < STOP_DURATION_MS:
            for motor in motors:
                motor.send_throttle_command(0)
        for motor in motors:
            motor.drain()
            motor.stop()
        candidate_rx.active(0)
        candidate_rx.restart()
    print()


def test_bidir_rx_speed_sweep():
    print("=== Bidirectional RX Speed Sweep Spike (W1) ===")
    print()
    for dshot_speed, label, rx_speeds in CANDIDATES:
        for rx_speed in rx_speeds:
            run_one(dshot_speed, label, rx_speed)
    print("=== Sweep Complete ===")


test_bidir_rx_speed_sweep()
