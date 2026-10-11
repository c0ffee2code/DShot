"""
Checks the bidirectional receive path against AM32's own reply encoder.

AM32's make_dshot_package() (Src/dshot.c) is ported below line for line. Every
12-bit eRPM payload it can emit is encoded the way AM32 encodes it, rendered as
a waveform at a chosen reply bit period, received by the project's PIO model of
dshot_bidir_rx_frame (scripts/simulate_frame_receiver.py's run() - imported, not
copied), and decoded by driver/gcr_decode.analyze_frame(). No capture data or
hardware needed. Run from the project root:
  python scripts/verify_am32_reply.py

Three checks:
1. The decoder alone, on every payload: CRC, marker, payload and eRPM must match.
2. The receiver's pass band: identical frames against the reply's bit length in
   receiver cycles (FRAME_CYCLES_PER_BIT when the ESC matches the profile).
3. The reply bit period each AM32 MCU family actually emits, and whether the
   receiver, tuned by BIDIR_PROFILES, reads it. AM32 clocks the reply from the
   input-capture timer switched to PWM output, one timer period per bit:
   (PSC + 1) * (ARR + 1) / f_timer, PSC = output_timer_prescaler from
   Src/signal.c checkDshot(). ARR is per family in Mcu/*/Src/IO.c.

Source reference: am32-firmware/AM32 at 55c96847 (2026-09-25).
"""

import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "driver"))
sys.path.insert(0, str(ROOT / "scripts"))

import gcr_decode
from dshot_profiles import BIDIR_PROFILES, DSHOT_SPEEDS, frame_rx_speed
from simulate_frame_receiver import run

# Verbatim from AM32 Src/dshot.c
AM32_GCR_ENCODE_TABLE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]

# (family, timer clock MHz, ARR + 1). Clocks from Inc/targets.h CPU_FREQUENCY_MHZ,
# ARR from each family's sendDshotDma()/changeToOutput() in Mcu/*/Src/IO.c.
AM32_FAMILIES = [
    ("F051/F031", 48, 62),
    ("F421 (AT32)", 120, 77),
    ("E230 (GD32)", 72, 101),
    ("G071/G031", 64, 93),
    ("L431", 80, 111),
    ("G431", 160, 109),
    ("F415 (AT32)", 144, 96),
    ("V203 (CH32)", 48, 64),
]

SPEEDS = [("DSHOT300", DSHOT_SPEEDS.DSHOT300), ("DSHOT600", DSHOT_SPEEDS.DSHOT600)]


def am32_make_dshot_package(com_time):
    """
    make_dshot_package()'s eRPM path. Returns (12-bit payload, gcr[] levels from
    gcr[buffer_padding] on), a level of 1 being the timer output's active state.
    """
    com_time &= 0xFFFF
    shift_amount = 0
    for i in range(15, 8, -1):
        if (com_time >> i) == 1:
            shift_amount = i + 1 - 9
            break
    payload = (shift_amount << 9) | (com_time >> shift_amount)

    csum = 0
    csum_data = payload
    for _ in range(3):
        csum ^= csum_data
        csum_data >>= 4
    csum = (~csum) & 0xF
    full = (payload << 4) | csum

    gcrnumber = (AM32_GCR_ENCODE_TABLE[full >> 12] << 15
                 | AM32_GCR_ENCODE_TABLE[0xF & (full >> 8)] << 10
                 | AM32_GCR_ENCODE_TABLE[0xF & (full >> 4)] << 5
                 | AM32_GCR_ENCODE_TABLE[0xF & full])

    levels = [0, 1]  # gcr[bp] = 0 (idle), gcr[bp + 1] = marker
    for i in range(19, -1, -1):
        levels.append(((gcrnumber >> i) & 1) ^ levels[-1])
    levels.append(0)  # gcr[bp + 22] is never written, so stays 0 (idle)
    return payload, levels


def reply_pins(levels):
    """Pin levels of the marker and 20 data bits. The output is active-low
    (e.g. F421's cctrl = 0x3), so an active level drives the line low."""
    return [1 - level for level in levels[1:22]]


def expected_frame(levels):
    frame = 0
    for pin in reply_pins(levels):
        frame = (frame << 1) | pin
    return frame


def waveform(levels, bit_cycles):
    """level(t) in receiver cycles, the marker edge at t=0, idle-high around it."""
    pins = reply_pins(levels)

    def level(t):
        if t < 0:
            return 1
        bit = int(t // bit_cycles)
        return pins[bit] if bit < len(pins) else 1

    return level


def am32_com_times():
    """One com_time per distinct payload AM32 emits on the eRPM path, plus the
    65535 it substitutes when the motor is not running."""
    yield 65535
    for exponent in range(8):
        for mantissa in range(512):
            if exponent > 0 and mantissa < 256:
                continue  # AM32 normalises: the mantissa's top bit is set once shifted
            yield mantissa << exponent


def receive_rate(bit_cycles, com_times, rng, phases=2):
    """Fraction of replies the receiver model rebuilds identically."""
    same = total = 0
    for com_time in com_times:
        _, levels = am32_make_dshot_package(com_time)
        expected = expected_frame(levels)
        for _ in range(phases):
            total += 1
            if run(waveform(levels, bit_cycles), 1.0, rng.random()) == expected:
                same += 1
    return same / total


def check_decoder(com_times):
    failures = 0
    for com_time in com_times:
        payload, levels = am32_make_dshot_package(com_time)
        result = gcr_decode.analyze_frame(expected_frame(levels))
        period_us = (payload & 0x1FF) << (payload >> 9)
        erpm = None if period_us == 0 else 60_000_000 / period_us
        if not (result["crc_ok"] and result["marker_ok"]
                and result["data12"] == payload and result["erpm"] == erpm):
            failures += 1
            print("  decode mismatch: com_time=%d payload=0x%03X %r" % (com_time, payload, result))
    return failures


def main():
    rng = random.Random(1)
    com_times = list(am32_com_times())

    tables_match = AM32_GCR_ENCODE_TABLE == gcr_decode.GCR_ENCODE_TABLE
    print("GCR encode table identical to AM32's: %s" % tables_match)

    print("\n1. Decoder on all %d payloads AM32 emits:" % len(com_times))
    failures = check_decoder(com_times)
    print("  %d mismatches" % failures)
    payload, levels = am32_make_dshot_package(65535)
    print("  not-running reply: payload 0x%03X decodes as %.1f eRPM"
          % (payload, gcr_decode.analyze_frame(expected_frame(levels))["erpm"]))

    sample = com_times[::7]
    print("\n2. Receiver model, identical frames by reply bit length (tuned: %d cycles):"
          % 16)
    for tenths in range(136, 180, 2):
        bit_cycles = tenths / 10
        print("  %4.1f cycles  %6.2f%%" % (bit_cycles, 100 * receive_rate(bit_cycles, sample, rng)))

    print("\n3. What each AM32 MCU family emits, received with BIDIR_PROFILES' tuning:")
    print("  %-12s %-9s %10s %11s %8s %10s" % ("family", "speed", "period_us", "vs_nominal",
                                             "cycles", "identical"))
    for speed_name, speed in SPEEDS:
        rx_hz = frame_rx_speed(speed)
        nominal_us = 1e6 / (speed / 8 * 5 / 4)
        for name, timer_mhz, ticks in AM32_FAMILIES:
            if speed == DSHOT_SPEEDS.DSHOT300:
                prescaler = 3 if timer_mhz > 100 else 1
            else:
                prescaler = 1 if timer_mhz > 100 else 0
            period_us = (prescaler + 1) * ticks / timer_mhz
            bit_cycles = period_us * 1e-6 * rx_hz
            print("  %-12s %-9s %10.4f %+10.2f%% %8.2f %9.2f%%" % (
                name, speed_name, period_us, 100 * (nominal_us / period_us - 1), bit_cycles,
                100 * receive_rate(bit_cycles, sample, rng)))

    print("\n  BIDIR_PROFILES' measured periods:")
    for speed_name, speed in SPEEDS:
        profile = BIDIR_PROFILES[speed]
        print("  %-9s %.4f us" % (speed_name, profile["expected_ratio"] / profile["rx_speed"] * 1e6))

    return 0 if tables_match and failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
