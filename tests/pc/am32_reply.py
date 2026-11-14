"""
am32_reply.py - AM32's own reply encoder, ported line for line, and a waveform of its reply for the PIO receiver model.
Test support for tests/pc (not a test itself): test_am32_reply_path.py and test_gcr_decode.py use it.

make_dshot_package() is from AM32 Src/dshot.c (am32-firmware/AM32 release 2.22; the file is unchanged from the 2026-09-25 reading). The reply bit period is the
F421's, the only MCU family supported (the bench ESC's, read from the ESC with tools/esc_buddy): AM32 clocks the reply from
the input-capture timer switched to PWM output, one timer period per bit, (PSC + 1) * (ARR + 1) / f_timer, with PSC =
output_timer_prescaler from Src/signal.c checkDshot() and ARR from Mcu/f421/Src/IO.c (pr = 76, checked at v2.22, the
bench ESC's firmware).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for folder in (ROOT / "driver", ROOT / "scripts"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from dshot_profiles import DSHOT_SPEEDS, frame_rx_speed  # noqa: E402
from simulate_frame_receiver import run  # noqa: E402  (the PIO model of dshot_bidir_rx_frame)

# Verbatim from AM32 Src/dshot.c
AM32_GCR_ENCODE_TABLE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]

F421_TIMER_MHZ = 120                # CPU_FREQUENCY_MHZ of the F421 targets
F421_REPLY_TICKS = 77               # Mcu/f421/Src/IO.c: IC_TIMER_REGISTER->pr = 76


def am32_make_dshot_package(com_time):
    """make_dshot_package()'s eRPM path. Returns (12-bit payload, gcr[] levels from gcr[buffer_padding] on), a level of 1
    being the timer output's active state."""
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
    """Pin levels of the marker and 20 data bits. The output is active-low (F421's cctrl = 0x3), so an active level drives
    the line low."""
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
    """One com_time per distinct payload AM32 emits on the eRPM path, plus the 65535 it substitutes when the motor is not
    running."""
    yield 65535
    for exponent in range(8):
        for mantissa in range(512):
            if exponent > 0 and mantissa < 256:
                continue  # AM32 normalises: the mantissa's top bit is set once shifted
            yield mantissa << exponent


def f421_bit_cycles(speed):
    """The F421's reply bit period in receiver cycles at this DSHOT speed."""
    prescaler = 3 if speed == DSHOT_SPEEDS.DSHOT300 else 1     # output_timer_prescaler for a >100 MHz timer
    period_us = (prescaler + 1) * F421_REPLY_TICKS / F421_TIMER_MHZ
    return period_us * 1e-6 * frame_rx_speed(speed)


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
