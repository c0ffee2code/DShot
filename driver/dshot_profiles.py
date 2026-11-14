"""
DShot speeds and the reply receiver's profile per speed, as plain data.
No hardware imports, so PC tools and tests read the values the Pico uses;
dshot_pio imports and re-exports them. Design: ADR-002.
"""

# The DShot speeds offered: DSHOT300 and DSHOT600, the two AM32 documents.
# DSHOT150 and DSHOT1200 are not offered: AM32 does not document them and has
# no path of its own for DSHOT1200 (it falls into a coarse input-rate band).
# Values are PIO clock frequencies: bit rate x 8 cycles per bit.
class DSHOT_SPEEDS:
    DSHOT300 = 2_400_000 # 300,000 bit/s * 8 cycle/bit
    DSHOT600 = 4_800_000 # 600,000 bit/s * 8 cycle/bit

# Reply receiver settings per DShot speed, measured on the bench ESC (ADR-002);
# BidirectionalDShot accepts only the speeds listed here.
#
#   rx_speed         reference clock in Hz, 9 x the nominal reply bit rate
#   expected_ratio   the reply's measured bit period in rx_speed cycles. AM32
#                    times its reply from its MCU's own timer, so the rate is
#                    a fixed offset from nominal. frame_rx_speed() derives the
#                    frame receiver's clock from these two.
BIDIR_PROFILES = {
    # rx_speed = 9 x the nominal 375kHz reply rate
    DSHOT_SPEEDS.DSHOT300: {"rx_speed": 3_375_000, "expected_ratio": 8.7069},
    # rx_speed = 9 x the nominal 750kHz reply rate
    DSHOT_SPEEDS.DSHOT600: {"rx_speed": 6_750_000, "expected_ratio": 8.7129},
}

# Cycles per reply bit in dshot_pio.dshot_bidir_rx_frame, fixed by its instructions
FRAME_CYCLES_PER_BIT = 16


def frame_rx_speed(dshot_speed):
    """
    The frame receiver's clock for a DShot speed, in Hz: FRAME_CYCLES_PER_BIT
    times the measured reply bit rate, rx_speed / expected_ratio.
    """
    profile = BIDIR_PROFILES[dshot_speed]
    return round(FRAME_CYCLES_PER_BIT * profile["rx_speed"] / profile["expected_ratio"])
