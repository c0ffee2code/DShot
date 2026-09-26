"""
DSHOT_SPEEDS / BIDIR_PROFILES - pure data, no hardware imports (unlike
dshot_pio.py, which imports machine/rp2/utime at module level and can
therefore only run under MicroPython on the Pico). They live here so PC-side
tooling (e.g. scripts/verify_gcr_decode_port.py) can read the live values
directly instead of keeping a hand-maintained copy that can drift out of sync.

driver/dshot_pio.py imports and re-exports both names, so
`from dshot_pio import BIDIR_PROFILES, DSHOT_SPEEDS` keeps working. This file
is deployed to the Pico alongside dshot_pio.py, which needs it too.
"""

# The DShot speeds this project supports: DSHOT300 and DSHOT600, the two AM32
# documents (its README and wiki.am32.ca). AM32's source is this project's
# ground truth for ESC behaviour, and DSHOT150/DSHOT1200 fall outside it -
# AM32 has no distinct DSHOT1200 path, it only happens to accept that signal
# through its coarse input-rate bands - so they are not offered as named speeds.
# Values are PIO clock frequencies: bit_rate * 8 cycles per bit.
class DSHOT_SPEEDS:
    DSHOT300 = 2_400_000 # 300,000 bit/s * 8 cycle/bit
    DSHOT600 = 4_800_000 # 600,000 bit/s * 8 cycle/bit

# Settings for the bidirectional reply receiver, per DShot speed. Only speeds
# with an entry here can be used with BidirectionalDShot.
#
#   rx_speed         the sample receiver's own clock (dshot_bidir_rx, used
#                    directly by the standalone calibration tool, and by
#                    rle_rx_speed() below to derive the frame receiver's).
#                    Independent of the DShot speed (TX and RX have separate
#                    clock dividers on the same PIO block) and sets how many
#                    samples land in each bit of the ESC's reply.
#   expected_ratio   the reply's measured bit period in RX clock cycles. It is
#                    measured on hardware rather than derived from the nominal
#                    reply rate, because ESC oscillators run a few percent off
#                    nominal. gcr_decode.estimate_bit_period_fixed uses it as a
#                    fixed divisor instead of searching for the period on every
#                    capture, which is what keeps decoding cheap enough to run
#                    on the Pico.
#   ratio_tolerance  0.0 uses expected_ratio as a bare divisor; above 0.0 the
#                    decoder searches a band of that half-width around it, for
#                    a profile whose measured spread is too wide to trust bare.
#
# See decision/ADR-002-bidirectional-dshot.md's fixed-ratio RX sampling section
# for how these values were measured and chosen.
BIDIR_PROFILES = {
    # rx_speed = 9 x the nominal 375kHz reply rate; the measured 8.7069 cycles per bit means the ESC replies ~3% faster than nominal
    DSHOT_SPEEDS.DSHOT300: {"rx_speed": 3_375_000, "expected_ratio": 8.7069, "ratio_tolerance": 0.0},
    # rx_speed = 9 x the nominal 750kHz reply rate
    DSHOT_SPEEDS.DSHOT600: {"rx_speed": 6_750_000, "expected_ratio": 8.7129, "ratio_tolerance": 0.0},
}

# Cycles per reply bit that dshot_pio.dshot_bidir_rx_rle's per-bit path takes,
# fixed by its instructions. Its receiver clock is this many times the reply bit
# rate, which is expected_ratio's measured rate: rx_speed / expected_ratio.
RLE_CYCLES_PER_BIT = 16


def rle_rx_speed(dshot_speed):
    """The frame receiver's clock for a DShot speed, in Hz."""
    profile = BIDIR_PROFILES[dshot_speed]
    return round(RLE_CYCLES_PER_BIT * profile["rx_speed"] / profile["expected_ratio"])
