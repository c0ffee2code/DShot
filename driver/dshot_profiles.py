"""
DSHOT_SPEEDS / BIDIR_PROFILES - pure data, no hardware imports (unlike
dshot_pio.py, which imports machine/rp2/utime at module level and can
therefore only run under MicroPython on the Pico). Split out specifically
so PC-side tooling (e.g. scripts/verify_gcr_decode_port.py) can read the
live profile values directly instead of duplicating them in a hand-
maintained mirror - the same fragility this project already hit once
with scripts/analyze_bidir_stress_log.py's own BIDIR_PROFILES copy.

driver/dshot_pio.py imports and re-exports both names from here, so every
existing `from dshot_pio import BIDIR_PROFILES`/`DSHOT_SPEEDS` call site
keeps working unchanged. Deployed to the Pico by scripts/deploy.py
alongside dshot_pio.py - it needs this file too, not just the PC side.
"""

# The DShot speeds this project supports. Restricted to what AM32 itself
# documents (its README and wiki.am32.ca both list DShot300/600 only) -
# DSHOT150 and DSHOT1200 were never part of that support matrix (DSHOT1200
# was measured working once, but only via undocumented rate-detection
# overlap with DSHOT600 - see BIDIR_PROFILES below), so this project doesn't
# carry them as named speeds per CLAUDE.md's design constraint (AM32's
# source/docs are ground truth; no configuration surface for cases outside
# the two ESC families this project targets).
class DSHOT_SPEEDS:
    DSHOT300 = 2_400_000 # 300,000 bit/s * 8 cycle/bit
    DSHOT600 = 4_800_000 # 600,000 bit/s * 8 cycle/bit

# rx_speed to use for dshot_bidir_rx per DShot request speed - hardware-verified
# (bidirectional_dshot_review.md's W1 item), not a fixed ratio of dshot_speed.
# DSHOT1200 is deliberately absent: AM32 documents bidirectional support for
# DSHOT300/600 only (its own README, and wiki.am32.ca) - Src/signal.c's
# checkDshot() has no distinct DSHOT1200 path, it just bins detected input
# rate into two coarse reply-timing bands (~150/300 and ~600/1200) with loose
# pulse-width thresholds, so DSHOT1200 happening to fall in the "600" band and
# getting a CRC-valid reply on this specific ESC is undocumented incidental
# behavior, not a feature AM32 tests or guarantees - per this project's
# AM32-source-is-ground-truth constraint (CLAUDE.md), that makes it
# unsupported here too, even though it was observed working (4/4 CRC-valid
# at rx_speed=8MHz, same measured ~1.28-1.29us reply bit period as DSHOT600).
#
# Each entry is {"rx_speed": ..., "expected_ratio": ..., "ratio_tolerance": ...}.
# rx_speed is the RX state machine's own clock (see BidirectionalDShot.__init__ in
# dshot_pio.py - independent of dshot_speed, TX and RX have separate clock
# dividers on the same PIO block). expected_ratio/ratio_tolerance feed
# gcr_decode.py's estimate_bit_period_fixed (a Betaflight-style fixed
# divisor - or narrow band, if ratio_tolerance>0 - replacing the
# brute-force sweep) - see decision/ADR-002-bidirectional-dshot.md's
# fixed-ratio RX sampling section. expected_ratio is None until a rate is
# retuned to a clean integer ratio and verified on real hardware (see that
# ADR section's plan) - poll_telemetry() passes these straight through to
# analyze_capture() for every profile, so a None here keeps the
# brute-force sweep as that speed's live behavior. Both DSHOT300 and
# DSHOT600 were retuned and verified 2026-09-12 (both K=9) and now always
# use the fixed-ratio path - see decision/ADR-002-bidirectional-dshot.md's
# fixed-ratio RX sampling section for the retirement of the sweep itself
# from driver/gcr_decode.py that followed. ratio_tolerance defaults to 0.0
# (bare fixed divisor) and is only meaningful once expected_ratio is set.
BIDIR_PROFILES = {
    DSHOT_SPEEDS.DSHOT300: {"rx_speed": 3_375_000, "expected_ratio": 8.7069, "ratio_tolerance": 0.0},  # K=9, 100% CRC-valid (4743/4743), std=0.0477 - see ADR-002 fixed-ratio retune, captures/2026-09-12_13-04-36
    DSHOT_SPEEDS.DSHOT600: {"rx_speed": 6_750_000, "expected_ratio": 8.7129, "ratio_tolerance": 0.0},  # K=9, 100% CRC-valid (2463/2463), std=0.0576 - see ADR-002 fixed-ratio retune, captures/2026-09-12_16-01-39
}
