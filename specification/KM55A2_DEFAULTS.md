# Skystars KM55A2: vendor defaults as read from the ESCs

The bench ESC (Skystars KM55A2) as it came from the vendor - identity and stored settings - read from the ESC itself. This page is the vendor's state only and stays as recorded: what we changed afterwards and what the ESCs hold now is in the top-level README, "ESC settings as configured". Every section says when it was recorded.

**Recorded 2026-11-04**, by two independent reads of all four ESCs through the ESC buddy (`tools/esc_buddy/`), no
writes:

- about 20:22 CET: the AM32 web configurator, through the buddy (owner; its downloads and a screenshot);
- 20:34:07 CET: `esc_buddy_run.py --read` (raw files in `captures/esc_buddy/2026-11-04_20-34-07/`, not in git).

Both reads, and the buddy's own record of the first one, are byte-for-byte identical. The four ESCs hold identical
settings.

## Identity

| What | Value | Source |
|------|-------|--------|
| Board | Skystars KM55A2, a 4-in-1: four separate MCUs, one per motor | owner; [R-003](REFERENCES.md) |
| Firmware target | `SKYSTARS_KM55_V2_F421` (AM32 `Inc/targets.h`, hardware group AT_B, dead time 60) | firmware file name read from each ESC |
| Firmware version | **AM32 2.16** ("KM55 F4") | settings block bytes 3-16 |
| MCU | F421 (configurator's reading); bootloader signature `0x1F06` | configurator, device info |
| Settings layout | EEPROM layout version 2 as shipped (the vendor's AM32 2.16); AM32 2.22 migrates the page to layout 4 on its first boot (ADR-007) | settings block byte 1 |
| Bootloader | version 1, input pin **PB4**, device-info protocol `0x01` - an older (AlkaMotors-era) AM32 bootloader | device info `34 37 31 14 1F 06 06 01 30` |

**The firmware image (dumped 2026-11-09, `esc_buddy_run.py --dump-firmware`, application area 0x1000..0x7BFF):** the four ESCs of the spare
board hold byte-identical images (SHA-256 `275f1399...d0ca08`). It is not the binary of any AM32 release for `SKYSTARS_KM55_V2_F421` from 2.16 to 2.22:
against the 2.16 release 19,178 of 27,648 bytes differ, but aligned with it (a shift of 0x30, later 0x2c and 0x28 bytes from about 0x2400)
the code matches 96-100% per 1 KB window; 2.17 and later fit worse. So it is a 2.16 build that differs from the release in a few small places.
The "2.16" label is the firmware's own stamp: AM32 2.16 `Src/main.c` rewrites settings bytes 3-4 (and the name in 5-16) from its compiled-in
VERSION_MAJOR/MINOR at boot when they differ. No copy of this image is kept: the dumps were discarded on 2026-11-10 (no downgrade is planned, ADR-007).

Not the E230 target the retailers list. The F421 target matches the ESC's bidirectional reply bit time measured earlier
(1.280-1.285 us at DSHOT600; F421: 1.2833 us).

**On the DShot wire** (recorded by the wingman's spy 2026-10-28, bidirectional, one ESC on channel 1, throttle 100,
DSHOT600 x2 and DSHOT300 x1): the reply starts 23.2 us (DSHOT600) / 25.8 us (DSHOT300) after our frame; reply bit time
1.284 / 2.565 us (F421 figure 1.2833 / 2.5667 us); first reply after 104 of our frames (AM32's latch is 101); then every
frame answered outside the arming silence, which began 1020 ms after our first frame and lasted 301 ms (the source's
1.02 s gate and 3 x 100 ms tune); 0 replies with a bad CRC. Our frames as this ESC sees them: the 0-bit pulse 0.20 / 0.39 us
below AM32's bit threshold and the 1-bit pulse 0.43 / 0.86 us above it (DSHOT600 / DSHOT300), worst frame length 0.2% off
the window it learns. Not covered: other channels, several ESCs, other throttles, cold power-on.

**Talking to this bootloader** (isolated 2026-11-05): it starts its reply **172-176 us after the end of our last stop
bit**, so the host must have let go of the line by then. Byte spacing, stop-bit handling and the idle level make no
difference to it. Details: ADR-006 and `tools/esc_buddy/README.md`.

## Stored settings (all four ESCs, as recorded above)

| Setting | Byte | Raw | Value |
|---------|------|-----|-------|
| Motor KV | 26 | 55 | **2220** (raw x 40 + 20) |
| Motor poles | 27 | 14 | **14** |
| Startup power | 25 | 100 | **100 %** |
| Timing advance | 23 | 2 | **15 deg** (old format, 7.5 deg steps) |
| Auto timing advance | 47 | 0 | off |
| PWM frequency | 24 | 24 | 24 kHz |
| Variable PWM | 21 | 1 | on (configurator shows 24-48 kHz) |
| Complementary PWM | 20 | 1 | on |
| Stuck rotor protection | 22 | 1 | on |
| Stall protection | 29 | 0 | **off** |
| Sinusoidal startup | 19 | 0 | off (range 15 %, power 6 stored) |
| 3D mode (bi-direction) | 18 | 0 | off |
| Direction reversed | 17 | 0 | off |
| Brake on stop | 28 | 0 | off |
| Running brake level / brake strength | 42 / 41 | 10 / 10 | 10 / 10 |
| Temperature limit | 43 | 141 | disabled (valid 70-140) |
| Current limit | 44 | 102 | disabled (valid 1-100) |
| Low voltage cut off | 36 | 0 | off (threshold 3.00 V per cell stored) |
| Input type | 46 | 0 | auto |
| Beeper volume | 30 | 5 | 5 |
| 30 ms interval telemetry | 31 | 0 | off |

Whole block as shipped (layout 2, 184 bytes): `01020102104b4d35352046342020202020000000010101021864370e0000050080808032003200000f0a0a8d66060000`
followed by 136 bytes of `ff`. Decoder: `tools/esc_buddy/esc_buddy_settings.py`, tested against the configurator's
display (`tests/pc/test_esc_buddy_settings.py`).

## What this means

- These are **AM32's generic template defaults** (they equal the AM32 wiki's EEPROM-format defaults, [R-002](REFERENCES.md):
  KV byte 55, 14 poles, startup power 100, 15 deg, 24 kHz). Nobody has tuned them for the fitted motors: the ESC assumes
  a 14-pole, ~2220 KV motor whatever is on the shaft.
- In layout 2, bytes 5-16 hold the firmware name. `minimum_duty_cycle` and `max_ramp`, settings in the newer layout,
  are **not settings in firmware 2.16** but built-in values. Any reasoning about startup duty must use the **2.16**
  source, not the newer commit `AM32.md` is pinned to.
- The stored KV and poles assumed a 14-pole, 2220 KV motor; the fitted motors are BETAFPV LAVA 1104 (7200 KV, 12 magnets). What was changed to match: the top-level README, "ESC settings as configured".
