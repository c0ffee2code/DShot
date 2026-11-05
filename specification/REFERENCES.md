# External References

Registry of external material (videos, articles, repositories) relevant to this project. This is a
pointer list, not evidence: a reference gives background or a lead, and a claim taken from one is
verified against AM32's source (`AM32.md`) or a bench measurement (README's Operational Envelope, the bug reports, `KM55A2_DEFAULTS.md`) before the
project relies on it.

## How to add an entry

One row per item, newest last. Fill in only what was actually checked; leave "Notes" short and say
who the claim comes from (the video, the owner, or the repo).

| Column | Meaning |
|--------|---------|
| ID | `R-NNN`, next free number |
| Type | video, article, repo, datasheet |
| Topic | what it covers, in a few words |
| Title / link | as published |
| Added | date (YYYY-MM-DD) |
| Notes | why it matters here; what has and has not been checked against the repo |

## Videos

| ID | Type | Topic | Title / link | Added | Notes |
|----|------|-------|--------------|-------|-------|
| R-001 | video | Tuning AM32 ESCs | [Tuning AM32 ESCs](https://www.youtube.com/watch?v=3SHzyUaypFw) (title as given by the owner; check against the page) | 2026-11-03 | Added by the owner. Not yet watched or summarised; nothing from it has been checked against AM32's source. |

## Articles and repositories

| ID | Type | Topic | Title / link | Added | Notes |
|----|------|-------|--------------|-------|-------|
| R-002 | article | AM32 settings layout | [Open ESC EEPROM Format](https://wiki.am32.ca/development/Open-ESC-EEPROM-Format.html) | 2026-11-03 | Lists defaults (KV byte 55, 14 poles, startup power 100, 24 kHz PWM, ...) for an old layout (firmware 1.04, bytes 0-41). Not tied to the KM55A2's flashed values; newer fields (max ramp, minimum duty, current limit) are missing. |
| R-003 | article | KM55A flashing | [Flashing SkyStars KM55A](https://wiki.am32.ca/guides/Flashing-SkyStars-KM55A.html) | 2026-11-03 | Each of the four ESCs is flashed on its own pads (separate chips). No settings values. |
| R-004 | article | KV and poles | [Recommended Settings For Freestyle](https://wiki.am32.ca/general/Recommended-Settings-For-Freestyle.html) | 2026-11-03 | Pasted by the owner: KV and poles set AM32's low/high-speed thresholds. The low-RPM duty limit matches `Src/main.c` (pinned commit); the high-speed stall side was not checked. |
| R-005 | repo | Passthrough reference | [BrushlessPower/BlHeli-Passthrough](https://github.com/BrushlessPower/BlHeli-Passthrough/tree/main/RP2040) @ `ee58a4a` | 2026-11-03 | Found by the owner. Arduino C++ MSP + 4-way passthrough for RP2040; the model for `tools/esc_buddy/`. |
| R-006 | repo | AM32 bootloader | [am32-firmware/AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader) @ `578ff29` | 2026-11-03 | Separate from the firmware repo. Entry on a high line (`checkForSignal()`), 19200 baud, command set. The ESCs run whatever bootloader the vendor flashed, not necessarily this version. |
| R-007 | repo | AM32 configurator | [am32-firmware/am32-configurator](https://github.com/am32-firmware/am32-configurator) @ `06955fe` | 2026-11-03 | The web app. The connect sequence the buddy answers; the Pico's USB vendor ID `0x2E8A` is on its flight-controller list; it writes settings on its own when the bootloader version reads `0xFF` (`four_way.ts`). |
