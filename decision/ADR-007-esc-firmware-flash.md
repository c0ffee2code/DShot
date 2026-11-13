# ADR-007: Flashing ESC Firmware Through the Main Pico (the ESC buddy's `--flash`)

**Status:** Accepted (used on all eight bench ESCs, 2026-11-09 and 2026-11-10)
**Date:** 2026-11-09
**Builds on:** [ADR-006](ADR-006-esc-configuration-access.md) (the Pico speaks the AM32 bootloader protocol on each ESC's signal
line and refuses every write except guarded ones)

## Context

The bench ESCs (Skystars KM55A2, four AM32 F421 chips per board, bootloader version 1) shipped with AM32 2.16, and the owner wanted
newer firmware on them (2.22 was released 2026-11-07). Flashing means writing the application area, 0x1000..0x7BFF, over the bootloader
protocol the buddy already speaks. ADR-006 made the buddy write-guarded; a flash writes 27 KB, so the guard has to grow with it.

The AM32 configurator flashes like this (recorded 2026-11-09 with `--passthrough --record-writes`, nothing reached an ESC): a settings
write with the boot byte (settings byte 0) set to 0 and every other byte as read, 108 chunks of 256 bytes at 0x1000..0x7B00, a settings
write with the boot byte back at 1. No erase command (the bootloader erases a 1 KB page when a write starts on its first byte) and **no
read-back of the image** before the boot byte returns to 1. The image it sent was the official `AM32_SKYSTARS_KM55_V2_F421_2.22.hex`
byte for byte, with 0xFF where the file has no data.

## Scope

**Forward flashes only:** from the shipped firmware to newer AM32 releases for the same target. There is no plan to downgrade an ESC's firmware
(owner, 2026-11-10), so the design has no rollback path and does not need one:
- the tool does not compare versions or check the direction of a flash; an older image would be written like any other, and is out of scope;
- the buddy's settings code speaks one layout (`.claude/rules/esc-buddy.md`), so an ESC flashed back to a build with another layout would be one
  the buddy can no longer read or write;
- the bootloader is not updated either.

The dump of the shipped firmware is kept for another reason (decision 4), not as a way back.

## Decision

**Flashing is a separate action of the buddy, `--flash`, driven by the PC tool; the Pico accepts only a write it can check against the
image the owner chose. The configurator is not used for flashing.** `--dump-firmware` reads an ESC's application area as an image.

1. **The Pico holds the image and judges every write.** The PC uploads the parsed image to the Pico as a file and starts the session for
   one ESC. In a flash session the Pico accepts, on that ESC only and only if its bootloader is the F421's (0x1F06): a 256-byte chunk equal
   to the image at its 256-aligned address inside 0x1000..0x7BFF, and only after the boot byte was cleared; and the settings block with
   only the boot byte changed. Nothing below 0x1000 (the bootloader) and nothing at 0x7C00 and above except that block is ever written.
2. **The boot byte protects against a bad flash.** The tool clears it first, writes the chunks, reads back the whole area, and sets it to 1
   only when every chunk equals the image. The Pico enforces the last part itself: it marks a chunk "read back equal" when a read reply
   equals the image, a write to that chunk removes the mark, and the boot byte is accepted at 1 only when all 108 chunks are marked. A
   failed or interrupted flash therefore leaves the ESC parked in its bootloader and the same command can be run again - also when it
   stopped after the last 1 KB page was erased, which wipes the file name (it sits in the last chunk, at 0x7B00): the target check then
   accepts an unreadable name, but only when the boot byte is 0, which only this tool clears. Tested on the
   bench bootloader (version 1): with the boot byte at 1 the first init answer came 2.69 s after a 4-way reset (the firmware ran first),
   with the byte at 0 it came 0.21 s after, on the first try. The power-on path (the bootloader deciding by itself at reset) was not tested
   separately.
3. **The tool refuses what is not a firmware for this ESC:** a file with data outside the application area, a first word that is no stack
   pointer or a reset vector outside the area, an image whose file name (at 0x7BE0) differs from the name the ESC reports, a bootloader
   other than the F421's, and a settings page holding data beyond the settings block (a block write erases the whole page). All of it is
   checked before the first byte is sent.
4. **Backup and dump first.** The settings page is saved (`--backup`, never overwritten), as for `--write`, and the ESC's firmware can be
   dumped. Downgrading is out of scope (above), but a dump is needed anyway: the shipped firmware is a 2.16 build that is
   not the binary of any AM32 release (`specification/KM55A2_DEFAULTS.md`), so a dump was its only copy and the only image with which a flash
   could be rehearsed without changing anything. The owner discarded the dumps on 2026-11-10 (no downgrade is planned and the KPI baseline restarts
   on 2.22), so no copy of the shipped build is kept; `--dump-firmware` remains the way to take one. The image given to `--flash` is an AM32 `.hex` file or a `--dump-firmware` JSON.
5. **Asking again, never skipping.** A settings write (boot byte 0 or 1) that gets no answer is asked again, up to three times; a chunk write
   likewise, and so are the reads (chunks, the settings page, the file name). The report lists every acknowledgement it saw. Every chunk is written, including those that already
   equal the image.
6. **One ESC per run, the owner's go for each ESC.** After the flash the tool writes the settings block back as read; the firmware migrates
   it itself (below).

## What the firmware does after a flash

AM32 2.22 (`Src/main.c`): on its first boot it compares the stored version and layout with its own and, when they differ, writes the new
version into the settings buffer and saves all 192 bytes of the new layout; fields an older layout lacks get defaults in
`loadEEpromSettings`. The tool therefore needs no new-layout block of its own. The buddy's settings code follows the firmware on the bench
and keeps no older layout (`.claude/rules/esc-buddy.md`).

## Alternatives considered

- **The configurator through the passthrough.** Needs the same Pico-side check (the image must reach the Pico anyway) and gives up the
  read-back before the boot byte returns and the choice of what the settings page holds after the flash. The owner picked the tool first;
  the guard here would also serve a configurator front-end later.
- **Pass the PC tool's frames through unchecked.** A script bug could then write the bootloader area or a wrong image; the Pico checking
  each write against the file is what keeps ADR-006's rule that no byte from the PC is trusted.
- **Skip chunks that already equal the image.** Faster for a re-flash, but a same-image run would not exercise the write path, which is
  what a first rehearsal on a new ESC is for.
- **A read-only preview (`--plan-flash`, and `--plan` / `--plan-restore` for settings).** Built and used 2026-11-09, removed 2026-11-10
  (owner: low value). `--flash` checks everything it can before it sends a byte, and its report says why it refused.
- **Downgrade support and bootloader updates.** Out of scope, not built. `--flash` with an ESC's own dump has been run once, as a same-image
  rehearsal (2.16 onto 2.16) on one ESC; going back from a migrated layout 4 page to the shipped firmware has never been tried. Whether
  the ESC boards have SWD pads is unknown.

## Consequences

- AM32 2.17 to 2.22 change start-up, sine mode, demag and stuck-rotor handling and eRPM values. Every arming, start-quality and eRPM figure in
  the README was measured on 2.16 and has to be measured again. One early observation, not a measurement: after the flash the
  median eRPM at throttle 215 read about 11 % lower on the two ESCs with a motor (a few decoded samples per run). One discarded four-channel
  run on 2.22 did not arm: one ESC gave no reply for 18 s; the next run, after a power cycle, armed. One occurrence, not counted as evidence
  either way for the arming rate, which has to be measured again anyway.
- A flash takes a few minutes per ESC (108 chunks of about 0.2 s, then a read-back of the same size).
- A retried chunk that does not start a page programs over flash that is already written. Real flash might reject that; it has not been
  observed. The remedy would be the same as for any stopped flash: run it again (the boot byte is still 0).
- Fallback after a flash that leaves a bad application (never needed): the boot byte is still 0; start the buddy (lines on the pull-up, as
  ADR-006 saw the bootloader park), have the owner power-cycle the ESC, and run `--flash` again with the image that was being flashed (an ESC's
  dump is only a rehearsal image and a copy of the shipped build).

## Verification

- **PC unit tests** (`tests/pc/test_esc_buddy_flash.py`): the image parser; every Pico guard (chunk before the boot byte is cleared, wrong
  content, wrong place, misaligned or short, bootloader area, settings page, boot byte back before the read-back, a rewritten or differing
  chunk, another ESC, another bootloader); the tool's sequence against a fake F421 whose flash only clears bits and erases a page only
  when a write starts on its first byte (good flash, same image again, other target, missing acknowledgement, a chunk that reads back
  wrong, retries of the settings writes, backup before the first write).
- **Test population:** two identical Skystars KM55A2 boards (four ESC chips each, the same stock firmware and settings); every ESC of both
  boards survived the flashes, none was bricked. One model of one ESC: nothing here says how another ESC would behave.
- **Bench, 2026-11-09 and 2026-11-10, GPIO 6-9, owner's go for every write:**
  - The ESCs of both boards were dumped first: all eight images are identical.
  - ESC 1 of one board was flashed back with its own dump as a rehearsal: 108 chunks written and read back equal, boot byte restored, the
    settings block unchanged, the ESC still arms and spins.
  - All eight ESCs were then flashed with the official 2.22 image, one run each (one stopped at its last step, below): 0 read-back mismatches
    on every one, and a following comparison with the image showed 0 differing chunks.
  - The firmware migrated every settings page itself on its first boot, identically: layout byte 2 -> 4, minor version 16 -> 22, the 2.16
    name bytes (5-16) became new settings, timing advance 2 -> 26 (the same 15 degrees in the new unit), temperature limit 141 -> 255. Motor
    KV, poles and beeper volume kept their values.
  - One flash stopped at its last step: the write that sets the boot byte back to 1 got no acknowledgement. The page was intact with byte 0
    at 0 and the ESC parked, as designed; finishing it by hand worked at once. The cause is unknown (the session record was overwritten);
    the tool now retries and reports the acknowledgements. It has not recurred in the five flashes since.
  - After the flashes a discarded four-channel bidirectional DShot 600 run on each board armed and spun the motors with CRC-valid replies,
    except the one run that did not arm (Consequences).
- **Not verified on a real ESC:** the boot byte's effect on the power-on path; the fallback above; a flash that stops in the middle of the chunk
  writes (the re-run after one is covered by a PC test, including the case where the file name page was erased).
  Open items of the settings code (first real write of the layout 4 fields, restore after a real failure) are in ADR-006.
