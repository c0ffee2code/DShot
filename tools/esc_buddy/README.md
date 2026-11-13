# ESC buddy

Access to the bench ESCs' **AM32 configuration and firmware** through the **main Pico**, with no flight controller: Claude (through the
`esc-buddy` skill) and the owner (through the AM32 web configurator) can read every ESC's settings, change a named set of them, and flash a
new firmware image. The Pico speaks the AM32 bootloader protocol on each ESC's signal line (19200 baud, half duplex); to the configurator it
looks like a Betaflight flight controller in passthrough mode. Why it is built this way: ADR-006 (access) and ADR-007 (flashing).

## What is supported

| | |
|---|---|
| **ESC firmware** | **AM32 2.22, settings layout 4** (192-byte settings block). The mechanism is the layout version stored in byte 1 of the block: the decoder and the write guards accept layout 4 only and refuse a block of any other layout, which `--read` reports as an error with the raw block. No older layouts or firmware versions are supported or kept working: when the bench moves to a new firmware the buddy moves with it (`.claude/rules/esc-buddy.md`). |
| **MCU / bootloader** | AM32 bootloader on an F421 (signature `0x1F06`, settings page at `0x7C00`, application area `0x1000`-`0x7BFF`). Flashing refuses any other signature. |
| **Wiring** | 1 to 8 ESC signal lines on the main Pico, given as `--pins` (the order is ESC 1..n in the configurator and in every report). |

**Not supported:** other firmware families, other settings layouts, other MCUs, updating the bootloader, anything during a motor run (the buddy
stops DShot on all lines), changing what the throttle means (direction, 3D mode, input type: see "Properties"), and the configurator writing
through the passthrough (it is read-only; its saves are refused).

## What it can do

| Action | What | Needs the owner's go |
|---|---|---|
| `--read` | every ESC's firmware name and version, bootloader and all settings, decoded, as JSON | no |
| `--passthrough` | the browser configurator reads the ESCs through the Pico; its saves are refused (`--record-writes` answers them OK without sending and logs them) | no |
| `--write --backup PATH` | changes named settings on one ESC, reads the page back | **yes, each change** |
| `--restore BACKUP --backup PATH` | puts a saved settings block back on one ESC | **yes, each change** |
| `--dump-firmware` | one ESC's firmware as JSON on stdout; usable as the image of the two actions below | no |
| `--flash IMAGE --backup PATH` | flashes the image onto one ESC | **yes, each ESC** |
| `--debug` | the buddy's record of the last session | no |

```
python tools/esc_buddy/esc_buddy_run.py --read --pins 6 7 8 9 > esc.json
python tools/esc_buddy/esc_buddy_run.py --passthrough --pins 6 7 8 9                       # then use the configurator, Disconnect, close the tab
python tools/esc_buddy/esc_buddy_run.py --write --pins 6 7 8 9 --esc 2 --set "beeper volume=1" --backup tmp/esc2_before.json
python tools/esc_buddy/esc_buddy_run.py --dump-firmware --pins 6 7 8 9 --esc 2 > tmp/esc2_firmware.json
python tools/esc_buddy/esc_buddy_run.py --flash IMAGE.hex --pins 6 7 8 9 --esc 2 --backup tmp/esc2_page.json
```

Tools write no files except `--backup`: they print one JSON report on stdout and progress on stderr; the caller decides what to keep.
Exit status 1 when nothing could be read, or the write, restore or flash did not go through (`error` in the JSON says why).

## Properties

`--read` decodes the whole block into named values (`byte`, `raw`, `value`) and keeps the raw hex. `--set "NAME=VALUE"` changes the fields
below, one ESC per run. Every range is the one the AM32 2.22 firmware accepts (`loadEEpromSettings`; it ignores or replaces anything else),
cross-checked with the AM32 configurator's own field definitions. Flags are written as exactly 0 or 1 (`on`/`off`); the multi-state fields
take their name (any case) or number.

**Editable (28):**

| Byte | Field | Accepted values |
|---|---|---|
| 26 | `motor KV` | 20 to 10220 in steps of 40 (stored as (KV - 20) / 40) |
| 27 | `motor poles` | even, 2-36 |
| 30 | `beeper volume` | 0-11 (above 11 the firmware plays 5) |
| 19 | `sinusoidal startup` | on / off |
| 21 | `PWM type` | fixed / variable / by RPM |
| 24 | `PWM frequency` | 8-144 kHz |
| 23 | `timing advance steps` | 0-32 steps of 0.9375 deg (16 steps = 15 deg); stored plus 10 |
| 47 | `auto timing advance` | on / off |
| 25 | `startup power` | 50-150 % |
| 5 | `ramp rate` | 0.1-20 % duty cycle per ms, steps of 0.1 |
| 6 | `minimum duty cycle` | 0-25 %, steps of 0.5 |
| 14 | `demag compensation` | off / coast only / 1/32 power cut / 1/16 power cut / 1/8 power cut (0-4) |
| 40 | `sine mode range` | 5-25 % |
| 45 | `sine mode power` | 1-10 |
| 7 | `disable stick calibration` | on / off |
| 28 | `brake on stop` | off / brake on stop / active brake |
| 41 | `brake strength` (drag brake) | 1-10 |
| 12 | `active brake power` | 0-5 % |
| 22 | `stuck rotor protection` | on / off |
| 29 | `stall protection` | on / off |
| 36 | `low voltage cut off` | off / cell based / absolute |
| 37 | `low voltage threshold` (cell based) | 2.50-3.50 V per cell, steps of 0.01 |
| 8 | `absolute voltage cutoff` (absolute) | 0.5-50 V, steps of 0.5 |
| 43 | `temperature limit` | 70-140 C, or `disabled` (stores 255, as the firmware does) |
| 44 | `current limit` | 2-200 A, even, or `disabled` (stores 0: the firmware then applies no limit) |
| 9, 10, 11 | `current limit P gain`, `I gain`, `D gain` | 0-255 each |

**Read-only (decoded, never written):** boot byte (0), layout version (1), bootloader version (2), firmware version (3-4, stamped by the
firmware itself at boot); `brake on zero throttle` (13: the configurator does not show it and the source gives no list of its values, so it is
shown raw); and the fields that change what the throttle means or are outside this bench: `direction reversed` (17), `3D mode` (18),
`complementary PWM` (20), `30 ms interval telemetry` (31), the four servo fields (32-35), `car type reverse braking` (38), `use hall sensors`
(39), `running brake level` (42: it changes the dead time and start-up duty), `input type` (46).

**Not decoded:** the reserved bytes (15-16), the startup tune (48-175) and the CAN settings (176-191). They are carried through every write
exactly as read; bytes 0-4 and 176-191 are additionally protected by the Pico (below).

## Permissions and safety rules

With an ESC attached every action puts signals on its line (no throttle). The `esc-buddy` skill repeats these rules for Claude:

- **No permission needed:** every action without a "yes" above (`--read`, `--passthrough`, `--debug` and `--dump-firmware`).
  They cannot write: the Pico refuses writes, erases and settings writes except in a session started for `--write`, `--restore` or `--flash`.
- **The owner's explicit go for each change:** `--write`, `--restore` and `--flash`. Show what changes first (which ESC, which field, old -> new, from a fresh `--read`) and give
  `--backup tmp/<file>` (the git-ignored `tmp/` folder). Claude Code's allow-list matches the start of the command, so put the action first.
- **Never during a motor run** (it stops DShot on all lines). The wingman must not run the ESC mock at the same time (it shares the lines).

## How it works

- **The Pico judges every write.** The Pico builds every bootloader frame itself and nothing from the PC is forwarded raw. Only these
  configurator commands reach an ESC: alive check, interface name and version, set mode, exit, init flash access, read, reset. Write, erase,
  EEPROM write and verify are answered "invalid command" without touching the line, and logged as `refused`. A session started for one ESC
  and one action may additionally write, once and only under these guards (`Passthrough.write_refusal` / `flash_refusal`):
  - **Settings write:** the session's one ESC, the settings address, exactly the block size, bytes 0-4 and 176-191 equal to what the Pico
    last read from that ESC.
  - **Restore:** only the block the session was started with.
  - **Flash:** only 256-byte chunks equal to the image it was started with, inside `0x1000`-`0x7BFF`, after the boot byte was cleared; the
    settings block with only the boot byte changed; and the boot byte back to 1 only after the Pico has itself seen every chunk read back
    equal to the image.
- **Getting an ESC into its bootloader.** A running AM32 ESC has no command for it: with no DShot on its line it reboots itself after 2 s
  disarmed, and the bootloader stays when the line reads high. The first read of an ESC can fail until it has rebooted; the tools retry
  (reads three times, a read of a whole area chunk by chunk, init up to ten times).
- **Line handling copies Betaflight's:** every line idles as an input on the Pico's pull-up and is never driven while idle; a byte is driven
  from one extra idle bit before its start bit to its last data bit. The PIO program releases the line the moment the last stop bit ends,
  because the ESC replies 172-176 us later. Never put `sm.exec(str)` on a timing path (MicroPython assembles the string per call, 4.7 ms).
- **Getting back to normal running.** On exit every line is driven low (the state `MotorGroup.disarm()` leaves); the bootloader starts the
  firmware after 20 ms of low, and the ESC resumes its no-signal beeps.
- **Startup.** The Pico's USB port belongs to the buddy while it runs (no REPL, Ctrl-C off: a request byte `0x03` would otherwise interrupt
  the program while it starts, so the start command switches it off first and the PC tools send a harmless request until the buddy answers).
  It returns to the REPL on Disconnect or `exit`, if the configurator does not connect within 30 min, after 5 min of silence once it has, or
  when USB is unplugged. If it stopped before you clicked Connect, the configurator reports "invalid result": start it again.
- **The record** (`esc_buddy_log.txt` on the Pico, shown by `--debug`): one line per event - `init` (the bootloader's device info), `read`
  (address and bytes), `refused` (the command and its parameters; also a write the guards turned down), `write` (the settings block sent),
  `chunk` (address and checksum of a flashed chunk) and `end`. Each session overwrites it.

## Writing settings

There is no dry run: `--read` shows what an ESC holds now, and `--write` checks everything it can before it sends a byte - the names and values, the
ESC's answer, and its whole 1 KB settings page, refusing when anything beyond the 192-byte block is not `0xFF` (a block write erases the whole
page) or when nothing would change - and its report lists each change with old and new. `--write` needs `--backup <path>` (never overwritten): the page is saved there first, then
the Pico writes (set address, set buffer, payload with CRC, program), the tool reads the page back and compares, and the ESC restarts its
firmware on exit. `--restore` accepts a backup made by `--write` or `--flash` of the same ESC; a backup of another layout is refused.

## Flashing firmware

There is no dry run here either: `--flash` checks the image, the ESC's target name and its settings page before it sends a byte. `--flash` saves the settings page (`--backup`), clears the
boot byte (settings byte 0) so the ESC stays in its bootloader, writes all 108 chunks of 256 bytes, reads the whole area back and compares, and
only when every chunk equals the image sets the boot byte back to 1; a settings write that gets no answer is asked again, and every
acknowledgement is in the report. A failed flash leaves the boot byte at 0: run the same command again (also after one that stopped once the file name page was erased: an unreadable name is accepted when the boot byte is 0). The image is an AM32 `.hex` file or a
`--dump-firmware` JSON; the tool refuses one whose file name (at `0x7BE0`) differs from the one the ESC reports, one with data outside the
application area or that is not a firmware image, another MCU, and a page with data beyond the settings block. After the flash the ESC boots
the new firmware, which migrates its own settings page on first boot, so run `--read` afterwards. A flash takes a few minutes per ESC.

## Files

| File | Runs on | What |
|------|---------|------|
| `esc_buddy_protocol.py` | Pico and PC | MSP and 4-way answers, bootloader frames, the allow-list, the write and flash guards (pure Python; `tests/pc/test_esc_buddy_protocol.py`, `test_esc_buddy_write.py`, `test_esc_buddy_flash.py`) |
| `esc_buddy_board.py` | main Pico | USB serial loop and the signal-line wire: two PIO programs, 19200 baud 8N1, half duplex |
| `esc_buddy_run.py` | PC | finds the main Pico by ID, uploads and starts the buddy, runs the action |
| `esc_buddy_settings.py` | PC | decodes a layout 4 settings block into named values and encodes the editable fields (`tests/pc/test_esc_buddy_settings.py`, against a block read from a bench ESC) |
| `esc_buddy_image.py` | PC | turns an AM32 `.hex` file or a dump into the 27,648 bytes of the application area (0xFF where the file has nothing, as the configurator does) and checks it is a firmware image |

Modelled on [BlHeli-Passthrough (RP2040)](https://github.com/BrushlessPower/BlHeli-Passthrough/tree/main/RP2040); checked against
[AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader), [AM32](https://github.com/am32-firmware/AM32) and
[am32-configurator](https://github.com/am32-firmware/am32-configurator). The bench ESC (vendor identity, settings as shipped):
`specification/KM55A2_DEFAULTS.md`; how it is configured now: the top-level README, "ESC settings as configured".
