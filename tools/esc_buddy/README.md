# ESC buddy

Gives the **AM32 configurator** (the web app) - and Claude, through the `esc-buddy` skill - access to the
ESCs' configuration through the **main Pico**, with no flight controller. The configurator takes the Pico for a
Betaflight flight controller in passthrough mode; the Pico relays its requests to each ESC's bootloader on that ESC's
signal line.

**What it can do**

| Action | What | Needs the owner's go |
|---|---|---|
| `--read` | every ESC's firmware name and version, bootloader and all stored settings, as JSON | no |
| `--passthrough` | the browser configurator reads the ESCs through the Pico (its saves are refused; `--record-writes` logs them) | no |
| `--plan` | shows what `--write` would change on one ESC; reads, writes nothing | no |
| `--write --backup PATH` | changes named settings (19 of them: motor, start-up and protection, "Writing settings") on one ESC, reads the page back | **yes, for each change** |
| `--plan-restore BACKUP` | shows what `--restore` would change on one ESC; reads, writes nothing | no |
| `--restore BACKUP --backup PATH` | puts a saved settings block back on one ESC | **yes, for each change** |
| `--debug` | the buddy's record of the last session | no |

It cannot flash firmware or the bootloader (see "To do"). Of the writable settings only motor KV, motor poles and beeper volume
have been written to real ESCs; the other 16 are encoded and tested against the firmware source and a fake bootloader only.

What the bench ESC is and how it came from the vendor: `specification/KM55A2_DEFAULTS.md`; how it is configured now: the top-level README, "ESC settings as configured".

**Reading and the configurator are read-only.** Only these configurator commands reach an ESC: alive check, interface name/version,
set mode, exit, init flash access, read, reset. Write, erase, EEPROM write and verify are answered "invalid command" without
touching the line, and logged as `refused`. The Pico builds every bootloader message itself; nothing from the PC is forwarded raw.
The one exception is a session started for `--write` or `--restore`: it may write the settings block of the one ESC
it was started for, once, under the guards in "Writing settings".
Why it matters: the vendor's settings exist only in the ESC chips, and the configurator itself writes settings on its
own in some cases (`four_way.ts`, bootloader version reads `0xFF`).

## Files

| File | Runs on | What |
|------|---------|------|
| `esc_buddy_protocol.py` | Pico and PC | MSP and 4-way answers, bootloader frames, the allow-list, the write guards (pure Python; tested by `tests/pc/test_esc_buddy_protocol.py`) |
| `esc_buddy_board.py` | main Pico | USB serial loop and the signal-line wire: two PIO programs, 19200 baud 8N1, half duplex |
| `esc_buddy_run.py` | PC | finds the main Pico by ID, uploads and starts the buddy; one action required: `--read` (settings as JSON), `--passthrough` (browser configurator; `--record-writes` logs its save attempt), `--plan` / `--write` and `--plan-restore` / `--restore` (show, then make, a change of settings on one ESC), `--debug` (the buddy's record, plain text) |
| `esc_buddy_settings.py` | PC | decodes a settings block (EEPROM layout 2, firmware 2.16) into named values, and encodes the writable fields (motor, start-up and protection settings, listed under "Writing settings"); tested by `tests/pc/test_esc_buddy_settings.py` against the configurator's display and `tests/pc/test_esc_buddy_write.py` |

## How to run

1. `python tools/esc_buddy/esc_buddy_run.py --passthrough --pins 6 7 8 9` - the GPIO numbers of the ESC signal lines, in order =
   ESC 1..n in the configurator and in the report. The only input the tool needs; the caller knows the wiring.
2. In the AM32 configurator: select the Pico's port, Connect, read the ESCs, Disconnect.
3. Optional: `python tools/esc_buddy/esc_buddy_run.py --debug` - prints the buddy's record of the session (plain text,
   stdout; nothing saved).

Without the configurator: `python tools/esc_buddy/esc_buddy_run.py --read --pins 6 7 8 9` starts
the buddy, plays the configurator's read for every ESC (init flash access, firmware name, the 184-byte settings
block), exits, and prints one JSON report on stdout; progress goes to stderr. It writes no files: the caller decides
what to keep (`> esc.json`, a pipe, a session's metadata), Unix-style. Exit status 1 when no ESC could be read. It uses
only commands on the read-only allow-list. The report gives, per ESC: GPIO, bootloader signature and input pin,
firmware file name, firmware version and name, every decoded setting (`byte`, `raw`, `value`) and the whole block in
hex; `identical` says whether all ESCs hold the same block. An ESC that did not answer has an `error` instead.

While it runs, the Pico's USB port belongs to the configurator (no REPL, Ctrl-C off). It returns to the REPL on
Disconnect, if the configurator does not connect within 30 min, after 5 min of silence once it has, or when USB is
unplugged. If it stopped before you clicked Connect, the configurator reports "invalid result" (it reads the REPL's
prompt instead of a reply): start it again.

To change settings, see "Writing settings" below.

## Permissions and safety rules

With an ESC attached every action puts signals on its line (no throttle). The rules, which the `esc-buddy` skill repeats for Claude:

- **No permission needed (owner, 2026-11-04):** `--read`, `--passthrough`, `--debug`, `--plan` and `--plan-restore`. They cannot write: the
  Pico refuses writes, erases and settings writes except in a session started for `--write` or `--restore`.
- **The owner's explicit go for each change:** `--write` and `--restore`. Show the plan first (what changes, on which ESC) and give
  `--backup tmp/<file>` (the git-ignored `tmp/` folder).
- **Claude Code's allow-list** (`.claude/settings.local.json`, local to the machine) runs the first group without a prompt and asks before
  `--write` and `--restore`. It matches the start of the command, so put the action first (`esc_buddy_run.py --plan --pins ...`).
- **Never during a motor run** (it stops DShot on all four lines); the report names the pins used. The wingman must not run the ESC mock
  at the same time (it shares the lines).

## The record (`esc_buddy_log.txt` on the Pico, shown by `--debug`)

One line per event: `init` (the bootloader's 9-byte device info for that channel - pin code, flash size, protocol
version), `read` (address and the bytes read, hex), `refused` (the command byte the configurator tried, then its
parameters; also a write the guards turned down), `write` (the settings block a `--write` / `--restore` session sent), and `end`
(exit or idle timeout). `ch` is the ESC index from 0, `pin` its GPIO. This is the saved copy of what each ESC holds,
independent of what the configurator shows.

## How it works, and its limits

- **Getting an ESC into its bootloader.** A running AM32 ESC has no command for it: with no DShot on its line it reboots
  itself after 2 s disarmed (AM32 `Src/main.c`), and the bootloader stays when the line reads high (AM32-bootloader
  `checkForSignal()`). The first read of an ESC can fail until it has rebooted; the configurator and `--read` retry.
- **Line handling copies Betaflight's** (`serial_4way.c`, `serial_4way_avrootloader.c`): every line idles as an input
  on the Pico's pull-up and is never driven while idle; a byte is driven from one extra idle bit before its start bit to
  its last data bit, and the pull-up holds the stop bit. What made it work is that the **PIO program releases the line
  the moment the last stop bit ends**: the ESC replies 172-176 us later. The first buddy released from Python with
  `sm.exec("set(pindirs, 0)")`, which MicroPython assembles at call time (about 4.7 ms on the Pico 2), so it was still
  driving the line high through the ESC's whole reply. Isolated 2026-11-05: the idle bit, the stop-bit handling and the
  idle level make no difference to this bootloader. Never put `sm.exec(str)` on a timing path.
- **Getting it back to normal running.** On exit every line is driven low, the state `MotorGroup.disarm()` leaves; the
  bootloader starts the firmware after 20 ms of low (`serialreadChar()`). Seen on the bench: the ESC resumes its
  no-signal beeps shortly after Disconnect.
- Modelled on [BlHeli-Passthrough (RP2040)](https://github.com/BrushlessPower/BlHeli-Passthrough/tree/main/RP2040);
  checked against [AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader) and
  [am32-configurator](https://github.com/am32-firmware/am32-configurator).

## Recording the configurator's writes (`--passthrough --record-writes`)

Still read-only on the ESC. The configurator's save commands (4-way erase all `0x38`, page erase `0x39`, write `0x3B`,
EEPROM write `0x3E`, verify `0x40`) are answered OK so it carries on, never sent to the line, and logged as `refused`
with the whole frame: the command byte, then the parameters (the data it wanted to write), at the address given. Purpose:
see exactly what the configurator sends on Save and check it against the AM32 bootloader source before the buddy writes
anything itself.
**What the configurator sent on Save (2026-11-05, two recorded sessions, GPIO 6-9, owner changed motor KV, and in the second also
motor poles):** per ESC exactly one 4-way write (`0x3B`) at `0x7C00` carrying 184 bytes, then a fresh init and a read-back of
`0x7C00`. No page erase, erase all or verify command. Against the 184 bytes read just before: the changed fields (byte 26 KV
`0x37` -> `0xB4`; second session also byte 27 poles `14` -> `12`); the 12 firmware-name bytes (5-16) sent as `0xFF` instead of
the name read; the last 8 bytes (176-183) sent as `0xFD` where the block held `0xFF`. All four ESCs got the identical frame
(the owner's changes applied to all four). So the configurator's block is not a safe thing to forward: a write path built here
must keep bytes 0-16 and the tail exactly as read and change only named fields.
On the wire (the wingman's spy, 19200 baud decode of all four lines, valid recording): in the Save step each line carried only
the init request, set address `0x7C00` and the 184-byte read-back; byte counts per line match the known frames exactly (572 per
line over the whole session), no set-buffer, program or erase frame. Nothing reached an ESC.
Source cross-check (AM32-bootloader master, version 19; the bench bootloader is version 1 and no public commit matches it):
set address, set buffer (`0xFE`), payload with CRC, program (`0x01`); a write at `0x7C00` starts a 1 KB page, so the bootloader
erases the whole page before programming (`save_flash_nolib`), and only addresses at or above the application are writable.
The bench bootloader (version 1) follows the same sequence (first write, below).

## Writing settings (`--plan`, `--write`)

```
python tools/esc_buddy/esc_buddy_run.py --plan --pins 6 7 8 9 --esc 4 --set "motor KV=2220" "motor poles=14"                 # plan: reads, writes nothing
python tools/esc_buddy/esc_buddy_run.py --write --pins 6 7 8 9 --esc 4 --set ... --backup tmp/esc4_before.json               # write
```

(Until 2026-11-08 the plan was `--write` and the real write `--write --apply`; the records below use the spelling of their day.)

One ESC per run. These fields can be named (names as `--read` shows them; every range is the one the AM32 2.16 firmware accepts, from `loadEEpromSettings`, because it ignores or replaces anything else):

| Group | Field: accepted values |
|---|---|
| Motor | `motor KV`: steps of 40 from 20; `motor poles`: even, 2-32 (above 32 the firmware divides by zero); `beeper volume`: 0-11 (above 11 it plays 5) |
| Start-up and running | `sinusoidal startup`, `variable PWM`, `auto timing advance`, `brake on stop`: on/off; `timing advance`: 0-3 (levels of 7.5 deg; anything else the firmware replaces by 2); `PWM frequency`: 8-48 kHz; `startup power`: 50-150 %; `sine mode range`: 5-25 %; `sine mode power`: 1-10; `brake strength` (drag brake): 1-10 |
| Protections that can stop a running motor | `stuck rotor protection`, `stall protection`, `low voltage cut off`: on/off; `low voltage threshold`: 2.50-3.50 V per cell; `temperature limit`: 70-140 C or `disabled`; `current limit`: 2-198 A, even, or `disabled` (stores 0, which the firmware ignores, so there is no limit) |

Flags are written as exactly 0 or 1 (the firmware reads any other byte as off). Not writable: direction, 3D mode, complementary PWM, hall sensors, input type, servo fields, car reverse braking (they change what the throttle means or are outside this bench) and `running brake level` (it changes the dead time and start-up duty). A wrong protection value shows up on the bench as a failed run, so read the plan before asking for the go. The plan
reads the ESC's whole 1 KB settings page, shows old and new, and stops if anything beyond the 184-byte block is not
`0xFF`. `--write` needs `--backup <path>` (a plan takes none): the page is saved there first (never overwritten), then the Pico writes, the
tool reads back and compares, and the ESC restarts its firmware on exit. Output: one JSON report; exit status 1 if the
plan or the write failed. The Pico enforces the limits itself (`Passthrough.write_refusal`): the session's one ESC, one
write, the settings address, exactly 184 bytes, bytes 0-16 and 176-183 equal to what it last read from that ESC. The
write sequence on the line is set address, set buffer (no ack), payload with CRC, program flash; its tests use a fake
bootloader built from the public source (version 19), which the bench ESC's version 1 may differ from.

Found 2026-11-05 while testing it: a request whose bytes include `0x03` (Ctrl-C) - ESC 4's init request is one - that
reaches the Pico while it is still starting (importing the board file, setting up) interrupts the program, and every
later request goes unanswered. Pico only, GPIO 10-13, nothing attached, ESC 4's init as the very first request, 6 trials
each: the old board code 0 answered, Ctrl-C switched off first thing in `main` 5, Ctrl-C switched off in the start command
before the import 6. The start command now does that, and the PC tools send a harmless request (interface name) until the
buddy answers before the first real one.
Betaflight's write (`serial_4way_avrootloader.c`, `BL_WriteA`) matches the sequence used: set address with ack, set buffer
`FE 00 <hi> <len>` with no ack, payload with ack, then `01 01`; it waits 500 ms for the program ack (ours 2 s).

`--restore <backup> --esc N --backup <new file>` puts a saved block back (`--plan-restore <backup> --esc N` shows it first; the new file saves the page as it is then): the
Pico accepts only a write equal to that block, so it also works on a page a failed write left blank.

**First write on an ESC (2026-11-05 22:21, ESC 2 on GPIO 7; owner's go for this change):** motor KV 2220 -> 7220 (byte 55 -> 180)
and motor poles 14 -> 12, `--write --apply --backup tmp/esc2_before_kv7220_poles12.json`. Result: write acknowledged, the whole
1 KB page read back equal to the new block plus blank (`ok` true). Wire (the wingman's spy, valid recording, 19200 baud decode):
set address `0x7C00` acknowledged, set buffer `FE 00 00 B8` with no reply, the 184-byte block with its CRC (equal to the new block)
acknowledged `0x30`, program `01 01` acknowledged `0x30` 9.3 ms later (page erase and programming). The bench bootloader
(version 1) therefore follows the public sequence. Lines 6, 8 and 9 carried no bytes. Eight bytes of unknown origin
(`09 31 21 2a 62 4a a2 f5`, twice, 0.16 s and 0.37 s into the window, before the buddy's first frame) were on line 7 before the
buddy sent anything. The configurator, connected afterwards through passthrough, read ESC 2 as KV 7220 and 12 poles (owner); the buddy's log of that session shows ESCs 1, 3 and 4 byte-for-byte unchanged.

**Restore on a real ESC (2026-11-05, ESC 2, owner's go):** `--restore tmp/esc2_before_kv7220_poles12.json --esc 2 --apply --backup tmp/esc2_before_restore.json`: KV 7220 -> 2220 and poles 12 -> 14, write acknowledged, the page read back equal to the original vendor block, same sequence on the wire.

**All four ESCs set (2026-11-05, owner's go, one ESC per run, no spy):** motor KV 2220 -> 7180 (the nearest storable step below 7200:
values go in steps of 40 from 20) and beeper volume 5 -> 1, ESC 1 to 4, each acknowledged and read back equal; a `--read` afterwards
found all four blocks identical, differing from the vendor block only in bytes 26 and 30. Poles stay 14. Backups of the vendor
pages: `tmp/esc<N>_before_kv7180_beep1.json` (local, git-ignored). `specification/KM55A2_DEFAULTS.md` holds the vendor values, the top-level README ("ESC settings as configured") the current ones.

**Motor poles 14 -> 12, the four ESCs on GPIO 6-9 (2026-11-08, owner's go, one ESC per run):** `--write --backup tmp/esc<N>_before_poles12.json`, each acknowledged and read back equal; a `--read` afterwards found four identical blocks, KV 7180 and poles 12. Same day, `--passthrough --record-writes`: the configurator's Save sent one 184-byte write at `0x7C00` to each ESC (identical frames) with startup power 100 -> 95 and poles 14 -> 12 against the block read, the 12 name bytes as `0xFF` and the last 8 bytes as `0xFD`; the Pico's guard would refuse that frame, nothing reached an ESC.

**The primary board (2026-11-05, owner's go "in same fashion, all 4"):** read first: same model as the spare (firmware 2.16
`SKYSTARS_KM55_V2_F421`, bootloader `0x1F06` on PB4), all four blocks byte-for-byte equal to the spare's vendor block. Then the same
two changes, ESC 1 to 4, one run each, all acknowledged and read back equal; a `--read` afterwards found four identical blocks,
differing from the vendor block only in bytes 26 (KV 7180) and 30 (beeper volume 1). Backups: `tmp/primary_esc<N>_before_kv7180_beep1.json`.

## To do

- **ESC configuration as session metadata.** A session (`scripts/run_session.py`) could pipe `--read` into its own
  record before the run, so every result carries the configuration it ran with, and the effect of a later settings
  change can be judged by comparing sessions. Open: whether the ~30 s read and the bootloader round trip before each
  session are acceptable, or once per batch is enough.
- **First real write of each new field.** The 16 fields added after motor KV, poles and beeper volume are encoded and tested
  against the firmware source and a fake bootloader only; none has been written to a real ESC yet (each needs the owner's go).
- **Restore after a failed write on a real ESC.** `--restore` is proven on a healthy ESC and, in tests, on a blank page; not yet on
  an ESC that a real failed write left blank.
- **Firmware or bootloader update: a separate session (owner, 2026-11-07).** Not built: passthrough refuses every write and erase
  (the exception is the one settings write above), so the configurator cannot flash through the buddy yet. The evidence baseline
  is AM32 2.16, and the ESCs' only copy of the vendor firmware is in the chips (dump it first). Facts gathered for that session:
  the F421 holds bootloader 0x0000-0x0FFF, firmware 0x1000-0x7BFF, settings page 0x7C00-0x7FFF; the official 2.16 and 2.21 images
  for `SKYSTARS_KM55_V2_F421` (release assets) cover 0x1000 to about 0x7247 plus the 32-byte name at 0x7BE0 and nothing at
  0x7C00 or above; 2.21 uses settings layout 4 (192 bytes), which this tool's decoder, `SETTINGS_SIZE` and protected-byte guard do
  not handle. The public bootloader (master, not the bench's version 1) decides at reset from the line level whether to stay, and
  jumps to the app only if settings byte 0x7C00 is 0x01 and the app's first two words are in range, so a broken app should leave
  the bootloader reachable; unproven on this ESC. A write that does not start on a 1 KB page start does not erase that page.
  Whether the ESC boards have SWD pads is unknown.

## Verified so far

### Pico only, GPIO 10-13, nothing attached (2026-11-03)

- The configurator's connect sequence played from a PC script: MSP answers (Betaflight, battery state, motor count 4,
  passthrough 4), 4-way version, init and read on ESC 1-4 twice fail cleanly with no ESC (general error), ESC 5 is an
  invalid channel, two write attempts refused and logged, exit returns the board to the REPL, the record could be read back (now `--debug`).
### On the bench, Skystars KM55A2 on GPIO 6-9

- Real AM32 configurator in the browser, GPIO 6-9, ESC attached but not powered (2026-11-03): accepted the Pico as a
  Betaflight FC (API 1.42.0, battery 0 V, motor count 4, passthrough); init flash on ESC 1-4, two tries each, all
  empty (no ESC power, as expected); Disconnect ended the session (`end exit`). A first attempt failed with "invalid
  result" because the buddy had already timed out before Connect (fixed: it now waits 30 min for the first
  connect).
- The first line handling (idle lines driven high, bytes back to back) got no answer from a powered ESC in four sessions; what was ruled out
  is in ADR-006, "Approaches tried", steps 1 to 3.
- **Betaflight-style line handling (2026-11-04): it answers.** 225 of 225 init requests answered with
  `34 37 31 14 1F 06 06 01 30` (AM32 bootloader, input pin PB4, signature 0x1F06, protocol version 1, OK) and 75 of 75
  keep-alives with `C1`. The buddy was switched to this line handling (Pico-only checks on GPIO 10-13:
  18 of 18 switch loopbacks, the output driver off whenever idle, rehearsal echoes intact).
- With it, the real configurator read all four ESCs (owner): bootloader v1 on PB4, MCU F421, EEPROM layout v2, firmware
  `SKYSTARS_KM55_V2_F421` 2.16. The configurator's downloads (owner) and the buddy's record are byte-for-byte
  identical, all four ESCs hold the same settings, and `--read` read the same bytes again on its own (20:34:07). The
  settings and what they mean: `specification/KM55A2_DEFAULTS.md`.
- **Isolation (2026-11-05, GPIO 6, wingman's recording):** Betaflight framing with one difference changed back (no idle
  bit / driven stop bit / driven-high idle) 80 of 80 answered; the first framing 0 of 20. The first framing's own bytes
  released by the PIO program at the end of the last stop bit 80 of 80 (pull-up or driven-high idle); the first framing
  0 of 40. The first buddy's Python release measured 4.7-5.2 ms after the last stop bit (`sm.exec` with a string: 4.6-5.1
  ms). Cause and record: ADR-006, "Approaches tried", step 5.
