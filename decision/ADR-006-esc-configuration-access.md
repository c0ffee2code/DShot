# ADR-006: ESC Configuration Access Through the Main Pico (the ESC buddy)

**Status:** Accepted
**Date:** 2026-11-05 (work and bench sessions 2026-11-03 to 2026-11-04; writing added 2026-11-05)
**Builds on:** [ADR-002](ADR-002-bidirectional-dshot.md) (the ESC signal lines are bidirectional and the Pico can drive and
read them)
**Later:** [ADR-007](ADR-007-esc-firmware-flash.md) (flashing; the ESCs now run AM32 2.22 and the buddy speaks settings layout 4)

## Context

Every result on this bench depends on how the ESCs are configured - motor KV and pole count, startup power, timing
advance, PWM, protections - and nobody had ever looked. The ESC is a Skystars KM55A2 4-in-1 (four separate MCUs) running
AM32, used as it came from the vendor. The vendor publishes no defaults, AM32's source has no per-target defaults (the
firmware reads whatever is stored in its own flash), so the only copy of the configuration was in the four ESC chips.

AM32 configuration is normally read with the AM32 web configurator, which talks to a flight controller over USB
(MSP), asks it to enter passthrough, and then sends BLHeli "4-way interface" commands that the flight controller turns
into AM32 bootloader commands on each ESC's signal wire: 19200 baud, 8N1, half duplex on the one wire. The bench has no
flight controller and no USB-serial adapter - only two Raspberry Pi Pico 2 boards (the main Pico, wired to the four
ESC signal lines on GPIO 6-9, and the wingman, which records those lines).

A running AM32 ESC has no command that sends it to its bootloader. It gets there on its own: with no valid DShot frames
it resets itself after 2 s while disarmed, and the bootloader then stays (instead of starting the firmware) if the
signal line reads high. On a line held low for more than 20 ms the bootloader starts the firmware.

Constraints: the vendor's values must not be put at risk (a write overwrites the only copy), the tool must not be able
to spin a motor, and the project runs MicroPython on the Pico.

## Scope

**Supported:** the bench ESCs - Skystars KM55A2, F421 MCU, AM32 bootloader version 1 - running **AM32 2.22, settings layout 4**. The
mechanism is the layout version in byte 1 of the settings block: the buddy decodes and writes layout 4 only and refuses a block of any other
layout. **Older firmware versions and layouts are not supported and not kept working** (owner, 2026-11-09 and 2026-11-10; CLAUDE.md, "ESC
buddy"): when the bench moves to a new firmware the buddy moves with it. Other firmware families and other MCUs are out of scope. What the
tool supports, down to the property list: `tools/esc_buddy/README.md`.

## Decision

**The main Pico acts as the flight controller: it answers the configurator's MSP and 4-way commands itself and speaks the AM32 bootloader
protocol on each ESC's signal line, with Betaflight's line handling. It is read-only by construction, apart from guarded writes that
exist only in a session started for one ESC and one action (settings write, restore, flash). The same path lets Claude read and change the
configuration without the configurator.**

1. **One MicroPython program on the main Pico** (`tools/esc_buddy/`): the USB serial port carries MSP and 4-way frames
   (Ctrl-C handling off while it runs), two PIO state machines carry bytes on the selected ESC line at 19200 baud
   (16 PIO cycles per bit, divider computed for the board's own clock). The protocol logic is pure Python and unit
   tested on the PC; the board file is only the wire.
2. **Read-only by an allow-list, not a deny-list.** Only these 4-way commands reach an ESC: alive check, protocol and
   interface version/name, set mode, exit, init flash access, read, reset. Everything else - write, page erase, erase
   all, EEPROM write, verify - is answered "invalid command" without touching the line, and logged. The Pico builds
   every bootloader frame itself; no byte from the PC is forwarded raw. This matters beyond user error: the
   configurator writes settings on its own in three cases (an unset bootloader version byte, settings that read all
   0xFF or all 0x00, one firmware version with low timing advance). The guarded exceptions are on the Pico, so a mistake in the PC script
   cannot get past them: the session's one ESC, the settings address, exactly the block size, bytes 0-4 and 176-191 equal to what the Pico
   last read from that ESC (a settings write); the one block the session was started with (a restore); the image checks of ADR-007 (a flash).
3. **Betaflight's line handling** (Betaflight `serial_4way.c`, `serial_4way_avrootloader.c`): every ESC line idles as
   an input on the Pico's pull-up and is never driven while idle; a byte is driven from one extra idle-high bit before
   its start bit to its last data bit, and the pull-up holds the stop bit. On exit every line is driven low, the state
   the motor driver's `disarm()` leaves, which makes each bootloader start the firmware.
4. **A command-line tool shaped like a Unix filter.** `esc_buddy_run.py` takes exactly one action flag and one input, `--pins`
   (the actions, and which need the owner's go, are in the tool README). Reports go to stdout as JSON and progress to stderr; the tool
   writes no files except the `--backup` of a write; the caller decides what to keep (for example a session storing the report as its
   metadata). It does not read a scenario file: the pins are the only thing it needs to know.
5. **A project skill** (`.claude/skills/esc-buddy/`: a router plus `configuration.md` and `passthrough.md`) so any Claude session can poll
   the configuration, compare it with the recorded vendor defaults (`specification/KM55A2_DEFAULTS.md`) and the current configuration
   (README, "ESC settings as configured") and hand the owner the configurator link. Reads run without asking (owner 2026-11-04); every
   write needs the owner's go, each time (owner 2026-11-05); never during a motor run.
6. **Backup before any settings write** (`--backup`, never overwritten) and a read-back after it, because a write replaces the only copy
   of the configuration. The bootloader erases the whole 1 KB page when a write starts at its first byte, so the tool refuses unless the page
   beyond the settings block is blank. There is no preview action: the write checks everything it can before it sends a byte.

## Approaches tried

In order, with what each showed. All requests and replies below were checked on the wire by the wingman's edge
recorder (its own clock) as well as by the Pico.

1. **Port the RP2040 Arduino passthrough** ([BrushlessPower/BlHeli-Passthrough](https://github.com/BrushlessPower/BlHeli-Passthrough))
   to MicroPython, but with idle lines *driven* high (the AM32 bootloader source requires a high line to stay in the
   bootloader and exits on a low one, and a driven line cannot lose to the ESC's own pull-down at boot) and bytes sent
   back to back with one stop bit. Pico-only checks on unused pins all passed (loopback, line levels, the
   configurator's connect sequence played from a script). With the real configurator: the flight-controller half
   worked at once (Betaflight identity, 4 motors, passthrough). A first failure was the Pico's own 5-minute idle
   timeout expiring before Connect (now 30 minutes to the first connect).
2. **With the ESC powered, no ESC ever answered the init request**, on all four lines, in four sessions. Ruled out one
   by one: our bytes were exactly as built on the wire (52.0 us bits, no framing errors, the Pico's echo intact); the
   Pico's output driver read off after `send()` returned (RP2350 IO status register - which, as step 5 shows, was
   checked too late to see the moment that mattered); the wingman was not driving the lines; both accepted request forms were tried (17-byte, and the 21-byte form the configurator's own
   direct mode sends); requests from before power-on to 27 s after got no answer either. The ESC was silent the whole
   time with the lines high and restarted only when they went low - AM32 firmware plays its startup tune on every
   boot, so the firmware was not running.
3. **`00 00 00 00` with the line held high.** The bootloader's "start the firmware" command needs no reply. The ESC
   beeped right after it, and the line showed the firmware starting 7.6 ms later. So the bootloader *decoded* our bytes;
   only its answers never appeared - while every published AM32 bootloader (current, and the older F421 v5 and F051
   v10 sources) answers on the same pin it listens on.
4. **Betaflight's line handling, as a standalone test.** Every request answered: 225 of 225 init requests returned
   the bootloader's device info (`34 37 31 14 1F 06 06 01 30`: AM32 bootloader, input pin PB4, signature 0x1F06,
   **protocol version 1**, OK) and 75 of 75 keep-alives returned `C1`. The device info shows an older bootloader than
   the one whose source was read (protocol version 3). The buddy was switched to this line handling and the real
   configurator then read all four ESCs.

5. **Isolation (2026-11-05, GPIO 6, variants interleaved round by round, judged by the wingman's recording).** Run 1:
   Betaflight framing with exactly one difference changed back - no idle bit, the stop bit driven, or the idle line
   driven high - answered **80 of 80** (each variant 20 of 20 init requests); the first framing answered **0 of 20**.
   Run 2: the first framing's own bytes (stop bit driven, no idle bit), released by the PIO program the moment the last
   stop bit ends, answered **40 of 40** with the idle line on the pull-up and **40 of 40** with it driven high; the first
   framing itself again **0 of 40**. The ESC's reply starts **172-176 us** after our last stop bit.

**Cause: the first buddy released the line about 4.7 ms too late.** It released from Python with
`sm.exec("set(pindirs, 0)")`; MicroPython assembles a string instruction at call time, which measured 4.6-5.1 ms on the
Pico 2 (GPIO 10, nothing attached), so the release came 4.7-5.2 ms after the last stop bit - after the ESC had sent its
whole reply (9 bytes, about 4.7 ms) into a line the Pico still drove high. The ESC answered every request all along;
none of the idle bit, the stop-bit handling or the idle level matters to this bootloader. Betaflight's framing worked
because its release is inside the PIO program. Rule kept: never put `sm.exec(str)` on a timing path - release in the PIO
program, or pre-encode the instruction once (`rp2.asm_pio_encode`) and exec the integer.

## Alternatives considered

- **Read each MCU over its debug (SWD) pads**, with a Pico as the probe. Bypasses the bootloader entirely and would show
  the flashed bootloader itself, but needs physical access to the pads and hardware the owner would have to provision.
  Kept for when software options are exhausted; not needed.
- **The configurator's direct mode with a USB-serial adapter** (it speaks the bootloader protocol itself, one ESC at a
  time, expecting the adapter's echo). No adapter on the bench; it would also give up read-only enforcement, since the
  bridge would have to forward raw bootloader bytes, writes included.
- **Run the Arduino C++ passthrough as-is.** Means reflashing the main Pico away from MicroPython and gives no
  read-only guarantee (it implements write, erase and verify).
- **Deny-list writes instead of an allow-list.** A command the list forgot would reach the ESC.
- **Forward the configurator's own save.** Its block is not safe to forward: recorded on 2.16, it blanked the firmware name and padded the
  block's tail (record below). The write path builds its block from the one just read, with named fields replaced.
- **A read-only preview of a write** (`--plan`, `--plan-restore`). Built 2026-11-05, removed 2026-11-10 (owner: low value): the write refuses
  before it sends a byte, and its report lists each change with old and new.
- **Take the pins from a scenario file.** Couples a standalone tool to the test runner's file format for four numbers.
- **Save reports to disk by default.** Decides storage for the caller; a filter on stdout lets any tool or session keep
  what it needs.

## Consequences

- The vendor configuration is known and recorded (`specification/KM55A2_DEFAULTS.md`): AM32 2.16 target `SKYSTARS_KM55_V2_F421` (an F421 MCU -
  not the E230 target retailers list), bootloader version 1, all four ESCs identical, AM32's generic template values (KV 2220, 14 poles,
  startup power 100 %, timing 15 deg, 24 kHz variable PWM, stall protection off), untuned for the fitted motors. What the ESCs hold now:
  the top-level README, "ESC settings as configured".
- The AM32 knowledge pages were checked against a newer firmware commit than the bench runs, and the settings layout differs between 2.16
  (layout 2) and 2.22 (layout 4). Claims about the bench must hold for 2.22.
- Reading puts each ESC through its bootloader and back (about 2 s each, 20-60 s for four). Not compatible with a
  motor run; a run that counts as evidence should note that the ESCs were just rebooted.
- The settings write is guarded on the Pico (decision 2) and checked on the PC before the first byte: names and values, the ESC's answer,
  the settings page. The set of fields that can be named is in the tool README; limits for the few fields the configurator does not bound
  are the firmware's own (`loadEEpromSettings`). The bench bootloader (version 1) has no public source; the public one (version 19) was used for
  the frame sequence, and the first write showed version 1 follows the same sequence.
- The tool imports two modules from the wingman's folder (the main Pico's board ID and the start-line pin); harmless
  today, a shared home for them would be cleaner.

## Verification

- **Test population:** two identical Skystars KM55A2 boards (four ESC chips each, the same stock firmware and settings); every ESC of both
  boards survived all of it (reads, settings writes, restores, and the flashes of ADR-007), none was bricked. One model of one ESC: nothing here
  says how another ESC would behave.
- PC unit tests: the allow-list (every one of the 247 non-allowed 4-way commands refused without touching the line),
  bootloader frames and CRCs, MSP answers where the configurator reads them, the settings decoder against a layout 4 block read from a
  bench ESC; the write path (encoders and ranges, every Pico-side guard, the bootloader sequence against a fake bootloader built from the
  public source, write and restore end to end).
- Pico only (GPIO 10-13, nothing attached), after each change: byte loopback, channel switching, output driver states,
  the configurator's connect sequence.
- Bench (GPIO 6-9, ESCs powered): the real configurator read all four ESCs through the buddy on 2.16 (2026-11-04; its downloads, the buddy's
  record and later `--read` runs were byte-for-byte identical) and again on 2.22 (2026-11-09). Settings writes on 2.16 (2026-11-05 to
  2026-11-08, record below): the first write on one ESC recorded on the wire, a restore, then KV, beeper volume and poles on all eight ESCs,
  each read back equal and a final `--read` identical.
- Bench, layout 4 (2026-11-10, the four ESCs on GPIO 6-9, AM32 2.22): `--read` returned all 44 decoded fields of every ESC, no error, the four
  blocks identical (KV 7180, 12 poles, beeper volume 1, layout 4), bootloader signature `0x1F06` on PB4 on all four.
- Bench, first write and restore on layout 4 (2026-11-10, ESC 1 on GPIO 6, owner's go): `motor poles` 12 -> 14 written (acknowledged, page read
  back equal, one field changed, backup saved first), a discarded four-channel DShot run, then `--restore` from that backup (acknowledged,
  read back equal); a final `--read` showed all four ESCs identical again at 12 poles.

## Bench record of the settings writes (AM32 2.16, settings layout 2, 184-byte block; 2026-11-05 to 2026-11-08)

Kept here because the evidence belongs with the decision. The tool README describes the capability only.

- **What the configurator sends on Save** (`--passthrough --record-writes`, two sessions on 2026-11-05 and one on 2026-11-08, GPIO 6-9): per ESC
  exactly one 4-way write (`0x3B`) at `0x7C00` carrying the whole block, then a fresh init and a read-back of `0x7C00`; no page erase, erase all
  or verify. Against the block read just before: the fields the owner changed, the 12 firmware-name bytes sent as `0xFF` instead of the name,
  and the last 8 bytes sent as `0xFD` where the block held `0xFF`. All four ESCs got the identical frame. So its block is never a safe thing
  to forward. On the wire (the wingman's spy, valid recording) nothing but the init, set address and read-back frames reached the lines.
- **Frame sequence against the source:** set address, set buffer (`0xFE`), payload with CRC, program (`0x01`); a write at `0x7C00` starts a
  1 KB page, so the bootloader erases the whole page first (`save_flash_nolib`, AM32-bootloader version 19). Betaflight's write
  (`serial_4way_avrootloader.c`, `BL_WriteA`) matches (it waits 500 ms for the program ack, ours 2 s).
- **First write on a real ESC (2026-11-05, ESC 2, GPIO 7):** motor KV 2220 -> 7220 and poles 14 -> 12. The write was acknowledged and the whole
  page read back equal. On the wire: set address acknowledged, set buffer `FE 00 00 B8` with no reply, the 184-byte block with its CRC
  acknowledged `0x30`, program `01 01` acknowledged `0x30` 9.3 ms later. So the bench bootloader (version 1) follows the public sequence. The
  configurator, connected afterwards, read the new values; the other three ESCs were unchanged. Restore of the original block on the same ESC:
  acknowledged, page equal to the vendor block.
- **Ctrl-C in the request stream (2026-11-05, Pico only, GPIO 10-13):** a request containing `0x03` (ESC 4's init request does) that reaches the
  Pico while it is still starting interrupts the program and every later request goes unanswered. Six trials each, ESC 4's init as the first
  request: old board code 0 answered, Ctrl-C switched off first thing in `main` 5, switched off in the start command before the import 6.

## Not yet verified on a real ESC

- A first write of each editable settings field except motor poles (written and restored on layout 4, above), motor KV and beeper volume
  (written on layout 2). The other fields are tested for writing only against the 2.22 source, the configurator's field definitions and a
  fake bootloader.
- `--restore` after a real failed write that left the page blank: proven on healthy ESCs (layout 2 and layout 4) and, in tests, on a blank page only.
- The ESC configuration as session metadata: `scripts/run_session.py` could pipe `--read` into its own record before the run, so every
  result carries the configuration it ran with. Open: whether a read of about 30 s before each session is acceptable, or once per batch.
