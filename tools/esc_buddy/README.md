# ESC buddy

Gives the **AM32 configurator** (the web app) - and Claude, through `--read` and the `esc-buddy` skill - access to the
ESCs' configuration through the **main Pico**, with no flight controller. The configurator takes the Pico for a
Betaflight flight controller in passthrough mode; the Pico relays its requests to each ESC's bootloader on that ESC's
signal line. Current phase: read (poll) the configuration. Tuning (writing) is backlog, below.

What the bench ESC is and how it is configured, as read with this tool: `specification/KM55A2_DEFAULTS.md`.

**Read-only.** Only these configurator commands reach an ESC: alive check, interface name/version, set mode, exit,
init flash access, read, reset. Write, erase, EEPROM write and verify are answered "invalid command" without touching
the line, and logged as `refused`. The Pico builds every bootloader message itself; nothing from the PC is forwarded raw.
Why it matters: the vendor's settings exist only in the ESC chips, and the configurator itself writes settings on its
own in some cases (`four_way.ts`, bootloader version reads `0xFF`).

## Files

| File | Runs on | What |
|------|---------|------|
| `esc_buddy_protocol.py` | Pico and PC | MSP and 4-way answers, bootloader frames, the allow-list (pure Python; tested by `tests/pc/test_esc_buddy_protocol.py`) |
| `esc_buddy_board.py` | main Pico | USB serial loop and the signal-line wire: two PIO programs, 19200 baud 8N1, half duplex |
| `esc_buddy_run.py` | PC | finds the main Pico by ID, uploads and starts the buddy; one action required: `--read` (settings as JSON), `--passthrough` (browser configurator), `--debug` (the buddy's record, plain text) |
| `esc_buddy_settings.py` | PC | decodes a settings block (EEPROM layout 2, firmware 2.16) into named values; tested by `tests/pc/test_esc_buddy_settings.py` against the configurator's display |

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

With an ESC attached this puts signals on the ESC lines; being read-only with no throttle, it needs no permission
(CLAUDE.md, "The ESC buddy skill needs no permission"), but never during a motor run. The wingman must not run the ESC
mock at the same time (it shares the lines).

## The record (`esc_buddy_log.txt` on the Pico, shown by `--debug`)

One line per event: `init` (the bootloader's 9-byte device info for that channel - pin code, flash size, protocol
version), `read` (address and the bytes read, hex), `refused` (the command byte the configurator tried), and `end`
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

## To do

- **ESC configuration as session metadata.** A session (`scripts/run_session.py`) could pipe `--read` into its own
  record before the run, so every result carries the configuration it ran with, and the effect of a later settings
  change can be judged by comparing sessions. Open: whether the ~30 s read and the bootloader round trip before each
  session are acceptable, or once per batch is enough.
- **Tune the configuration (backlog).** Writing settings, behind its own explicit switch and the owner's go per change,
  with a read-back before and after and the old block kept. Today the buddy cannot write at all (allow-list); that
  guarantee must stay for the read path.
## Verified so far

### Pico only, GPIO 10-13, nothing attached (2026-11-03, first line handling)

- Line loopback on GPIO 10: 28 of 28 bytes sent came back unchanged on the same pin, 528 us per byte (19200 baud 8N1
  is 521 us).
- Four lines, GPIO 10-13: 18 channel switches with a loopback after each, 0 failures; the three other lines read high
  every time, the active line high again after each reply window, all four low after exit.
- The configurator's connect sequence played from a PC script: MSP answers (Betaflight, battery state, motor count 4,
  passthrough 4), 4-way version, init and read on ESC 1-4 twice fail cleanly with no ESC (general error), ESC 5 is an
  invalid channel, two write attempts refused and logged, exit returns the board to the REPL, the record could be read back (now `--debug`).
### On the bench, Skystars KM55A2 on GPIO 6-9

- Real AM32 configurator in the browser, GPIO 6-9, ESC attached but not powered (2026-11-03): accepted the Pico as a
  Betaflight FC (API 1.42.0, battery 0 V, motor count 4, passthrough); init flash on ESC 1-4, two tries each, all
  empty (no ESC power, as expected); Disconnect ended the session (`end exit`). A first attempt failed with "invalid
  result" because the buddy had already timed out before Connect (fixed: it now waits 30 min for the first
  connect).
- Powered ESC (Skystars KM55A2), GPIO 6-9, four sessions 2026-11-04: the configurator connects, but no ESC ever answers
  "init flash access" - both request forms (17 and 21 bytes), two tries each, on all four lines. The Pico's own
  echo matches what it sent every time, and the line never went low during any 200 ms reply window. The Pico's output
  driver is verifiably off during the reply window (IO bank status register, checked on GPIO 10), and the wingman is
  not driving GPIO 6-9. The wingman's spy, on its own clock, decoded every request exactly as built (52.0 us bits, no
  framing errors) and saw no other edge on any line for the whole 42 s session, including the ESC's power-cycle.
- What the ESC did (owner, by ear): silent for the whole session with the lines high (no startup tune, no beeps);
  restarted with its normal no-signal beeps once the buddy exited and drove the lines low. AM32 firmware plays
  its startup tune on every boot (`Src/main.c`), so the firmware was not running; inferred: the ESC sat in its
  bootloader and did not answer. The AM32-bootloader history shows no change to 19200 baud or the request check since
  its first commit (2024-08-20). Open: which bootloader the vendor flashed.
- `00 00 00 00` (the bootloader's "start the firmware") sent on GPIO 6 with the line held high: the ESC beeped right
  after it (owner) and the spy saw a 34 ms low 7.6 ms later - so the bootloader decoded our bytes; only its answers
  never appeared. Init requests every 150 ms from before power-on to 27 s after: no answer either.
- **Betaflight-style line handling (2026-11-04): it answers.** 225 of 225 init requests answered with
  `34 37 31 14 1F 06 06 01 30` (AM32 bootloader, input pin PB4, signature 0x1F06, protocol version 1, OK) and 75 of 75
  keep-alives with `C1`. The buddy was switched to this line handling (Pico-only checks on GPIO 10-13 repeated:
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
