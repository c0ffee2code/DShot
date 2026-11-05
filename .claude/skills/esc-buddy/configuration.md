# ESC buddy: ESC configuration - read (current) and write (backlog)

## Before running

- **Permission:** none needed (CLAUDE.md, "The ESC buddy skill needs no permission"): no throttle, no writes. Say which
  pins it used.
- **The ESC must be powered.** Ask if unsure; the Pico cannot see power. No answer from every ESC usually means no power.
- **COM10 must be free:** no AM32 configurator tab open, no motor test or other script running on the main Pico. Never
  run it during a motor run - it stops DShot on all four lines.
- The wingman must not be in ESC-mock mode (it shares the lines).
- `--pins` lists the ESC signal lines in order = ESC 1..n (bench wiring today: GPIO 6-9; confirm with the owner if the
  wiring may have changed).

## Run

```
python tools/esc_buddy/esc_buddy_run.py --read --pins 6 7 8 9
```

Takes 20-60 s: each ESC needs about 2 s without DShot to reboot into its bootloader. Progress goes to stderr; stdout is
exactly one JSON report and nothing is written to disk - redirect or pipe it if it is to be kept (e.g. into a session's
metadata). Exit status 1: no ESC answered.

## Report

- Per ESC: `firmware` (version, name), `firmware_file`, `bootloader` (signature, input pin), `settings` (each with
  `byte`, `raw`, `value`), `raw_hex`; `identical` across ESCs; `error` for an ESC that did not answer.
- Compare with `specification/KM55A2_DEFAULTS.md` (recorded configuration and its date). Report any difference - field, old
  value, new value - and whether the four ESCs still agree. Do not update the specification page without the owner's go.
- Lead with what changed or what the user asked about, in plain words; the full table only if asked.

## After

On exit the buddy drives all lines low and each ESC starts its firmware again (no-signal beeps shortly after). Before a
run that counts as arming or start evidence, mention that the ESCs were just through their bootloader.

## Limits

- Read-only by construction: writes, erases and settings writes are refused on the Pico (`esc_buddy_protocol.ALLOWED_4WAY`).
- Decodes EEPROM layout 2 (AM32 firmware 2.16) only; another layout comes back with `raw_hex` and an `error`.
- Settings address is known only for bootloader signatures 0x1F06 and 0x3506 (`SETTINGS_ADDRESS` in `esc_buddy_run.py`).
- Details, line handling and bench history: `tools/esc_buddy/README.md`.

## Write (tune) - not supported yet

The buddy cannot write: writes, erases and settings writes are refused on the Pico by design. Tuning is backlog
(`tools/esc_buddy/README.md`, "To do"): its own explicit action flag, the owner's go per change, a read before and after,
and the old settings block kept. Until then, if asked to change a setting, say it is not possible yet: neither `--read`
nor passthrough can write (the configurator's Save is refused too). Offer to read the current value instead.
