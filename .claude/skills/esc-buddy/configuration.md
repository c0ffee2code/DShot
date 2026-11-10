# ESC buddy: ESC configuration - read and write

## Before running

- **Permission:** none needed for reading (SKILL.md): no throttle, no writes. `--write`, `--restore` and `--flash` need the owner's go. Say which
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

- Per ESC: `firmware` (version, as the firmware stamped it into the block), `firmware_file` (the name at 0x7BE0), `bootloader` (signature, input pin), `settings` (each with
  `byte`, `raw`, `value`), `raw_hex`; `identical` across ESCs; `error` for an ESC that did not answer.
- Compare with the top-level README's "ESC settings as configured" (the recorded current configuration and its date; the vendor's values are in `specification/KM55A2_DEFAULTS.md`). Report any difference - field, old
  value, new value - and whether the four ESCs still agree. Do not update the specification page without the owner's go.
- Lead with what changed or what the user asked about, in plain words; the full table only if asked.

## After

On exit the buddy drives all lines low and each ESC starts its firmware again (no-signal beeps shortly after). Before a
run that counts as arming or start evidence, mention that the ESCs were just through their bootloader.

## Limits

- `--read` and passthrough cannot write (`--record-writes` only logs the configurator's save attempt, see passthrough.md); the only writes are `--write` and `--restore` below, guarded on the Pico (`esc_buddy_protocol.ALLOWED_4WAY`).
- Decodes EEPROM layout 4 (AM32 firmware 2.22, 192 bytes) only; another layout (an ESC still on 2.16) comes back with `raw_hex` and an `error`. No backward compatibility (owner, 2026-11-09).
- Settings address is known only for bootloader signatures 0x1F06 and 0x3506 (`SETTINGS_ADDRESS` in `esc_buddy_protocol.py`).
- Firmware flashing is `--flash` and `--dump-firmware` (ADR-007; owner's go for `--flash`, each ESC); the bootloader is not updated by the buddy. Do not try either through the configurator's passthrough, which refuses writes.
- What is supported, the properties and the guards: `tools/esc_buddy/README.md`.

## Write (tune)

Only the fields listed in `tools/esc_buddy/README.md` ("Properties": the complete editable set, each with the range the firmware accepts), one ESC at a time, the owner's go for every write.

1. **Read the ESC** (`--read`, no go needed) and work out the change: which ESC (and GPIO), which field, old value -> new value. Names and values
   are validated before anything is sent (KV goes in steps of 40 from 20: 7200 is refused, 7180 or 7220).
2. **Show the owner that change and ask for the go.** A write can leave that ESC without valid settings until it is written again; the
   bootloader area cannot be touched.
3. **Write** (only after the go): `python tools/esc_buddy/esc_buddy_run.py --write --pins 6 7 8 9 --esc 4 --set "motor KV=7180" --backup tmp/esc<N>_<what>.json`
   (the git-ignored `tmp/` folder; an existing backup file is never overwritten). Before the first byte is sent the tool reads the ESC and refuses
   when the page holds anything beyond the settings block or nothing would change; the report lists each change with old and new, `ok`,
   `write_ack` and whether the read-back matches.
4. Afterwards run `--read` to confirm, tell the owner to expect the ESC's normal start beeps, and ask before touching
   the README's "ESC settings as configured". The backup holds the whole page; restoring is another write.

If a write fails or the read-back differs, do not retry blindly: tell the owner, and put the saved block back with
`--restore tmp/<backup>.json --esc N --backup tmp/<new file>.json` (with `--pins`, and again with the owner's go).

Exit status 1 when the write did not go through (`error` in the JSON says why). Details: `tools/esc_buddy/README.md`.
