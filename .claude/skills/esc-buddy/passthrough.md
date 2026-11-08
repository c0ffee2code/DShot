# ESC buddy: passthrough for the AM32 web configurator

## Before

- **Permission:** none needed (SKILL.md): no throttle, no writes. Say which
  pins it used.
- **The ESC must be powered.** Ask if unsure; the Pico cannot see power.
- **COM10 must be free:** no configurator tab still open from before (closing the tab, not just Disconnect, frees it),
  no motor test or other script running on the main Pico. Never during a motor run - it stops DShot on all four lines.
- The wingman must not be in ESC-mock mode (it shares the lines).
- `--pins` lists the ESC signal lines in order = ESC 1..n in the configurator (bench wiring today: GPIO 6-9; confirm with
  the owner if the wiring may have changed).

## Start

```
python tools/esc_buddy/esc_buddy_run.py --passthrough --pins 6 7 8 9
```

## Then give the owner, every time

1. The link: **https://am32.ca/configurator** (Chrome or Edge - it needs Web Serial).
2. Wait until the ESC is silent (a few seconds without DShot: it reboots into its bootloader), then select **COM10**,
   **Connect** (normal mode, "Direct" unticked), **Read**.
3. Do not Save/Write: the buddy refuses writes and the configurator then shows errors.
4. When done: **Disconnect and close the tab**, then tell Claude.

## Timing and trouble

- The buddy waits up to 30 min for Connect; once connected it returns after 5 min of silence, or on Disconnect.
- "invalid result" in the configurator: the buddy had already timed out - start it again.
- "Port in use" / COM10 busy: a configurator tab still holds it - close the tab.

## After

- `python tools/esc_buddy/esc_buddy_run.py --debug` prints the buddy's own record of the session as plain text (each ESC's device info,
  every block read, refused writes, line diagnostics). It also ends a buddy that is still running.
- On exit the buddy drives all lines low and each ESC starts its firmware again (no-signal beeps shortly after). Before a
  run that counts as arming or start evidence, mention that the ESCs were just through their bootloader.
- If the owner reports values, compare them with the README's "ESC settings as configured"; don't update that section without their go.

Details: `tools/esc_buddy/README.md`.

## Recording what the configurator would write (no write reaches the ESC)

`python tools/esc_buddy/esc_buddy_run.py --passthrough --record-writes --pins 6 7 8 9` answers the configurator's
save commands (erase, write, verify) "OK" without sending anything to the ESC, and logs each one in full: command,
address, data. The owner clicks Save on one ESC; `--debug` then shows the `refused` lines, to be checked against the
bootloader source. The configurator's read-back shows the old settings, so it may report a failed save - expected.
