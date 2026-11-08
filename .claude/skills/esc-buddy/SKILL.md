---
name: esc-buddy
description: Access the bench ESCs' AM32 configuration through the main Pico with tools/esc_buddy - read (poll) firmware, bootloader and every stored setting as JSON and compare it with the README's "ESC settings as configured" and specification/KM55A2_DEFAULTS.md (vendor), or enable passthrough so the owner can open the AM32 web configurator. Use when the user asks what the ESCs are set to, wants their settings checked or re-read, a question depends on an ESC setting (KV, poles, startup power, timing, protections), asks to change an ESC setting, or asks to enable passthrough / open the ESC configurator. Writes (named motor, start-up and protection settings, one ESC, owner's go each time) go through --plan, then --write. Flashing firmware is not built.
---

# ESC buddy

Access to the bench ESCs' AM32 configuration through the main Pico (`tools/esc_buddy/`). Pick the file for the task and
follow it:

| Task | File |
|------|------|
| Read the ESC configuration (Claude gets it as JSON), compare it with the record; any request to change a setting | `configuration.md` |
| Enable passthrough so the owner can use the AM32 web configurator (gives them the link) | `passthrough.md` |

Both talk to the ESCs' bootloaders on their signal lines. Rules (owner, 2026-11-04; full text in the README's "Permissions and safety rules"):

- No go needed: `--read`, `--passthrough`, `--debug`, `--plan`, `--plan-restore`. Name the pins used in the report.
- **`--write` and `--restore` write to an ESC and need the owner's explicit go every time**, after you showed the plan (what changes, on which ESC).
  The allow-list prompts for them too; that prompt is not the go.
- Never during a motor run. Put the action first on the command line, or the allow-list prompts for a read as well.

Tool details, line handling and bench history: `tools/esc_buddy/README.md`. The recorded configuration:
the top-level README's "ESC settings as configured" (what the ESCs hold now) and `specification/KM55A2_DEFAULTS.md` (the vendor's values).
