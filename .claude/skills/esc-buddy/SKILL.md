---
name: esc-buddy
description: Access the bench ESCs' AM32 configuration through the main Pico with tools/esc_buddy - read (poll) firmware, bootloader and every stored setting as JSON and compare it with specification/KM55A2_DEFAULTS.md, or enable passthrough so the owner can open the AM32 web configurator. Use when the user asks what the ESCs are set to, wants their settings checked or re-read, a question depends on an ESC setting (KV, poles, startup power, timing, protections), asks to change an ESC setting, or asks to enable passthrough / open the ESC configurator. Read-only today; writing (tuning) is backlog.
---

# ESC buddy

Access to the bench ESCs' AM32 configuration through the main Pico (`tools/esc_buddy/`). Pick the file for the task and
follow it:

| Task | File |
|------|------|
| Read the ESC configuration (Claude gets it as JSON), compare it with the record; any request to change a setting | `configuration.md` |
| Enable passthrough so the owner can use the AM32 web configurator (gives them the link) | `passthrough.md` |

Both talk to the ESCs' bootloaders on their signal lines and are read-only on the ESC; neither needs the owner's go
(CLAUDE.md, "The ESC buddy skill needs no permission"). Never during a motor run.
Tool details, line handling and bench history: `tools/esc_buddy/README.md`. The recorded configuration:
`specification/KM55A2_DEFAULTS.md`.
