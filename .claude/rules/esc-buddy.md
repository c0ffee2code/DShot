---
paths:
  - "tools/esc_buddy/**"
---

# ESC buddy design principle

The ESC buddy gives access to the ESCs' AM32 configuration through the main Pico. It is not a test group and not part of a motor run.

- **The buddy serves the firmware that is on the bench, and only that** (owner, 2026-11-10). There is no other hardware to keep working.
- **No backward compatibility** with older AM32 versions or settings layouts. When the ESCs move to a new firmware, the buddy moves with them in the same piece of work: decoder, field names and units, ranges, guards, tests, docs.
- **Old encodings, names and branches are deleted, not kept behind a shim.** An old name is refused like any unknown name. An ESC on another layout is refused, not guessed at.
- History stays in git and in the dated records of ADRs and READMEs.
- Today: AM32 2.22, settings layout 4 (ADR-007).
- Why it is built this way: ADR-006. The bench ESC and its settings: `specification/KM55A2_DEFAULTS.md` (vendor) and the README's "ESC settings as configured" (now).
