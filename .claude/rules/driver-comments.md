---
paths:
  - "driver/**/*.py"
---

# Comments and docstrings in `driver/`

A comment or docstring in the driver says what the code is for and what its caller must know. Everything else lives elsewhere.

## What goes in

- **Purpose and contract.** What it is for; arguments, return value and what it raises; units; constraints the caller must respect (which core may call it, call order, timing).
- **The concept behind the code, when the code cannot show it.** One or two sentences, for example why a pin is released between frames. Not a walk-through of the code.
- **A non-obvious line** gets one short "why" clause at that line.
- **One pointer line per module or class** to the decision records that explain the design, for example `Design: ADR-002, ADR-005`.

## What stays out

- **Implementation narrative.** If code needs a story to be understood, change the code.
- **History.** No dates, no "used to", no "was removed", no bench results, no measurements, no alternatives tried, no who decided. That belongs to the decision record or bug report that owns the claim.
- **Bug numbers, backlog IDs, review references.** The pointer line above is the only reference to records.
- **Statements that are no longer true.** A comment is checked against the code whenever the code under it changes.

## Length

- First line: one sentence that summarises.
- Method docstring: normally up to 8 lines. Class: up to 15. Module header: up to 10.
- Longer needs a real contract to justify it, not more background.

## PIO programs

- One block above `@asm_pio` says what the program does in protocol terms: cycles per bit, the pin and interrupt contract with its partner program, what it hands to the CPU.
- A comment on an instruction is a short tag (cycle count, level, branch), not a sentence.

## Constants

- One line: meaning and unit.
- Where the value comes from is a pointer, not a story.
- A temporary limit stays marked as temporary, in one line, with a pointer to where the limit is recorded.

## When a comment is removed

A fact that exists only in a comment (a measurement, the reason for a value, the failure that led to a design) is moved to the record that owns it before the comment is deleted. It is not thrown away.

## Examples: before and after

Taken from `driver/dshot_pio.py` as it was when the convention was written; names have changed since (`send_throttle_command()` is gone, `stop()` is `deactivate()`).

**A method docstring.** History, a backlog ID and the reason for an optimisation go; the purpose stays.

```python
# Before
"""
Send an all-zero-throttle command - equivalent to
send_throttle_command(0), but skips the validation, packet build and
CRC calculation that produce the exact same result every time (see
self.zero_packet). Used only on MotorGroup.update()'s ARMING-branch
hot path, where every motor is sent a forced zero on every tick (see
bidirectional_dshot_review.md's W33).
"""

# After
"""
Queue one frame with throttle 0; same as send_throttle_command(0),
without building the packet. For MotorGroup.update()'s arming ticks.
"""
```

**A constant.** One line with meaning and unit. How the value was chosen belongs to the record.

```python
# Before
# One shutdown-only wait, not on any hot path: the ESC's reply to the last
# frame TX sent is still in flight for a while after TX's own last bit -
# dshot_bidir_rx_frame's own predelay is explicitly a lower bound on when that
# reply starts, not a measured one, so the actual turnaround can't be
# computed exactly here. A generous fixed margin, comfortably longer than
# a whole reply at either supported speed, before handing the pin to SIO
# avoids fighting the ESC's own drive.
STOP_REPLY_MARGIN_US = 300

# After
# Wait in stop() for a reply still in flight; longer than a whole reply at either speed
STOP_REPLY_MARGIN_US = 300
```

**A temporary limit.** Marked as temporary, with a pointer; no date and no story.

```python
# Before
# TEMPORARY bench ceiling (owner's decision 2026-10-19): throttles above this are not tested yet, so
# send_throttle_command() and MotorGroup.set_throttle() reject them instead of sending them. Raise or
# remove it when higher throttles are tested; MAX_THROTTLE stays the protocol's own limit.
THROTTLE_CEILING = 250

# After
# TEMPORARY: highest throttle accepted until higher ones are tested (BUG-005).
THROTTLE_CEILING = 250
```

**A PIO instruction.** A tag, not a sentence.

```python
# Before
pull()            .side(1)   [1] # blocks here with the pin already released - this is the gap/RX window
set(pindirs, 1)   .side(1)   [1] # a new command is ready - reclaim the pin as output

# After
pull()            .side(1)   [1] # pin released; waits here between frames (reply window)
set(pindirs, 1)   .side(1)   [1] # reclaim pin
```

**A statement that must be true.** A docstring states a fact about behaviour only if the code does it.

```python
# Before (false: at some throttles a capture of the driver's own frame passes the CRC)
"""... crc_ok is the validity signal ..."""

# After
"""... A capture is not proof of a reply: with a silent ESC the receiver captures
the driver's own next frame, and at some throttles that capture passes the CRC."""
```
