---
paths:
  - "bug-reports/**"
  - "decision/**"
---

# Reasoning about bench results

The rules are in `CLAUDE.md` ("Analysing bench results"). This file holds the reasoning behind them and the worked example.

## A shift between batches: look for a code change first

When two batches of bench runs, taken hours or days apart, behave differently, check for an actual code change before reaching for "thermal drift", "power sag", "day-to-day factor" or any other unnamed environmental cause. Those can be real, but they are rarely the first thing to check. Naming one without evidence launders a gap in the investigation as a finding.

**Committed timestamps lag deployed code.** Code is routinely written, deployed to the Pico and bench-tested *before* it is committed. So "what commit was HEAD at that wall-clock time" is not "what code was running on the bench". A commit's own message often says so (for example "bench-verified: 10 more runs"): that verification happened before the commit, not after.

When comparing two batches, run `git log --date=iso-local -- driver tests/bench tools scripts` for every commit between them. Read what each one changed, and check whether it can cause the observed shift by its mechanism, not just whether it is close in time.

Worked example, BUG-002's investigation of 2026-09-30: a jump in first-contact acceptance on the evening of 09-27 lined up with a commit timestamped minutes *after* the jump. The change had been bench-verified first and committed once confirmed.

## No physical cause without a measurement

Do not offer physical causes for a bench result (wiring, connectors, crosstalk between signal wires, supply sag or inrush, grounds, solder joints, mechanical coupling through the lever, a "bad" ESC or motor) unless a measurement in this repository supports it. The Pico sees only DShot frames and replies. It cannot see the power or the wiring, so a hardware cause cannot be inferred from its data. This rule is stricter than the one above, which covers environmental explanations of a shift between batches.

- Report what the data shows (counts, timings, which motor on which channel and when) and what has been ruled out by a test, in those words.
- When asked "why", propose only hypotheses the repository can test in software (order of motors in a group, simultaneous against staggered starts, throttle, polling pattern, PIO block assignment). Say which result would confirm or reject each.
- Hardware actions (inspect, swap, measure the supply) are the user's to choose. List them as options without naming a suspected cause. The user knows the bench and holds the hardware facts; ask for them instead of guessing.
- A quantity taken from AM32's source (a timeout, a limit) may be quoted as a source fact. Saying that it explains a bench result is a hypothesis and is marked as one.
