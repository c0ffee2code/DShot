# Bidirectional DShot on Raspberry Pi Pico 2 — Conceptual & Implementation Review

**Review basis:** the current project files and ADR material provided in this conversation, especially `dshot_pio.py`, `motor_throttle_group.py`, and `ADR-002-bidirectional-dshot.md`.

**Review status:** 2026-08-25

---

## Executive summary

The RX waveform/capture/decode work is substantially stronger than the surrounding production architecture.

The hardware evidence is excellent:

- 59/59 CRC-valid captures in the first broad throttle sweep.
- 162/162 CRC-valid captures in the subsequent up/down sweep.
- Monotonic eRPM behavior.
- Repeatability across separate runs/days.
- No meaningful up/down hysteresis.

The ADR itself correctly states that this proves Phase 3, but that the result is **not yet integrated into `MotorThrottleGroup` / `DShotPIO`'s public API**.

The main risks are therefore no longer "can the Pico decode the AM32 reply?" but:

1. TX/RX synchronization semantics.
2. RX FIFO backpressure.
3. Continuous operation under load.
4. State-machine lifecycle/reset semantics.
5. Multi-motor resource allocation.
6. Cross-core ownership/concurrency.
7. Generalizing beyond the currently characterized DShot300/AM32 setup.

---

# P0 — Critical issues

## 1. PIO IRQ handshake may lose frame boundaries

### Concern

The current dual-SM design appears to use a PIO IRQ as a per-frame synchronization signal:

```text
TX:
    release GPIO
    raise IRQ
    start next frame

RX:
    wait for IRQ
    capture reply
    wait for next IRQ
```

The problem is that a PIO IRQ is a flag, not a queue of timestamped/event-counted IRQ objects.

If TX produces another IRQ while RX is still consuming the previous capture, there is no inherent representation of:

```text
IRQ #184
IRQ #185
IRQ #186
```

as three queued events.

### Failure scenario

```text
TX:  frame A ---- IRQ ---- frame B ---- IRQ ---- frame C ---- IRQ
RX:             capture A ----------------------------->
```

If RX is still capturing A when the IRQ for B occurs, the synchronization relationship can be lost.

This becomes especially relevant because the TX SM can continue consuming queued frames from its FIFO.

### Why current testing may not expose it

The current hardware tests strongly validate the physical reply and decoder, but they do not yet establish that the production driver can sustain an uninterrupted frame-by-frame transaction at maximum rate.

### Recommendation

Do not use a bare PIO IRQ flag as the sole identity of a transaction.

Preferred approaches:

- Make TX and RX explicitly lockstep.
- Prevent TX from getting ahead of RX.
- Or move to a single-SM transaction architecture similar to the Betaflight reference.
- At minimum, introduce explicit sequence/epoch tracking at the driver level.

### Acceptance test

Run at least 10,000–1,000,000 consecutive frames and prove:

```text
TX frame N <-> RX capture N
```

for every frame, with no lost/misaligned transactions.

---

## 2. RX FIFO can stall the RX state machine

### Concern

The current capture produces:

```text
128 samples
= 4 × 32-bit words
```

which exactly fills the RX FIFO.

The public `rx_read()` API exposes one FIFO word at a time.

If the application does not drain all four words before another capture reaches `autopush`, the RX SM can stall when the FIFO is full.

### Failure chain

```text
RX capture
    ↓
FIFO fills
    ↓
consumer is late
    ↓
autopush blocks
    ↓
RX SM stalls
    ↓
TX continues
    ↓
TX/RX synchronization is lost
```

This is tightly coupled to issue #1.

### Recommendation

Make a complete capture the internal unit of work.

For example:

```python
capture = read_capture()
```

where the implementation guarantees:

```text
word 0
word 1
word 2
word 3
```

are consumed as one capture.

Better still, make raw capture a diagnostic API and expose decoded telemetry to normal callers.

### Acceptance test

Deliberately stop consuming RX data for several milliseconds, then resume.

Verify:

- no permanent RX stall,
- deterministic recovery,
- no stale capture interpreted as current telemetry,
- explicit missed-frame accounting.

---

## 3. TX FIFO streaming and bidirectional transactions are conceptually at odds

Normal DShot naturally wants:

```text
Python → TX FIFO → continuous PIO stream
```

Bidirectional DShot naturally wants:

```text
TX frame
→ release
→ ESC reply
→ RX capture
→ next TX frame
```

The current design combines a potentially free-running TX path with an independently synchronized RX path.

That is the root of several timing/association problems.

### Recommendation

For bidirectional mode, make the unit of scheduling a **transaction**, not merely a TX frame:

```text
submit command
    ↓
TX
    ↓
turnaround
    ↓
RX
    ↓
decode
    ↓
transaction complete
```

PIO should still handle all timing-critical work. The CPU does not need to participate in bit timing.

---

# P1 — High-priority issues

## 4. Bidirectional RX is currently DShot300-specific

The current implementation hard-codes:

```python
rx_speed = 4_000_000
```

and the source comments explicitly say:

> Tuned for DSHOT300 only.

Yet the constructor still permits general DShot speed selection.

### Recommendation

Until characterized, reject unsupported combinations:

```python
if bidirectional and dshot_speed != DSHOT300:
    raise ValueError(...)
```

Eventually replace this with explicit calibrated profiles:

```text
BIDIR_PROFILE[DShot300]
BIDIR_PROFILE[DShot600]
...
```

Do not silently imply that bidirectional operation is supported at speeds that have not been validated.

---

## 5. The ~4.7 µs turnaround should be treated as an empirical/ESC-specific parameter

The current implementation uses a fixed post-release delay around 4.7 µs.

The ADR also records a substantially different turnaround figure from the Betaflight implementation/reference material.

Therefore, the safest conceptual interpretation is:

```text
ESC/firmware-specific reply latency
```

rather than:

```text
universal bidirectional-DShot protocol constant
```

### Recommendation

Make turnaround a configurable profile parameter.

Preferably, capture a sufficiently broad post-release window and detect the actual reply start rather than relying on one hard-coded latency.

---

## 6. Phase-3 verification does not prove the production driver

The current evidence proves:

- physical RX capture,
- GCR reconstruction,
- CRC,
- eRPM decoding,
- repeatability.

It does **not** yet prove:

- continuous RX synchronization,
- RX FIFO handling under load,
- multi-motor operation,
- lifecycle robustness,
- cross-core operation,
- DShot600/1200 bidirectional support,
- production API integration.

Keep these verification levels explicitly separate.

### Suggested status table

| Layer | Status |
|---|---|
| Inverted DShot TX | Verified |
| ESC bidirectional detection | Verified |
| Physical RX capture | Verified |
| GCR decode | Verified |
| CRC validation | Verified |
| eRPM calculation | Verified |
| Continuous RX synchronization | Not proven |
| RX FIFO management | Not proven |
| Multi-motor operation | Not proven |
| Public API integration | Not implemented |
| DShot600/1200 bidirectional | Not supported |
| Telemetry loss/fault handling | Not implemented |

---

## 7. `MotorThrottleGroup` does not yet integrate bidirectional mode

The current constructor creates:

```python
DShotPIO(i, pin, dshot_speed)
```

without enabling bidirectional mode or allocating RX SMs.

Simply adding `bidirectional=True` would not be sufficient.

### Resource issue

Each motor potentially needs:

```text
TX SM
RX SM
```

and both need to reside in a compatible PIO block because they share the GPIO / synchronization mechanism.

### Recommendation

Create an explicit PIO resource allocator.

Do not derive RX SM IDs from motor index with assumptions about PIO block placement.

---

## 8. `stop()` / `start()` need explicit telemetry-epoch semantics

The ADR correctly notes that `StateMachine.restart()` does not clear FIFOs.

Therefore:

```text
run
→ partial/stale RX FIFO
→ stop/restart
→ start
```

can leave stale RX data unless the FIFO is explicitly flushed.

### Recommendation

A bidirectional `start()` should establish a clean epoch:

```text
stop
→ disable SMs
→ clear RX FIFO
→ clear TX FIFO as appropriate
→ reset RX parser state
→ clear synchronization state
→ arm RX waiting state
→ start TX
```

The first telemetry result after restart should have an explicit validity state.

---

## 9. Startup ordering should make RX ready before TX can emit

Current startup activates TX before RX.

If TX already has data queued, the first frame/IRQ can theoretically happen before RX is ready.

### Recommendation

Use:

```text
initialize RX
→ clear RX FIFO
→ put RX into waiting state
→ activate RX
→ activate TX
```

or otherwise guarantee that the first TX event cannot outrun RX initialization.

---

# P2 — Important robustness/API issues

## 10. Raw 128-sample capture is excellent diagnostics, but probably not the final telemetry API

The dense capture strategy was a very good debugging choice.

It solved the accumulated-phase-error problem and made it possible to establish the true signal period empirically.

However, production telemetry probably should not expose:

```text
128 raw samples
```

as the normal interface.

### Suggested split

Diagnostic mode:

```python
capture_raw()
```

Normal mode:

```python
telemetry.erpm
telemetry.valid
telemetry.timestamp
```

The raw waveform path can remain available for troubleshooting.

---

## 11. AM32-specific GCR behavior should be explicit in the API/documentation

The ADR found that AM32's real GCR table differs from the table in the project's generic DShot specification and matches Betaflight's implementation.

This means the decoder should be described accurately.

Instead of implying:

```text
generic bidirectional DShot decoder
```

consider documenting:

```text
AM32-compatible bidirectional DShot GCR decoder
```

unless additional ESC firmware families are tested.

---

## 12. `send_throttle_command()` has a misleading exception message

Current logic permits zero throttle, but the exception says:

```text
Throttle should be greater than 0.
```

It should say something equivalent to:

```text
Throttle must be >= 0.
```

This is minor, but worth fixing before treating the API as stable.

---

## 13. Cross-core `MotorThrottleGroup.disarm()` is not actually synchronized

The code comments describe `disarm()` as safe from another core.

However, the state flag is not a lock around PIO operations.

One core can be inside a send operation while another core begins:

```text
state = DISARMED
→ enqueue zero frames
→ drain
→ stop SMs
```

The comments reason about timing, but timing is not a synchronization primitive.

This becomes more serious with bidirectional mode because there are now:

```text
TX SM
RX SM
shared GPIO
IRQ synchronization
RX FIFO state
```

### Recommendation

Establish explicit ownership:

```text
one core owns PIO lifecycle
other core writes command state
```

For emergency stop, use a dedicated synchronization mechanism rather than relying on scheduling assumptions.

---

## 14. Multi-motor throttle updates are not coherent

The current throttle array uses atomic element writes, but a batch update is not atomic as a whole.

That is acceptable for the current use case, but closed-loop control will eventually benefit from coherent snapshots.

### Recommendation

Use a double-buffer or generation-number model:

```text
command buffer
generation
```

and have `update()` consume one coherent generation across all motors.

---

## 15. Separate eRPM from mechanical RPM

The low-level DShot decoder should ideally return:

```text
eRPM
```

rather than applying motor pole-count conversion.

Motor pole count is a property of the motor/system, not the DShot protocol.

Suggested layering:

```text
GCR
 ↓
DShot telemetry value
 ↓
eRPM
```

then:

```text
eRPM
 ↓
motor pole count
 ↓
mechanical RPM
```

The ADR already notes that the current pole-count assumption is an unverified constant scale factor.

---

## 16. Add explicit telemetry health state

CRC-valid telemetry is not sufficient for a real-time controller.

Suggested state:

```python
telemetry.valid
telemetry.erpm
telemetry.timestamp
telemetry.age_us
telemetry.crc_errors
telemetry.frames_received
telemetry.frames_missed
telemetry.consecutive_failures
```

A controller should be able to distinguish:

```text
"RPM = 22000"
```

from:

```text
"last valid RPM = 22000, but telemetry is 30 ms old"
```

---

# Recommended verification plan

## Test A — continuous saturated operation

Run:

```text
10,000+ frames
```

and eventually:

```text
1,000,000 frames
```

with no intentional application pacing.

Measure:

- CRC-valid count
- missed frames
- FIFO stalls
- sequence mismatches
- capture-to-frame association
- latency distribution
- maximum sustainable rate

Repeat for:

```text
1 motor
2 motors
4 motors
```

---

## Test B — RX starvation/recovery

Procedure:

```text
capture normally
→ stop consuming RX FIFO for 5 ms
→ resume
```

Expected:

- RX recovers,
- stale frames are not silently accepted,
- missed frames are counted,
- synchronization is restored deterministically.

---

## Test C — lifecycle stress

Repeatedly:

```text
start
→ transmit
→ receive
→ stop
→ start
```

for thousands of cycles.

Verify:

- no stale FIFO data,
- no lost synchronization,
- no first-frame corruption,
- no stuck SMs.

---

## Test D — multi-motor simultaneous telemetry

Run all four motors simultaneously.

Measure:

- per-motor CRC-valid rate,
- FIFO occupancy,
- synchronization,
- CPU load,
- cross-motor interference,
- PIO resource allocation.

---

## Test E — DShot speed matrix

Do not enable a speed until independently validated.

Suggested matrix:

| TX | RX | Status |
|---|---|---|
| DShot300 | calibrated | Current |
| DShot600 | unknown | Do not assume |
| DShot1200 | unknown | Do not assume |

---

# Recommended architecture

A clean conceptual model is:

```text
                 ONE MOTOR TRANSACTION

             ┌──────────────────────────┐
             │        TX PIO            │
             │                          │
command ───► │  16-bit inverted DShot   │
             └────────────┬─────────────┘
                          │
                     release GPIO
                          │
                     ESC turnaround
                          │
                          ▼
             ┌──────────────────────────┐
             │        RX PIO            │
             │                          │
             │ capture / oversample     │
             └────────────┬─────────────┘
                          │
                    complete capture
                          │
                          ▼
             ┌──────────────────────────┐
             │      CPU decoder         │
             │                          │
             │ GCR → frame → CRC        │
             │           │              │
             │          eRPM             │
             └────────────┬─────────────┘
                          │
                          ▼
                  telemetry state
```

The key invariant should be:

> **One transaction has one identity.**

The driver should be able to reason about:

```text
TX sequence 18472
      ↕
RX capture 18472
      ↕
telemetry result 18472
```

rather than relying on an unnumbered PIO IRQ flag to establish correspondence.

---

# What I would fix first

1. **Prove/fix IRQ event loss.**
2. **Make RX FIFO draining atomic per capture.**
3. **Prevent TX from outrunning RX.**
4. **Define ownership of TX/RX SMs and lifecycle operations.**
5. **Make start/stop create clean telemetry epochs.**
6. **Reject unsupported bidirectional DShot speeds.**
7. **Add sequence/missed-frame/age telemetry state.**
8. **Integrate explicit PIO SM allocation.**
9. **Add continuous-rate and RX-starvation stress tests.**
10. **Only then promote bidirectional mode into the public `MotorThrottleGroup` API.**

---

# Overall assessment

The encouraging conclusion is that the hardest physical/protocol problem appears to be solved.

The dense oversampling + run-length reconstruction approach was a strong engineering decision. The subsequent hardware sweeps provide convincing evidence that the AM32 reply is being captured and decoded correctly.

The remaining risk is primarily architectural:

> **The experimental capture path is proven; the continuous transaction engine is not yet proven.**

I would therefore avoid changing the GCR table, nibble ordering, CRC polarity, or the successful dense-capture decoder unless a new failing test specifically implicates them. The investigation has already eliminated those as the likely remaining problems.

The first engineering target should instead be **deterministic TX/RX transaction scheduling and FIFO/IRQ behavior under continuous load**.

---

# Maintainer assessment (2026-08-25)

Every factual claim in the review was checked against the code and ADR-002 before this
backlog was written. The review's core thesis is correct and adopted: **the capture/decode
path is proven; the continuous transaction engine is not**, and that engine — not the GCR
table, nibble order, CRC polarity, or dense-capture decoder — is where the remaining work
lives.

**Design constraint (2026-08-25, also recorded in CLAUDE.md):** this is a pet/exploration
project. Supported ESC firmware families are exactly two: **BLHeli_S** (unidirectional
baseline — stock BLHeli_S has no bidirectional DShot) and **AM32** (the bidirectional
target). AM32's decisive advantage is being open source: its firmware source is this
project's ground truth, so "what does the ESC actually do" is answered by reading
`Src/dshot.c`/`Src/signal.c`, not by generalizing from articles. Consequences for this
review: the executive summary's risk 7 ("generalizing beyond the currently characterized
DShot300/AM32 setup") is **out of scope by design**, not an open gap — no generic-ESC
abstraction layers, no profile machinery for hypothetical firmware families. Turnaround,
GCR behavior, and reply timing are characterized against AM32-as-shipped and that is
sufficient.

Per-finding verdicts (review issues referenced as R1–R16, its tests as TA–TE):

**Adopted as written:** R1, R2 (both verified against `dshot_bidir_tx`/`dshot_bidir_rx`: the
IRQ-4 flag is sticky and unnumbered, and an undrained RX FIFO stalls `in_()` mid-capture,
after which the next `wait(1, irq, 4)` fires on a stale flag and `wait(0, pin, 0)` can
trigger on TX's own LOW bits — capturing TX's waveform as a "reply"), R4, R6, R7, R8 (also:
the IRQ-4 flag is PIO-block state, not SM state, so it survives `restart()` — a stale set
flag is part of the epoch that `start()` must clear), R9, R10, R12 (confirmed at
`driver/dshot_pio.py:335`), R15, R16.

**Adopted with modification:**

- **R3** (transaction model): correct diagnosis, but the review does not account for this
  project's hardest-won constraint — this ESC's arming fragility (ADR-002: only true
  back-to-back framing arms it; even a clean 250µs-paced loop failed). ADR-002 already
  evaluated and rejected the Betaflight single-SM per-frame stop/restart pattern (its
  "Option B") for exactly this reason. Any lockstep/transaction design must leave the arm
  sequence's framing cadence untouched — e.g. free-run TX during arming (drain and discard
  RX), enter lockstep only after arm. The decision is item W6, deliberately gated on
  measured data from W4/W5 rather than adopted on argument alone.
- **R5** (turnaround): the review implies the reply start is found by the fixed delay alone.
  It isn't — `wait(0, pin, 0)` already detects the actual reply edge; the ~4.7µs predelay is
  a lower-bound guard against re-triggering on TX's own still-LOW tail. So the actionable
  part is parameterizing predelay + `rx_speed` per (speed, ESC) profile (folded into W1),
  not adding a window-scan.
- **R11** (AM32-specific GCR): the framing is off. ADR-002 established that AM32's table
  matches Betaflight's `gcrs[]` exactly — this is the ecosystem's real bidirectional DShot
  table, not an AM32 quirk. What's actually wrong is `specification/DSHOT_PROTOCOL.md`'s own
  table (ADR-002 flags it, unfixed). The item (W2) fixes the spec with provenance rather than
  relabeling the decoder. The review's suggestion to test "additional ESC firmware families"
  is moot under the design constraint above: AM32 is the only bidirectional target, and its
  source is checkable directly.
- **R13** (cross-core disarm): real, but the current TX-only reasoning is deliberate and
  documented (ADR-001, `disarm()` comments), and no bug has been demonstrated in it. Scoped
  as an ownership-design input to bidirectional integration (W11), not a standalone fix to
  the existing unidirectional path.

**Deferred, no backlog item:** R14 (batch throttle coherence) — the review itself calls the
current behavior acceptable, the limitation is documented in `set_all_throttles()`, and
nothing consumes coherent snapshots yet. Revisit when closed-loop control exists.

## ADR-002 review sweep (2026-08-25) — findings A1–A7

A second sweep over `decision/ADR-002-bidirectional-dshot.md` itself, cross-checked against
`driver/dshot_pio.py` and `scripts/decode_bidir_capture.py`. These are new findings, not in
the third-party review; remediation lands in W15/W16 and amendments to W1/W4/W9 below.

- **A1 — the "~4.7µs" predelay figure is stale at the current clock.** The predelay is 14 RX
  cycles (`set(x, 1)` + two 7-cycle iterations). 4.7µs was correct at the superseded slotted
  design's 3MHz clock; the verified redesign runs `rx_speed = 4MHz`, where 14 cycles ≈
  **3.5µs**. The wrong figure appears as current fact in `driver/dshot_pio.py`'s
  `dshot_bidir_rx` comment ("~4.7us fixed delay ... empirically confirmed correct"),
  `tests/test_bidir_rx_raw.py`'s header, and ADR-002's Timing Coordination note ("the real
  fixed delay ... is ~4.7µs"). What hardware actually verified is *14 cycles at 4MHz ≈
  3.5µs is enough*. Functionally harmless today; a trap the moment anyone treats 4.7µs as a
  calibrated constant (e.g. when building W1's profiles or a DShot600 profile).
- **A2 — a disproven claim lacks its inline supersession flag.** Bug-list item 4 (slotted-
  design section) states run-length analysis "confirmed the RX clock and the 8-cycles/bit
  design were both correct all along — `rx_speed`'s 5/4 multiplier is right and should not
  be touched." The verified redesign then measured the real bit period at 10.1–10.4 cycles
  @4MHz ≈ 2.5–2.6µs — a few percent *off* the 5/4-derived 2.67µs, and that small mismatch's
  accumulated phase error is exactly what the ADR itself concludes defeated the slotted
  design. The section is marked superseded as a whole, but the ADR's own convention is to
  flag wrong claims inline, and this one reads like a standing instruction ("should not be
  touched").
- **A3 — the CRC check accepts both polarities.** `check_crc()` in
  `scripts/decode_bidir_capture.py` passes a frame if its CRC matches *either* the plain or
  the inverted formula. This retroactively explains the ADR's otherwise-odd
  "P(CRC match | valid symbols) = 2/16" figure — the tooling really does have two accepting
  outcomes — but it doubles the false-accept probability of a 4-bit CRC, and the ADR's
  headline results (17/17, 59/59, 162/162) never record *which* polarity hit (expected:
  inverted, per the AM32 `make_dshot_package` brute-force finding). Fine for exploration;
  the production decoder (W9) must pin one polarity, and the ADR should record what the
  hardware produced.
- **A4 — the decode pipeline assumes every reply is an eRPM frame.** The decoder computes
  eRPM from any CRC-valid frame. Bidirectional DShot also defines a stopped-motor/max-period
  sentinel, and AM32 supports Extended DShot Telemetry (EDT), where some replies carry
  temperature/voltage/current in the same 12-bit field — decoding those as eRPM would give
  confidently wrong RPM. All sweeps so far ran at steady spin, which would not surface
  either case. W9 must check AM32 source for when EDT frames are emitted and discriminate
  frame types; zero-throttle/stopped telemetry has never been tested.
- **A5 — before the ESC arms, every RX capture is a TX echo by construction.** Per the ADR's
  own AM32-source finding, replies start only once `armed && dshot_telemetry`. During the
  3000ms arm window the RX SM still fires per frame (IRQ → predelay → `wait(0, pin, 0)`),
  no reply comes, and the wait triggers on the *next TX frame's own LOW bits* — capturing
  TX's waveform. At back-to-back DShot300 (~53µs/frame) that is tens of thousands of echo
  captures per arm; with the pipeline's ~0.8% garbage-pass rate (symbol-valid ~6.25% ×
  dual-polarity CRC ~12.5%), a handful will pass CRC as plausible-looking junk. The tests
  drain and discard during arming, so nothing observed was wrong — but the ADR never draws
  this consequence, and the production validity model (W8/W10) must gate telemetry on arm
  state, not on CRC statistics. W4's harness should expect and count these echoes.
- **A6 — inconsistent GCR-table agreement counts.** The GCR table section says the ADR's
  original table agreed with AM32 on "7 of 16" entries; the Implementation Update says the
  spec's table agrees on "6 of 16". Two different wrong tables, or one miscount — reconcile
  when fixing the spec (W2/W15).
- **A7 — "Recommendation: Bluejay" is dead under the design constraint.** The Firmware
  Compatibility section still recommends flashing Bluejay onto the BLHeli_S ESCs. Under the
  supported-ESC constraint (BLHeli_S = unidirectional as-is, AM32 = the bidirectional
  target) no Bluejay flash is planned; the section needs a superseded marker so a future
  session doesn't pursue it.
- **Checked and benign:** the two confirmation sweeps report *identical* throttle-100
  statistics (mean 3,092, range 3,088–3,097) — it looks like a copy-paste error but is
  real quantization: at ~21.6k eRPM the encoding sits at exponent=3, where adjacent
  mantissas (347/346) map to exactly 3,088 and 3,097 RPM. The "range" is two adjacent
  quantization levels, expected to repeat exactly across runs. The eRPM math in the ADR
  (exponent stepping 3→2→1, the ~21.6k/~48.8k/~76.1k figures vs the sweep tables) was also
  re-derived and checks out.

---

## Backlog — sequenced work plan

### How to work this backlog (instructions for future sessions)

1. Read the findings referenced by your item (R-numbers above in this document) **and** the
   matching ADR-002 sections before touching code. Where this review and ADR-002 conflict,
   ADR-002's hardware-verified findings win — in particular, do NOT re-investigate the GCR
   table, nibble ordering, CRC polarity, the 21-bit frame model, or the run-length decode
   method; ADR-002 proves them and this review explicitly endorses leaving them alone.
2. Take items in order unless the status table says otherwise. The order encodes
   dependencies: W4's harness is what makes W7 and W8 verifiable; W6's decision gates W7's
   implementation; W9–W11 build on a proven transaction engine (post-W8 gate). If you must
   skip, record why in the status table.
3. One item per commit, message prefixed with the item ID:
   `fix(driver): W1 — bidir speed profiles (R4, R5)`. Put the item's ground-truth citation
   in the commit body.
4. After completing an item: flip its status-table row, and note any deviations directly
   under the item.
5. Verification: hardware tests deploy via `python scripts/deploy.py tests/<name>.py` (the
   `/deploy` skill). The bench is live — motors bolted down, channel 1 = GPIO 2
   (bidirectional, motor+prop, SMs 0/1 on PIO0), channels 2–4 = GPIO 3/4/5 (TX-only, SMs
   4/5/6 on PIO1). DShot300 only. Offline decode of printed captures:
   `scripts/decode_bidir_capture.py`. Arming needs 3000ms of back-to-back frames.
6. Ground truth: ADR-002 "Implementation Update" (authoritative); AM32 firmware source
   at https://github.com/am32-firmware/AM32
   (`Src/dshot.c` `gcr_encode_table`, `Src/signal.c` `transfercomplete()`); Betaflight
   RP2350 PR #14618 (`src/platform/PICO/dshot.pio`, `dshot_bidir_pico.c`).
   Design constraint (see the maintainer assessment above and CLAUDE.md): supported ESCs
   are BLHeli_S (unidirectional) and AM32 (bidirectional) only. When in doubt about ESC
   behavior, read AM32's source — do not design for, or add configuration surface for,
   other ESC families.
7. Code style (CLAUDE.md): MicroPython, snake_case, no `_`-prefix visibility convention, no
   f-strings on error paths under `driver/`, and never add `_thread` under `driver/`
   (ADR-004).
8. **ADRs must stay self-contained.** When an item has you editing an ADR (or code
   comments), never write references to this review document or its finding IDs (R-numbers,
   A-numbers, W-numbers) into it — this file is a working document and such links go stale,
   costing the ADR part of its context. Either restate the relevant context in place, in
   the ADR's own words, or leave the reference out. The IDs are for this backlog's internal
   bookkeeping and commit messages only.

### Status table

| ID | Title | Findings | Effort | Status |
|---|---|---|---|---|
| W1 | Characterize bidir RX at faster DShot speeds; add profile/guard only if warranted | R4, R5 | M | DONE |
| W2 | Fix GCR table in `specification/DSHOT_PROTOCOL.md` | R11 | S | TODO |
| W3 | Add verification-status table to ADR-002 | R6 | S | TODO |
| — | **Phase 1 gate: docs and API stop overstating what is verified — safe to pause the project here** | — | — | — |
| W4 | Saturated continuous-capture stress harness | R1, R2, R3, TA | M | TODO |
| W5 | RX starvation and recovery characterization | R2, TB | M | TODO |
| — | **Phase 2 gate: transaction failure modes characterized with data, not argument** | — | — | — |
| W6 | Decide the transaction model (ADR) | R1, R3 | M | BLOCKED — needs user decision (present W4/W5 data first) |
| W7 | Implement transaction model + atomic `read_capture()` | R1, R2, R3 | L | TODO |
| W8 | Epoch-clean `start()`/`stop()` + startup ordering | R8, R9, TC | M | DONE |
| — | **Phase 3 gate: continuous transaction engine proven — integration may build on it** | — | — | — |
| W9 | On-Pico eRPM decoder (returns eRPM, not RPM) | R10, R15 | L | TODO |
| W10 | Telemetry health state | R16 | M | TODO |
| W11 | `MotorThrottleGroup` bidir integration + PIO allocator | R7, R13 | L | TODO |
| W12 | Multi-motor simultaneous telemetry test | TD, R7 | M | TODO |
| — | **Phase 4 gate: bidirectional mode promoted to the public API; ADR-002 flips to Accepted** | — | — | — |
| W13 | DShot600 bidir calibration (speed matrix) | R4, TE | L | TODO |
| W14 | Hygiene batch | R12 | S | TODO |
| W15 | ADR-002 accuracy fixes | A1, A2, A3, A6, A7 | S | TODO |
| W16 | Verify MOTOR_POLES against the bench magnetic encoder | R15, A4 | M | TODO |

### Work items

**W1 — Characterize bidir RX at faster DShot speeds; add profile/guard only if warranted (R4, R5)** ·
`driver/dshot_pio.py:190-248`, new spike test, `scripts/decode_bidir_capture.py`

**Reframed 2026-08-25** (see conversation this date): the original framing — add a
`ValueError` in `__init__` for any `dshot_speed != DSHOT300` when `bidirectional=True` —
turned out to be validating a combination nothing in the repo can currently reach. Checked
by grep: every existing bidir call site (`test_bidir_rx_raw.py`, `test_bidir_tx_arm.py`,
`test_bidir_rx_sweep.py`) already hardcodes `DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300`, and
`MotorThrottleGroup` doesn't expose `bidirectional` at all yet (W11). Per CLAUDE.md, adding
validation for a scenario nothing can currently produce isn't warranted on its own.

Reframed as: measure whether the current dense-oversampling RX design (`dshot_bidir_rx`,
`rx_speed=4MHz`, 14-cycle/~3.5µs predelay — see finding A1) actually breaks at a faster
DShot speed, or keeps working, *before* deciding whether a guard/profile table is needed.
Back-of-envelope prediction (not yet a measurement): ADR-002's own pre-implementation 5/4×
bitrate table puts DSHOT600's real GCR bit period at roughly half DSHOT300's measured
~2.5-2.6µs, i.e. ~1.3µs. At the current `rx_speed`, the RX program's effective sample period
is 0.5µs (2 PIO cycles @ 4MHz — see `scripts/decode_bidir_capture.py`'s module comment), so
DSHOT600 would drop from ~5 samples/bit (the design's own stated minimum, "5-6+ samples per
plausible real bit") to ~2.5 samples/bit — plausibly too sparse for `reconstruct_bits()`'s
run-length method to resolve bit boundaries reliably. This item replaces that prediction
with data.

Plan:
1. Before hardware: check AM32 firmware source (`Src/signal.c`, the reply-generation path)
   for whether ESC turnaround-to-reply-start scales with the request DShot rate or is a
   roughly fixed processing delay — this determines whether the 14-cycle predelay needs its
   own retuning at a faster speed, independent of `rx_speed`.
2. Write a standalone spike script (does not touch `driver/` or `DShotPIO`'s public shape)
   that builds channel 1's TX/RX state-machine pair directly at `DSHOT_SPEEDS.DSHOT600`,
   sweeping `rx_speed` across a couple of candidates (current 4MHz as a control, and ~8MHz to
   restore the same oversampling density) while reusing the existing `dshot_bidir_rx` program
   unchanged.
3. Capture raw replies at each `rx_speed` candidate in the same word format
   `test_bidir_rx_raw.py` already prints, and decode them with
   `scripts/decode_bidir_capture.py` — note its `RX_CLOCK_HZ = 4_000_000` constant (line 101)
   needs to vary per capture set; parameterize it for this spike rather than hand-editing.
4. Record CRC-valid rate and measured real bit period per candidate `rx_speed`.

Decision this produces: if 4MHz reliably decodes DSHOT600 too, the guard is unneeded and
this item should close as `SKIPPED` with the data as the reason. If 4MHz garbles/fails and a
different `rx_speed` fixes it, that's direct evidence the guard belongs (it prevents exactly
this silent-wrong-decode failure) — and it also hands W13 a head start on its DSHOT600
profile, so this item would instead close by populating `BIDIR_PROFILES` with both DSHOT300
and DSHOT600 entries plus the `ValueError` for anything not in the table, rather than a bare
guard with no second profile behind it.

**Done when:** the spike's captures are decoded offline, CRC-valid rate and measured bit
period at each tested `rx_speed` are recorded in this document (exploratory — not yet a
verified ADR-002 finding), and the status-table row reflects the resulting decision (either
`SKIPPED (reason)`, or `DONE` with a populated `BIDIR_PROFILES` + guard).

**DONE 2026-08-29.** Ran the spike (`tests/test_bidir_rx_speed_sweep.py`) on hardware, decoded
offline (parameterizing `RX_CLOCK_HZ` per candidate):

| DShot speed | rx_speed | CRC-valid | Measured real bit period |
|---|---|---|---|
| DSHOT600 | 4MHz (old hardcoded default) | 4/8 (50%) | pinned at the decoder's search floor — undersampled |
| DSHOT600 | **8MHz** | **6/6 (100%)** | ~10.1-10.3 cycles → ~1.28µs |
| DSHOT1200 | 4MHz | 2/3, unreliable | same undersampling artifact |
| DSHOT1200 | **8MHz** | **4/4 (100%)** | ~10.3-10.4 cycles → ~1.28-1.29µs |
| DSHOT1200 | 16MHz | 0/8 (0%) | ~20.5 cycles → ~1.28-1.29µs (same as 8MHz!) |

eRPM at each throttle step matched across every *working* config (~21.6k / ~48.2-48.5k /
~75.2-75.9k) regardless of DShot speed — expected, since real motor RPM doesn't depend on
which protocol speed commanded it, and a good cross-check that the decodes are correct, not
coincidentally CRC-passing garbage.

The step 1 firmware check (AM32 `Src/signal.c`) explains the surprising DSHOT1200 result: its
`checkDshot()` only bins detected input rate into **two** reply-timing bands (roughly
150/300 and 600/1200), each with its own fixed `output_timer_prescaler`/`buffer_padding` for
the reply — it does not scale continuously per exact speed. So DSHOT600 and DSHOT1200
produce an *identical* real GCR reply bit period on this ESC, confirmed by the matching
~1.28-1.29µs measurement at both once `rx_speed` was adequate. The 16MHz attempt for
DSHOT1200 wasn't just unnecessary, it actively broke decoding: `dshot_bidir_rx`'s 128-sample
capture window shrinks in wall-clock time as `rx_speed` rises, and at 16MHz that window
(~16µs) fell below the ~27µs real frame duration, truncating every capture before the CRC
bits arrived.

**Amendment 2026-08-29: DSHOT1200 excluded after checking AM32's documented support.** Both
AM32's own README ("Dshot(300, 600) motor protocol support") and wiki.am32.ca ("Compatible
with PWM and BiDirectional DShot300/600 protocols") state bidirectional support for DSHOT300
and DSHOT600 only - DSHOT1200 is never mentioned. `Src/signal.c`'s `checkDshot()` confirms why
the sweep still got clean replies at 1200: it has no distinct DSHOT1200 path, it just bins
detected input rate into two coarse bands with loose pulse-width thresholds, and DSHOT1200's
faster pulses happen to fall inside the same "fast" (~600) band. So the 4/4 CRC-valid result
above is undocumented incidental behavior on this specific ESC/firmware build, not a feature
AM32 tests or guarantees - per this project's AM32-source-is-ground-truth constraint
(CLAUDE.md), that makes it unsupported here too, regardless of it having worked once.

**Outcome: one new profile.** Implemented `BIDIR_PROFILES` in `driver/dshot_pio.py` (module
level, right after `DSHOT_SPEEDS`): `{DSHOT300: 4_000_000, DSHOT600: 8_000_000}` - DSHOT1200
deliberately absent per the amendment above. `DShotPIO.__init__` looks up `dshot_speed` in it
when `bidirectional=True` and raises `ValueError` for anything absent (DSHOT150 - never
measured; DSHOT1200 - measured working but excluded on documentation grounds). Verified:
`tests/test_bidir_rx_raw.py`'s DSHOT300 sweep unaffected (17/17 CRC-valid, matching the prior
baseline); `tests/test_bidir_profile_check.py` confirms the `ValueError` fires for both
DSHOT150 and DSHOT1200, and that DSHOT600 decodes CRC-valid through the real public
`DShotPIO` API (not the spike's hand-built bypass). This gives W13 a head start on DSHOT600
only - its `BIDIR_PROFILES` entry is already validated and populated; DSHOT1200 stays
unsupported, matching W13's original default.

**W2 — Fix GCR table in `specification/DSHOT_PROTOCOL.md` (R11)** · `specification/DSHOT_PROTOCOL.md`
ADR-002 established the spec's GCR symbol table is wrong (agrees with AM32's real
`gcr_encode_table[16]` on only 6 of 16 entries) and records the corrected table, which also
matches Betaflight's `gcrs[]` exactly. Replace the spec's table with the corrected one and
add a provenance note (AM32 `Src/dshot.c`, cross-checked against Betaflight — i.e. the
ecosystem table, not an AM32 quirk). Check the spec's surrounding bidir sections for the
same stale assumptions ADR-002 disproved (22-bit frame, separate seed bit, 30µs turnaround
as universal).
**Done when:** the spec table matches ADR-002's corrected table entry-for-entry, with the
provenance note, and a cross-reference links spec ↔ ADR-002.

**W3 — Add verification-status table to ADR-002 (R6)** · `decision/ADR-002-bidirectional-dshot.md`
Adopt the review's layer table (§6) into ADR-002 near the status header: inverted TX /
detection / capture / GCR / CRC / eRPM = Verified; continuous sync / FIFO management /
multi-motor / lifecycle / API integration / other speeds / fault handling = Not proven or
Not implemented. Future sessions update this table as gates pass; the ADR's Accepted flip
(Phase 4 gate) requires every row resolved.
**Done when:** the table is in ADR-002, the status header points to it, and each row's
claim is consistent with the ADR body.

**W4 — Saturated continuous-capture stress harness (R1, R2, R3, TA)** · new `tests/test_bidir_rx_stress.py`
The harness that makes the P0 findings observable instead of theoretical. After arming, run
≥10,000 frames with no application pacing (back-to-back sends, drain RX every iteration),
counting: frames sent, complete 4-word captures, CRC-valid decodes (this needs at least the
word-grouping done on-Pico; full decode can stay offline on a sampled subset), partial or
misaligned captures, and max observed RX FIFO occupancy. Compare captures-seen against
frames-sent to measure the reply-per-frame association rate. This is characterization —
"Done" is the data, not a pass. Note: sending to 4 channels per iteration paces the loop; a
1-channel variant is the true saturation case. Per finding A5, captures taken during the
arm window are TX echoes by construction (the ESC does not reply until armed) — count them
separately and expect a small number of them to pass CRC as garbage; do not let them
pollute the post-arm statistics.
**Done when:** the harness runs ≥10,000 frames on hardware and its counters are recorded in
ADR-002 (new subsection), whatever they show.

*Note (2026-08-29):* R1/R2's stale-IRQ-4 corruption mechanism is now fixed (see W8) and
verified at small scale (`tests/test_bidir_rx_stall_recovery.py`). This harness's job is
still open — it characterizes sustained/saturated throughput and association rate, which the
small repro doesn't — but it now runs against a driver whose known corruption path is
closed, not the one R1/R2 originally described.

**W5 — RX starvation and recovery characterization (R2, TB)** · extends W4's harness
Deliberately stop draining the RX FIFO for ~5ms mid-run, then resume, repeatedly. Record
what actually happens: does the RX SM stall mid-capture (expected: `autopush` blocks
`in_()`), what the first post-resume captures contain (expected: garbage spanning frames,
possibly TX's own waveform via the stale IRQ-4 flag — see R1/R2 verdict above), and whether
capture validity returns to 100% deterministically or requires a restart. This data is W6's
main input.
**Done when:** observed stall/garbage/recovery behavior is documented in ADR-002 with
counters (captures lost, invalid captures after resume, frames until recovery).

*Note (2026-08-29):* a single-shot, smaller-scale version of exactly this scenario was run
as part of fixing W8 (`tests/test_bidir_rx_stall_recovery.py` — 10 withheld frames, one
withhold/resume cycle, not the "repeatedly" this item calls for) and is recorded there with
before/after CRC-valid rates. That confirmed the fix and is not a substitute for this item's
full repeated-cycle, counter-driven characterization, which W6 still needs.

**W6 — Decide the transaction model (R1, R3)** · ADR (extend ADR-002 or new ADR-005)
**BLOCKED — needs user decision.** With W4/W5 data in hand, choose the synchronization
design. Options to present:
(a) *Lockstep after arm* — TX free-runs during arming (RX drained and discarded); once
armed, the driver never queues frame N+1 until capture N is consumed. Keeps the arm-proven
TX program and cadence untouched; costs peak command rate.
(b) *Sequence/epoch tracking* — TX free-runs; driver stamps captures against a frame
counter and discards unassociable ones. Keeps throughput; association is inferred, not
guaranteed.
(c) *Single-SM transaction program* (Betaflight port) — strongest invariant, but replaces
the arm-verified TX program; ADR-002 already flags this ESC's arming fragility as the
specific risk. Fallback only.
The arm sequence's back-to-back framing requirement is a hard constraint on all options.
**Done when:** the user has picked, and the ADR records the decision, the W4/W5 evidence,
and the rejected options.

**W7 — Implement transaction model + atomic `read_capture()` (R1, R2, R3)** · `driver/dshot_pio.py`
Implement W6's decision. Regardless of option chosen: replace the public single-word
`rx_read()` with `read_capture()` returning a complete 4-word capture (or None), so a
capture is the atomic unit and partial drains can't desynchronize word grouping; keep raw
word access as a diagnostic path. Update `tests/test_bidir_rx_raw.py` /
`tests/test_bidir_rx_sweep.py` to the new API.
**Done when:** the W4 harness re-run shows every capture associated with its frame per the
chosen model's invariant over ≥100,000 frames (target ≥99.9% association, 0 misassociations),
and the W5 starvation scenario recovers deterministically with losses counted, not silent.

**W8 — Epoch-clean `start()`/`stop()` + startup ordering (R8, R9, TC)** · `driver/dshot_pio.py:250-320`
**DONE 2026-08-29.** Re-triaging the backlog surfaced that this bug's mechanism is not
restart-specific: IRQ 4 is a single sticky, block-level flag, so it goes stale exactly the
same way *within a single continuous run* whenever `dshot_bidir_rx`'s `autopush` stalls on a
full RX FIFO (R1/R2 — the same stale flag also drives R8/R9's restart-survival case; both
findings share one fix). Fixed by adding `irq(clear, 4)` at the top of `dshot_bidir_rx`'s
`wrap_target()`, before `wait(1, irq, 4)`, so every iteration — including the first one after
`start()` — blocks for a genuinely fresh release signal instead of a possibly-stale one.
Also added to `start()`: flush the RX FIFO before activating, and activate `rx_sm` before
`sm` (was TX-first).

Verified on hardware with a targeted repro (`tests/test_bidir_rx_stall_recovery.py`):
withhold draining `rx_read()` for 10 frames at settled throttle (enough to fill the 4-word
RX FIFO and stall the SM), then resume and decode offline.
- **Pre-fix:** 19/22 CRC-valid, with 3 consecutive corrupted captures at the stall boundary —
  one measured a 6.0-cycle bit period against a ~10.2-10.3 cycle baseline, i.e. a plausible-
  looking but wrong decode, the exact silent-corruption failure mode R1/R2 describes.
- **Post-fix:** 20/21 CRC-valid, with exactly 1 affected capture — a cleanly truncated,
  obviously-invalid word pattern (correctly rejected, not silently misdecoded).
- **Regression check** (`tests/test_bidir_rx_raw.py`'s standard sweep): 17/17 CRC-valid,
  eRPM still monotonic across throttle steps 100/200/300 (~21.6k / ~48.8k / ~75.6k),
  matching the original ADR-002 baseline exactly.

The original "Done when" (a ≥1,000-cycle `start()`→`stop()` lifecycle stress test) is not
required to close this item: per the same re-triage, no current code path cycles
`start()`/`stop()` at all (every existing test does exactly one of each), so that stress
scenario remains unproven-to-occur rather than a live risk — building it now would repeat
the "build before measuring" pattern already triaged out of W1/W9. Revisit if W11 introduces
real arm/disarm restart cycling in production use.

This also gives W4/W5 a corrected baseline: their "Done when" targets (a saturated
≥10,000/≥100,000-frame harness and repeated starvation characterization with counters) are
still open and not satisfied by this smaller repro, but they now characterize a driver whose
known stale-flag corruption path is closed, rather than one where R1/R2 was still live.

**W9 — On-Pico eRPM decoder (R10, R15)** · new `driver/` module + `scripts/decode_bidir_capture.py`
Port the offline run-length reconstruction decode (marker edge → period estimate →
`reconstruct_bits()` → GCR reverse-lookup → CRC → eRPM) into a driver-side module callable
per capture on the Pico. It returns **eRPM** (plus mantissa/exponent and CRC status) — pole
count and mechanical RPM are the application's business; `MOTOR_POLES = 14` is unverified
(ADR-002) and must not be baked into the driver. Per findings A3/A4, the production decoder
must go beyond the offline script in three ways: accept exactly one CRC polarity (the one
the hardware sweeps produced — recorded by W15), handle the stopped-motor/max-period
sentinel explicitly, and check AM32 source for when Extended DShot Telemetry (EDT) frames
are emitted, discriminating frame types before interpreting the 12-bit field as an eRPM
period — the steady-spin sweeps never exercised either case, and zero-throttle telemetry
has never been tested. Measure decode time per capture on the
RP2350 (plain MicroPython first; `@micropython.viper`/`native` only if measurably needed) —
this number decides how often telemetry can be decoded inline vs sampled. Keep the offline
script as the reference implementation; add a cross-check mode.
**Done when:** an on-hardware sweep decodes live captures with CRC-valid rate matching the
offline decoder on the same run (100% at settled throttle per ADR-002 baselines), on-Pico
results agree with the offline decode of the same printed captures, and per-capture decode
time is measured and recorded.

**W10 — Telemetry health state (R16)** · driver module from W9
Wrap decoded telemetry in explicit health state: `valid`, `erpm`, `timestamp` (ticks),
`age_us`-style accessor, and counters — `frames_received`, `crc_errors`, `frames_missed`,
`consecutive_failures`. Mirrors `update_age_ms()`'s philosophy (report facts, application
sets thresholds). A controller must be able to distinguish "eRPM = 22,000" from "last valid
eRPM = 22,000, 30ms ago". Per finding A5, validity must also gate on arm state: captures
taken before the ESC arms are TX echoes and occasionally pass CRC — a CRC hit alone is not
proof of telemetry.
**Done when:** a deployed test induces loss (starvation from W5's technique, or signal
interruption) and shows the counters and age reflect it correctly while normal operation
shows steady `frames_received` and near-zero errors.

**W11 — `MotorThrottleGroup` bidir integration + PIO allocator (R7, R13)** · `driver/motor_throttle_group.py`
Promote bidirectional mode into the facade. Requires an explicit SM allocator: TX/RX pairs
must share a PIO block (GPIO function-select + IRQ scope, see `DShotPIO.__init__`
docstring), 4 SMs per block, 3 blocks on RP2350 — stop deriving SM id from motor index
(`:107`). Constructor API shape (per-motor bidir flags? group-level? how telemetry is read —
inside `update()` or a separate call?) has user-facing decision points: sketch options and
confirm with the user before building. Define cross-core ownership for the new RX/IRQ/FIFO
state (R13): one core owns PIO lifecycle, the other writes command state; document what
`disarm()` guarantees in bidir mode.
**Done when:** the ADR-002 sweep scenario runs through the public `MotorThrottleGroup` API
(arm → throttle steps → telemetry per motor → disarm) on hardware with the W7-level
association invariant holding, and the allocator rejects impossible placements with clear
errors.

**W12 — Multi-motor simultaneous telemetry test (TD, R7)** · new test
Both bench motors (channel 1 + one more channel made bidirectional) running simultaneously
with telemetry. Measure per-motor CRC-valid rate, FIFO occupancy, and Python-loop headroom;
check for cross-motor interference. Requires wiring the second motor's channel for bidir —
confirm bench state with the user before the hardware session.
**Done when:** ≥10,000 frames per motor simultaneously with per-motor CRC-valid rates
recorded in ADR-002 and no cross-motor misassociation.

**W13 — DShot600 bidir calibration (R4, TE)** · `driver/dshot_pio.py` profiles
*Superseded in part 2026-08-29 by W1.* Characterization never actually needed the Phase 4
gate or the full W4 harness — DSHOT300 itself was originally validated the same way, via bare
`DShotPIO` in a standalone test script, not through the (still unbuilt) transaction engine.
W1's spike did exactly this for DSHOT600 and populated `BIDIR_PROFILES` with a validated
entry (`8_000_000`): measured bit period ~1.28µs, small-scale CRC-valid 6/6, confirmed working
through the real public `DShotPIO` API. DShot1200 was also swept and measured working
(4/4 CRC-valid, same ~1.28-1.29µs reply timing as DShot600, explained by AM32's coarse
2-band rate detection) but was deliberately excluded from `BIDIR_PROFILES`: AM32's own
README and wiki.am32.ca both document bidirectional support for DSHOT300/600 only, so
DShot1200 working here is undocumented incidental behavior, not a supported feature — it
stays unsupported (`ValueError`), matching this item's original default.

**Remaining scope, now genuinely gated on Phase 4/W4/W8:** the *full* W4/W8-level acceptance
bar (≥10,000+ frames, saturated/starvation conditions, association-rate counters) has only
ever been run for DSHOT300 — DSHOT600 has only the small-scale spike data above, not this
larger-scale characterization. That part still needs the W4 harness to exist first.
**Done when:** DShot600 bidir passes the same W4/W8-level acceptance as DSHOT300 (not just
the small-scale spike check); the speed matrix in ADR-002 updated.

**W14 — Hygiene batch (R12)** · `driver/dshot_pio.py:334-337`
The `throttle < 0` guard's message says "Throttle should be greater than 0." — zero is
legal; change to "Throttle must be >= 0." Sweep for other comment/message drift introduced
by the bidir work (e.g. comments still describing `rx_read()` if W7 renamed it). No
functional changes in this diff.
**Done when:** messages match behavior; grep for the old message returns nothing; no
functional diff.

**W15 — ADR-002 accuracy fixes (A1, A2, A3, A6, A7)** · `decision/ADR-002-bidirectional-dshot.md`, `driver/dshot_pio.py` comments, `tests/test_bidir_rx_raw.py` header
Doc-only; no dependencies — may be taken at any point, and fits naturally alongside
Phase 1. Findings are argued in full in the "ADR-002 review sweep" section above; the
fixes: (A1) correct every "~4.7µs" predelay claim to the verified fact — 14 RX cycles ≈
3.5µs at the current 4MHz `rx_speed`, 4.7µs was the superseded 3MHz design's figure —
in the ADR's Timing Coordination note, `dshot_bidir_rx`'s comment, and the test header;
(A2) add the inline supersession flag to the "8-cycles/bit confirmed correct all along /
5/4 multiplier ... should not be touched" claim; (A3, doc half) complete the "eRPM
Decoding" pseudocode with the reply-CRC validation formula and expected polarity, and
record which polarity the hardware sweeps actually hit (rerun
`scripts/decode_bidir_capture.py` — it prints the polarity per capture); (A6) reconcile
the "7 of 16" vs "6 of 16" GCR-table agreement counts (coordinate with W2); (A7) mark
"Recommendation: Bluejay" superseded by the ESC design constraint. Per preamble rule 8:
none of these edits may cite this review or its finding IDs inside the ADR — each
correction carries its full context in the ADR's own words (the finding texts above have
everything needed to restate in place).
**Done when:** each claim is corrected or flagged in place; a grep for "4.7" across
`driver/`, `tests/`, and `decision/` finds only historical mentions that name the 3MHz
context; the recorded CRC polarity matches the decoder's actual output.

**W16 — Verify MOTOR_POLES against the bench magnetic encoder (R15, A4)** · `scripts/decode_bidir_capture.py:100`, ADR-002 "eRPM Decoding"
No backlog dependencies (usable with the existing offline pipeline before W9). The bench
has a magnetic encoder (CLAUDE.md's sensor list); `MOTOR_POLES = 14` is an unverified
EEPROM-default guess, and the ADR's original guess was 12 — every RPM figure in ADR-002
carries this as a constant scale factor. Run a throttle sweep capturing eRPM telemetry and
encoder-measured mechanical RPM simultaneously (or in matched runs at settled throttle);
the eRPM/RPM ratio is the pole-pair count directly. Confirm with the user which motor the
encoder reads and that it is channel 1 before the hardware session. This also validates the
entire telemetry chain end-to-end against an independent sensor — the strongest single
check available on this bench.
**Done when:** the measured ratio pins the pole count to an integer consistently across
≥3 throttle levels; `MOTOR_POLES` is corrected (or confirmed) with provenance in the
script and ADR-002, and the ADR's "unverified" caveats are resolved.
