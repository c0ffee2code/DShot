# BUG-009: Motors sometimes start rough

**Status:** OPEN. One specific telemetry artifact has a candidate mechanism, not yet fully
reconciled with the bench data. The mechanical question (does the motor actually turn unevenly,
separate from telemetry) is untouched.
**Recorded:** 2026-10-10, but present from the start, per the user - confirmed to predate every
code change made that day (W33 performance work, `@micropython.native`, BUG-002's `arm_group()`
fix). None of that day's own code changes are the cause.

## KPI: what a rough start is, and how it is counted (defined 2026-10-18; limits pending)

**Current KPI (re-scoped 2026-11-02, owner).** Channel 4 did not spin up with RPM disks mounted, on either of the owner's two AM32 ESCs, so tests with the disks and
with heavy or draggy props are paused. The KPI is now: **all four motors spin up with Gemfan 2218 3-Blade props.** Two measures stay in scope:

1. **Start success (the headline).** A run succeeds when **all four motors** spin up and hold - a drone with fewer than four motors turning does not fly, so one motor short is
   a failed run (owner, 2026-11-02) - **and the slowest spin-up is within the limit of item 2.** A motor spins up and holds when it reaches 80% of its own settled speed (3 replies
   in a row), the ESC never cuts it (not-running that stays at least 500 ms with the throttle on), its settled speed is within 15% of the median of the four motors in the same run
   (a motor stuck at a low reading is not spun up just because it settled there), and after spin-up at most 1% of its replies fall outside +-35% of its settled speed (a stall without
   a cut is not holding). The unit is the run: the motors start together, so four motor-starts are one trial. Target 99% of runs (about 300 clean runs would support it; the bound is
   the exact one-sided 95% one, the convention of the arming evidence: 100 of 100 -> 97%, 300 of 300 -> 99%). Baseline 2026-11-01: all four spun up and held in 28 of 28 runs (90%
   lower bound), and 26 of 28 runs succeed with the limit applied (93%, 79% lower bound): the two failures are the slow GPIO 6 spin-ups, "Result 3" below.
   Counting: only a real ESC, four bidirectional motors, the KPI props and a 0 -> 215 step on every motor make a KPI run; a run that never reached the step is counted as *not started*
   and reported both ways (the owner decides whether it is a failure); a session whose spy record lost data is an instrument problem, not judged.
   Limits of the measurement: only the ESC's own eRPM (no shaft check while the disks are paused), and back-to-back warm restarts of an already powered ESC in one evening;
   cold power-on, idle gaps and several days are not covered (the first run after a pause armed late and had damaged replies).
2. **Spin-up duration (part of the success test).** Per motor, the time from the first nonzero throttle frame to 80% of its settled speed ("Spin-up" below). **Limit 0.5 s (owner, 2026-11-02, provisional).** The baseline gives
   0.09-0.16 s medians per motor, an ordinary slowest of 0.33 s (GPIO 9) and two outliers at 0.9 and 1.05 s (GPIO 6), which the limit flags; it rests on those two
   outliers, so it is revisited when more runs exist (`start_kpi.SPIN_UP_LIMIT_MS`).

Both are computed from the spy's telemetry (`scripts/start_kpi.py` holds the definition; `run_scenarios.py` prints the totals at the end of a batch, `start_kpi.py` on any folder of
sessions) and are in every session's `summary.json`: a `start` block (`in_kpi_scope`, `all_spun_up`, `not_spun_up_or_cut`,
`slowest_spin_up_ms`, `slowest_gpio`) and `spin_up_ms` per line, with a line in the printed table. A session counts toward the KPI only if the replies came from a real
ESC, four bidirectional motors were commanded, and the scenario's propellers are all Gemfan 2218 3-Blade; otherwise the block says why it does not.

**Paused:** everything that needs RPM disks or the reflective sensor - the optical items below (first turn, settling, jerks, stall) and the RPM-accuracy tests, which were
removed from the codebase 2026-11-02 and are a standing backlog item (W41 in `bidirectional_dshot_review.md`, with the commit to restore them from). The
optical definition below stays as written for when the disks return.


**Goal.** At least **99%** of start attempts are smooth. Same evidence rule as arming: about 300
smooth attempts on the same code support 99%, 100 support about 97%.

**One attempt** = one Pico reset, then: arm, jump straight to throttle 215 on the first pass after
`ARMED`, watch the shaft for 4 s (the same window every time). The recorder runs with the scenario
runner's own mechanics (same Core 1 loop, Core 0 reading every reply every 1 ms and decoding every
20th), so a start is comparable with the protocol baseline. An attempt that does not arm is `excluded`
(the arming KPI owns it); a shaft that never completes a turn is a `fail`.

**Smooth** = all four hold, measured by the optical sensor on the RPM disk (never by telemetry, which
cannot tell a telemetry glitch from a motor that turns unevenly):
1. **Gets going:** the first full turn ends within a time limit.
2. **Settles:** from some moment on every turn stays within +-10% of the final speed, and that moment
   comes within a time limit.
3. **No jerks:** no deceleration below -D or acceleration above +A (RPM/s) after the first turn. "Sharp"
   is set against healthy starts, because spin-up is itself the largest acceleration of the window.
4. **No stall:** no gap between sensor pulses longer than a set time after the first turn.

**Spin-up (from the spy's telemetry, added 2026-11-01).** The time from the first nonzero throttle frame to the motor reaching 80% of its own settled speed
(the median eRPM of its replies 1.5-4 s after the step; 3 replies in a row, so a lone spike does not count). It needs no optical disk and no eRPM written into
the tool, so it holds for any props. `summary.json` carries it per bidirectional line as `spin_up_ms` (with `settled_erpm`), the printed table has a "spin up"
column, and `spy_start_summary.py --table` repeats it. A motor that never settled (cut early, too few real replies) has none, shown as "none". It is judged against the 0.5 s limit
(`start_kpi.SPIN_UP_LIMIT_MS`). It is the wire counterpart of the optical "gets going" item; the 28-run baseline of 2026-11-01 ("Result 3") has medians of
0.09-0.16 s per motor and two GPIO 6 starts at 0.9 and 1.05 s.

**Evidence status (2026-10-19).** Every start recorded so far (the 20-start baselines of 2026-10-18 and all the channel, motor, disk, order and single-channel diagnostics of 2026-10-18/19) ran on the old unpaced command loop, `update()` back to back, which no longer exists (the only mode is the loop paced at 1045us). They are not a baseline for the KPI and no limit may be set from them. The baseline has to be recorded again on the paced loop; `run_start_quality.py` now reports `schedule_behind` per start, and a start where the loop fell behind is excluded. Whether the pacing changes the wild or stalled starts seen on some channels is open: there is no evidence either way.

**Our output at the pin is verified (2026-10-20, Pico only).** `tests/bench/pio_and_cores/test_wire_frames.py`
records the four transmit pins at 10 MHz with the production loop running and decodes every frame from the waveform
alone: all frames have a valid inverted CRC and a clear telemetry bit; the bit period is 1.667 us (min 1.60, max 1.70),
the low part of a 1 bit 1.250 us and of a 0 bit 0.623-0.626 us; the throttle sequence across the arming end, a
0 -> 60 step, steady 60 and a 60 -> 0 step is exactly what was commanded, with no frame of any other value and no
frame missing or doubled where the group leaves its arming path. That was on GPIO 10-13 with nothing attached,
so it says what the Pico puts on a pin, not what an ESC receives. The frame spacing has its own finding (BUG-011).

**Limits not set yet** (first turn, settling, D, A, stall). They come from a baseline of about 20
recorded starts at throttle 215, placed in the gap between healthy and bad starts the way 3.0 s was
placed for arming. If every baseline start is smooth there is no gap: the limits are then the largest
healthy value plus a margin, marked provisional. D and A must sit clearly above `accel_noise`
(the jitter of the steady tail).

**KPI throttle moved from 60 to 215 (owner, 2026-11-01).** Below about 200 the bench's motors did not spin
up reliably (channel 4 with props: held 190-215, quit at 170 and 125; throttle 60 fails on channel 4 even with
props). Every start recorded at throttle 60 is therefore not evidence for this KPI; the baseline is recorded at 215.

**What AM32's source says about low-throttle starts** (read 2026-11-01 from `Src/main.c` at the commit AM32.md
cites; duty runs 0-2000). Source facts:
- Duty is `map(input, 47, 2047, minimum_duty_cycle, 2000)`, about 1 duty unit per throttle unit above 47
  (when `minimum_duty_cycle` is 0). Throttle 60 maps to about 13, throttle 125 to 78, throttle 215 to 168.
- While `zero_crosses` is below 30 (15 with stall protection) the duty is floored at `min_startup_duty` (120 in
  the source's static default; with `startup_power` between 50 and 150 it is `minimum_duty_cycle + startup_power`)
  and capped at `minimum_duty_cycle + 400`. So every throttle up to about 167 asks for the same startup duty (the
  floor); only after 30 zero crossings does the duty drop to the mapped value (throttle 125: 78).
- Stuck-rotor protection counts 22.5 ms intervals with no back-EMF zero crossing. The limit is 100 intervals
  (2.25 s) if the adjusted throttle is below 150, and 10 intervals (0.225 s) at 150 or above. Past the limit
  AM32 cuts the phases and forces input to 0, which then reports "not running".
- The low-RPM duty cap (`throttle_max_at_low_rpm` = 400) is above 215's duty (168), so it does not limit these throttles.
- The ESC's `startup_power` and `minimum_duty_cycle` EEPROM values on this bench are unknown (the Pico cannot read them).

**Hypothesis (marked, not tested): the "starts then quits" shape is the stuck-rotor limit.** Quit after about
2.2-2.3 s at throttle 125 matches 100 x 22.5 ms; quit after about 0.25-0.3 s at 170 matches 10 x 22.5 ms; the
boundary at 150 would also explain why 170 quits sooner than 125 (not monotonic in throttle). The 190+ holds
fit too: once enough zero crossings are seen the counter clears. Confirm or reject with the existing 125 and 170 runs:
the quit time measured from the last real eRPM reply should be about 2.25 s and about 0.225 s. A quit time away from
those rejects it. The "never starts" shape (disks, 1 g props) is not covered by this.

**Expect a slow-down after the first surge to be normal.** The floor above applies during the startup window, so
a deceleration seen in every healthy start may be the firmware's normal profile, not a jerk. Not confirmed; the
baseline decides, and the limits must be set above it.

**Stuck-rotor timing test (prepared 2026-11-01, not yet run).** Question: are the "starts, then quits" runs (throttle 125 quit after about
2.2-2.3 s, 170 after about 0.25-0.3 s, on channel 4 with props) AM32's stuck-rotor protection cutting the motor? Source facts (`Src/main.c`
at the commit AM32.md cites): while `running`, every 22.5 ms interval without a back-EMF zero crossing counts; past the limit the ESC cuts the
phases and reports "not running". The limit is 100 intervals when the adjusted throttle is below 150 and 10 at 150 or above, and the cut comes when the counter *exceeds* it (`>`), so
after 101 intervals (2.27 s) or 11 (0.25 s). In normal (non-3D) mode the adjusted throttle is the DShot value itself (`main.c` L1141 `adjusted_input = newinput;`,
`dshot.c` L102/L131 `newinput = tocheck`, the raw 11-bit value); whether this ESC is in 3D mode is unknown, but the 125 and 170 observations fit normal mode
(in 3D mode the step would sit near DShot 100). So the quit time should step sharply between DShot 149 and 150.

*Test.* The four `experiments/four_channel_bidirectional_600_throttle{125,149,150,170}.json` scenarios (removed from the tree in commit e82c232, recoverable from commit 362b322; all four motors hold that throttle for
10 s), 3 runs each with the props mounted, read per channel from the spy: `python tests/bench/wingman/tools/spy_start_summary.py
captures/<session>/spy.bin --table`, column "back to idle" (ms after the throttle step).

*Prediction, written down before any data:* a channel whose rotor does not catch quits at 2,000-2,700 ms at throttles 125 and 149 (101 x 22.5 ms = 2.27 s
plus the time to its last zero crossing), and at 200-700 ms at 150 and 170 (11 x 22.5 ms = 0.25 s plus the same). The jump between 149 and 150 is the
discriminator. *Rejected* if the 149 and 150 quit times are within 25% of each other, or if a quit lies outside its window (then the adjusted-throttle
mapping or the counter differs from what is assumed; read `Src/dshot.c` next). A channel that does not quit says nothing about the hypothesis.
*Not answered by it:* why the rotor does not catch at those throttles in the first place.

**Result of the stuck-rotor timing test (2026-11-01 21:00-21:05; props: Gemfan 2218 3-Blade on all 4 motors, stated by the owner afterwards; the sessions' own `load` label reads "Gemfan D2.2 3-blade, 1-1.25 g each (assumed same set)", my earlier label, and whether D2.2 and 2218 are the same props was not checked - the scenarios now carry the propeller, so this cannot be missed again).**
9 of the 12 planned runs: the batch stopped after three errors in a row, each the no-spin failsafe stopping channel 4 (a motor commanded to spin with no
real eRPM for 2 s). Channels 1-3 held in all 9 runs (no cut; median eRPM about 27.2k at 125, 32.7-33.1k at 149 and 150, 37.4-38.0k at 170). Channel 4:

| Throttle | Runs | Channel 4: cut that stayed (ms after the step) | Brief 'not running' stretches that recovered |
|---|---|---|---|
| 125 | 2 | 2,320 and 2,403 | none |
| 149 | 2 | none (one run held, one recovered) | 1,088-1,110 ms (22 replies) |
| 150 | 3 | 315 and 274 | 92-114 ms (22 replies) in the run that did not cut |
| 170 | 2 | 272 and 277 | 98-111 ms (14 replies) before the cut of the first run |

*Two readings, both recorded.* (1) By the letter of the rule written above (the first return to 'not running', the "back to idle" column), three values fall
outside their windows: 1,089 ms at 149, 93 ms at 150 and 98 ms at 170, so the prediction would be rejected. (2) Those three are brief 'not running'
stretches of 14-22 replies after which the motor ran again; they are not cuts, and they are too early to be the 22.5 ms counter (11 intervals need about
245 ms at the earliest). The measurement as written down caught them by mistake. `spy_start_summary.py --table` now has a "final cut" column (not running
that stays for at least 500 ms with the throttle on). On that reading all 6 cuts fall in their windows: 2,320 and 2,403 ms at 125 (101 x 22.5 ms = 2,272
ms plus 48-131 ms), 315, 274, 272 and 277 ms at 150 and 170 (11 x 22.5 ms = 248 ms plus 24-67 ms).

*Status.* Consistent with stuck-rotor protection being what cuts channel 4 at 125, 150 and 170. **Not confirmed at the step:** channel 4 produced no
cut at 149 in 2 runs (it started or recovered both times), and a channel that does not cut says nothing about the hypothesis, so whether the limit changes
exactly between 149 and 150 is still open. Channel 4 cut in 6 of the 9 runs, so more 149 runs should produce cuts. Unexplained and not investigated:
the three brief 'not running' stretches (a 22-reply stretch is about one 22.5 ms interval, which may be coincidence).

**Result 2: motor order reversed (2026-11-01 21:13-21:17, temporary scenario copies, not in the repo).** The motor list was reversed so GPIO 9 (channel 4)
is first in the group and GPIO 6 last (each motor keeps its own pin and state machines; the spy confirms GPIO 9 > 8 > 7 > 6 on the wire). 8 runs, 4 at
149 and 4 at 150, props as above; the final-cut definition was fixed before this batch. Read per GPIO (the tables label spy lines, which follow the list
order, so GPIO 9 appears as line 0 / "ch1"):

| Throttle | Runs | GPIO 9 cut that stayed (ms after the step) | GPIO 9 otherwise | GPIO 6, 7, 8 |
|---|---|---|---|---|
| 149 | 4 | **2,473** (1 run) | held in 2 runs; 1 run read about 1k eRPM with a 95 ms dip and then ran | held in all 4 (GPIO 6 one 72 ms dip) |
| 150 | 4 | **277, 260, 269** (3 runs) | held in 1 run | held in all 4 |

*Findings.* (1) **The failing motor is still GPIO 9 although it now starts first** (and was last before): the failure follows the pin/channel, not the
position in the group order. GPIO 6, 7 and 8 held in 8 of 8 runs here and 9 of 9 in the first batch. (2) **The 149/150 step is now observed:** below 150
the cuts that stayed came at 2,320, 2,403 (125, first batch) and 2,473 ms (149); at 150 and above at 260-315 ms (7 cuts: 315, 274, 272, 277 in the first
batch, 277, 260, 269 here). Nothing was cut between 315 and 2,320 ms. All 10 cuts lie in the predicted windows (2,000-2,700 and 200-700 ms), the 149 and
150 times are not within 25% of each other, and 2,473 / 269 is about 9, the ratio of 101 to 11 intervals, so the prediction above is **not rejected** on the
final-cut definition. Limits: only one cut at 149 so far (n=1), the props' identity is assumed, and the three early dips of the first batch (and one of
this one) remain unexplained. (3) GPIO 9 failed (cut) in 4 of these 8 runs and in 6 of 9 in the first batch; at 149 it cut in 1 of 6 runs and at 150 in
5 of 7, so the failure is more frequent at 150 than at 149 in this data (small numbers).

*What the repo cannot say.* Why GPIO 9's rotor fails to catch. Already ruled out by test (2026-10-31): the state machine / PIO block assignment and the
motors themselves (rotated between channels, all work on the other channels, owner), and now the position in the group order. Hardware actions that
remain are yours to choose; the software-testable one left is the throttle profile around the start (a hold at 60 and a ramp, which crosses 150 on the way).

**Result 3: start-health baseline at the KPI throttle 215 (2026-11-01 21:29-21:52, 28 runs, Gemfan 2218 3-Blade on all 4 motors, four-channel bidirectional
scenario, `decode_every` 50, spy recording; telemetry-based, because the optical disks are not mounted).**

- **Held:** in 28 of 28 runs all four motors ran the full 20 s with no cut and no dip that stayed. **The unit is the run, not the motor-start:** the four motors
  start at the same moment from the same power and ESC, so their starts are not independent trials and 112 motor-starts are not 112 pieces of evidence. 28 of 28
  runs support about 90% at 95% confidence (exact one-sided bound; the first write-up said 88% with a different method); per motor it is also 28 of 28, so 90% for each of GPIO 6, 7, 8 and 9 (pooling them would hide the
  weakest). This is not progress toward the 99% promise, which needs about 300 clean runs (CLAUDE.md, Operational Envelope). **GPIO 9, the motor that cut in 5 of 7 runs at 150, never cut at 215.** All 28 runs armed within 3.0 s (27 at 2,112-2,114 ms;
  the first run, after a pause of a few minutes, at 2,635 ms). No schedule slip in any run. `decode_every` 50 on a real ESC: 34 decodes per motor, all CRC ok.
- **Time from the throttle step to 38,000 eRPM (about 80% of the 48,000 these props read at 215), median / slowest:** GPIO 8 89 / 103 ms, GPIO 7 115 / 125 ms,
  GPIO 6 117 / **1,052** ms, GPIO 9 160 / 325 ms. GPIO 9 is consistently the slowest to speed (124-325 ms) but always gets there. **GPIO 6 had two slow starts**
  (895 ms in run 9, 1,052 ms in run 13): about 0.9-1.0 s at the 2-3k eRPM "low reading" and then a jump to about 47k; also 291 ms in run 1. They are not cuts,
  so they pass as "held", and they are exactly what a smooth-start measure has to see. 2 of the 28 runs (7%) contained a start more than 5 times the
  motor's own median, both on GPIO 6. No limit is set from this; the optical KPI limits stay unset.
- **Run 1 fails the session summary** (CRC-valid replies 98.1% on GPIO 8 and 97.7% on GPIO 9, threshold 99%) although the runner passed it: the damaged
  replies are all in the first 0.75 s, before the throttle step, in the run that followed a pause and armed late (2,635 ms). The other 27 sessions pass. Whether the
  scenario's CRC expectation should judge only the commanded phase is open.
- **These props read lower than the earlier set:** median eRPM at 215 is 47.6-48.5k here against 52.3-53.0k in the morning's runs (labelled "Gemfan D2.2"), about
  9% lower, so the earlier set was probably not the same props. The prop name is now recorded in the scenario. `spy_start_summary.NORMAL_ERPM[215]` (52,600) comes
  from the earlier set.
- **Prediction check:** GPIO 6, 7 and 8 started every time and GPIO 9 never failed at 215, as predicted; there was no failure to test the 200-700 ms cut prediction at 215.

**Result 4: KPI characterization batch (2026-11-02 21:02-21:27, 32 runs, same scenario, Gemfan 2218 3-Blade on all 4 motors, throttle 0 -> 215).**
Pre-registered prediction: no cuts, all four motors spin up every run, GPIO 6 slower than 0.5 s in about 1-4 of 32 runs, any other motor slow would be new.
- **All 32 runs armed and passed** (2,111-2,560 ms; median 2,134 ms; the first run, 2,560 ms, is the slowest), no schedule-behind events.
- **All four motors spun up and held in 32 of 32 runs** (exact one-sided 95% lower bound 91.1%). No cuts.
- **Runs fully succeeding (all four spun up, slowest within 0.5 s): 30 of 32 (93.8%, lower bound 81.6%).** The two failures are GPIO 6 again: 1,559 ms (run 20, session
  2026-11-02_21-17-10, the motor sat at about 5% of its settled speed for about 1.5 s) and 596 ms (run 32).
- **Per motor, spin-up median / p90 / max (ms):** GPIO 6 123 / 358 / 1,559; GPIO 7 107 / 117 / 126; GPIO 8 93 / 107 / 128; GPIO 9 156 / 224 / 281.
- **Prediction check:** confirmed. No cuts, GPIO 6 slow in 2 of 32 (inside 1-4), no other motor over the limit. Together with Result 3, GPIO 6 was over the limit in
  4 of 60 runs and no other motor in any (counted from the two batches; the KPI script counts them per batch).
- **Owner-reported:** supply showed about 8 W, 0.75 A, 12.19 V during the hold, fluctuating a bit (not measured by the repo).
- **Open:** why GPIO 6 occasionally starts late. Software-testable: order test with GPIO 6 first and last in the group (if the slow start follows the pin, as the
  GPIO 9 stuck-rotor failures did), and a different state-machine/PIO block assignment for it. The KPI target (>= 99%) is not met: about 94%.

**Result 5: is the slow GPIO 6 start something our frames did? (2026-11-02, wire record only, ESC powered off, no new run).** Question: could MicroPython overhead in
the driver's loop (frame content, spacing or phase) make GPIO 6 start late? The spy recorded every frame of all 60 runs of Results 3 and 4 (240 line-starts, 4 of them
the slow GPIO 6 starts: 596, 896, 1,052 and 1,559 ms). Window per start: 50 ms before the step to the end of its spin-up (at least 300 ms).
- **Nothing separates the 4 slow starts from the 56 normal GPIO 6 starts.** Frame spacing p1 / p99 / max, frames that are not throttle 0 or 215 or have a bad CRC (0 in all),
  reply delay (min 22.7-23.0, max 23.3-25.1 us), time from ARMED to the step (2.1 ms every time) and the number of 30,000,000-eRPM replies after the step (slow: 0, 3, 6, 13;
  normal: 0-20) all lie inside the normal range. The step reached all four lines within 0.1 ms in both slow runs checked.
- **GPIO 6 is the tightest line, not the jittery one:** spacing p99 1,184 us (median over normal runs) against 1,204 (GPIO 7), 1,222 (GPIO 8), 1,240 us (GPIO 9); max 1,254 against 1,268, 1,285, 1,324.
  It is motor 0, called first in each `update()`. Loop jitter therefore does not single out GPIO 6.
- **Result:** in the slow starts the ESC received the same frames, spacing and phase as in normal ones; the difference begins after the wire. The 30,000,000 reading is AM32's
  not-yet-commutated state (the candidate in "What's confirmed" above); its length varies 0-20 replies in normal runs, so it does not predict a slow start either.
- **Not tested:** anything that moves GPIO 6 within the group or onto other state machines; the wire record cannot show the ESC's own state. Prepared for the next powered session
  (temporary files, not in git; they would be run with `run_scenarios.py --scenario <file> --rounds N`): GPIO 6 called last in the group with every pin keeping its own state machines
  (position only), and GPIO 6 on state machines 6/7 with GPIO 9 on 0/1 with the order unchanged (state machine / PIO block only). Prediction: if the slow start follows GPIO 6, neither
  the call order nor the state machine is the cause; if it follows position 0 or the state machine, it is. At a base rate of about 7% each arm needs 50-60 runs to say anything
  (30 runs without a slow start would still happen about 12% of the time by chance).

**Result 6: shape of the slow GPIO 6 starts, and what AM32's start code says (2026-11-02, recorded data and source only, ESC powered off).**
- **Data.** In all four slow starts the motor reports a flat 2,400-2,900 eRPM (about 5% of its settled speed) from about 100 ms after the step until it jumps to full speed within 100-200 ms;
  only the length of the plateau differs (about 0.5, 0.8, 0.95 and 1.4 s). No cut, no "not running" reply. The jump is a mode change, not a slow ramp. Slow starts are not a separate class:
  GPIO 6's spin-up times form a tail (313, 321, 358, 406, 596, 896, 1,052 and 1,559 ms are its eight slowest of 60; the other 52 are below 300 ms), and the slowest ones cluster late in a batch (runs 11 and 14 of the first batch; 2, 20, 21, 23, 28 and 32 of 32 in the second).
  Not separating them: wire features (Result 5), time since the previous run (47-48 s in every case).
- **Source (AM32 `Src/main.c` at commit 55c96847).** A start from rest runs in the polling mode (`old_routine = 1`): zero crossings are found by software polling (`bemfcounter > min_bemf_counts`), the
  duty is clamped to `min_startup_duty`..`startup_max_duty_cycle` while `zero_crosses < 30`, the duty change per ramp step is `max_ramp_startup` while `zero_crosses < 150`, and the ESC changes to interrupt
  mode only once `commutation_interval < polling_mode_changeover`. A plateau flat for up to 1.4 s at 240-290 zero crossings a second passes both zero-crossing counts within 0.6 s, so those two clamps do not
  hold it by themselves.
- **Hypothesis (not tested, marked as such):** the ESC stays in the polling mode until `commutation_interval` falls below the changeover, a threshold the motor crosses at a variable moment. This would
  make the plateau length a property of the motor-ESC start, not of our frames. Confirming or rejecting it needs the ESC's own state (not on the wire) or a source-level model with the ESC's EEPROM settings
  (unknown); neither can be done with the ESC off. Software-testable next, with the ESC powered: the order and state-machine scenarios of Result 5.

**Next powered session (runbook, needs the owner's go: it spins motors; props mounted, ESC powered).** In this order:
1. One four-channel bidirectional run at 215 with `decode_every` 50 (`run_session.py`): expect ARMED about 2,112 ms, about 34 decodes per motor,
   all CRC ok - the first check that Core 0 keeps up with the new setting on a real ESC.
2. The stuck-rotor timing test above: `run_scenarios.py --scenario <each of the four files> --rounds 3 --load "<props>"` (12 runs, interleaved).
3. The start-health baseline at 215 with props: 20 runs of `four_channel_bidirectional_600.json`. The optical disks are not mounted, so the signal is
   the spy's (first real eRPM time, "not running" after the first real eRPM, per channel), not the optical KPI.

**Reported, not judged:** how often the ESC's speed report shows a physically impossible value
(30,000,000 eRPM, `data12=2`) and when; ESC RPM against optical RPM once steady. They show whether
that report quirk goes with rough shaft behaviour or is harmless.

**Scope.** AM32; motor 1 (BETAFPV LAVA 1104, 7200KV) with the RPM disk; throttle 215 from 0 right after
arming; DSHOT600 bidirectional; one start per Pico reset. Not covered: other throttles, motor 2 (no
sensor), cold ESC power-on, DSHOT300, other motors, props or loads.

**Blind spot.** One sensor reading per slot interval (a half turn on the 2-hole disk, about 25 ms at
throttle 60): a disturbance shorter than that is invisible. A disk with more slots sees finer detail.

**Removed 2026-11-02 (owner).** The optical recording test (`tests/bench/start_quality/`: `record_motor_start.py` and 14 start configs) and its KPI runner
(`scripts/run_start_quality.py`, with its test) were deleted: the configs had grown to differ in every aspect and nobody could say what each tested. They are in git
(the commit before this removal) if the optical judging is wanted again; the KPI definition above stands, but nothing records or judges it from the shaft now.
Start health is read from ordinary scenario runs: the spy's spin-up and final cut in each session's `summary.json`. Keep README.md and CLAUDE.md's
"Operational Envelope" tables in agreement once limits exist.

## Before touching this: BUG-005 (now fixed) is why a ramp from 0 cannot be written

*Update 2026-10-13:* `MotorGroup.set_throttle()` now raises for 1-47 and scenario profiles that reach it fail to load, so a ramp from 0 can no longer walk through the command range by accident. A smooth-start experiment must jump from 0 to 48 or above. The text below is the original warning.

**Read `bug-reports/BUG-005-throttle-1-47-sent-as-dshot-commands.md` first.** It is OPEN,
independent of this investigation, and directly relevant: DShot values 1-47 are commands, not
throttle (beep, direction reverse, 3D-mode toggle, **EEPROM save**, programming mode), and this
driver does not clamp them. No current scenario hits it (none ramp from 0 - the lowest hold is 60),
but a "ramp gradually from 0 for a smoother start" experiment walks straight into it, and can
persist changes to the ESC across power cycles. Fix BUG-005, or at minimum clamp any experimental
ramp to 48+, before trying gradual starts here.

## What's confirmed: one specific CRC-valid glitch, reproducible and timed precisely

A standalone tool (`tools/measure_rough_start.py`) that jumps straight to one fixed throttle right
after arming and logs every capture (no on-device sampling) found captures decoding to `data12=2`
(`erpm = 60,000,000/2 = 30,000,000` - physically absurd, yet CRC-valid), byte-for-byte identical
(raw word `0x8bb89`) across every session that hit it (17 of 25 runs at `BASE_THROTTLE=65`, 6/10
without a pause, 11/15 with a 500ms pause added between arming and the first throttle command).

**Timing confirms it's tied to the throttle-crosses-arming-threshold moment, not to the arm event
itself.** Without the pause, every hit landed within ~2.5ms of `armed_ticks_us`. With the 500ms
pause added before the first nonzero throttle command, every hit shifted to 501,451-502,573us after
`armed_ticks_us` - matching the pause almost exactly. That shift is the strongest evidence in this
file: whatever produces `data12=2`, it happens at the moment throttle first goes nonzero, not at
`ARMED`.

**Candidate mechanism, from AM32's own source - re-verified at the pinned commit (`55c96847`).**
A first re-read (below, as "the first correction to this entry") concluded the zero-array story
didn't hold. A second, independent re-read (a side agent) found a gating condition the first read
missed, which reverses that conclusion back toward the original theory. This is a candidate again,
not a settled mechanism - two source reads disagreeing with each other once already is itself a
reason for caution, not confidence:
`commutation_intervals[6]`, the array AM32 averages to compute eRPM, is zero-initialized at boot
(line 432) and written only inside `commutate()` (line 904:
`commutation_intervals[step - 1] = commutation_interval`).

The first correction to this entry (below) worried that `startMotor()`'s one-shot `commutate()`
call would poison the array with `commutation_interval`'s declared initial value (12500, line 570)
instead of 0. **That worry doesn't apply on the normal boot path, because `startMotor()` is
gated off and never runs there at all:**
```c
// line 1205-1213, inside setInput()/the throttle-threshold check
if (input >= 47 + (80 * eepromBuffer.use_sine_start)) {
    if (running == 0) {
        allOff();
        if (!old_routine) {
            startMotor();
        }
        running = 1;
        ...
```
`old_routine` is declared `= 1` (line 458) and is only ever cleared to 0 *inside* `zcfoundroutine()`
- which itself only runs while `running` is already true (its call sites are all gated on
`running`). So on every fresh boot, `old_routine` is still 1 when throttle first crosses the start
threshold, `!old_routine` is false, **`startMotor()` is skipped**, and `running` flips to 1 directly
with `commutation_intervals[]` never touched by anything - genuinely `[0, 0, 0, 0, 0, 0]`. The first
`e_com_time` computed after that (any reply sent before the motor's first real commutation event)
is `(0+0+0+0+0+0+4) >> 1 = 2` - `data12=2`, exactly the anomaly. This also explains the
reproducibility: since every bench run resets the ESC (BUG-002's own well-established finding),
`old_routine` resets to its `=1` default every single run, reproducing the same gated-off path every
time.

This is a candidate explanation, not the best-supported one - it reasons from source alone, not
from a bench re-test designed around this specific claim, and it already reversed once on a second
read. If this gets revisited, the clean test is: does `data12=2` ever appear on a build/config
where `old_routine` would already be 0 or `use_sine_start` is set (changing the threshold and the
gate)? **Stop re-deriving this from source alone - the next real move on this specific question is
a bench test, not another re-read.**

**TX-echo checked, not fully ruled out.** This driver's RX receiver can capture its own TX frame
when the ESC stays silent, and some of those echoes happen to pass CRC by chance (`BUG-008`'s own
documented mechanism, `scripts/classify_reply_timeline.py`'s `EchoModel`). Computed the predicted
echo words directly for throttle 60, 65, 100 and 150 at DSHOT600: `0x8bb89` is not among them at any
of those throttles. That only means the model, as it stands, doesn't predict this word at these
throttles - the model itself has not been independently validated against a known-real echo from
this bench's own history, so this is evidence against the echo explanation, not a ruled-out
verdict. (`driver/gcr_decode.py`'s docstring says the same thing - keep both in agreement if either
changes.)

**On the wire, 2026-10-28: the `data12=2` word is the ESC's own reply, not an echo.** A second Pico recorded the
signal line (every edge, 0.1us resolution, decoded from the waveform alone - CLAUDE.md's "wingman"). Three
single-motor runs, channel 1 on GPIO 6, throttle 0 -> 100, the scenario runner's own mechanics: DSHOT600 twice and
DSHOT300 once. In every run, the reply to the first throttle-100 frame is still the not-running word (0xFFF);
the next replies are `data12 = 0x002` (30,000,000 eRPM) for 6, 9 and 8 consecutive replies (about 6-9 ms), then
`0xD1A` (3,324 eRPM) for 20 or more, then the eRPM climbs to a steady 21,802-21,866 (median). Each of those replies
is a 21-bit GCR frame at the ESC's reply bit time (1.284us at DSHOT600, 2.565us at DSHOT300), CRC-valid, starting
23.1-23.2us (DSHOT600) or 25.8us (DSHOT300) after the end of our frame - the same delay as every ordinary reply,
and not our own 16-bit frame shape - so the echo explanation above is ruled out for these occurrences. The word
also matches the source arithmetic exactly (`e_com_time = (sum of six commutation intervals + 4) >> 1`, all six
still zero, gives 2). What this does not test is *why* the intervals are still zero (the `old_routine` reading
above stays a candidate) or the open question below, why 30 earlier runs at throttle 60 never showed it; this
was throttle 100. The count of such replies after the step differed between runs (6, 9, 8). Recordings are
regenerable with `tests/bench/wingman/tools/wingman_run.py` + the single-channel scenarios; tables: `tests/bench/wingman/tools/spy_am32_table.py`.

## Wire-level findings, 2026-10-28 evening (all four channels with motors, wingman recording)

Single-motor starts, throttle 0 -> 60 right after arming, 6 s, DSHOT600, recorded on the wire by the second Pico
(`scripts/run_listened.py` - removed 2026-10-29, in git at `febda45`; `tests/bench/wingman/tools/spy_start_summary.py`, scenarios `single_channel{1-4}_bidirectional_start60_600.json`).
"Derails" means: between 0.6 s and 4 s after the step the ESC's replies leave the normal reading (about 8,400 eRPM at
throttle 60, 21,800 at 100) for 1% or more of the window, or return to the not-running word.

- **Channel 4 derails; channels 1-3 do not.** 0 of 24 starts on channels 1-3 (15 single-channel at throttle 60, 9 more in
  four-channel runs) derailed; 13 of 17 on channel 4 did (6 of 8 at throttle 60, 2 of 3 at 100, 2 of 3 with its motor on
  the state machines of PIO block 0, 3 of 3 in four-channel runs, where channels 1-3 stayed normal in the same runs).
- **It is a cycle, not a plateau.** A derailing start alternates between about 1,000 eRPM ("low", 45-300 ms) and 23,000-30,000
  ("high", 4-650 ms), over and over. The ESC keeps answering every frame throughout (no silence over 5 ms). In some runs
  the cycling stops by itself after 0.1-1 s and the start continues normally; in two (203924, 204147) the ESC went to
  not-running 2.37-2.40 s after the step, and in others at 0.4-2.4 s.
- **The low phases last whole multiples of 22.5 ms** (measured on the wire, 25 phases): 45 (2.00 x), 90-91 (4.04 x), 114 (5.06 x),
  136-137 (6.04-6.09 x). AM32's back-EMF timeout interval is 22.5 ms (source fact: `INTERVAL_TIMER_COUNT > 45000` at 0.5us); that
  the low phases are that timer counting is a hypothesis, not tested. The high phases do not cluster.
- **Our signal does not differ between channels.** With every ESC attached: bit margins to AM32's threshold (0.18 / 0.44 us),
  frame length, shortest edge interval, reply delay (23.1-23.2 us), arming silence (1019-1022 ms after our first frame, 301 ms
  long), first reply after 104 frames and reply bit time (1.280-1.285 us) are the same on all four channels. Only the start
  differs.
- **Not the first-start path.** `data12=2` (30,000,000 eRPM) runs after the throttle step vary from 0 to 22 replies on every
  channel and do not separate derailing from normal starts (a derailing start with 0 and a normal one with 8 exist). Two starts
  inside one ESC boot (throttle 60 for 2 s, 0 for 1 s, 60 for 3 s): channel 4's second start failed 3 of 3 times (a plateau at 23.3k,
  or stuck at about 1,000 eRPM for the whole 1.5 s) with no `data12=2` at all; channel 1's second start was normal.
- **Not the throttle, not the PIO block, not the other channels.** Throttle 100 and the PIO 0 state machines did not remove it (small
  numbers: 3 runs each). Four-channel runs left channels 1-3 normal.
- **One-off: our frame garbled by the ESC's first reply after its arming silence.** In 2 of 34 recordings (both channel 4), the frame
  right after the ESC's 301 ms arming silence shows 10 or 2 extra edges at ESC reply timing (the first reply overlapped our frame), so our
  frame decodes as a different word (one of them, 0x8007 = throttle 1024, still passes the CRC by chance). The ESC did not start. A reply
  that overlaps a frame is a path by which an armed ESC could be handed a wrong throttle; it needs one more look.
  **Correction (2026-10-31): the "our frame decoded as a different word" reading above came from the analyzer, not from the wire.** The wire analyzer
  took the first 32 edges of each frame-and-reply group as our frame, so an ESC reply sitting just before our frame was read as the first half of it.
  Re-reading 40 four-motor bidirectional recordings (2026-10-30, bare motors, spy on) with a frame finder that locates our frame inside the group
  (`spy_edge_log.frame_offset`): in 10 of 20 bidirectional runs, once, at about 1.33 s (just after the arming silence), one line shows 15-16 edges that
  decode as a CRC-valid "not running" reply, starting about one frame period (1.05 ms) after the frame it would answer and ending 2-8 us before our next
  frame; our frame after it is intact (same bits and CRC as its neighbours) in every case checked (3 sessions by hand, all 10 by the check verdicts). In one
  of them the reply to the following frame starts 1.4 us after our frame and has a bad CRC. Channel 4 in 7 of the 10, channel 3 in 2, channel 2 in 1. No effect
  on arming, spin or outcome in any of the 40. The earlier recordings of this section are gone, so whether their "garbled frame" was the same misreading
  cannot be re-checked; treat that claim as unconfirmed. Why the reply comes late is untested (hypothesis: the ESC's first reply after its arming silence).
  **What in our code allows the overlap (read 2026-10-28):** (1) `dshot_bidir_tx` blocks in `pull()` with the pin released and, the moment a word is
  queued, executes `set(pindirs, 1)` and drives the frame - it never looks at the line first (`driver/dshot_pio.py`, frame_start). (2) The word is queued
  by `MotorGroup.update()` on the fixed 1045 us period (`send_throttle_command` -> `sm.put`), so a frame starts when the schedule says, not when the
  previous reply is over. (3) The only coordination between a transmitter and its receiver is one-way: TX raises `irq(rel(1))` at release and RX waits for it;
  RX never tells TX anything. Nothing in the driver compares what it drove with what the line shows, so an overlap is invisible to it. The design's
  only protection is the gap: a normal reply starts 23 us after our frame and is over within about 50 us, 1 ms before the next frame. In the two recordings
  the ESC's reply to the previous frame arrived about one frame period late (the previous frame has no reply; the extra edges sit at the start of
  the next frame), right after its arming silence. Why the ESC replied late is ESC-side and not explained. A listen-before-talk check cannot go into the TX
  program as it stands: the TX/RX pair fills its PIO block (13 + 19 of 32 words). A check in `update()` (skip the tick if a reply is in progress) is possible
  but costs Core 1 time on the loop whose pacing matters. Whether the ESC would accept such a frame is not known: in both cases it did not start, even
  with a word that passes the CRC (0x8007, throttle 1024).
- **Arming 2.6 s in 2 of 21 starts** (first of a batch, channel 1 and 2): one frame, then 1.9-2.0 s with no frames, then normal. The ESC
  behaved normally relative to our first frame; the extra time is on the main Pico's side, cause not found.

**Start-sequence variations on channel 4 (2026-10-28, a second bench window, 22 runs, scenarios `single_channel{1,4}_bidirectional_start{48,80,150}_600`,
`..._hold48_then60_600`, `..._kick{100,150}_then60_600`):** none of them avoids the cycling. Channel 4 derailed in 18 of 18 runs (jumps to 48, 80 and 150;
48 then 60; a kick of 100 or 150 then 60) and the no-spin failsafe stopped all 18 early (no real eRPM for 2 s: the ESC had gone to not-running). The channel 1
controls at 80, 150 and kick 150 then 60 stayed normal; at throttle 48 (just above AM32's start threshold of 47) channel 1 itself was unstable, so 48 is not
clean evidence. Across both bench windows channel 4 derailed in 31 of 35 starts and channels 1-3 in 0 of 24 (at 60 and in the controls).

**The two readings of a derailing start do not follow the throttle.** On channel 4's derailing starts the "high" reading has a median of 23.4-24.0k eRPM at throttle
48, 60, 80 and 100 alike (24-28k at 150, few samples) and the "low" reading about 1,000 eRPM (950-1,170) at all of them; a normal start follows the command (7.6k at
48, 8.4k at 60, 15.7k at 80, 21.8k at 100, 35k at 150, channel 1). Earlier optical-sensor sessions (2026-10-17..19, in this file's history) measured the shaft at 170-300
RPM during the 23k reading, which is about 3,900 RPM if it were real, so the reported value is not the rotation speed. That the ESC is then commutating open loop
(without sensed zero-crossings) is a hypothesis; the fixed values and the 22.5 ms multiples fit it, a rotation measurement during a derailing start on the wire would
test it.

**History, 2026-10-28 (`scripts/derail_history.py`, 2,732 motor starts in the archived sessions from 2026-09-26 to 2026-10-20, current log format only).**
A start is flagged when 0.6-4 s after its throttle step it shows a low phase (500-3,000 eRPM for 30 ms or more) or a not-running stretch of 100 ms or more with the
throttle on. The throttle differs between scenarios, so this counts a signature, not an eRPM verdict.
- Low phases are almost absent before 2026-10-04: about 1,200 starts on channels 1 and 3 from 09-26 to 10-03 show 2 (both channel 3, 09-27). The 15 + 15 flagged on 09-27
  are not-running stretches without low phases (the not-starting behaviour of that day, not the cycling).
- From 10-04 the low phases appear on **channel 3**: 2 of 149 (10-04), 2 of 80 (10-06), 1 of 115 (10-08), 23 of 211 (10-10, 11%), 3 of 5 (10-13), 2 of 15 (10-17), 2 of 18
  (10-18); channel 1 stays at 0-2 starts per day through 10-18.
- On 10-19, the first day with motors on all four channels: channel 1 12 of 22, channel 4 12 of 18, channel 3 4 of 18, channel 2 0 of 22. On 10-20: channel 1 1 of 4,
  channel 4 1 of 3. Today (10-28, spy recordings): channel 4 31 of 35, channels 1-3 0 of 24.
- So which channel shows it has changed over the weeks (3, then 1 and 4, then 4 only) while channel 2 is clean on every day. What changed on the bench at 10-04, 10-10 and
  10-19 and between 10-20 and 10-28 is a fact the owner holds; it is not inferred here.

**Owner fact, 2026-10-29:** the owner has rotated the motors between the channels and every motor works on the other channels, so the motors are ruled out as the difference (owner's statement, not re-tested here).

**Spy validated, 2026-10-29** (`tests/bench/wingman/SPY_EVIDENCE.md`): loss detectors fire on purpose; known gaps from 1 ms to 3.5 s read back to 0.21 us; 44,522 frames per line over 46.5 s with nothing lost. The test found and fixed two defects (long intervals read 92 ppm low; an edge right after a 3.28 ms marker was lost), neither of which changes a finding in this report. The "one frame, then ~2 s" start did not reproduce in 10 resets with the pacing-probe start sequence on the Pico alone (first frame always 21.6 ms after the start line, no gap over 5 ms). That harness has no reply gate, SD session or scenario loading, while the two cases came through the scenario runner's start path, so the next step is the scenario runner's start, Pico-only.

Open leads: whether the 22.5 ms multiples are the stuck-rotor timer; the owner's AM32 settings of channel 4's ESC against channel 1's
(startup power, sine start, stall protection); what changed on the bench on the dates the affected channel changed (see History above); the MCU target (the reply bit time fits F421 to 0.1% and F051/F031 to 0.5-0.9%). The spy has been attached to every one of
these runs; a few runs with its wires removed would show whether it matters (owner's option).

## What's NOT yet reconciled - do not call this settled

**Zero occurrences in 30 `two_channel_divergent_600.json` runs that set a nonzero throttle (60)
immediately after arming, at the same nominal `poll_ms=1`.** Across 30 full-resolution-checked
sessions, none ever produced `data12=2` (their worst early eRPM reading was 159,151, a different
and smaller-magnitude anomaly - see below). If the AM32 mechanism above is both correct and
genuinely throttle-independent, this is unexplained.

**The "lower effective poll rate" hypothesis is ruled out, by its own numbers, not just untested.**
Early-window record rates were measured directly: ~463/s for `measure_rough_start.py`, ~262/s for
`divergent_600` (56% of that). If the glitch's hit rate scaled proportionally with poll rate,
`measure_rough_start.py`'s observed 68% (17/25) would predict roughly 38% for `divergent_600` - and
observing literally 0 hits in 30 trials at a true 38% rate has probability under 1e-6. A real but
partial effective-poll-rate difference cannot produce a clean 0/30. Something else explains the gap
- possibly a genuine throttle dependence after all, or some other structural difference between the
two scripts not yet identified. **Next session's first task should be resolving this**, starting
with `measure_rough_start.py` at `BASE_THROTTLE=60` (matching `divergent_600`'s own first throttle
exactly) to see whether the hit rate collapses to match `divergent_600`'s 0/30 - that result would
itself argue for a genuine throttle dependence, since the polling-rate explanation is now excluded.

**Separately, `decode_every=20` (the on-device sampling every scenario uses for its pass/fail
verdict) cannot see any of this reliably** - it only decodes 1 in 20 replies, and none of the runs
that hit the `data12=2` spike printed `Test FAILED`. Full-resolution decode (every capture) is the
only check that has caught this so far; treat any scenario "pass" verdict on this motor's eRPM as
uninformative about brief spikes.

**The broader, smaller-magnitude eRPM-variance spikes in `two_channel_divergent_600.json`** (motor
2/channel 3's coefficient of variation up to 142.7%, max eRPM up to 159,151, in 3 of 20 runs from
one batch, spread across roughly the first 2 seconds rather than one single frame) are an open
question, not a confirmed instance of the same mechanism. A 6-slot moving average warming up does
not obviously explain noise spread across 2 full seconds rather than concentrated in the first few
commutation cycles; this needs its own check before assuming it's the same cause.

**That spike batch, and a separate 4-run batch where the same motor never spun at all in
isolation (throttle 65/100/150), are confounded**: the user was changing hardware on channel 3 (a
slotted-disk RPM sensor, an imbalanced prop) during that same window. Don't treat either result as
a clean software/firmware finding until re-run with a known, stable physical setup on that channel.

**Is there real mechanical roughness at all, separate from telemetry artifacts?** Nothing in this
investigation distinguishes "AM32's eRPM reporting is briefly unreliable" from "the motor is
actually turning unevenly." The independent slotted-disk + light-sensor RPM rig (in progress,
independent of DShot telemetry entirely) is the right tool to settle this - hold off on further
DShot-telemetry digging into "is it rough" until that's ready.

## Decided, not open

**The driver will not filter implausible eRPM values.** Considered a value-based floor in the
decoder, a time-based "not yet reliable" window in `MotorGroup`, and leaving it to applications;
the user chose the last: document the behavior (done, in `driver/gcr_decode.py`'s `decode_result()`
docstring) and leave filtering to whatever application needs it. Real applications don't poll eRPM
within milliseconds of arming anyway.

## Starting point for the next session

1. Resolve the echo-vs-AM32 question first (see above): validate `EchoModel` against a known-real
   echo before trusting it either way, and run `measure_rough_start.py` at `BASE_THROTTLE=60`.
2. Fix or work around BUG-005 before any gradual-ramp-from-0 experiment.
3. Re-run the channel-3 isolation and divergent-600 spike batches only once the physical setup on
   that channel is known and stable.
4. A full backup of `captures/` as it stood 2026-10-10 is at `captures Backup/` (sibling directory,
   repo root) - captures themselves are being dropped from git; use this backup, not git history,
   for any historical-capture analysis (e.g. bucketing spike timing by first-nonzero-throttle
   record instead of `armed_ticks_us`, or checking for eRPM sentinel re-entry after a motor has
   genuinely spun).

## Related

- BUG-002 - the arming-reliability investigation this spun off from, now resolved. A different
  problem; its `arm_group()` fix is unrelated to this one.
- BUG-005 - landmine for any fix attempted here (ramping from 0), see above.
- BUG-008 - tracks the harness safety-net gaps; this file's `data12=2` finding was a confirmed
  instance there. Since 2026-10-13 the permanent latch it defeated is replaced by a continuous
  "no real eRPM for 2 s while commanded" check (`decode_tally.SpinWatch`).
