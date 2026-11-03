# BUG-011: Core 0's USB printing and SD-card writes make the command loop's frame spacing uneven

**Status:** OPEN. Measured 2026-10-20 on the Pico with no ESC and no motor. Whether the unevenness affects
an ESC is **not known**; this report records the measurement, not a consequence.

## What was measured

A pin recorder (`tests/bench/pin_sampler.py`: a spare PIO state machine plus DMA reads the four transmit pins
every microsecond while the real command loop runs) shows when each DShot frame leaves the Pico. The set-up is
the production one: four bidirectional motors in a `MotorGroup`, `Core1Runner` at `MotorGroup.UPDATE_INTERVAL_US`,
armed at zero throttle, on unused GPIO 10-13 with nothing attached. `test_core0_effect_on_pacing.py` keeps one
loop running and changes only what Core 0 does, eight 205 ms recordings per mode, four channels each.

| Core 0 does | paced: p1-p99 spread | paced: gaps >25% off | paced: extremes | unpaced: mean | unpaced: spread | unpaced: gaps >50% off |
|---|---|---|---|---|---|---|
| sleeps 1 ms (baseline) | 77 us | 0 | 864-1109 us | 392 us | 98 us | 0 |
| tight loop, no sleep | 62 us | 0 | 980-1206 us | 684 us | 40 us | 0 |
| small allocation per ms | 147 us | 0 | 872-1202 us | 405 us | 138 us | 0 |
| `gc.collect()` every 10 ms | 116 us | 0 | 788-1175 us | 390 us | 107 us | 0 |
| **`print()` over USB every 10 ms** | **482 us** | **93** | 669-1397 us | 407 us | 328 us | **471** |
| **one SD capture record per ms** | **424 us** | **33** | 770-1394 us | 466 us | 325 us | **95** |

"Paced" is the 1045 us loop (mean 1044.4-1044.9 us in every mode). "Unpaced" is the old loop at commit `e7cc8c4`
(`UPDATE_INTERVAL_US = 0`, removed 2026-10-19), run from a worktree. Two earlier runs agree: with SD logging a
paced channel's spacing went from 1023-1065 us (p1-p99) to 848-1265 us, and the unpaced loop's from 518-547 us to
354-646 us.

## What it shows

- Core 0's USB printing and SD writes widen the spacing of the frames Core 1 sends, by hundreds of microseconds,
  in both loops. The paced loop keeps its mean exactly (anchored deadlines) but its single frames move.
- The unpaced loop's frame period depends on Core 0: the same loop measured 392, 448, 500 and 532 us in
  different modes and runs, and 684 us with a spinning Core 0. An unpaced pace was never a stable property.
- Allocation and garbage collection on Core 0 widen the spacing slightly (116-147 us against 77 us), far less
  than USB and SD I/O.
- The scenario runner does both: it writes an SD record for every new capture (about 1 kHz) and prints status
  lines. The start recorder does neither during its window (it prints after the run) and still showed the
  stuck-rotor pattern of BUG-009, so this does not explain BUG-009 by itself.

## Narrowing it down (same day, same loop, paced 1045 us; p1-p99 spread of the frame spacing)

`test_core0_io_options.py` - ways to log and print:

| Core 0 does | spread | gaps >25% off |
|---|---|---|
| sleeps 1 ms (baseline) | 70 us | 0 |
| SD record every ms (`sink.write_record`) | 484 us | 62 |
| the same record packed into RAM only | 343 us | 8 |
| packed into RAM, 4 KB written to the SD file when full | 345 us | 20 |
| 16 KB per write | 305 us | 2 |
| 16 KB per write, flushed each time | 303 us | 6 |
| USB line every 10 ms | 460 us | 85 |
| USB line every 100 ms | 146 us | 20 |
| USB line every second | 138 us | 0 |

`test_core0_operation_cost.py` - one plain operation per pass, then a sleep (two runs, which agree within 5 us):

| Core 0 does per pass | spread |
|---|---|
| sleep 1 ms (baseline) | 70-74 us |
| sleep 200 us | 84 us |
| sleep 5 ms | 155 us |
| `utime.ticks_us()` | 117-123 us |
| two bytearray stores | 104-115 us |
| 20 integer additions | 171-173 us |
| `struct.pack_into` of nine constants | 200-202 us |
| `struct.pack_into` with `ticks_us()` | 221-224 us |

## What the narrowing shows

- The spread grows with how much Python Core 0 executes per pass, before any SD or USB device is involved:
  70 us idle, about 110 us for one or two trivial operations, about 170 us for 20 additions, about 220 us for
  packing one record, about 480 us for the whole SD record call. A tight `pass` loop on Core 0 stays at the
  baseline (54-62 us).
- Batching the SD writes does not help: the writes are not the main contributor; the per-pass record work is.
  The SD write itself adds roughly 140 us on top of packing (484 against 343 us).
- USB printing is harmless at one line per 100 ms or less (138-146 us) and costly at one per 10 ms (460 us).
- A longer sleep does not help either: sleeping 5 ms per pass is wider (155 us) than 1 ms (70 us).
- INFERENCE, not tested: both cores execute the MicroPython interpreter from the same flash cache, so Core 0
  running varied code paths evicts what Core 1 needs, while a tight loop keeps the same few paths hot.
  A test would run the same Core 0 work in a form that does not go through the interpreter (a viper function)
  and compare.

## Not known

- Whether AM32 is sensitive to frame-to-frame jitter of this size after arming. BUG-002 found that extra Core 0
  work raised `update()`'s cost and disturbed arming; this is the same family of effect, now measured on the pins.
- The mechanism by which Core 0 I/O delays Core 1 (shared resource inside the MicroPython port or the chip).
  Not investigated; no claim.
- The effect with an ESC replying: the loop also drains replies, which costs time the test does not include.

## Next steps (all runnable on the Pico without an ESC)

1. Find which part of the SD path matters (the write, the flush, the file system) and whether larger, rarer
   writes (a buffer written every 100 ms) bring the spacing back to the baseline.
2. The same for USB printing: how much status output the runner can print before the spacing changes.
3. Repeat with the real receive drain in the loop by looping the transmit pins back into the receivers, so the
   measurement includes the cost of a reply.
4. If a Core 0 change brings the spread to the baseline, apply it to the scenario runner and re-measure.

## Where the evidence is

`tests/bench/pio_and_cores/test_pin_sampler.py` (the recorder self-test, passed), `test_core0_effect_on_pacing.py`
(the first table), `test_core0_io_options.py` and `test_core0_operation_cost.py` (the narrowing), all driven by
`tests/bench/pacing_probe.py`; `tests/bench/pin_timing.py` and
its PC tests (`tests/pc/test_pin_timing.py`) hold the analysis.
