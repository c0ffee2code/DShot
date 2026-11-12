# ESC mock

The wingman's second role (the first is the spy, `SPY.md`): **replays recorded ESC replies, so the main Pico's reply path runs with
no ESC and no motor.** It tests our side of the link (receiver state machine, FIFO drain, mailbox, Core 0 decoding, the scenario
mechanics). It says nothing about an ESC, a motor, AM32's behaviour or arming on real hardware.

| Piece | File | Runs on |
|-------|------|---------|
| The mock (one PIO state machine and one DMA channel per line, no CPU work per frame) | `board/esc_mock.py` | wingman |
| Fixture format and the word the mock shifts out | `shared/esc_mock_format.py` | wingman and PC |
| The replies (4 lines x 1024 real replies) | `board/esc_mock_replies_dshot600.bin` | uploaded to the wingman |
| Build the fixture from a recorded session | `tools/esc_mock_fixture.py <captures/session>` | PC |
| Launch | `python scripts/run_session.py <scenario.json> --load "..." --esc-mock` (or `tests/bench/wingman/tools/wingman_run.py --esc-mock`) | PC |

## How to run

**The ESC must be disconnected from the signal lines.** The mock drives the scenario's bidirectional motor pins on
the wingman while it answers; a connected ESC would be driven against. Wiring is the wingman's convention (README.md): the same GPIO
numbers on both boards plus a shared ground. `run_session.py --esc-mock` starts the mock and the recorder together on the spy
board (the recorder keeps recording the true wire, so the mock's timing is verified from the recording), runs the
scenario on the main Pico, and resets the wingman afterwards so it cannot keep answering. The session's
`session.json` carries `esc_mock`, and `summary.json` notes that the replies were replayed.

The mock never outlives its run: the launcher stops it in a `finally` and resets the board, and `run_session.py` resets the board
again in its own `finally` for every `--esc-mock` session (a killed launcher runs no cleanup of its own).

## What it does per frame

1. Waits until the line has been high for about 164 us, so only a frame's first falling edge starts it.
2. Samples the 11 throttle bits of the frame (0.9 us after each bit's fall: still low = a 1-bit).
3. All zero -> replies with AM32's "not running" word; otherwise with the next recorded reply (the fixture loops).
4. Starts the reply 48.8 us after the frame's first edge and shifts out the 21 levels at 1.2845 us per bit, driving both
   levels, then releases the line.

## Not like the real ESC

- It answers from the first frame; the real ESC is silent for the first 104 frames and for about 301 ms after its arming
  gate. A mock arming takes about 2.0 s, the real one about 2.11 s.
- The speed is replayed from the fixture, not derived from the throttle; there is no start-up transient.
- The AM32 checks about the real ESC's own behaviour (first reply after the latch, arming silence) are reported as not
  applicable on a mock session; the reply integrity, bit-time and coverage checks stay judged.
- Mock sessions are not evidence for the arming rate or for start quality. They are marked in `session.json`. Their session verdict (`scripts/session_summary.py`) is never `pass`: the arming and spin proofs are `not shown`, so a clean mock session reads `unknown`.

## Fixture provenance

`esc_mock_replies_dshot600.bin` was built on 2026-11-01 from `captures/2026-11-01_11-36-52/spy.bin` (four-channel bidirectional
DSHOT600, throttle 215, Gemfan D2.2 props, git commit `02e08c8`), the first 1024 CRC-valid real-eRPM replies per line
after 5 s: median eRPM 52,632 / 53,004 / 52,265 / 52,632. `captures/` is deleted from time to time, so the fixture is
committed; rebuild it with `tests/bench/wingman/tools/esc_mock_fixture.py` from any session of the same kind.

The same facts are machine-readable in `board/esc_mock_replies_dshot600.json`, written next to the fixture by that tool (source session,
scenario, git commit, load, build date, replies per line, medians). The wingman plan requires it, and every mock session's `session.json`
carries it under `esc_mock.source`, so a mock session says which recorded replies it played.

## Results (2026-11-01, ESC disconnected, GPIO 6-9 + 15, `four_channel_bidirectional_600.json`, `decode_every` 50)

- **Waveform on the true wire** (the recorder, 21,083 replies per line): 0 CRC errors; reply starts 48.7-48.9 us after our
  frame's first edge (the real ESC: 48.5-51.0 us); bit time 1.279-1.290 us, median 1.285 (real: 1.284); a zero-throttle frame
  was always answered with the not-running word and a nonzero one never.
- **First version released the line for the high levels** and let the main Pico's pull-up raise it: the falls landed exactly,
  the rises about 1 us late, short highs vanished, replies decoded as garbage and the group never armed. Driving both
  levels during the reply window fixed it.
- **Main Pico, 5 runs in a row:** all reached ARMED (2003-2004 ms), 34 decodes per motor with 34 CRC ok, median eRPM equal to
  the fixture's, verdict PASS. The longest wait between passes with a new capture was 21.9-25.0 ms. So at `decode_every` 50
  Core 0 keeps up and every motor now has enough decodes to be judged (the old 17 per motor were not). The run does not time a
  single decode.

## Scenario-driven check (2026-11-01, after the wingman started reading the scenario, ESC disconnected, GPIO 6-9 + 15)

- `four_channel_bidirectional_600.json` with the mock: ARMED 2003 ms, 34 decodes per motor, all CRC ok, PASS (same as before).
- A mixed scenario (motor 1 bidirectional, motors 2-4 unidirectional): the mock answered line 1 only; ARMED 2003 ms, 36 decodes on motor 1,
  and the three unidirectional lines were recorded and judged as unidirectional with no replies on them. PASS.
- `four_channel_unidirectional_600.json`, spy only: completed, four lines recorded, PASS.
- Refused before any session folder or hardware was touched: `--esc-mock` with a scenario that has no bidirectional motor, `--esc-mock`
  with `dshot_speed` DSHOT300 (recorded replies exist for DSHOT600 only), `--spy` on a device script without a config.
