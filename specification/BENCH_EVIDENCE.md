# Bench evidence behind the operational envelope

Moved out of CLAUDE.md on 2026-10-31 (CLAUDE.md keeps the guarantees and the conventions; this page keeps the
measurements). Text is verbatim; dates and counts are as of when each was written.

## AM32 input match (what our frames look like to AM32)

**Measured on the wire, no ESC, 2026-10-28** (unidirectional frames, 4 lines, 3 runs at DSHOT600 and 2 at DSHOT300, `tests/bench/wingman/tools/spy_am32_table.py`): 0 of 74,752 bits per run that AM32's own threshold would read differently from ours; the 0-bit pulse sits 0.18 us below that threshold at DSHOT600 (0.37 us at DSHOT300) and the 1-bit pulse 0.44 us above it (0.88 us); worst frame length 2.7% off the window AM32 learns (limit 6.25%); no wrong CRC, no command value, largest gap between frames 1.1 ms (limit 500 ms). The readings were identical run to run (they are properties of the PIO waveform; the spy resolves 0.1 us). **Bidirectional, one ESC attached (single-motor scenarios, channel 1, throttle 100, DSHOT600 x2 and DSHOT300 x1, same day):** same input-side figures (0-bit 0.20 / 0.39 us and 1-bit 0.43 / 0.86 us from AM32's threshold at DSHOT600 / DSHOT300), worst frame length 0.2% off the learned window; the ESC's first reply after 104 frames (the source's latch is 101), then 100% of frames answered outside its arming silence, which began 1020 ms after our first frame and lasted 301 ms (the source's 1.02 s gate and 3 x 100 ms tune); 0 replies with a bad CRC; the reply starts 23.2 us (DSHOT600) / 25.8 us (DSHOT300) after our frame; reply bit time 1.284 / 2.565 us (F421 figure to 0.1%, F051/F031 figure -0.6% / -0.7%, E230 and the others excluded: the pinned source's only Skystars KM55 target is an E230 build, which would read 1.403 us, so the ESC is F421 or F051/F031 - the owner's configurator shows which); after the throttle step the ESC sent 6, 9 and 8 replies of data12=2 (30,000,000 eRPM) before settling at 21.8k eRPM (BUG-009). Not yet covered: other motors/channels, several ESCs, throttles other than 100, cold ESC power-on; the tick-based limits default to the F051 capture clock (`--mcu F421|E230` changes it; the 1.02 s arming gate is the same on F051, E230 and F421). Definition: `tests/bench/wingman/tools/spy_check_am32.py`, table: `tests/bench/wingman/tools/spy_am32_table.py`

## Arming: where the bound sits, and the evidence for the rate

**Where the bound sits.** Across the 698 sessions archived on the bench (mostly earlier code),
clean arms took 2.08-2.59 s and arms that needed an ESC reboot took 4.06 s or more, so 3.0 s falls
in the empty gap between the two. That archive justifies the bound, not the 99% rate.

**Evidence for the rate (2026-10-13 and 2026-10-14), arming-only scenario (`two_channel_arming_600.json`,
removed 2026-10-17 - it is in git history), two bidirectional motors, DSHOT600, max-rate pacing (the library default until 2026-10-19; it is now the 1045us paced loop, so this evidence describes the old default and the long batch must run on the new one):**
- 104 of 104 runs passed on the code of those two days (before the one-word capture slot of 2026-10-17; the long batch should run on the current code) (1 on 2026-10-13, then 3 and 100 on
  2026-10-14). Arming time: min 2026 ms, median 2027 ms, max 2516 ms; 99 of the 100 runs landed
  within 2 ms of each other, and the one 2516 ms arm (484 ms of room) is inside the archive's
  historical clean range. Across those 100 sessions the post-ARMED telemetry was 138,720 of 138,720
  records CRC-valid, with no failure streak.
- 30 more of 30 passed on the same driver with an earlier harness (2026-10-13); 10 earlier runs
  used the previous teardown code and are not counted.
- That is not yet enough to support 99%: 100 clean runs support about 97%, and about 300 clean
  runs on the same code are needed, so roughly 195 more. The long batch is waiting for the owner to
  schedule it; with the arming-only scenario gone it would use a spinning scenario such as
  `four_channel_bidirectional_600.json` (arming is judged in every scenario run), which is not the same
  footing as the 104 runs above.
- The captures begin at ARMED (the arming phase is not logged, by design), so an outlier such as the
  2516 ms arm cannot be explained from them.

**Scope of that evidence.** One scenario on one bench: an ESC that was already powered (each run
resets the Pico, not the ESC), DSHOT600, two bidirectional motors, over two days. Not covered: a
cold ESC power-on, DSHOT300, a single motor, other days or conditions.
