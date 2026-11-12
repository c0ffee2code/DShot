# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Writes a SAMPLED record of a scenario run (see tests/bench/dshot_protocol/run_scenario.py) to a
# session folder (named by the PC) on the PicoBell Adalogger's SD card. The SD mount and session-folder
# lifecycle are in capture_sink.py; this adds the sample record format. Nothing is written per reply:
# the run writes one record each time it decodes a capture (every decode_every-th per motor), so the
# card sees about 40 small writes a second at most instead of one for every pass of the poll loop
# (I/O on Core 0 costs the command loop; every wire edge is recorded by the wingman instead).
# The scenario's own JSON is not copied here: the PC that runs the session keeps it.

import struct

from capture_sink import CaptureSinkBase
from dshot_profiles import frame_rx_speed

FLAG_FAILED = 0x80      # in a record's flags byte: the decode of the triggering motor's capture failed
FLAG_MOTOR_MASK = 0x03  # in a record's flags byte: which motor triggered the sample


class BidirCaptureSink(CaptureSinkBase):
    # One record per sampled decode: ticks_us (when Core 0 decoded it), flags (FLAG_* above), throttle0..3, then
    # capture0..3 - each motor's latest capture seen so far, as MotorGroup.raw_telemetry() handed it out: the
    # 21-bit reply frame with the 9-bit publish token above it (0 = none seen yet, and any non-bidirectional
    # motor). The token lets the PC tell a fresh capture from a repeated one and find the reply on the wire.
    RECORD_FMT = "<IB4H4I"

    def init_session(self, scenario, update_interval_us=None):
        """Create the run directory (named by the PC) and open the capture log."""
        bidir_indices = ",".join(str(i) for i in scenario.bidir_indices)
        rx_clock_hz = frame_rx_speed(scenario.dshot_speed) if scenario.bidir_indices else 0

        meta = {
            "dshot_speed": str(scenario.dshot_speed),
            "rx_clock_hz": str(rx_clock_hz),
            "record_fmt": self.RECORD_FMT,
            "capture_kind": "sampled",
            "sample_every": str(scenario.decode_every),
            "bidir_motor_indices": bidir_indices,
            "outcome": "running",
        }
        # The period update() ran at. A session without this key was recorded before 2026-10-19, on
        # the old unpaced loop, and the analysis scripts label it that way instead of guessing.
        if update_interval_us is not None:
            meta["update_interval_us"] = str(update_interval_us)
        # What each spinning motor physically is, so a session's eRPM can be read
        # back as shaft RPM without the scenario file at hand.
        for index, motor in enumerate(scenario.motors):
            hardware = motor.hardware
            if hardware is not None:
                prefix = "motor" + str(index) + "_"
                meta[prefix + "name"] = hardware.name
                meta[prefix + "kv"] = str(hardware.kv)
                meta[prefix + "magnets"] = str(hardware.magnets)
                meta[prefix + "propeller"] = hardware.propeller
        self.create_session_dir(meta)

        self.open_capture()

    def finalize(self, outcome, capture_passes, samples_written, samples_failed, missed, largest_gap_us, published, tallies,
                 extra=None):
        """Record how the run ended and its final device-side stats in meta.txt.

        `missed` is the number of captures the group published that the run
        never saw, and `published` maps each bidirectional motor's index to the number of
        captures it published since ARMED (counted from the token steps), so the analyzer can relate the
        samples in capture.bin to what the ESC actually sent. `tallies` maps
        each bidirectional motor's index to the DecodeTally of the captures the
        run decoded on the device, which the analyzer repeats and compares.
        `outcome` is "completed" or "aborted: <reason>". Facts only: the session
        verdict is made on the PC (decision/ADR-008).
        """
        fields = {
            "capture_passes": str(capture_passes),
            "samples_written": str(samples_written),
            "samples_failed": str(samples_failed),
            "captures_missed": str(missed),
            "largest_gap_us": str(largest_gap_us),
        }
        for index in published:
            fields["motor" + str(index) + "_captures_published"] = str(published[index])
        for index in tallies:
            tally = tallies[index]
            prefix = "motor" + str(index) + "_decode_"
            fields[prefix + "sampled"] = str(tally.sampled)
            fields[prefix + "crc_ok"] = str(tally.crc_ok)
            fields[prefix + "crc_fail"] = str(tally.crc_fail)
            fields[prefix + "invalid"] = str(tally.invalid)
        if extra:
            for key in extra:
                fields[key] = str(extra[key])
        self.finalize_meta(outcome, fields)

    def write_sample(self, ticks_us, motor, failed, throttles, captures):
        """Write one sample record to capture.bin: `motor` is the motor whose capture was just decoded, `failed`
        whether that decode failed, `throttles` the 4 motors' throttle values and `captures` the 4 motors' latest
        capture values (see RECORD_FMT). A plain file write: it reaches the card when FatFS's sector buffer fills."""
        struct.pack_into(self.RECORD_FMT, self.pack_buf, 0, ticks_us, motor | (FLAG_FAILED if failed else 0),
                         throttles[0], throttles[1], throttles[2], throttles[3],
                         captures[0], captures[1], captures[2], captures[3])
        self.file.write(self.pack_buf)
