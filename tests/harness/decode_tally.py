# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Tallies how the telemetry captures a run decoded, for one motor, so a scenario
# can say how much garbage it tolerates. run_scenario.py decodes every Nth new
# capture on the device - the decode a real application would do on Core 0, in
# MicroPython - and feeds each result here; scripts/analyze_bidir_capture_log.py
# replays the same sampling rule over the logged captures on a PC and compares.
#
# A decoded capture lands in exactly one of three classes:
#   crc_ok    - GCR symbols and CRC both valid: a real reply.
#   crc_fail  - decoded to GCR symbols but the CRC did not match (bit errors).
#   invalid   - not a reply at all: no edges, or a 5-bit group outside the GCR
#               code (a torn read, noise, or the transmitter's own echo).
# The split matters: crc_fail points at the line, invalid at the receiver or at
# what reached it.
#
# Pure Python (array only), so tests/unit exercises it on a PC as well.

from array import array

# Fewer decoded captures than this cannot support a percentage: a threshold on
# an almost-empty sample would pass or fail by luck
MIN_SAMPLES = 20

ERPM_KEPT = 256


def is_sampled(count, decode_every):
    """True when the `count`-th (1-based) capture seen for a motor is decoded."""
    return decode_every > 0 and count % decode_every == 0


class DecodeTally:
    def __init__(self):
        self.sampled = 0
        self.crc_ok = 0
        self.crc_fail = 0
        self.invalid = 0
        # A preallocated array rather than a growing list: a long run must not
        # fragment the heap the command loop's garbage collector works on. It
        # holds an even spread over the whole run, not just its start: when full,
        # every second value is dropped and only every stride-th value is kept
        # from then on, so the median is not the median of the first seconds.
        self.erpms = array('f', [0.0] * ERPM_KEPT)
        self.erpm_count = 0
        self.erpm_seen = 0
        self.erpm_stride = 1

    def add(self, result):
        """Record one decode. `result` is what MotorGroup.decode_telemetry() returned."""
        self.sampled += 1
        if result is None or result["full"] is None:
            self.invalid += 1
        elif not result["crc_ok"]:
            self.crc_fail += 1
        else:
            self.crc_ok += 1
            erpm = result["erpm"]
            if erpm is not None:
                if self.erpm_seen % self.erpm_stride == 0:
                    if self.erpm_count == ERPM_KEPT:
                        half = ERPM_KEPT // 2
                        for i in range(half):
                            self.erpms[i] = self.erpms[2 * i]
                        self.erpm_count = half
                        self.erpm_stride *= 2
                    if self.erpm_seen % self.erpm_stride == 0:
                        self.erpms[self.erpm_count] = erpm
                        self.erpm_count += 1
                self.erpm_seen += 1

    def rejected(self):
        return self.crc_fail + self.invalid

    def crc_valid_pct(self):
        return 100.0 * self.crc_ok / self.sampled if self.sampled else 0.0

    def median_erpm(self):
        if not self.erpm_count:
            return 0.0
        values = sorted(self.erpms[:self.erpm_count])
        return values[self.erpm_count // 2]

    def check(self, min_crc_valid_pct=None, min_median_erpm=None):
        """
        The thresholds this tally misses, as a list of messages (empty when met).
        A threshold given on a sample smaller than MIN_SAMPLES is itself a miss.
        """
        failures = []
        if min_crc_valid_pct is None and min_median_erpm is None:
            return failures
        if self.sampled < MIN_SAMPLES:
            failures.append("only " + str(self.sampled) + " captures decoded, need " + str(MIN_SAMPLES))
            return failures
        if min_crc_valid_pct is not None and self.crc_valid_pct() < min_crc_valid_pct:
            failures.append("crc_valid=" + str(round(self.crc_valid_pct(), 1)) + "% < min_crc_valid_pct=" +
                            str(min_crc_valid_pct) + "% (" + str(self.crc_fail) + " CRC failures, " +
                            str(self.invalid) + " invalid)")
        if min_median_erpm is not None and self.median_erpm() < min_median_erpm:
            failures.append("median eRPM=" + str(round(self.median_erpm())) + " < min_median_erpm=" +
                            str(min_median_erpm))
        return failures

    def summary(self):
        return ("decoded=" + str(self.sampled) + " crc_ok=" + str(self.crc_ok) +
                " crc_fail=" + str(self.crc_fail) + " invalid=" + str(self.invalid) +
                " median_erpm=" + str(round(self.median_erpm())))
