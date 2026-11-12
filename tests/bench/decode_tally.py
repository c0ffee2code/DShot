# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Tallies how the telemetry captures a run decoded, for one motor, as facts: it
# judges nothing (the session verdict is made on the PC from the wire recording,
# decision/ADR-008). run_scenario.py decodes every Nth new capture on the device -
# the decode a real application would do on Core 0, in MicroPython - and feeds each
# result here; scripts/analyze_bidir_capture_log.py replays the same sampling rule
# over the logged captures on a PC and checks that the two decoders agree.
#
# A decoded capture lands in exactly one of three classes:
#   crc_ok    - GCR symbols and CRC both valid: a real reply.
#   crc_fail  - decoded to GCR symbols but the CRC did not match (bit errors).
#   invalid   - not a reply at all: no edges, or a 5-bit group outside the GCR
#               code (a torn read, noise, or the transmitter's own echo).
# The split matters: crc_fail points at the line, invalid at the receiver or at
# what reached it.
#
# Pure Python (array only), so tests/pc exercises it on a PC as well.

from array import array

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

    def summary(self):
        return ("decoded=" + str(self.sampled) + " crc_ok=" + str(self.crc_ok) +
                " crc_fail=" + str(self.crc_fail) + " invalid=" + str(self.invalid) +
                " median_erpm=" + str(round(self.median_erpm())))


class SpinWatch:
    """
    Per-motor "commanded to spin but no real eRPM" timer, for one motor.

    A motor commanded at or above min_throttle must decode a real eRPM (CRC-valid,
    not AM32's not-running sentinel) at least once every limit_ms. The clock
    restarts whenever a real eRPM is decoded or the commanded throttle drops
    below min_throttle, so a motor deliberately idle never trips, and neither
    does one that stops and is restarted. It is a continuous streak, not a
    latch: one stray CRC-valid frame (BUG-009's data12=2) restarts the clock
    once and the motor still trips limit_ms later if it is truly stuck, where
    a "ever seen a real reply" flag would have been defeated for the rest of
    the run (BUG-008).

    Times are milliseconds since the run started; the caller supplies them.
    """

    def __init__(self, limit_ms, min_throttle=48):
        self.limit_ms = limit_ms
        self.min_throttle = min_throttle
        self.since_ms = 0

    def note_throttle(self, throttle, now_ms):
        """Call every pass with the motor's currently commanded throttle."""
        if throttle < self.min_throttle:
            self.since_ms = now_ms

    def note_spin(self, now_ms):
        """Call when a sampled decode is CRC-valid and not 'not running'."""
        self.since_ms = now_ms

    def stuck_ms(self, throttle, now_ms):
        """Milliseconds commanded to spin without a real eRPM; 0 when not commanded."""
        if throttle < self.min_throttle:
            return 0
        return now_ms - self.since_ms

    def tripped(self, throttle, now_ms):
        return self.stuck_ms(throttle, now_ms) > self.limit_ms


# TEMPORARY (owner, 2026-10-20): the over-speed guard is switched off while the plateau reading on some
# channels is investigated, so runs are not ended at the start. The scenario runner and the start
# recorder both read this. Set it to True to bring the guard back; the SpinWatch failsafe and the
# throttle ceiling are not affected.
OVERSPEED_GUARD_ENABLED = False


class OverspeedWatch:
    """
    Per-motor over-speed guard for the bench, for one motor.

    A motor must not report a real eRPM far above what its commanded throttle
    gives. The limit is erpm_per_throttle x the highest throttle commanded in
    the last hold_ms (a motor that was just slowed down is still coasting, so
    the lower throttle does not tighten the limit at once). The default 300
    sits about 1.4x above this bench's slowest healthy case (throttle 60 reads
    about 8,400 eRPM, at most 12,500) and about 1.2x above its fastest (throttle
    300 reads about 75,000). Throttle 0 is never judged.

    `consecutive` decoded readings in a row above the limit trip it, so one
    stray CRC-valid value (BUG-009's single 30,000,000 eRPM reading) does not,
    while a motor that really runs away does, within a few decodes. Times are
    milliseconds since the run started; the caller supplies them.
    """

    def __init__(self, erpm_per_throttle=300, hold_ms=2000, consecutive=5):
        self.erpm_per_throttle = erpm_per_throttle
        self.hold_ms = hold_ms
        self.consecutive = consecutive
        self.peak = 0
        self.peak_ms = 0
        self.run = 0

    def note_throttle(self, throttle, now_ms):
        """Call with the motor's commanded throttle just before note_erpm()."""
        if throttle >= self.peak or now_ms - self.peak_ms > self.hold_ms:
            self.peak = throttle
            self.peak_ms = now_ms

    def limit_erpm(self):
        return self.erpm_per_throttle * self.peak

    def note_erpm(self, erpm):
        """Call for a decode that is CRC-valid, not 'not running' and has an eRPM.
        Returns True once `consecutive` readings in a row exceed the limit."""
        if self.peak > 0 and erpm > self.erpm_per_throttle * self.peak:
            self.run += 1
        else:
            self.run = 0
        return self.run >= self.consecutive

