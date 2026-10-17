# tests/bench/decode_tally.py: how a run's decoded captures are classified and
# checked against a scenario's thresholds.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))

import fakes  # noqa: E402,F401
from decode_tally import DecodeTally, SpinWatch, MIN_SAMPLES, ERPM_KEPT, is_sampled  # noqa: E402


def good(erpm=21000.0):
    return {"full": 0x1234, "crc_ok": True, "erpm": erpm}


CRC_FAIL = {"full": 0x1234, "crc_ok": False, "erpm": None}
BAD_SYMBOL = {"full": None, "crc_ok": False, "erpm": None}


class SamplingRuleTest(unittest.TestCase):
    def test_every_nth_capture_is_decoded(self):
        self.assertEqual([n for n in range(1, 21) if is_sampled(n, 5)], [5, 10, 15, 20])

    def test_zero_turns_decoding_off(self):
        self.assertFalse(any(is_sampled(n, 0) for n in range(1, 50)))

    def test_one_decodes_everything(self):
        self.assertTrue(all(is_sampled(n, 1) for n in range(1, 50)))


class ClassificationTest(unittest.TestCase):
    def test_each_decode_lands_in_one_class(self):
        tally = DecodeTally()
        for result in (good(), good(), CRC_FAIL, BAD_SYMBOL, None):
            tally.add(result)
        self.assertEqual((tally.sampled, tally.crc_ok, tally.crc_fail, tally.invalid), (5, 2, 1, 2))
        self.assertEqual(tally.rejected(), 3)

    def test_no_edges_counts_as_invalid_not_as_nothing(self):
        tally = DecodeTally()
        tally.add(None)
        self.assertEqual((tally.sampled, tally.invalid), (1, 1))

    def test_crc_valid_percentage(self):
        tally = DecodeTally()
        for _ in range(3):
            tally.add(good())
        tally.add(CRC_FAIL)
        self.assertEqual(tally.crc_valid_pct(), 75.0)

    def test_an_empty_tally_is_zero_percent(self):
        self.assertEqual(DecodeTally().crc_valid_pct(), 0.0)


class ErpmTest(unittest.TestCase):
    def test_median(self):
        tally = DecodeTally()
        for erpm in (10.0, 30.0, 20.0, 50.0, 40.0):
            tally.add(good(erpm))
        self.assertEqual(tally.median_erpm(), 30.0)

    def test_a_reply_without_an_erpm_value_is_valid_but_adds_none(self):
        tally = DecodeTally()
        tally.add(good(None))
        self.assertEqual((tally.crc_ok, tally.erpm_count), (1, 0))
        self.assertEqual(tally.median_erpm(), 0.0)

    def test_the_kept_values_never_exceed_the_array_and_every_reply_still_counts(self):
        tally = DecodeTally()
        for _ in range(ERPM_KEPT * 5 + 7):
            tally.add(good())
        self.assertLessEqual(tally.erpm_count, ERPM_KEPT)
        self.assertEqual(tally.crc_ok, ERPM_KEPT * 5 + 7)

    def test_the_median_follows_the_whole_run_not_only_its_start(self):
        # eRPM climbs steadily from 10k to 90k over 20x more replies than the array holds
        tally = DecodeTally()
        total = ERPM_KEPT * 20
        for i in range(total):
            tally.add(good(10000.0 + 80000.0 * i / (total - 1)))
        self.assertAlmostEqual(tally.median_erpm(), 50000.0, delta=2000.0)

    def test_the_kept_values_are_spread_over_the_run(self):
        tally = DecodeTally()
        total = ERPM_KEPT * 8
        for i in range(total):
            tally.add(good(float(i)))
        kept = sorted(tally.erpms[:tally.erpm_count])
        self.assertLess(kept[0], total * 0.05)
        self.assertGreater(kept[-1], total * 0.90)


class CheckTest(unittest.TestCase):
    def filled(self, ok, bad=0, invalid=0, erpm=21000.0):
        tally = DecodeTally()
        for _ in range(ok):
            tally.add(good(erpm))
        for _ in range(bad):
            tally.add(CRC_FAIL)
        for _ in range(invalid):
            tally.add(BAD_SYMBOL)
        return tally

    def test_no_thresholds_means_nothing_to_miss(self):
        self.assertEqual(DecodeTally().check(), [])

    def test_thresholds_met(self):
        self.assertEqual(self.filled(99, bad=1).check(min_crc_valid_pct=98.0, min_median_erpm=10000), [])

    def test_too_much_garbage_is_reported_with_its_kinds(self):
        failures = self.filled(90, bad=6, invalid=4).check(min_crc_valid_pct=98.0)
        self.assertEqual(len(failures), 1)
        self.assertIn("90.0%", failures[0])
        self.assertIn("6 CRC failures", failures[0])
        self.assertIn("4 invalid", failures[0])

    def test_a_motor_that_is_not_turning_is_reported(self):
        failures = self.filled(100, erpm=917.0).check(min_median_erpm=10000)
        self.assertEqual(len(failures), 1)
        self.assertIn("median eRPM=917", failures[0])

    def test_a_threshold_on_a_tiny_sample_is_a_miss(self):
        failures = self.filled(MIN_SAMPLES - 1).check(min_crc_valid_pct=50.0)
        self.assertEqual(len(failures), 1)
        self.assertIn("need " + str(MIN_SAMPLES), failures[0])

    def test_a_sample_of_exactly_the_minimum_is_enough(self):
        self.assertEqual(self.filled(MIN_SAMPLES).check(min_crc_valid_pct=99.0), [])

    def test_summary_names_every_class(self):
        text = self.filled(3, bad=1, invalid=2).summary()
        for part in ("decoded=6", "crc_ok=3", "crc_fail=1", "invalid=2"):
            self.assertIn(part, text)


class SpinWatchTest(unittest.TestCase):
    """A commanded motor must keep decoding a real eRPM; a glitch frame must
    not defeat the check for the rest of the run (BUG-008, BUG-009)."""

    def test_a_motor_that_never_spins_trips_after_the_limit(self):
        watch = SpinWatch(2000)
        for now in range(0, 2001, 100):
            watch.note_throttle(60, now)
            self.assertFalse(watch.tripped(60, now))
        watch.note_throttle(60, 2100)
        self.assertTrue(watch.tripped(60, 2100))

    def test_real_eRPM_keeps_it_from_tripping(self):
        watch = SpinWatch(2000)
        for now in range(0, 20000, 100):
            watch.note_throttle(60, now)
            if now % 1000 == 0:
                watch.note_spin(now)
            self.assertFalse(watch.tripped(60, now))

    def test_one_glitch_frame_restarts_the_clock_but_does_not_end_the_check(self):
        watch = SpinWatch(2000)
        watch.note_spin(500)          # the glitch: CRC-valid, not 'not running'
        self.assertFalse(watch.tripped(60, 2400))
        self.assertTrue(watch.tripped(60, 2600))

    def test_an_idle_motor_never_trips(self):
        watch = SpinWatch(2000)
        for now in range(0, 60000, 500):
            watch.note_throttle(0, now)
            self.assertFalse(watch.tripped(0, now))
        self.assertEqual(watch.stuck_ms(0, 60000), 0)

    def test_the_clock_starts_when_the_motor_is_first_commanded(self):
        watch = SpinWatch(2000)
        for now in range(0, 10000, 100):
            watch.note_throttle(0, now)
        watch.note_throttle(60, 10000)
        self.assertFalse(watch.tripped(60, 11900))
        self.assertTrue(watch.tripped(60, 12100))

    def test_a_motor_commanded_back_to_zero_and_up_again_gets_a_fresh_clock(self):
        watch = SpinWatch(2000)
        watch.note_throttle(60, 0)
        watch.note_throttle(0, 1500)
        watch.note_throttle(60, 1600)
        self.assertFalse(watch.tripped(60, 3000))
        self.assertTrue(watch.tripped(60, 3700))


if __name__ == "__main__":
    unittest.main()
