# driver/dshot_profiles.py: the DShot speeds and the tuned reply-receiver profile
# for each. These are constants, and the checks are about how they relate to each
# other and to the protocol, so that retuning one number cannot silently leave
# another inconsistent.

import unittest

import fakes  # noqa: F401  puts driver/ on sys.path
from dshot_profiles import BIDIR_PROFILES, DSHOT_SPEEDS, FRAME_CYCLES_PER_BIT, frame_rx_speed

BIT_RATE = {DSHOT_SPEEDS.DSHOT300: 300_000, DSHOT_SPEEDS.DSHOT600: 600_000}


class DShotSpeedsTest(unittest.TestCase):
    def test_a_speed_is_its_bit_rate_times_8_cycles_per_bit(self):
        self.assertEqual(DSHOT_SPEEDS.DSHOT300, 300_000 * 8)
        self.assertEqual(DSHOT_SPEEDS.DSHOT600, 600_000 * 8)


class BidirProfilesTest(unittest.TestCase):
    def test_both_supported_speeds_have_a_profile(self):
        self.assertEqual(sorted(BIDIR_PROFILES), sorted(BIT_RATE))

    def test_profiles_carry_the_fields_the_driver_reads(self):
        for speed, profile in BIDIR_PROFILES.items():
            self.assertEqual(sorted(profile), ["expected_ratio", "ratio_tolerance", "rx_speed"], speed)

    def test_the_reply_runs_at_five_quarters_of_the_command_rate_within_a_few_percent(self):
        # AM32 answers at 5/4 of the DShot bit rate; oscillators run a few percent off
        for speed, profile in BIDIR_PROFILES.items():
            reply_rate = profile["rx_speed"] / profile["expected_ratio"]
            nominal = BIT_RATE[speed] * 5 / 4
            self.assertAlmostEqual(reply_rate / nominal, 1.0, delta=0.05, msg=speed)

    def test_the_receiver_clock_gives_about_nine_cycles_per_reply_bit(self):
        for profile in BIDIR_PROFILES.values():
            self.assertTrue(8.5 < profile["expected_ratio"] < 9.0, profile["expected_ratio"])

    def test_the_bare_divisor_path_is_used(self):
        for profile in BIDIR_PROFILES.values():
            self.assertEqual(profile["ratio_tolerance"], 0.0)


class RunLengthReceiverClockTest(unittest.TestCase):
    def test_the_clock_is_sixteen_cycles_per_reply_bit(self):
        for speed, profile in BIDIR_PROFILES.items():
            reply_rate = profile["rx_speed"] / profile["expected_ratio"]
            self.assertAlmostEqual(frame_rx_speed(speed) / reply_rate, FRAME_CYCLES_PER_BIT, places=3)

    def test_known_clocks(self):
        self.assertEqual(frame_rx_speed(DSHOT_SPEEDS.DSHOT300), 6_201_978)
        self.assertEqual(frame_rx_speed(DSHOT_SPEEDS.DSHOT600), 12_395_414)

    def test_the_clock_fits_the_pio_divider_range(self):
        # the PIO clock divides the 150 MHz system clock by at least 1 and less than 65536
        for speed in BIDIR_PROFILES:
            self.assertTrue(150_000_000 / 65536 < frame_rx_speed(speed) <= 150_000_000)


if __name__ == "__main__":
    unittest.main()
