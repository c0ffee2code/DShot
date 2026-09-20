# The 32-bit word send_throttle_command() puts on the TX FIFO, and the checks
# motors make when they are built.
#
# The word is the 16-bit DShot packet (11-bit throttle, telemetry bit 0, 4-bit
# CRC) in the top half; StateMachine.put(value, 16) does the shift. This checks
# the packet against an independent restatement of the DShot formula and a few
# literal values, in both CRC polarities (bidirectional DShot sends the CRC
# inverted). That the C-side put() really shifts into the top half is not
# something a PC can show: a wrong shift would keep every ESC from arming, so
# every scenario in tests/harness/ fails on it.

import unittest

import fakes
from fakes import Pin
from dshot_pio import (BidirectionalDShot, UnidirectionalDShot, InvalidThrottleException,
                       UnsupportedOperationException, DSHOT_SPEEDS, MAX_THROTTLE)


def expected_word(throttle, inverted):
    """The DShot frame for `throttle`: SSSSSSSSSSSTCCCC in the top 16 bits."""
    value = throttle << 1  # telemetry request bit = 0
    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF
    if inverted:
        crc ^= 0xF
    return ((value << 4) | crc) << 16


def uni():
    return UnidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300)


def bidir():
    return BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1)


class SendThrottleCommandTest(unittest.TestCase):
    def send(self, motor, throttle):
        motor.send_throttle_command(throttle)
        return motor.sm.sent[-1]

    def test_literal_values(self):
        self.assertEqual(self.send(uni(), 0), 0x00000000)
        self.assertEqual(self.send(bidir(), 0), 0x000F0000)
        self.assertEqual(self.send(uni(), 100), 0x0C840000)
        self.assertEqual(self.send(bidir(), 100), 0x0C8B0000)

    def test_every_throttle_matches_the_formula_in_both_polarities(self):
        for inverted, motor in ((False, uni()), (True, bidir())):
            for throttle in range(MAX_THROTTLE + 1):
                self.assertEqual(self.send(motor, throttle), expected_word(throttle, inverted),
                                 "throttle %d inverted %s" % (throttle, inverted))

    def test_the_telemetry_request_bit_is_never_set(self):
        motor = bidir()
        for throttle in (0, 1, 999, MAX_THROTTLE):
            self.assertEqual((self.send(motor, throttle) >> 20) & 1, 0)

    def test_out_of_range_throttle_is_rejected_and_sends_nothing(self):
        motor = uni()
        for bad in (-1, MAX_THROTTLE + 1):
            with self.assertRaises(InvalidThrottleException):
                motor.send_throttle_command(bad)
        self.assertEqual(motor.sm.sent, [])


class MotorConstructionTest(unittest.TestCase):
    def test_bidirectional_motor_needs_its_receiver_id(self):
        with self.assertRaises(ValueError):
            BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300)

    def test_receiver_must_be_transmitter_id_plus_one(self):
        with self.assertRaises(ValueError):
            BidirectionalDShot(0, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=2)

    def test_receiver_must_share_the_transmitters_pio_block(self):
        with self.assertRaises(ValueError):
            BidirectionalDShot(3, Pin(6), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=4)

    def test_speed_without_a_bidirectional_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            BidirectionalDShot(0, Pin(6), 1_200_000, rx_state_machine_id=1)

    def test_state_machines_are_created_inactive(self):
        motor = bidir()
        self.assertFalse(motor.sm.is_active)
        self.assertFalse(motor.rx_sm.is_active)

    def test_start_and_stop_drive_both_state_machines(self):
        motor = bidir()
        motor.start()
        self.assertTrue(motor.sm.is_active and motor.rx_sm.is_active)
        motor.stop()
        self.assertFalse(motor.sm.is_active or motor.rx_sm.is_active)

    def test_start_flushes_stale_receiver_words(self):
        motor = bidir()
        motor.rx_sm.feed([1, 2, 3])
        motor.start()
        self.assertEqual(motor.rx_sm.rx_fifo(), 0)

    def test_unidirectional_motor_has_no_telemetry(self):
        motor = uni()
        for call in (motor.rx_read, motor.latest_capture, lambda: motor.decode_capture((0, 0, 0, 0))):
            with self.assertRaises(UnsupportedOperationException):
                call()

    def test_kinds_are_told_apart_by_the_bidirectional_flag(self):
        self.assertFalse(uni().bidirectional)
        self.assertTrue(bidir().bidirectional)


if __name__ == "__main__":
    unittest.main()
