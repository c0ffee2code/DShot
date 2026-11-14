# The 32-bit word send_frame() puts on the TX FIFO, and the checks
# motors make when they are built.
#
# The word is the 16-bit DShot packet (11-bit value, telemetry bit 0, 4-bit
# CRC) in the top half; StateMachine.put(value, 16) does the shift. This checks
# the packet against an independent restatement of the DShot formula and a few
# literal values, in both CRC polarities (bidirectional DShot sends the CRC
# inverted). That the C-side put() really shifts into the top half is not
# something a PC can show: a wrong shift would keep every ESC from arming, so
# every scenario in tests/bench/dshot_protocol/ fails on it.

import unittest

import fakes
from fakes import Pin, CallLog
from dshot_pio import BidirectionalDShot, UnidirectionalDShot, DSHOT_SPEEDS, MAX_THROTTLE


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


class SendFrameTest(unittest.TestCase):
    def send(self, motor, value):
        motor.send_frame(value)
        return motor.sm.sent[-1]

    def test_literal_values(self):
        self.assertEqual(self.send(uni(), 0), 0x00000000)
        self.assertEqual(self.send(bidir(), 0), 0x000F0000)
        self.assertEqual(self.send(uni(), 100), 0x0C840000)
        self.assertEqual(self.send(bidir(), 100), 0x0C8B0000)

    def test_every_value_matches_the_formula_in_both_polarities(self):
        for inverted, motor in ((False, uni()), (True, bidir())):
            for throttle in range(MAX_THROTTLE + 1):
                self.assertEqual(self.send(motor, throttle), expected_word(throttle, inverted),
                                 "throttle %d inverted %s" % (throttle, inverted))

    def test_the_two_polarities_differ_in_the_low_four_bits_by_xor_0xf(self):
        plain, inverted = uni(), bidir()
        for throttle in range(MAX_THROTTLE + 1):
            self.assertEqual(self.send(plain, throttle) ^ self.send(inverted, throttle), 0xF << 16,
                             "throttle %d" % throttle)

    def test_the_telemetry_request_bit_is_never_set(self):
        motor = bidir()
        for throttle in (0, 1, 99, MAX_THROTTLE):
            self.assertEqual((self.send(motor, throttle) >> 20) & 1, 0)

    def test_no_value_is_refused(self):
        # Which values are valid is MotorGroup's concern: commands (1-47) and
        # every throttle up to the field's limit go out as given.
        motor = uni()
        for value in (1, 47, 48, MAX_THROTTLE):
            self.assertEqual(self.send(motor, value), expected_word(value, inverted=False))


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

    def test_activate_and_deactivate_drive_both_state_machines(self):
        motor = bidir()
        motor.activate()
        self.assertTrue(motor.sm.is_active and motor.rx_sm.is_active)
        motor.deactivate()
        self.assertFalse(motor.sm.is_active or motor.rx_sm.is_active)

    def test_activate_flushes_stale_receiver_words(self):
        motor = bidir()
        motor.rx_sm.feed([1, 2, 3])
        motor.activate()
        self.assertEqual(motor.rx_sm.rx_fifo(), 0)

    def test_a_motor_activated_again_after_deactivate_still_sends(self):
        # deactivate() then activate() again must leave the motor working, not just inactive
        motor = bidir()
        motor.activate()
        motor.send_frame(50)
        motor.deactivate()
        motor.activate()
        self.assertTrue(motor.sm.is_active and motor.rx_sm.is_active)
        motor.send_frame(75)
        self.assertEqual(motor.sm.sent[-1], expected_word(75, inverted=True))

    def test_deactivate_turns_both_state_machines_off_before_driving_pin_low(self):
        # A later refactor could reorder this and still pass every other
        # test - active(False) must happen before the pin is handed to SIO,
        # or deactivate() would fight its own state machines for the pin.
        motor = bidir()
        motor.activate()
        CallLog.reset()
        motor.deactivate()
        events = CallLog.events
        tx_off = events.index(("sm", motor.state_machine_id, "active", False))
        rx_off = events.index(("sm", motor.rx_state_machine_id, "active", False))
        pin_low = events.index(("pin", motor.pin.id, "init", (Pin.OUT,), {"value": 0}))
        self.assertLess(tx_off, pin_low)
        self.assertLess(rx_off, pin_low)

    def test_activate_reclaims_pin_and_state_machines_before_activating(self):
        motor = bidir()
        motor.activate()
        motor.deactivate()
        CallLog.reset()
        motor.activate()
        events = CallLog.events
        pullup = events.index(("pin", motor.pin.id, "init", (Pin.IN, Pin.PULL_UP), {}))
        tx_init = events.index(("sm", motor.state_machine_id, "init"))
        rx_init = events.index(("sm", motor.rx_state_machine_id, "init"))
        tx_on = events.index(("sm", motor.state_machine_id, "active", True))
        rx_on = events.index(("sm", motor.rx_state_machine_id, "active", True))
        self.assertLess(pullup, tx_init)
        self.assertLess(pullup, rx_init)
        self.assertLess(tx_init, tx_on)
        self.assertLess(rx_init, rx_on)

    def test_unidirectional_deactivate_resets_the_state_machine_then_drives_the_pin_low(self):
        motor = uni()
        motor.activate()
        CallLog.reset()
        motor.deactivate()
        events = CallLog.events
        off = events.index(("sm", motor.state_machine_id, "active", False))
        init = events.index(("sm", motor.state_machine_id, "init"))
        pin_low = events.index(("pin", motor.pin.id, "init", (Pin.OUT,), {"value": 0}))
        self.assertLess(off, init)
        self.assertLess(init, pin_low)

    def test_deactivate_leaves_nothing_queued_to_go_out_at_the_next_activate(self):
        for motor in (uni(), bidir()):
            with self.subTest(type(motor).__name__):
                motor.activate()
                motor.send_frame(200)
                motor.send_frame(200)
                self.assertTrue(motor.sm.pending)
                motor.deactivate()
                self.assertEqual(motor.sm.pending, [])
                motor.activate()
                self.assertEqual(motor.sm.pending, [])

    def test_deactivate_transmits_nothing(self):
        for motor in (uni(), bidir()):
            with self.subTest(type(motor).__name__):
                motor.activate()
                motor.send_frame(200)
                sent = list(motor.sm.sent)
                motor.deactivate()
                self.assertEqual(motor.sm.sent, sent)

    def test_unidirectional_activate_after_deactivate_reclaims_the_pin_first(self):
        motor = uni()
        motor.activate()
        motor.deactivate()
        CallLog.reset()
        motor.activate()
        events = CallLog.events
        init = events.index(("sm", motor.state_machine_id, "init"))
        on = events.index(("sm", motor.state_machine_id, "active", True))
        self.assertLess(init, on)

    def test_bidirectional_deactivate_discards_a_stored_capture_and_receive_words(self):
        motor = bidir()
        motor.activate()
        motor.rx_sm.feed([0x1234])
        motor.drain_rx()
        self.assertIsNotNone(motor.latest_capture())
        motor.rx_sm.feed([1, 2])
        motor.deactivate()
        self.assertEqual(motor.rx_sm.rx_fifo(), 0)
        self.assertIsNone(motor.latest_capture())

    def test_motors_report_their_state_machines_and_their_mailbox(self):
        self.assertEqual(uni().state_machine_ids, (0,))
        self.assertIsNone(uni().mailbox)
        motor = bidir()
        self.assertEqual(motor.state_machine_ids, (0, 1))
        self.assertIsNotNone(motor.mailbox)

    def test_each_motor_holds_the_value_its_crc_is_xored_with(self):
        self.assertEqual(uni().crc_xor, 0x0)
        self.assertEqual(bidir().crc_xor, 0xF)

    def test_the_zero_packet_is_the_crc_of_an_all_zero_payload(self):
        for motor in (uni(), bidir()):
            self.assertEqual(motor.zero_packet, motor.crc_xor)
            motor.send_zero()
            motor.send_frame(0)
            self.assertEqual(motor.sm.sent[0], motor.sm.sent[1])


if __name__ == "__main__":
    unittest.main()
