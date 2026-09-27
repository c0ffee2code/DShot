# MotorGroup: construction checks, lifecycle, throttle handling and the
# telemetry accessors, against fake state machines and a hand-moved clock.
#
# The lifecycle checks are about what the group transmits and when: nothing
# while disarmed, literal zeros for the whole arming window, the set throttles
# once armed, zeros again on disarm, then silence. On real hardware a group that
# kept writing to a deactivated state machine would block its caller forever;
# that is the reason for most of the "inert" checks here.

import unittest

import fakes
from fakes import Clock, Pin
from dshot_pio import (BidirectionalDShot, UnidirectionalDShot, UnsupportedOperationException,
                       DSHOT_SPEEDS, dshot_bidir_rx_frame)
from dshot_profiles import frame_rx_speed
from motor_group import (MotorGroup, MotorGroupException,
                                  DISARMED, ARMING, ARMED)

SPEED = DSHOT_SPEEDS.DSHOT300
ARM_MS = 200


def uni(sm, pin):
    return UnidirectionalDShot(sm, Pin(pin), SPEED)


def bidir(sm, pin):
    return BidirectionalDShot(sm, Pin(pin), SPEED, rx_state_machine_id=sm + 1)


def sent_zero(word):
    """True for a frame that commands throttle 0, whichever CRC polarity."""
    return word in (0x00000000, 0x000F0000)


class GroupTestCase(unittest.TestCase):
    def setUp(self):
        Clock.reset()

    def make(self, motors):
        return MotorGroup(motors)

    def arm_fully(self, group):
        group.arm(ARM_MS)
        self.run_until_armed(group)

    def run_until_armed(self, group):
        for _ in range(ARM_MS * 2):
            group.update()
            if group.is_armed():
                return
            Clock.advance_ms(1)
        self.fail("group never armed")


class ConstructionTest(GroupTestCase):
    def test_zero_and_five_motors_are_rejected(self):
        with self.assertRaises(MotorGroupException):
            self.make([])
        with self.assertRaises(MotorGroupException):
            self.make([uni(0, 6), uni(1, 7), uni(2, 8), uni(3, 9), uni(4, 10)])

    def test_one_to_four_motors_are_accepted(self):
        for count in range(1, 5):
            group = self.make([uni(i, 6 + i) for i in range(count)])
            self.assertEqual(group.motor_count, count)

    def test_two_motors_on_one_state_machine_are_rejected(self):
        with self.assertRaises(MotorGroupException):
            self.make([uni(0, 6), uni(0, 7)])

    def test_a_state_machine_used_as_another_motors_receiver_is_rejected(self):
        with self.assertRaises(MotorGroupException):
            self.make([uni(0, 6), bidir(2, 7), uni(3, 8)])

    def test_two_motors_on_one_pin_are_rejected(self):
        with self.assertRaises(MotorGroupException):
            self.make([uni(0, 6), uni(2, 6)])

    def test_state_machines_stay_inactive_until_armed(self):
        group = self.make([uni(0, 6), bidir(2, 7)])
        for motor in group.motors:
            self.assertFalse(motor.sm.is_active)
        self.assertEqual(group.state, DISARMED)


class ArmingTest(GroupTestCase):
    def test_update_is_inert_while_disarmed(self):
        group = self.make([uni(0, 6), bidir(2, 7)])
        group.update()
        for motor in group.motors:
            self.assertEqual(motor.sm.sent, [])

    def test_arm_starts_the_motors_and_zeroes_the_throttles(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_throttle(0, 500)
        group.arm(ARM_MS)
        self.assertEqual(group.state, ARMING)
        self.assertTrue(group.is_arming() and not group.is_armed())
        self.assertEqual(group.get_all_throttles(), [0, 0])
        for motor in group.motors:
            self.assertTrue(motor.sm.is_active)

    def test_arm_while_arming_or_armed_is_rejected(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        with self.assertRaises(MotorGroupException):
            group.arm(ARM_MS)
        self.run_until_armed(group)
        with self.assertRaises(MotorGroupException):
            group.arm(ARM_MS)

    def test_arming_window_sends_literal_zeros_even_if_a_throttle_is_set(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        group.set_throttle(0, 700)
        for _ in range(10):
            group.update()
            Clock.advance_ms(1)
        self.assertEqual(len(group.motors[0].sm.sent), 10)
        self.assertTrue(all(sent_zero(w) for w in group.motors[0].sm.sent))

    def test_armed_only_after_the_window_has_elapsed(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        for _ in range(ARM_MS - 1):
            group.update()
            Clock.advance_ms(1)
        self.assertFalse(group.is_armed())
        group.update()
        Clock.advance_ms(1)
        group.update()
        self.assertTrue(group.is_armed())

    def test_a_gap_in_updates_restarts_the_arming_window(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        for _ in range(ARM_MS - 20):
            group.update()
            Clock.advance_ms(1)
        Clock.advance_ms(group.ARM_GAP_TOLERANCE_MS + 5)
        group.update()  # the gap: the window starts over from here
        for _ in range(ARM_MS - 20):
            group.update()
            Clock.advance_ms(1)
        self.assertFalse(group.is_armed())
        self.run_until_armed(group)

    def test_set_throttles_are_transmitted_once_armed(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        self.arm_fully(group)
        group.set_throttle(0, 100)
        group.set_throttle(1, 250)
        for motor in group.motors:
            del motor.sm.sent[:]
        group.update()
        self.assertEqual(group.motors[0].sm.sent, [0x0C840000])
        self.assertEqual(group.motors[1].sm.sent, [(500 << 4 | ((500 ^ (500 >> 4) ^ (500 >> 8)) & 0xF)) << 16])


class UpdateOrderTest(GroupTestCase):
    """Every bidirectional motor is drained before any command is sent."""

    def logged_group(self):
        log = []
        motors = [bidir(0, 6), uni(2, 7), bidir(4, 8), uni(6, 9)]
        for i, motor in enumerate(motors):
            def send(throttle, i=i, real=motor.send_throttle_command):
                log.append(("send", i))
                real(throttle)
            motor.send_throttle_command = send
            if motor.bidirectional:
                def drain(publish, i=i, real=motor.drain_rx):
                    log.append(("drain", i, publish))
                    real(publish)
                motor.drain_rx = drain
        return self.make(motors), log

    def assert_drains_come_first(self, log, publish):
        self.assertEqual(log, [("drain", 0, publish), ("drain", 2, publish),
                               ("send", 0), ("send", 1), ("send", 2), ("send", 3)])

    def test_while_arming(self):
        group, log = self.logged_group()
        group.arm(ARM_MS)
        group.update()
        self.assert_drains_come_first(log, False)

    def test_while_armed(self):
        group, log = self.logged_group()
        self.arm_fully(group)
        del log[:]
        group.update()
        self.assert_drains_come_first(log, True)


class DisarmTest(GroupTestCase):
    def test_disarm_transmits_zeros_then_deactivates(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        self.arm_fully(group)
        group.set_throttle(0, 900)
        group.update()
        for motor in group.motors:
            del motor.sm.sent[:]
        group.disarm()
        for motor in group.motors:
            self.assertEqual(len(motor.sm.sent), group.DISARM_FRAMES)
            self.assertTrue(all(sent_zero(w) for w in motor.sm.sent))
            self.assertFalse(motor.sm.is_active)
        self.assertEqual(group.state, DISARMED)
        self.assertEqual(group.get_all_throttles(), [0, 0])

    def test_disarm_is_idempotent_and_sends_nothing_the_second_time(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.disarm()
        sent = len(group.motors[0].sm.sent)
        group.disarm()
        self.assertEqual(len(group.motors[0].sm.sent), sent)

    def test_a_redundant_disarm_does_not_stop_the_motors_again(self):
        # stop() is only meaningful on a state machine that was actually live -
        # a repeat disarm() (or one before ever arming) has nothing to stop.
        group = self.make([uni(0, 6), bidir(2, 8)])
        self.arm_fully(group)
        group.disarm()
        restarts = [motor.sm.restarts for motor in group.motors]
        rx_restarts = group.motors[1].rx_sm.restarts
        group.disarm()
        self.assertEqual([motor.sm.restarts for motor in group.motors], restarts)
        self.assertEqual(group.motors[1].rx_sm.restarts, rx_restarts)

    def test_disarm_before_arming_does_not_stop_the_motors(self):
        group = self.make([uni(0, 6), bidir(2, 8)])
        group.disarm()
        self.assertEqual([motor.sm.restarts for motor in group.motors], [0, 0])
        self.assertEqual(group.motors[1].rx_sm.restarts, 0)

    def test_disarm_before_arming_sends_nothing(self):
        group = self.make([uni(0, 6)])
        group.disarm()
        self.assertEqual(group.motors[0].sm.sent, [])

    def test_update_is_inert_again_after_disarm(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.disarm()
        sent = len(group.motors[0].sm.sent)
        group.update()
        self.assertEqual(len(group.motors[0].sm.sent), sent)

    def test_the_group_can_be_armed_again_after_disarm(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.disarm()
        self.arm_fully(group)
        self.assertTrue(group.is_armed())


class ThrottleTest(GroupTestCase):
    def test_set_throttle_clamps_to_the_valid_range(self):
        group = self.make([uni(0, 6)])
        group.set_throttle(0, -5)
        self.assertEqual(group.get_all_throttles(), [0])
        group.set_throttle(0, 99999)
        self.assertEqual(group.get_all_throttles(), [group.MAX_THROTTLE])

    def test_set_throttle_rejects_a_bad_index(self):
        group = self.make([uni(0, 6)])
        for index in (-1, 1):
            with self.assertRaises(MotorGroupException):
                group.set_throttle(index, 10)

    def test_set_all_throttles_needs_one_value_per_motor(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_all_throttles([10, 3000])
        self.assertEqual(group.get_all_throttles(), [10, group.MAX_THROTTLE])
        with self.assertRaises(MotorGroupException):
            group.set_all_throttles([1])

    def test_update_age_counts_from_the_last_transmission(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        group.update()
        Clock.advance_ms(7)
        self.assertEqual(group.update_age_ms(), 7)


class TelemetryTest(GroupTestCase):
    # A real DSHOT300 reply from the bench, reconstructed to the frame
    # receiver's own 1-word shape (gcr_decode.reconstruct_frame() on the raw
    # samples this used to be); decodes to eRPM 48859.9.
    CAPTURE = (0xC8BB3,)

    def setUp(self):
        super().setUp()
        self.group = self.make([bidir(0, 6), uni(2, 7)])
        self.motor = self.group.motors[0]

    def test_raw_telemetry_raises_for_a_unidirectional_motor(self):
        with self.assertRaises(UnsupportedOperationException):
            self.group.raw_telemetry(1)

    def test_raw_telemetry_rejects_a_bad_index(self):
        with self.assertRaises(MotorGroupException):
            self.group.raw_telemetry(9)

    def test_no_capture_is_handed_out_while_disarmed_or_arming(self):
        self.assertIsNone(self.group.raw_telemetry(0))
        self.group.arm(ARM_MS)
        for _ in range(20):
            self.motor.rx_sm.feed(self.CAPTURE)
            self.group.update()
            Clock.advance_ms(1)
            self.assertIsNone(self.group.raw_telemetry(0))

    def test_the_group_itself_refuses_captures_while_arming(self):
        # Not only the group's drain discards early replies: a capture that reached
        # the motor's slot by another route is still not handed out before ARMED
        self.group.arm(ARM_MS)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.motor.drain_rx(True)
        self.assertIsNotNone(self.motor.latest_capture())
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_replies_drained_while_arming_are_discarded(self):
        self.group.arm(ARM_MS)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.run_until_armed(self.group)
        self.assertEqual(self.motor.rx_sm.rx_fifo(), 0, "they were taken from the receiver...")
        self.assertIsNone(self.group.raw_telemetry(0), "...but not published")

    def test_publish_while_arming_keeps_arming_replies_on_the_motor_only(self):
        self.group.publish_while_arming = True
        self.group.arm(ARM_MS)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        self.assertEqual(self.motor.latest_capture()[1:], (1, self.CAPTURE),
                         "published to the motor's slot...")
        self.assertIsNone(self.group.raw_telemetry(0), "...but still withheld by the group")
        self.run_until_armed(self.group)
        self.assertEqual(self.group.raw_telemetry(0)[1:], (1, self.CAPTURE))

    def test_a_reply_after_arming_is_handed_out_with_its_sequence(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        ticks_us, sequence, words = self.group.raw_telemetry(0)
        self.assertEqual(sequence, 1)
        self.assertEqual(words, self.CAPTURE)

    def test_the_latest_capture_replaces_earlier_ones(self):
        self.arm_fully(self.group)
        for marker in (1, 2, 3):
            self.motor.rx_sm.feed([marker])
            self.group.update()
        self.assertEqual(self.group.raw_telemetry(0)[1:], (3, (3,)))

    def test_disarm_takes_the_captures_away_again(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        self.assertIsNotNone(self.group.raw_telemetry(0))
        self.group.disarm()
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_a_new_run_starts_without_the_previous_capture(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        self.group.disarm()
        self.group.arm(ARM_MS)
        self.run_until_armed(self.group)
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_decode_telemetry_uses_analyze_frame(self):
        motor = BidirectionalDShot(4, Pin(8), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=5)
        group = self.make([motor])
        result = group.decode_telemetry(0, self.CAPTURE)
        self.assertTrue(result["crc_ok"])
        self.assertAlmostEqual(result["erpm"], 48859.9, places=1)
        self.assertIsNone(result["period_cycles"], "the frame receiver measures no period per capture")

    def test_decode_telemetry_raises_for_a_unidirectional_motor(self):
        with self.assertRaises(UnsupportedOperationException):
            self.group.decode_telemetry(1, self.CAPTURE)

    def test_decode_telemetry_rejects_a_bad_index(self):
        with self.assertRaises(MotorGroupException):
            self.group.decode_telemetry(5, self.CAPTURE)

    def test_every_arm_reinitializes_the_receiver_program_and_jmp_pin(self):
        # start() calls rx_sm.init() on every arm(), the first one included,
        # not only a restart after a stop() - and dshot_bidir_rx_frame fills its
        # PIO block on its own (see BidirectionalDShot's constructor
        # docstring), so it is worth checking both calls get it right.
        self.arm_fully(self.group)
        first_program, first_freq, first_kwargs = self.motor.rx_sm.init_calls[-1]
        self.assertEqual(first_program, dshot_bidir_rx_frame)
        self.assertEqual(first_freq, frame_rx_speed(SPEED))
        self.assertEqual(first_kwargs["jmp_pin"], self.motor.pin)

        self.group.disarm()
        self.group.arm(ARM_MS)
        program, freq, kwargs = self.motor.rx_sm.init_calls[-1]
        self.assertEqual(program, dshot_bidir_rx_frame)
        self.assertEqual(freq, frame_rx_speed(SPEED))
        self.assertEqual(kwargs["jmp_pin"], self.motor.pin)


if __name__ == "__main__":
    unittest.main()
