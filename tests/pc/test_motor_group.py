# MotorGroup: construction checks, lifecycle, throttle handling and the
# telemetry accessors, against fake state machines and a hand-moved clock.
#
# The lifecycle checks are about what the group transmits and when: nothing
# while inactive, literal zeros for the whole arming window, the set throttles
# once armed, then silence from deactivate() on (it transmits nothing; the ESC's
# own timeout stops the motor - see ADR-004). On real hardware a group that
# kept writing to a deactivated state machine would block its caller forever;
# that is the reason for most of the "inert" checks here.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))

import fakes
from fakes import Clock, Pin
from dshot_pio import (BidirectionalDShot, UnidirectionalDShot, UnsupportedOperationException,
                       DSHOT_SPEEDS, dshot_bidir_rx_frame)
from dshot_profiles import frame_rx_speed
from capture_mailbox import FRAME_BITS, FRAME_MASK
from gcr_decode import AM32_NOT_RUNNING_FRAME
from motor_group import (MotorGroup, MotorGroupException,
                                  INACTIVE, ARMING, ARMED)
from group_without_reply_gate import GroupWithoutReplyGate

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
        """Tick until ARMED, every bidirectional ESC replying "not running" each tick."""
        for _ in range(ARM_MS + group.READY_SPAN_MS + 100):
            for motor in group.bidir_motors:
                motor.rx_sm.feed([AM32_NOT_RUNNING_FRAME])
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
        self.assertEqual(group.state, INACTIVE)


class ArmingTest(GroupTestCase):
    def test_update_is_inert_while_inactive(self):
        group = self.make([uni(0, 6), bidir(2, 7)])
        group.update()
        for motor in group.motors:
            self.assertEqual(motor.sm.sent, [])

    def test_arm_starts_the_motors_and_zeroes_the_throttles(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_throttle(0, 200)
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
        group.set_throttle(0, 200)
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

    def test_a_gap_in_updates_does_not_restart_the_arming_window(self):
        """AM32 does not reset its own arming counter on a gap in our
        transmissions - only non-zero throttle, or a gap long enough to be a
        reboot, does that (specification/AM32.md#16). The floor is plain
        elapsed time since arm(), unaffected by how unevenly update() gets
        called."""
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        for _ in range(ARM_MS - 20):
            group.update()
            Clock.advance_ms(1)
        self.assertFalse(group.is_armed())
        Clock.advance_ms(1000)  # a gap far larger than update() ever leaves in practice
        group.update()
        self.assertTrue(group.is_armed())  # armed on the very next call - the floor kept counting through the gap

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

class ArmingGateTest(GroupTestCase):
    """A group with bidirectional motors arms only once each ESC has been replying
    AM32's not-running frame for READY_SPAN_MS (bug-reports/BUG-003)."""

    def setUp(self):
        super().setUp()
        self.group = self.make([bidir(0, 6), uni(2, 7)])
        self.motor = self.group.motors[0]

    def tick(self, milliseconds, words=(AM32_NOT_RUNNING_FRAME,), motors=None):
        """Advance `milliseconds` 1ms ticks; each, the motors (default: every
        bidirectional one) first receive `words` (none if empty)."""
        motors = self.group.bidir_motors if motors is None else motors
        for _ in range(milliseconds):
            for motor in motors:
                motor.rx_sm.feed(list(words))
            self.group.update()
            Clock.advance_ms(1)

    def test_no_replies_keep_the_group_arming(self):
        self.group.arm(ARM_MS)
        self.tick(ARM_MS + 3 * self.group.READY_SPAN_MS, words=())
        self.assertTrue(self.group.is_arming())
        self.assertEqual(self.group.arming_status(), [None, None])

    def test_armed_after_the_span_of_replies(self):
        span = self.group.READY_SPAN_MS
        self.group.arm(ARM_MS)
        self.tick(span - 1)
        self.assertFalse(self.group.is_armed(), "the floor has passed, the span has not")
        self.tick(2)
        self.assertTrue(self.group.is_armed())

    def test_the_arming_window_is_still_a_floor(self):
        long_window = self.group.READY_SPAN_MS + 1000
        self.group.arm(long_window)
        self.tick(long_window - 1)
        self.assertFalse(self.group.is_armed())
        self.tick(2)
        self.assertTrue(self.group.is_armed())

    def test_the_span_counts_from_the_first_reply(self):
        self.group.arm(ARM_MS)
        self.tick(500, words=())
        self.tick(self.group.READY_SPAN_MS - 1)
        self.assertFalse(self.group.is_armed())
        self.tick(2)
        self.assertTrue(self.group.is_armed())

    def test_an_arming_tune_gap_does_not_restart_the_span(self):
        self.group.arm(ARM_MS)
        self.tick(1000)
        self.tick(300, words=())  # AM32's arming tune: interrupts off, no replies
        self.tick(self.group.READY_SPAN_MS - 1300 + 1)
        self.assertTrue(self.group.is_armed())

    def test_a_reboot_length_gap_restarts_the_span(self):
        self.group.arm(ARM_MS)
        self.tick(1500)
        self.tick(700, words=())  # startup tune plus the bidirectional latch
        self.tick(self.group.READY_SPAN_MS - 1)
        self.assertFalse(self.group.is_armed(), "the first 1500ms of replies no longer count")
        self.tick(2)
        self.assertTrue(self.group.is_armed())

    def test_a_stale_reply_blocks_promotion(self):
        self.group.arm(ARM_MS)
        self.tick(self.group.READY_SPAN_MS - 100)
        self.tick(400, words=())  # past the span, but the ESC has gone quiet
        self.assertFalse(self.group.is_armed())

    def test_one_silent_motor_holds_the_whole_group(self):
        group = self.make([bidir(0, 6), bidir(4, 8)])
        self.group = group
        group.arm(ARM_MS)
        self.tick(ARM_MS + 2 * group.READY_SPAN_MS, motors=[group.motors[0]])
        self.assertTrue(group.is_arming())
        status = group.arming_status()
        self.assertIsNotNone(status[0])
        self.assertIsNone(status[1])

    def test_only_the_not_running_frame_counts(self):
        self.group.arm(ARM_MS)
        self.tick(ARM_MS + 2 * self.group.READY_SPAN_MS, words=(0xC8BB3,))  # a real eRPM reply
        self.tick(ARM_MS + 2 * self.group.READY_SPAN_MS, words=(0,))  # the line held low
        self.assertFalse(self.group.is_armed())

    def test_arming_status_reports_the_run_of_replies(self):
        self.group.arm(ARM_MS)
        self.tick(20, words=())
        self.tick(100)
        self.tick(30, words=())
        replying_for_ms, last_reply_ms_ago = self.group.arming_status()[0]
        self.assertEqual(replying_for_ms, 130)
        self.assertEqual(last_reply_ms_ago, 31)
        self.assertIsNone(self.group.arming_status()[1], "unidirectional")

    def test_a_group_without_the_reply_gate_arms_on_the_window_alone(self):
        self.group = GroupWithoutReplyGate([bidir(0, 6), uni(2, 7)])  # the bench's no-ESC group
        self.group.arm(ARM_MS)
        self.tick(ARM_MS + 1, words=())
        self.assertTrue(self.group.is_armed())

    def test_arm_again_needs_new_replies(self):
        self.arm_fully(self.group)
        self.group.deactivate()
        self.group.arm(ARM_MS)
        self.tick(ARM_MS + self.group.READY_SPAN_MS, words=())
        self.assertFalse(self.group.is_armed())


class UpdateOrderTest(GroupTestCase):
    """Every bidirectional motor is drained before any command is sent."""

    def logged_group(self):
        log = []
        motors = [bidir(0, 6), uni(2, 7), bidir(4, 8), uni(6, 9)]
        for i, motor in enumerate(motors):
            def send(throttle, i=i, real=motor.send_frame):
                log.append(("send", i))
                real(throttle)
            motor.send_frame = send
            # update()'s ARMING branch sends through send_zero(), not
            # send_frame(0) (see dshot_pio.py's own note on why) -
            # patched the same way so "while arming" sees the same log shape
            # as "while armed".
            def send_zero(i=i, real=motor.send_zero):
                log.append(("send", i))
                real()
            motor.send_zero = send_zero
            if motor.mailbox is not None:
                def drain(i=i, real=motor.drain_rx):
                    log.append(("drain", i))
                    return real()
                motor.drain_rx = drain
                def discard(i=i, real=motor.discard_rx):
                    log.append(("discard", i))
                    return real()
                motor.discard_rx = discard
        return self.make(motors), log

    def assert_drains_come_first(self, log, kind):
        self.assertEqual(log, [(kind, 0), (kind, 2),
                               ("send", 0), ("send", 1), ("send", 2), ("send", 3)])

    def test_while_arming(self):
        group, log = self.logged_group()
        group.arm(ARM_MS)
        group.update()
        self.assert_drains_come_first(log, "discard")

    def test_while_armed(self):
        group, log = self.logged_group()
        self.arm_fully(group)
        del log[:]
        group.update()
        self.assert_drains_come_first(log, "drain")


class TxWaitTest(GroupTestCase):
    """update() sends only when every transmitter has taken its previous frame (StateMachine.tx_fifo() is 0)."""

    def test_no_queue_is_asked_while_inactive(self):
        group = self.make([uni(0, 6)])
        group.update()
        self.assertEqual(group.motors[0].sm.tx_polls, 0)

    def test_update_waits_until_every_transmitter_has_taken_its_frame(self):
        group = GroupWithoutReplyGate([bidir(0, 6), bidir(2, 7)])
        self.arm_fully(group)
        first, second = (motor.sm for motor in group.motors)
        second.tx_busy_polls = 3            # the second motor's frame of the last tick stays queued for 3 polls
        polls = second.tx_polls
        before = [len(first.sent), len(second.sent)]
        group.update()
        self.assertEqual(second.tx_polls - polls, 5)     # 3 busy, the one that sees it taken, the empty one
        self.assertEqual([len(first.sent), len(second.sent)], [n + 1 for n in before])

    def test_update_waits_while_arming_too(self):
        group = self.make([uni(0, 6)])
        group.arm(ARM_MS)
        group.update()
        sm = group.motors[0].sm
        sm.tx_busy_polls = 2
        group.update()
        self.assertEqual(sm.tx_polls, 1 + 4)     # first tick: one; second: 2 busy, the one that sees it taken, the empty one
        self.assertEqual(len(sm.sent), 2)

    def test_update_never_queues_a_frame_behind_another(self):
        group = GroupWithoutReplyGate([bidir(0, 6), uni(8, 8)])
        self.arm_fully(group)
        queued_before_put = []
        for motor in group.motors:
            def put(value, shift=0, sm=motor.sm, real=motor.sm.put):
                queued_before_put.append(len(sm.pending))
                real(value, shift)
            motor.sm.put = put
        group.motors[0].sm.tx_busy_polls = 5
        for _ in range(4):
            group.update()
        self.assertEqual(queued_before_put, [0] * 8)

    def test_deactivate_ends_the_wait_and_nothing_is_sent(self):
        group = GroupWithoutReplyGate([uni(0, 6)])
        self.arm_fully(group)
        sm = group.motors[0].sm
        sent = len(sm.sent)
        polls = []

        def never_empty():
            """The queue never empties; the third poll happens as another core deactivates the group."""
            polls.append(1)
            if len(polls) == 3:
                group.deactivate()
            return 1

        sm.tx_fifo = never_empty
        group.update()
        self.assertEqual(len(polls), 3)
        self.assertEqual(group.state, INACTIVE)
        self.assertEqual(len(sm.sent), sent)


class DisarmTest(GroupTestCase):
    def test_deactivate_transmits_nothing_and_deactivates_everything(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        self.arm_fully(group)
        group.set_throttle(0, 200)
        group.update()
        sent = [list(motor.sm.sent) for motor in group.motors]
        group.deactivate()
        for motor, before in zip(group.motors, sent):
            self.assertEqual(motor.sm.sent, before)
            self.assertFalse(motor.sm.is_active)
            self.assertEqual(motor.sm.pending, [])
        self.assertEqual(group.state, INACTIVE)
        self.assertEqual(group.get_all_throttles(), [0, 0])

    def test_a_word_queued_behind_deactivate_is_not_sent_at_the_next_arm(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.set_throttle(0, 200)
        group.update()
        group.deactivate()
        # Another core's update() put one more word after the reset
        group.motors[0].sm.put(0x0E100000)
        group.arm(ARM_MS)
        self.assertEqual(group.motors[0].sm.pending, [])

    def test_deactivate_is_idempotent_and_sends_nothing_the_second_time(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.deactivate()
        sent = len(group.motors[0].sm.sent)
        group.deactivate()
        self.assertEqual(len(group.motors[0].sm.sent), sent)

    def test_a_redundant_deactivate_does_not_reset_the_state_machines_again(self):
        # A motor's deactivate() is only meaningful on a state machine that was
        # actually live - a repeat (or one before ever arming) has nothing to reset.
        group = self.make([uni(0, 6), bidir(2, 8)])
        self.arm_fully(group)
        group.deactivate()
        restarts = [motor.sm.restarts for motor in group.motors]
        rx_restarts = group.motors[1].rx_sm.restarts
        group.deactivate()
        self.assertEqual([motor.sm.restarts for motor in group.motors], restarts)
        self.assertEqual(group.motors[1].rx_sm.restarts, rx_restarts)

    def test_deactivate_before_arming_does_not_reset_the_state_machines(self):
        group = self.make([uni(0, 6), bidir(2, 8)])
        group.deactivate()
        self.assertEqual([motor.sm.restarts for motor in group.motors], [0, 0])
        self.assertEqual(group.motors[1].rx_sm.restarts, 0)

    def test_deactivate_before_arming_sends_nothing(self):
        group = self.make([uni(0, 6)])
        group.deactivate()
        self.assertEqual(group.motors[0].sm.sent, [])

    def test_update_is_inert_again_after_deactivate(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.deactivate()
        sent = len(group.motors[0].sm.sent)
        group.update()
        self.assertEqual(len(group.motors[0].sm.sent), sent)

    def test_the_group_can_be_armed_again_after_deactivate(self):
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        group.deactivate()
        self.arm_fully(group)
        self.assertTrue(group.is_armed())


class UpdateIntervalTest(GroupTestCase):
    def test_the_update_period_is_the_one_bench_verified_value(self):
        # 1045us is what README's "Verified Parameters" table (20/20, then 13/13 clean) and the README's
        # Operational Envelope rest on. There is no unpaced mode (0 was the default until 2026-10-19):
        # changing this number is a deliberate act that needs new evidence and these docs updated.
        self.assertEqual(MotorGroup.UPDATE_INTERVAL_US, 1045)


class ThrottleTest(GroupTestCase):
    def test_set_throttle_rejects_a_negative_value(self):
        group = self.make([uni(0, 6)])
        group.set_throttle(0, 100)
        for value in (-1, -5, -2047):
            with self.assertRaises(MotorGroupException):
                group.set_throttle(0, value)
        self.assertEqual(group.get_all_throttles(), [100])

    def test_set_all_throttles_rejects_a_negative_value_without_a_partial_write(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_all_throttles([100, 200])
        with self.assertRaises(MotorGroupException):
            group.set_all_throttles([60, -1])
        self.assertEqual(group.get_all_throttles(), [100, 200])

    def test_validate_throttle_returns_a_valid_value_unchanged(self):
        group = self.make([uni(0, 6)])
        for value in (0, 48, 100, group.THROTTLE_CEILING):
            self.assertEqual(group.validate_throttle(value), value)

    def test_update_does_not_validate_what_it_sends(self):
        # Validation is at the set_* boundary only; the per-tick path sends the
        # array as it stands (a value above the ceiling written straight in).
        group = self.make([uni(0, 6)])
        self.arm_fully(group)
        value = group.THROTTLE_CEILING + 1
        group.throttles[0] = value
        group.update()
        packet = value << 1
        crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xF
        self.assertEqual(group.motors[0].sm.sent[-1], ((packet << 4) | crc) << 16)

    def test_set_throttle_rejects_above_the_temporary_bench_ceiling(self):
        group = self.make([uni(0, 6)])
        group.set_throttle(0, group.THROTTLE_CEILING)
        self.assertEqual(group.get_all_throttles(), [group.THROTTLE_CEILING])
        for value in (group.THROTTLE_CEILING + 1, 300, 2047, 99999):
            with self.assertRaises(MotorGroupException):
                group.set_throttle(0, value)
        self.assertEqual(group.get_all_throttles(), [group.THROTTLE_CEILING])

    def test_set_throttle_rejects_the_dshot_command_range(self):
        group = self.make([uni(0, 6)])
        for value in (1, 12, 36, 47):
            with self.assertRaises(MotorGroupException):
                group.set_throttle(0, value)
        self.assertEqual(group.get_all_throttles(), [0])
        for value in (0, 48, 250):
            group.set_throttle(0, value)
            self.assertEqual(group.get_all_throttles(), [value])

    def test_set_all_throttles_rejects_the_command_range_without_a_partial_write(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_all_throttles([100, 200])
        with self.assertRaises(MotorGroupException):
            group.set_all_throttles([300, 10])
        self.assertEqual(group.get_all_throttles(), [100, 200])

    def test_set_throttle_rejects_a_bad_index(self):
        group = self.make([uni(0, 6)])
        for index in (-1, 1):
            with self.assertRaises(MotorGroupException):
                group.set_throttle(index, 60)

    def test_set_all_throttles_needs_one_value_per_motor(self):
        group = self.make([uni(0, 6), uni(1, 7)])
        group.set_all_throttles([60, group.THROTTLE_CEILING])
        self.assertEqual(group.get_all_throttles(), [60, group.THROTTLE_CEILING])
        with self.assertRaises(MotorGroupException):
            group.set_all_throttles([1])
        with self.assertRaises(MotorGroupException):
            group.set_all_throttles([60, group.THROTTLE_CEILING + 1])
        self.assertEqual(group.get_all_throttles(), [60, group.THROTTLE_CEILING])

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

    def test_no_capture_is_handed_out_while_inactive_or_arming(self):
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
        self.motor.drain_rx()
        self.assertIsNotNone(self.motor.latest_capture())
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_replies_drained_while_arming_are_discarded(self):
        self.group.arm(ARM_MS)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.run_until_armed(self.group)
        self.assertEqual(self.motor.rx_sm.rx_fifo(), 0, "they were taken from the receiver...")
        self.assertIsNone(self.group.raw_telemetry(0), "...but not published")

    def test_a_reply_after_arming_is_handed_out_as_one_value(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        value = self.group.raw_telemetry(0)
        self.assertEqual(value & FRAME_MASK, self.CAPTURE[0])
        self.assertGreater(value >> FRAME_BITS, 0, "carrying a token above the frame")

    def test_the_latest_capture_replaces_earlier_ones(self):
        self.arm_fully(self.group)
        for marker in (1, 2, 3):
            self.motor.rx_sm.feed([marker])
            self.group.update()
        self.assertEqual(self.group.raw_telemetry(0) & FRAME_MASK, 3)

    def test_deactivate_takes_the_captures_away_again(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        self.assertIsNotNone(self.group.raw_telemetry(0))
        self.group.deactivate()
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_a_new_run_starts_without_the_previous_capture(self):
        self.arm_fully(self.group)
        self.motor.rx_sm.feed(self.CAPTURE)
        self.group.update()
        self.group.deactivate()
        self.group.arm(ARM_MS)
        self.run_until_armed(self.group)
        self.assertIsNone(self.group.raw_telemetry(0))

    def test_decode_telemetry_uses_analyze_frame(self):
        motor = BidirectionalDShot(4, Pin(8), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=5)
        group = self.make([motor])
        result = group.decode_telemetry(0, self.CAPTURE[0])
        self.assertTrue(result["crc_ok"])
        self.assertAlmostEqual(result["erpm"], 48859.9, places=1)

    def test_decode_telemetry_ignores_the_token_above_the_frame(self):
        motor = BidirectionalDShot(4, Pin(8), DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=5)
        group = self.make([motor])
        with_token = (300 << FRAME_BITS) | self.CAPTURE[0]
        self.assertEqual(group.decode_telemetry(0, with_token), group.decode_telemetry(0, self.CAPTURE[0]))

    def test_decode_telemetry_raises_for_a_unidirectional_motor(self):
        with self.assertRaises(UnsupportedOperationException):
            self.group.decode_telemetry(1, self.CAPTURE[0])

    def test_decode_telemetry_rejects_a_bad_index(self):
        with self.assertRaises(MotorGroupException):
            self.group.decode_telemetry(5, self.CAPTURE)

    def test_every_arm_reinitializes_the_receiver_program_and_jmp_pin(self):
        # activate() calls rx_sm.init() on every arm(), the first one included,
        # not only a restart after a deactivate() - and dshot_bidir_rx_frame fills its
        # PIO block on its own (see BidirectionalDShot's constructor
        # docstring), so it is worth checking both calls get it right.
        self.arm_fully(self.group)
        first_program, first_freq, first_kwargs = self.motor.rx_sm.init_calls[-1]
        self.assertEqual(first_program, dshot_bidir_rx_frame)
        self.assertEqual(first_freq, frame_rx_speed(SPEED))
        self.assertEqual(first_kwargs["jmp_pin"], self.motor.pin)

        self.group.deactivate()
        self.group.arm(ARM_MS)
        program, freq, kwargs = self.motor.rx_sm.init_calls[-1]
        self.assertEqual(program, dshot_bidir_rx_frame)
        self.assertEqual(freq, frame_rx_speed(SPEED))
        self.assertEqual(kwargs["jmp_pin"], self.motor.pin)


if __name__ == "__main__":
    unittest.main()
