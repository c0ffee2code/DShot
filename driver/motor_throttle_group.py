# SPDX-License-Identifier: GPL-3.0-or-later
# MotorThrottleGroup: facade over the PIO state machines and throttle state
# of a group of DShot motors.
#
# See decision/ADR-004-client-owned-command-loop.md for the threading model
# See decision/ADR-001-dual-core-motor-control.md for the lock-free throttle store

import utime
from array import array

from dshot_pio import DShotPIO, UnsupportedOperationException

# Lifecycle states, as reported by MotorThrottleGroup.state
DISARMED = 0
ARMING = 1
ARMED = 2


class MotorThrottleGroupException(Exception):
    def __init__(self, message):
        self.message = message


class MotorThrottleGroup:
    """
    Facade for controlling throttle on a group of DShot motors.

    This class owns the throttle values and the lifecycle of the motors it is
    given. The motors themselves (UnidirectionalDShot or BidirectionalDShot,
    one per motor, 1 to 4 of them) are built by the application, which picks
    their state machines and pins. It does NOT own a command loop: the application decides which core, thread, timer
    or main loop calls update(), because that is an architecture choice of the
    application, not of this library.

    The only requirement is that update() is called at least every
    UPDATE_INTERVAL_US while armed - ESCs disarm if commands stop arriving.

    Usage (application runs the loop on Core 1):
        from machine import Pin
        from dshot_pio import UnidirectionalDShot, BidirectionalDShot, DSHOT_SPEEDS
        from motor_throttle_group import MotorThrottleGroup

        group = MotorThrottleGroup([
            UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600),
            BidirectionalDShot(2, Pin(5), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=3),
        ])

        runner = Core1Runner(group.update)  # application-supplied, see tests/
        runner.start()

        group.arm()
        while not group.is_armed():         # arming is non-blocking
            utime.sleep_ms(10)

        group.set_throttle(0, 100)
        group.set_throttle(1, 150)

        group.disarm()
        runner.stop()

    Usage (application pumps update() from its own main loop):
        group.arm()
        while True:
            group.update()
            ...application work, kept under UPDATE_INTERVAL_US...
            utime.sleep_us(group.UPDATE_INTERVAL_US)
    """

    # How often the application must call update(). 0 means "as fast as
    # possible, no delay between calls", which is the rate the facade has been
    # hardware-verified at (see the "Verified Parameters" table in README.md).
    UPDATE_INTERVAL_US = 0

    # Default arming window in milliseconds. A telemetry reply only shows that
    # the ESC is armed, not that the motor is running: an armed ESC replies
    # with an at-rest eRPM even when the motor does not start. The library
    # cannot observe whether the motor started (or whether the ESC is beeping,
    # or the Pico has hung), so an application should not treat "armed" or
    # "replying" as "spinning". On the bench a 500ms window has spun the motor,
    # but some runs have not started it and the cause was not established; the
    # test bench arms for 3000ms. A longer window only costs startup time.
    DEFAULT_ARM_DURATION_MS = 500

    # A gap longer than this between update() calls restarts the arming window,
    # because the ESC resets its own arming counter when commands stop arriving
    ARM_GAP_TOLERANCE_MS = 10

    # Highest throttle a motor will transmit; set_throttle() clamps to it
    MAX_THROTTLE = DShotPIO.MAX_THROTTLE

    # Zero-throttle frames disarm() transmits before cutting the signal. One
    # commands the stop; the rest are margin against a frame lost to noise.
    DISARM_FRAMES = 4

    # Motors a group can drive: one ESC's worth
    MIN_MOTORS = 1
    MAX_MOTORS = 4

    def __init__(self, motors):
        """
        Initialize motor group.

        The motors' state machines stay inactive - arm() activates them.

        Args:
            motors: List of 1 to 4 UnidirectionalDShot / BidirectionalDShot
                instances, one per motor, in motor-index order. Bidirectional
                ones must have been built as such from the start: an ESC only
                detects bidirectional DShot during arming.
        """
        if len(motors) < self.MIN_MOTORS or len(motors) > self.MAX_MOTORS:
            raise MotorThrottleGroupException(
                "Expected 1 to 4 motors, got " + str(len(motors))
            )

        # The application chose each motor's state machines and pin, so check
        # they do not collide: two motors on one state machine would silently
        # replace each other, and two on one pin would fight over the line.
        # A bidirectional motor uses two state machines.
        used_state_machines = []
        used_pins = []
        for motor in motors:
            ids = [motor.state_machine_id]
            if motor.bidirectional:
                ids.append(motor.rx_state_machine_id)
            for state_machine_id in ids:
                if state_machine_id in used_state_machines:
                    raise MotorThrottleGroupException(
                        "State machine " + str(state_machine_id) + " is used by more than one motor"
                    )
                used_state_machines.append(state_machine_id)
            if motor.pin in used_pins:
                raise MotorThrottleGroupException("Two motors share the same pin")
            used_pins.append(motor.pin)

        self.motor_count = len(motors)
        self.motors = list(motors)

        # The subset whose reply must be drained on every update(), resolved
        # once so the command loop does no per-tick type checks
        self.bidir_motors = [m for m in self.motors if m.bidirectional]

        # Shared throttle array - lock-free access (atomic on ARM).
        # Using unsigned 16-bit integers ('H') for DShot throttle values.
        # See ADR-001: the application may write these from a different core
        # than the one calling update().
        self.throttles = array('H', [0] * self.motor_count)

        # One of DISARMED / ARMING / ARMED
        self.state = DISARMED

        self.arm_duration_ms = self.DEFAULT_ARM_DURATION_MS
        self.arm_started_ms = 0
        self.last_update_ms = utime.ticks_ms()

    def arm(self, duration_ms=DEFAULT_ARM_DURATION_MS):
        """
        Begin arming all ESCs.

        Activates the PIO state machines and starts the arming window. This
        does NOT block: arming completes inside update(), so the application
        must be calling update() for arming to progress. Poll is_armed().

        Any throttle set before arm() is discarded - arming always starts
        from zero.

        Only valid while disarmed. Restarting the motors under a live command
        loop would flush their RX FIFOs and reset their published telemetry
        mid-write and snap the throttles to zero, so calling it while ARMING or
        ARMED raises MotorThrottleGroupException; disarm() first to start over.

        Args:
            duration_ms: Arming duration (default: DEFAULT_ARM_DURATION_MS)
        """
        if self.state != DISARMED:
            raise MotorThrottleGroupException("arm() called while already arming or armed")

        for i in range(self.motor_count):
            self.throttles[i] = 0

        for motor in self.motors:
            motor.start()

        now = utime.ticks_ms()
        self.arm_duration_ms = duration_ms
        self.arm_started_ms = now
        self.last_update_ms = now

        # Set last: update() must not run before the state machines are active
        self.state = ARMING

    def disarm(self):
        """
        Stop all motors and disarm the ESCs.

        Commands zero throttle, waits for those frames to reach the ESCs, then
        deactivates the PIO state machines so the signal line stops carrying
        DShot transitions and the ESC cannot spin.

        This is the emergency stop. It transmits the zeros itself rather than
        leaving them for update(), so it works even when the application's
        command loop is dead - and it stops the motors in about a millisecond
        instead of waiting out the ESC's signal-loss timeout, which is over a
        hundred times longer.

        Blocks for a few hundred microseconds while the zeros shift out.

        Safe to call from any context, including a different core than the one
        calling update().

        Idempotent. Call arm() to bring the group back up.
        """
        # Cleared first, so a concurrent update() bails before we start cutting
        # the signal. The prior value also tells us whether the state machines
        # are live - putting to an inactive one fills the TX FIFO and then
        # blocks the caller forever.
        was_live = self.state != DISARMED
        self.state = DISARMED

        for i in range(self.motor_count):
            self.throttles[i] = 0

        if was_live:
            # DISARM_FRAMES fits the TX FIFO, so on an idle queue these do not
            # block at all, and on a full one they wait a few frame times for
            # an active state machine to drain - never indefinitely
            for _ in range(self.DISARM_FRAMES):
                for motor in self.motors:
                    motor.send_throttle_command(0)

            # Cutting the signal before the zeros are on the wire would leave
            # the motors spinning at their last commanded throttle
            for motor in self.motors:
                motor.drain()

        for motor in self.motors:
            motor.stop()

        # Re-assert. An update() already past its state check when we started
        # may have promoted the group to ARMED behind us; by now it has long
        # returned, because transmitting the zeros above took far longer than
        # a single update() call. Leaving ARMED set over inactive state
        # machines is what makes the next update() block forever.
        self.state = DISARMED

    def update(self):
        """
        Send one DShot command to each motor, drain each bidirectional
        motor's reply FIFO, and advance the arming sequence.

        The application calls this at least every UPDATE_INTERVAL_US, from
        whichever core or scheduling arrangement it chooses. Draining here is
        deliberate: an RX FIFO left undrained stalls the receiver and corrupts
        the captures that follow, so it must not depend on the application
        remembering a second call. Replies drained while ARMING are discarded
        (before the ESC arms, what the receiver hears is our own transmit).

        Does nothing while disarmed, so it is always safe to call - including
        before arm() or after disarm(), when the state machines are inactive
        and writing to them would eventually block on a full TX FIFO.
        """
        state = self.state
        if state == DISARMED:
            return

        now = utime.ticks_ms()

        if state == ARMING:
            # A transmission gap resets the ESC's arming counter, so restart
            # our window to match what the ESC actually saw
            if utime.ticks_diff(now, self.last_update_ms) > self.ARM_GAP_TOLERANCE_MS:
                self.arm_started_ms = now

            # Send literal zeros rather than the throttle array, so the arming
            # window stays genuinely at zero even if the application sets a
            # throttle early
            for motor in self.motors:
                motor.send_throttle_command(0)

            for motor in self.bidir_motors:
                motor.drain_rx(False)

            if utime.ticks_diff(now, self.arm_started_ms) >= self.arm_duration_ms:
                # Re-read rather than promoting from the snapshot above: a
                # disarm() on another core may have landed since, and writing
                # ARMED over it would leave the group "armed" with inactive
                # state machines - the case that makes put() block forever
                if self.state == ARMING:
                    self.state = ARMED
        else:
            throttles = self.throttles
            motors = self.motors
            for i in range(self.motor_count):
                motors[i].send_throttle_command(throttles[i])

            for motor in self.bidir_motors:
                motor.drain_rx(True)

        self.last_update_ms = now

    def is_armed(self):
        """
        Returns:
            True once the arming window has completed and throttle commands
            are being sent
        """
        return self.state == ARMED

    def is_arming(self):
        """
        Returns:
            True while the arming window is still in progress
        """
        return self.state == ARMING

    def raw_telemetry(self, motor_index):
        """
        Latest raw telemetry capture for one bidirectional motor.

        Returns (ticks_us, sequence, words) from that motor's
        latest_capture(), or None while the group is not ARMED or nothing has
        arrived yet. The group stores nothing itself - it only refuses to
        hand out captures taken while its own arming window was still open,
        when what the receiver hears is mostly our own transmit.

        ARMED means that window has elapsed, not that the ESC has armed: with
        a window shorter than the ESC needs, or an ESC without power, the
        captures handed out can still be echoes or noise. The CRC check in
        decode_telemetry() is what tells a real reply from those, and a
        CRC-failed capture should be discarded and asked for again later -
        retry timing is the application's decision.

        Safe to call from a different core than update().

        Raises UnsupportedOperationException for a unidirectional motor,
        whatever the state: asking one for telemetry is an application bug.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorThrottleGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        motor = self.motors[motor_index]
        if not motor.bidirectional:
            raise UnsupportedOperationException("Motor " + str(motor_index) + " is unidirectional")

        if self.state != ARMED:
            return None

        return motor.latest_capture()

    def decode_telemetry(self, motor_index, words):
        """
        Decode one capture returned by raw_telemetry() with that motor's own RX
        profile; see BidirectionalDShot.decode_capture() for the result.

        Costs about 1.3ms, several command-loop ticks, so call it at whatever
        pace the application can afford, never from the command loop. Raises UnsupportedOperationException for a
        unidirectional motor.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorThrottleGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        return self.motors[motor_index].decode_capture(words)

    def clamp_throttle(self, value):
        """Limit a throttle to the range a motor will transmit, 0 to MAX_THROTTLE."""
        if value < 0:
            return 0
        if value > self.MAX_THROTTLE:
            return self.MAX_THROTTLE
        return value

    def set_throttle(self, motor_index, value):
        """
        Set throttle for a single motor.

        Args:
            motor_index: Motor index (0-based)
            value: Throttle value (0-2047), clamped to range

        Accepted while disarmed, but nothing is transmitted until the group is
        armed, and arm() resets all throttles to zero.

        Lock-free: safe to call from a different core than the one calling
        update(). The write is atomic (see ADR-001).
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorThrottleGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        self.throttles[motor_index] = self.clamp_throttle(value)

    def set_all_throttles(self, values):
        """
        Set throttles for all motors.

        Args:
            values: List/tuple of throttle values, one per motor

        Each write is atomic but the batch is not atomic as a whole.
        For flight control this is acceptable (see ADR-001).
        """
        if len(values) != self.motor_count:
            raise MotorThrottleGroupException(
                "Expected " + str(self.motor_count) +
                " values, got " + str(len(values))
            )

        for i, value in enumerate(values):
            self.throttles[i] = self.clamp_throttle(value)

    def get_all_throttles(self):
        """
        Get current throttle values as a plain list.

        The throttles array itself is readable directly; this converts it for
        printing and comparison.
        """
        return [self.throttles[i] for i in range(self.motor_count)]

    def update_age_ms(self):
        """
        Milliseconds since update() last transmitted.

        This library does not own the command loop, so it cannot judge whether
        that loop is healthy - it only reports the fact. The application sets
        its own threshold, which should be well under the ESC's disarm timeout.

        Only meaningful while armed or arming; update() does not transmit, and
        so does not refresh this, while disarmed.
        """
        return utime.ticks_diff(utime.ticks_ms(), self.last_update_ms)
