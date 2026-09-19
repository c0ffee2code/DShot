# SPDX-License-Identifier: GPL-3.0-or-later
# MotorThrottleGroup: facade over the PIO state machines and throttle state
# of a group of DShot motors.
#
# See decision/ADR-004-client-owned-command-loop.md for the threading model
# See decision/ADR-001-dual-core-motor-control.md for the lock-free throttle store

import utime
from array import array

from dshot_pio import UnidirectionalDShot, DSHOT_SPEEDS

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

    This class owns the PIO state machines and the throttle values. It does
    NOT own a command loop: the application decides which core, thread, timer
    or main loop calls update(), because that is an architecture choice of the
    application, not of this library.

    The only requirement is that update() is called at least every
    UPDATE_INTERVAL_US while armed - ESCs disarm if commands stop arriving.

    Usage (application runs the loop on Core 1):
        from machine import Pin
        from dshot_pio import DSHOT_SPEEDS
        from motor_throttle_group import MotorThrottleGroup

        group = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)

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
    # possible, no explicit delay" - an AM32-firmware ESC would not complete
    # arming even at a clean 250us once Core1Runner's own per-call overhead
    # was added on top; back-to-back calls (no sleep) is the only rate
    # verified reliable through the real facade. See the "Verified
    # Parameters" table in README.md.
    UPDATE_INTERVAL_US = 0

    # Default arming duration in milliseconds. An earlier finding claimed an
    # AM32-firmware ESC never completed its own arm confirmation at 500ms,
    # even at max frame rate, and set this to 3000ms - that finding was
    # re-tested on 2026-09-12 after discovering the original test run(s)
    # predated a fix for a board-state corruption bug (`mpremote run` not
    # resetting the board between invocations - see scripts/deploy.py). Under
    # the corrected reset-before-run workflow, 500ms (and even 300ms) armed
    # cleanly, confirmed via genuine non-zero eRPM telemetry replies, not
    # just elapsed time - see bidirectional_dshot_review.md's W18 notes.
    # A longer hold is always safe for ESCs that need more (see README.md
    # "Verified Parameters").
    DEFAULT_ARM_DURATION_MS = 500

    # A gap longer than this between update() calls restarts the arming window,
    # because the ESC resets its own arming counter when commands stop arriving
    ARM_GAP_TOLERANCE_MS = 10

    # Highest value representable in an 11-bit DShot throttle field
    MAX_THROTTLE = 2047

    # Zero-throttle frames disarm() transmits before cutting the signal. One
    # commands the stop; the rest are margin against a frame lost to noise.
    DISARM_FRAMES = 4

    def __init__(self, pins, dshot_speed=DSHOT_SPEEDS.DSHOT600):
        """
        Initialize motor group.

        Creates the PIO state machines but leaves them inactive - arm()
        activates them.

        Args:
            pins: List of Pin objects for motor signal outputs
            dshot_speed: DShot protocol speed (default: DSHOT600)
        """
        if not pins:
            raise MotorThrottleGroupException("At least one pin required")

        self.motor_count = len(pins)

        # Create UnidirectionalDShot instances internally (SM index = motor index)
        self.motors = [
            UnidirectionalDShot(i, pin, dshot_speed) for i, pin in enumerate(pins)
        ]

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

        Args:
            duration_ms: Arming duration (default: DEFAULT_ARM_DURATION_MS)
        """
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
        Send one DShot command to each motor and advance the arming sequence.

        The application calls this at least every UPDATE_INTERVAL_US, from
        whichever core or scheduling arrangement it chooses.

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

        if value < 0:
            value = 0
        elif value > self.MAX_THROTTLE:
            value = self.MAX_THROTTLE

        self.throttles[motor_index] = value

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

        max_throttle = self.MAX_THROTTLE
        for i, value in enumerate(values):
            if value < 0:
                value = 0
            elif value > max_throttle:
                value = max_throttle
            self.throttles[i] = value

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
