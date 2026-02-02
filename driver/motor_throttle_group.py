# SPDX-License-Identifier: GPL-3.0-or-later
# MotorThrottleGroup: Dual-core facade for reliable motor throttle control
# See decision/ADR-001-dual-core-motor-control.md for architecture details

import _thread
import utime
from array import array

from dshot_pio import DShotPIO, DSHOT_SPEEDS


class MotorThrottleGroupException(Exception):
    def __init__(self, message):
        self.message = message


class MotorThrottleGroup:
    """
    Facade for controlling throttle on multiple motors with guaranteed timing.

    Runs a dedicated loop on Core 1 that sends DShot commands at 1kHz,
    ensuring reliable arming and consistent update rate regardless
    of Core 0 activity (UI, sensors, control algorithms).

    Usage:
        from machine import Pin
        from dshot_pio import DSHOT_SPEEDS
        from motor_throttle_group import MotorThrottleGroup

        group = MotorThrottleGroup([Pin(4), Pin(5)], DSHOT_SPEEDS.DSHOT600)
        group.start()
        group.arm()

        group.setThrottle(0, 100)  # Motor 0 at throttle 100
        group.setThrottle(1, 150)  # Motor 1 at throttle 150

        group.stop()
    """

    # Command loop interval in microseconds (1kHz = 1000us)
    UPDATE_INTERVAL_US = 1000

    # Default arming duration in milliseconds
    DEFAULT_ARM_DURATION_MS = 500

    def __init__(self, pins, dshot_speed=DSHOT_SPEEDS.DSHOT600):
        """
        Initialize motor group.

        Args:
            pins: List of Pin objects for motor signal outputs
            dshot_speed: DShot protocol speed (default: DSHOT600)
        """
        if not pins:
            raise MotorThrottleGroupException("At least one pin required")

        self._motor_count = len(pins)

        # Create DShotPIO instances internally (SM index = motor index)
        self._motors = [
            DShotPIO(i, pin, dshot_speed) for i, pin in enumerate(pins)
        ]

        # Shared throttle array - lock-free access (atomic on ARM)
        # Using unsigned 16-bit integers ('H') for DShot throttle values
        self._throttles = array('H', [0] * self._motor_count)

        # Core 1 thread state
        self._running = False
        self._armed = False

        # Heartbeat counter - incremented by Core 1, monitored by Core 0
        # Detects if Core 1 loop has crashed
        self._heartbeat = array('L', [0])  # Unsigned 32-bit

    def start(self):
        """
        Start the Core 1 command loop.

        Must be called before arm() or setThrottle().
        Commands are sent continuously at 1kHz.
        Throttles are reset to 0 (disarmed state).
        """
        if self._running:
            return

        # Reset to disarmed state
        for i in range(self._motor_count):
            self._throttles[i] = 0
        self._armed = False

        self._running = True
        self._heartbeat[0] = 0
        _thread.start_new_thread(self._core1_loop, ())

        # Wait for Core 1 to start (first heartbeat)
        timeout_ms = 100
        start = utime.ticks_ms()
        while self._heartbeat[0] == 0:
            if utime.ticks_diff(utime.ticks_ms(), start) > timeout_ms:
                self._running = False
                raise MotorThrottleGroupException("Core 1 failed to start")
            utime.sleep_ms(1)

        # start the PIO state machines
        for i in range(self._motor_count):
            self._motors[i].start()

    def stop(self):
        """
        Stop the Core 1 command loop.

        Sets all throttles to 0 before stopping.
        """
        if not self._running:
            return

        # Set all throttles to 0 first
        self.emergencyStop()

        # Give Core 1 time to send the zero commands
        utime.sleep_ms(10)

        self._running = False
        self._armed = False

        # Give Core 1 time to exit
        utime.sleep_ms(5)

    def arm(self, duration_ms=None):
        """
        Arm all ESCs by sending throttle=0 for the required duration.

        Args:
            duration_ms: Arming duration (default: 500ms)

        Returns:
            True if arming completed (Core 1 still running)
        """
        if not self._running:
            raise MotorThrottleGroupException("Must call start() before arm()")

        if duration_ms is None:
            duration_ms = self.DEFAULT_ARM_DURATION_MS

        # Ensure all throttles are at 0
        for i in range(self._motor_count):
            self._throttles[i] = 0

        # Wait for arming duration while Core 1 sends zero commands
        utime.sleep_ms(duration_ms)

        # Verify Core 1 is still running
        if not self.isHealthy():
            raise MotorThrottleGroupException("Core 1 stopped during arming")

        self._armed = True
        return True

    def disarm(self):
        """
        Disarm all ESCs by setting throttles to 0.
        """
        self.emergencyStop()
        self._armed = False

    def emergencyStop(self):
        """
        Immediately set all throttles to 0.

        Safe to call from any context. Each write is atomic.
        Core 1 will pick up zeros within 1ms.
        """
        for i in range(self._motor_count):
            self._throttles[i] = 0

    def setThrottle(self, motor_index, value):
        """
        Set throttle for a single motor.

        Args:
            motor_index: Motor index (0-based)
            value: Throttle value (0-2047)

        Lock-free: safe to call from Core 0 while Core 1 is running.
        """
        if motor_index < 0 or motor_index >= self._motor_count:
            raise MotorThrottleGroupException(f"Invalid motor index: {motor_index}")

        # Clamp value to valid DShot range
        value = max(0, min(2047, value))

        # Atomic write on ARM
        self._throttles[motor_index] = value

    def setAllThrottles(self, values):
        """
        Set throttles for all motors.

        Args:
            values: List/tuple of throttle values, one per motor

        Each write is atomic but the batch is not atomic as a whole.
        For flight control this is acceptable (see ADR-001).
        """
        if len(values) != self._motor_count:
            raise MotorThrottleGroupException(
                f"Expected {self._motor_count} values, got {len(values)}"
            )

        for i, value in enumerate(values):
            self._throttles[i] = max(0, min(2047, value))

    def getAllThrottles(self):
        """
        Get current throttle values for all motors.

        Returns:
            List of throttle values
        """
        return [self._throttles[i] for i in range(self._motor_count)]

    def isHealthy(self):
        """
        Check if Core 1 is still running by monitoring heartbeat.

        Returns:
            True if Core 1 has updated heartbeat recently
        """
        if not self._running:
            return False

        # Check if heartbeat has incremented
        initial = self._heartbeat[0]
        utime.sleep_ms(5)  # Wait for a few Core 1 cycles
        return self._heartbeat[0] != initial

    @property
    def motor_count(self):
        """Number of motors in this group."""
        return self._motor_count

    def _core1_loop(self):
        """
        Core 1 dedicated loop - sends commands at 1kHz.

        This runs on the second core, isolated from Core 0 activity.
        Maintains consistent timing regardless of UI or sensor operations.
        """
        while self._running:
            # Send command to each motor
            for i in range(self._motor_count):
                self._motors[i].sendThrottleCommand(self._throttles[i])

            # Increment heartbeat (wraps around naturally)
            self._heartbeat[0] += 1

            # Maintain 1kHz update rate
            utime.sleep_us(self.UPDATE_INTERVAL_US)
