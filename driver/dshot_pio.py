# SPDX-License-Identifier: GPL-3.0-or-later
# Original implementation: https://github.com/jrddupont/DShotPIO
# Licensed under GNU General Public License v3.0
# DShot protocol reference: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

from rp2 import PIO, StateMachine, asm_pio

class InvalidThrottleException(Exception):
    def __init__(self,message):
        self.message=message

# PIO assembly code for sending DShot throttle packets
# This is set up to transmit the 16 high order bits of a 32 bit input from high to low order
# Each bit is sent in 8 clock cycles
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def dshot():
    wrap_target()
    label("start")
    out(x, 1)            .side(0)    [1] # 2 cycle, Read the next bit into x register. Start at zero so the output is always low when waiting for new data
    jmp(not_x, "zero")   .side(1)    [2] # 3 cycles, Jump on x register
    jmp("start")         .side(1)    [2] # 3 cycles, "ONE" condition
    label("zero")
    jmp("start")         .side(0)    [2] # 3 cycles, "ZERO" condition
    wrap()

# The different DShot speeds. The Pico and Pico W should be fast enough to transmit at any of these speeds
class DSHOT_SPEEDS:
    DSHOT150  = 1_200_000 #   150,000 bit/s * 8 cycle/bit
    DSHOT300  = 2_400_000 #   300,000 bit/s * 8 cycle/bit
    DSHOT600  = 4_800_000 #   600,000 bit/s * 8 cycle/bit
    DSHOT1200 = 9_600_000 # 1,200,000 bit/s * 8 cycle/bit


class DShotPIO:
    # Once the class is initialized, it will create and enable the state machine
    def __init__(self, stateMachineID, pin, dshotSpeed=DSHOT_SPEEDS.DSHOT150):
        self._sm = StateMachine(stateMachineID, dshot, freq=dshotSpeed, sideset_base=pin)

    def start(self):
        self._sm.active(1)

    def sendThrottleCommand(self, throttle):
        """
        Send a throttle command to the ESC.

        Args:
            throttle: Throttle value (0-2047)

        Note: DShot protocol includes a telemetry request bit, but this implementation
        always sets it to 0. Telemetry requires bidirectional DShot which is not
        implemented (see ADR-002).
        """
        if throttle < 0:
            raise InvalidThrottleException("Throttle should be greater than 0.")
        if throttle > 2047:
            raise InvalidThrottleException("Throttle value is too high. Maximum value is 2047.")

        # Build 12-bit value: 11-bit throttle shifted left, telemetry bit = 0
        packetValue = throttle << 1

        # Calculate 4-bit CRC
        crc = (packetValue ^ (packetValue >> 4) ^ (packetValue >> 8)) & 0x0F

        # Build 16-bit packet: SSSSSSSSSSSTCCCC (S=throttle, T=telemetry=0, C=CRC)
        dShotPacket = (packetValue << 4) | crc
        
        # Since the state machine consumes the bits from high order to low order, we need to shift the
        #  data all the way to the high bit
        rightPaddedPacket = dShotPacket << 16

        # Put the packet into the PIO machine
        self._sm.put(rightPaddedPacket)
