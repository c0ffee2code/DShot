# SPDX-License-Identifier: GPL-3.0-or-later
# Original implementation: https://github.com/jrddupont/DShotPIO
# Licensed under GNU General Public License v3.0
# DShot protocol reference: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

import utime
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
    # Words the PIO TX FIFO holds before put() starts blocking
    TX_FIFO_DEPTH = 4

    # Creates the state machine but leaves it inactive - call start() to enable it
    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT150):
        self.sm = StateMachine(state_machine_id, dshot, freq=dshot_speed, sideset_base=pin)

        # Wall-clock time of one 16-bit frame at 8 PIO cycles per bit, rounded
        # up so a wait built from it is never short
        self.frame_us = (16 * 8 * 1_000_000 + dshot_speed - 1) // dshot_speed

    def start(self):
        self.sm.active(1)

    def drain(self):
        """
        Block until everything queued has been transmitted.

        Call this before stop() when the queued frames still need to reach the
        ESC - stop() cuts them off otherwise. It also parks the line low: with
        the FIFO empty the program stalls on its side(0) out instruction.

        Only meaningful on an active state machine. The wait is bounded rather
        than a spin on tx_fifo(), so calling it on an inactive one costs a few
        hundred microseconds instead of hanging.
        """
        for _ in range(self.TX_FIFO_DEPTH):
            if not self.sm.tx_fifo():
                break
            utime.sleep_us(self.frame_us)

        # tx_fifo() reaching zero only means the last word has been pulled into
        # the shift register - it is still going out on the wire
        utime.sleep_us(self.frame_us)

    def stop(self):
        """
        Deactivate the state machine.

        The signal line stops carrying DShot transitions, so the ESC times out
        and cannot spin.

        Stopping mid-frame truncates that frame - it fails CRC and the ESC
        drops it - and leaves the pin at whatever level the frame was driving.
        Call drain() first when that matters.

        Call start() to reactivate - the PIO program stays loaded.
        """
        self.sm.active(0)

        # Clears the shift state and jumps to the start of the program, so a
        # frame interrupted by active(0) cannot resume mid-bit on restart.
        # This does NOT drain the TX FIFO: whatever is still queued goes out on
        # the next start(), which is why drain() exists as a separate call.
        self.sm.restart()

    def send_throttle_command(self, throttle):
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
        self.sm.put(rightPaddedPacket)
