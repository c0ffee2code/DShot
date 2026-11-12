"""
esc_buddy_board.py - the main Pico's half of the ESC buddy: USB serial to the AM32 configurator on one side, the
ESC signal lines on the other (19200 baud 8N1, half duplex, one line at a time). The protocol lives in
esc_buddy_protocol.py; this file is only the wire. Started by tools/esc_buddy/esc_buddy_run.py, which names
the pins (its --pins); never part of a motor run.

The USB port is the REPL's: while this runs, Ctrl-C is off (binary traffic would contain 0x03) and nothing is printed.
It returns to the REPL on the configurator's "exit", if no configurator connects within FIRST_TRAFFIC_TIMEOUT_S, or after
IDLE_TIMEOUT_S of silence once one has; unplugging USB also works.

Line handling copies Betaflight's (src/main/io/serial_4way.c, serial_4way_avrootloader.c): every line idles as an
input with the Pico's pull-up and is never driven while idle; a byte is driven only from one extra idle bit before its
start bit to its last data bit, and the stop bit is held by the pull-up. Bench result 2026-11-04 (Skystars KM55A2, an
older AM32 bootloader, protocol version 1): it answered every request this way, and none while the Pico drove idle lines
high and sent bytes back to back. On exit every line is driven low, the state MotorGroup.disarm() leaves, which makes the
bootloader start the ESC's firmware.
"""

import select
import sys
import time

import micropython
import rp2
import ubinascii
from machine import Pin

from esc_buddy_protocol import FrameReader, Passthrough

BAUD = 19200
CYCLES_PER_BIT = 16
BYTE_US = 11 * 1000000 // BAUD + 1        # idle + start + 8 data + stop
REPLY_GAP_US = 3000                        # a reply has ended when no byte came for this long
FIRST_TRAFFIC_TIMEOUT_S = 1800              # waiting for the configurator to connect
IDLE_TIMEOUT_S = 300                        # silence once it has
LOG_PATH = "esc_buddy_log.txt"
IMAGE_PATH = "esc_buddy_image.bin"           # the firmware image of a flash session, uploaded by esc_buddy_run.py
TX_SM = 0
RX_SM = 1


# One byte out (Betaflight's framing): an extra idle bit, driven high, then start bit and 8 data bits LSB first, 16 cycles
# each; then the pin is released and the pull-up holds the stop bit (one bit time before the next byte can start).
@rp2.asm_pio(set_init=rp2.PIO.IN_HIGH, out_init=rp2.PIO.IN_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT, pull_thresh=8)
def onewire_tx():
    pull(block)
    set(pins, 1)
    set(pindirs, 1)[15]
    set(pins, 0)[15]
    label("bit")
    out(pins, 1)[14]
    jmp(not_osre, "bit")
    set(pins, 1)
    set(pindirs, 0)[14]


# One byte in: wait for the start bit's falling edge, sample each data bit in its middle (1.5 bit times later, then every
# 16 cycles), keep the byte only if the stop bit is high. The byte arrives in bits 24-31.
@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def onewire_rx():
    wrap_target()
    label("start")
    mov(isr, null)
    set(x, 7)
    wait(0, pin, 0)[23]
    label("bit")
    in_(pins, 1)[14]
    jmp(x_dec, "bit")
    jmp(pin, "good")
    jmp("start")
    label("good")
    push(block)
    wrap()


class OneWireLink:
    def __init__(self, pins):
        self.pins = pins
        self.pin = None
        self.line = None
        self.echo = b""            # diagnostics of the last send / receive, written into the record
        self.sent = b""
        self.low_samples = 0
        self.samples = 0
        self.tx = rp2.StateMachine(TX_SM)
        self.rx = rp2.StateMachine(RX_SM)
        for number in pins:
            Pin(number, Pin.IN, Pin.PULL_UP)

    def select(self, index):
        self.tx.active(0)
        self.rx.active(0)
        if self.pin is not None:
            Pin(self.pin, Pin.IN, Pin.PULL_UP)
        self.pin = self.pins[index]
        pin = Pin(self.pin, Pin.IN, Pin.PULL_UP)
        freq = BAUD * CYCLES_PER_BIT
        self.tx.init(onewire_tx, freq=freq, set_base=pin, out_base=pin)
        self.rx.init(onewire_rx, freq=freq, in_base=pin, jmp_pin=pin)    # the line stays on the pull-up
        self.line = Pin(self.pin)
        self.drain()
        self.tx.active(1)
        self.rx.active(1)

    def drain(self, into=None):
        while self.rx.rx_fifo():
            word = self.rx.get()
            if into is not None:
                into.append(word >> 24)

    def send(self, data):
        self.drain()
        echo = bytearray()                     # our own bytes come back on the shared line
        for byte in data:
            self.tx.put(byte)
            self.drain(echo)
        while self.tx.tx_fifo():
            self.drain(echo)
        time.sleep_us(BYTE_US + 40)            # the last byte is out; the PIO program has already released the line
        self.drain(echo)
        self.echo = bytes(echo)
        self.sent = bytes(data)

    def receive(self, count, timeout_ms):
        out = bytearray()
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        low = samples = 0
        line = self.line
        while not self.rx.rx_fifo():
            samples += 1
            if not line.value():
                low += 1
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                self.low_samples, self.samples = low, samples
                return bytes(out)
        self.low_samples, self.samples = low, samples
        last = time.ticks_us()
        while len(out) < count:
            if self.rx.rx_fifo():
                out.append(self.rx.get() >> 24)
                last = time.ticks_us()
            elif time.ticks_diff(time.ticks_us(), last) > REPLY_GAP_US:
                break
        return bytes(out)

    def restart_esc(self):
        self.tx.exec("set(pindirs, 1)")
        self.tx.exec("set(pins, 0)")
        time.sleep_ms(300)
        self.tx.exec("set(pins, 1)")
        self.tx.exec("set(pindirs, 0)")        # back on the pull-up

    def close(self):
        self.tx.active(0)
        self.rx.active(0)
        for number in self.pins:
            Pin(number, Pin.OUT, value=0)


def main(pins, start_pin=None, record_writes=False, write_channel=None, restore_block=None, flash=False):
    """write_channel: the one ESC (index from 0) whose settings block this session may write, once (esc_buddy_protocol.Passthrough);
    None = read-only. restore_block: with it, the only block that write may carry. flash: the session flashes the image in IMAGE_PATH
    onto that ESC (esc_buddy_protocol.Passthrough, flash_image). start_pin: the bench start line (tests/bench/wingman/main_pico/start_line.py), held high for the whole session
    so the wingman's observer records it, low again at the end."""
    micropython.kbd_intr(-1)       # first thing: a request byte 0x03 (ESC 4's init) is Ctrl-C and would kill the program in its setup
    start_line = Pin(start_pin, Pin.OUT, value=1) if start_pin is not None else None
    log_file = open(LOG_PATH, "w")
    log_file.write("pins %s\n" % " ".join(str(p) for p in pins))
    log_file.flush()

    def log(kind, channel, address, data):
        pin = "-" if channel is None else str(pins[channel])
        log_file.write("%s ch=%s pin=%s addr=0x%04x n=%d %s\n" % (
            kind, "-" if channel is None else str(channel), pin, address, len(data), ubinascii.hexlify(data).decode()))
        if kind != "refused":
            # what the Pico saw on the line itself: its own bytes coming back, and whether the line went low while it
            # waited for the reply (samples taken in the wait loop before the first byte)
            log_file.write("  echo %s (%d of %d bytes) wait_low %d of %d samples\n" % (
                "ok" if link.echo == link.sent else "DIFFERS " + ubinascii.hexlify(link.echo).decode(),
                len(link.echo), len(link.sent), link.low_samples, link.samples))
        log_file.flush()

    link = OneWireLink(pins)
    image = None
    if flash:
        with open(IMAGE_PATH, "rb") as image_file:
            image = image_file.read()
    passthrough = Passthrough(link, len(pins), log, record_writes, write_channel, restore_block, image)
    reader = FrameReader()
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    out = sys.stdout.buffer
    last_traffic = time.time()
    limit = FIRST_TRAFFIC_TIMEOUT_S
    try:
        while not passthrough.exit_requested and time.time() - last_traffic < limit:
            if not poll.poll(10):
                continue
            chunk = bytearray()
            while poll.poll(0) and len(chunk) < 300:
                chunk.extend(sys.stdin.buffer.read(1))
            last_traffic = time.time()
            limit = IDLE_TIMEOUT_S
            for frame in reader.feed(chunk):
                if frame[0] == "4way":
                    reply = passthrough.handle_four_way(frame[1])
                else:
                    reply = passthrough.handle_msp(frame[0], frame[1], frame[2], frame[3])
                out.write(reply)
    finally:
        micropython.kbd_intr(3)
        link.close()
        if start_line is not None:
            start_line.value(0)
        log_file.write("end %s\n" % ("exit" if passthrough.exit_requested else "idle timeout"))
        log_file.close()
