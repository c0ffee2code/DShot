"""
esc_buddy_protocol.py - the protocol half of the ESC buddy: what the AM32 configurator says to us and what we
say to each ESC's bootloader. Pure Python (runs under MicroPython on the Pico and under CPython for tests/pc); the wire
itself is the `link` object the board script passes in.

The configurator thinks it is talking to a flight controller: MSP to identify it and switch to passthrough, then the
BLHeli 4-way interface, which we turn into AM32 bootloader commands on one signal line at a time. Modelled on
github.com/BrushlessPower/BlHeli-Passthrough (RP2040), checked against am32-firmware/AM32-bootloader (bootloader/main.c)
and am32-firmware/am32-configurator (src/communication/).

Read-only by construction, but for one granted write: only the 4-way commands in ALLOWED_4WAY reach the ESC, every other one (write, erase, EEPROM
write, verify) is answered "invalid command" (or, with record_writes, "OK" so the configurator's save sequence can be
recorded) without touching the line, and every bootloader frame is built here, never
forwarded from the PC.
"""

# --- 4-way interface (configurator <-> us) ---
LOCAL_ESCAPE = 0x2F    # configurator -> interface
REMOTE_ESCAPE = 0x2E   # interface -> configurator

CMD_TEST_ALIVE = 0x30
CMD_PROTOCOL_GET_VERSION = 0x31
CMD_INTERFACE_GET_NAME = 0x32
CMD_INTERFACE_GET_VERSION = 0x33
CMD_INTERFACE_EXIT = 0x34
CMD_DEVICE_RESET = 0x35
CMD_DEVICE_INIT_FLASH = 0x37
CMD_DEVICE_READ = 0x3A
CMD_DEVICE_WRITE = 0x3B
CMD_INTERFACE_SET_MODE = 0x3F

ALLOWED_4WAY = (CMD_TEST_ALIVE, CMD_PROTOCOL_GET_VERSION, CMD_INTERFACE_GET_NAME, CMD_INTERFACE_GET_VERSION,
                CMD_INTERFACE_EXIT, CMD_DEVICE_RESET, CMD_DEVICE_INIT_FLASH, CMD_DEVICE_READ, CMD_INTERFACE_SET_MODE)

# Commands the configurator uses to save settings. Never sent to an ESC. With record_writes the buddy answers them "OK"
# so the configurator carries on and its whole save sequence lands in the record; without it they are "invalid command".
WRITE_CLASS_4WAY = (0x38, 0x39, 0x3B, 0x3E, 0x40)    # erase all, page erase, write, EEPROM write, verify

ACK_OK = 0x00
ACK_I_INVALID_CMD = 0x02
ACK_I_INVALID_CRC = 0x03
ACK_I_INVALID_CHANNEL = 0x08
ACK_I_INVALID_PARAM = 0x09
ACK_D_GENERAL_ERROR = 0x0F

PROTOCOL_VERSION_4WAY = 108
INTERFACE_NAME = b"m4wFCIntf"
INTERFACE_VERSION = (200, 5)     # 20005, as Betaflight and the reference report
MODE_ARM_BLB = 4

# --- AM32 bootloader (us <-> ESC), 19200 baud 8N1 on the signal line ---
BL_RUN = 0x00
BL_READ_FLASH = 0x03
BL_KEEP_ALIVE = 0xFD
BL_PROG_FLASH = 0x01
BL_SET_BUFFER = 0xFE
BL_SET_ADDRESS = 0xFF
BL_SUCCESS = 0x30

# Where the settings block sits, per bootloader signature, for bootloaders without the v3 devinfo block (am32-configurator
# src/mcu.ts variants). Only what this bench has met is listed; an unknown signature is refused, not guessed.
SETTINGS_ADDRESS = {0x1F06: 0x7C00, 0x3506: 0xF800}
SETTINGS_SIZE = 0xB8          # Mcu.LAYOUT_SIZE
# A write must leave these bytes as the buddy last read them from that ESC: the first 17 (boot byte, layout, bootloader
# version, firmware version and name) and the last 8. The configurator blanks the name and pads the tail when it saves.
PROTECTED_HEAD = 17
PROTECTED_TAIL = 8
BOOT_INIT = bytes([0, 0, 0, 0, 0, 0, 0, 0, 0x0D, ord("B"), ord("L"), ord("H"), ord("e"), ord("l"), ord("i"), 0xF4, 0x7D])
# The same request with 12 leading zeros, as the configurator's direct mode sends it (direct.ts init()); the bootloader
# accepts both (decodeInput() checks offsets 16, 20 and 40). Tried when the 17-byte form gets no answer.
BOOT_INIT_LONG = bytes(4) + BOOT_INIT
DEVICE_INFO_LEN = 9              # '4','7','1', pin code, flash size, 0x06, 0x06, protocol version, 0x30

# --- MSP (configurator <-> us, before passthrough) ---
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_MOTOR = 104
MSP_BATTERY_STATE = 130
MSP_MOTOR_CONFIG = 131
MSP_SET_PASSTHROUGH = 245


def crc_xmodem(data, crc=0):
    """CRC16-XMODEM (poly 0x1021, MSB first): the 4-way frame checksum."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def crc_bootloader(data, crc=0):
    """CRC16 (poly 0xA001, LSB first, init 0): the AM32 bootloader's frame checksum, sent low byte first."""
    for byte in data:
        for _ in range(8):
            if (byte ^ crc) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            byte >>= 1
    return crc


def crc8_dvb_s2(data, crc=0):
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def bootloader_frame(payload):
    crc = crc_bootloader(payload)
    return bytes(payload) + bytes([crc & 0xFF, crc >> 8])


def four_way_reply(cmd, address, params, ack):
    """params: 1..256 bytes (256 is sent as length 0)."""
    body = bytes([REMOTE_ESCAPE, cmd, address >> 8, address & 0xFF, len(params) & 0xFF]) + bytes(params) + bytes([ack])
    crc = crc_xmodem(body)
    return body + bytes([crc >> 8, crc & 0xFF])


def msp_v1_reply(cmd, payload, error=False):
    payload = bytes(payload)
    checksum = len(payload) ^ cmd
    for byte in payload:
        checksum ^= byte
    return b"$M" + (b"!" if error else b">") + bytes([len(payload), cmd]) + payload + bytes([checksum])


def msp_v2_reply(cmd, payload, error=False):
    payload = bytes(payload)
    body = bytes([0, cmd & 0xFF, cmd >> 8, len(payload) & 0xFF, len(payload) >> 8]) + payload
    return b"$X" + (b"!" if error else b">") + body + bytes([crc8_dvb_s2(body)])


class FrameReader:
    """Collects bytes from USB and hands out complete frames: ("msp1", cmd, payload, ok), ("msp2", cmd, payload, ok)
    or ("4way", frame_bytes). Bytes that start no known frame are dropped. (Slicing, not `del`: MicroPython's bytearray
    has no item deletion.)"""

    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        self.buffer += bytes(data)
        frames = []
        while True:
            frame = self.next_frame()
            if frame is None:
                return frames
            frames.append(frame)

    def drop(self, count):
        self.buffer = self.buffer[count:]

    def next_frame(self):
        while True:
            start = 0
            buf = self.buffer
            while start < len(buf) and buf[start] not in (36, LOCAL_ESCAPE):     # 36 = "$"
                start += 1
            if start:
                self.drop(start)
                buf = self.buffer
            if not buf:
                return None
            if buf[0] == LOCAL_ESCAPE:
                if len(buf) < 5:
                    return None
                total = 5 + (buf[4] or 256) + 2
                if len(buf) < total:
                    return None
                self.drop(total)
                return ("4way", buf[:total])
            if len(buf) < 3:
                return None
            if buf[1] == 77 and buf[2] == 60:          # "M<"
                if len(buf) < 6:
                    return None
                size = buf[3]
                total = 6 + size
                if len(buf) < total:
                    return None
                cmd = buf[4]
                payload = buf[5:5 + size]
                checksum = size ^ cmd
                for byte in payload:
                    checksum ^= byte
                self.drop(total)
                return ("msp1", cmd, payload, checksum == buf[total - 1])
            if buf[1] == 88 and buf[2] == 60:          # "X<"
                if len(buf) < 9:
                    return None
                cmd = buf[4] | (buf[5] << 8)
                size = buf[6] | (buf[7] << 8)
                total = 9 + size
                if len(buf) < total:
                    return None
                payload = buf[8:8 + size]
                self.drop(total)
                return ("msp2", cmd, payload, crc8_dvb_s2(buf[3:8 + size]) == buf[total - 1])
            self.drop(1)    # "$" followed by something that is not MSP


class Passthrough:
    """Answers the configurator. `link` reaches the ESCs:
        link.select(index)                  - talk to ESC `index` from now on
        link.send(data)                     - transmit bytes on the selected line
        link.receive(count, timeout_ms)     - bytes the ESC sent back (up to `count`, b"" on timeout)
        link.restart_esc()                  - line low ~300 ms, then high again
    `log(kind, channel, address, data)` records every ESC reply that carries information ("init", "read") and every
    command it refused ("refused", data = the command byte followed by the parameters the configurator sent, so what it
    tried to write can be checked against the bootloader source); channel is None before any ESC was selected.
    record_writes: answer the write-class commands "OK" instead of "invalid command" - still nothing reaches the line."""

    def __init__(self, link, esc_count, log=None, record_writes=False, write_channel=None, restore_block=None):
        self.record_writes = record_writes
        self.restore_block = restore_block      # put an earlier block back: the write must carry exactly this (bytes) or nothing
        self.write_channel = write_channel      # the one ESC (index from 0) that may be written, once; None = read-only
        self.writes_left = 0 if write_channel is None else 1
        self.signatures = {}                    # channel -> bootloader signature from its init reply
        self.last_block = {}                    # channel -> the settings block as last read from it
        self.refusal = None                     # why the last write was refused (text), for tests and the record
        self.link = link
        self.esc_count = esc_count
        self.log = log or (lambda kind, channel, address, data: None)
        self.channel = None
        self.exit_requested = False

    # ---- MSP ----
    def handle_msp(self, kind, cmd, payload, ok):
        reply = msp_v1_reply if kind == "msp1" else msp_v2_reply
        if not ok:
            return reply(cmd, b"", error=True)
        if cmd == MSP_API_VERSION:
            return reply(cmd, bytes([0, 1, 42]))
        if cmd == MSP_FC_VARIANT:
            return reply(cmd, b"BTFL")
        if cmd == MSP_FC_VERSION:
            return reply(cmd, bytes([4, 5, 0]))
        if cmd == MSP_BATTERY_STATE:
            return reply(cmd, bytes(11))   # no battery known: cells 0, 0 V, 0 A
        if cmd == MSP_MOTOR_CONFIG:
            # min 1070, max 2000, mincommand 1000, motor count, poles, telemetry, esc sensor (count read at byte 6)
            return reply(cmd, bytes([0x2E, 0x04, 0xD0, 0x07, 0xE8, 0x03, self.esc_count, 0, 0, 0]))
        if cmd == MSP_MOTOR:
            values = bytearray(16)
            for i in range(self.esc_count):
                values[2 * i] = 0xE8
                values[2 * i + 1] = 0x03
            return reply(cmd, values)
        if cmd == MSP_SET_PASSTHROUGH and not payload:
            return reply(cmd, bytes([self.esc_count]))
        return reply(cmd, b"", error=True)

    # ---- 4-way ----
    def handle_four_way(self, frame):
        cmd = frame[1]
        address = (frame[2] << 8) | frame[3]
        count = frame[4] or 256
        params = frame[5:5 + count]
        crc_in = (frame[5 + count] << 8) | frame[6 + count]
        if crc_xmodem(frame[:5 + count]) != crc_in:
            return four_way_reply(cmd, address, b"\x00", ACK_I_INVALID_CRC)
        if cmd == CMD_DEVICE_WRITE and self.write_channel is not None:
            return self.write(address, params)
        if cmd not in ALLOWED_4WAY:
            self.log("refused", self.channel, address, bytes([cmd]) + bytes(params))
            if self.record_writes and cmd in WRITE_CLASS_4WAY:
                return four_way_reply(cmd, address, b"\x00", ACK_OK)
            return four_way_reply(cmd, address, b"\x00", ACK_I_INVALID_CMD)
        param = params[0]

        if cmd == CMD_TEST_ALIVE:
            if self.channel is not None:
                self.link.send(bootloader_frame([BL_KEEP_ALIVE, 0]))
                self.link.receive(1, 50)
            return four_way_reply(cmd, address, b"\x00", ACK_OK)
        if cmd == CMD_PROTOCOL_GET_VERSION:
            return four_way_reply(cmd, address, bytes([PROTOCOL_VERSION_4WAY]), ACK_OK)
        if cmd == CMD_INTERFACE_GET_NAME:
            return four_way_reply(cmd, address, INTERFACE_NAME, ACK_OK)
        if cmd == CMD_INTERFACE_GET_VERSION:
            return four_way_reply(cmd, address, bytes(INTERFACE_VERSION), ACK_OK)
        if cmd == CMD_INTERFACE_SET_MODE:
            return four_way_reply(cmd, address, b"\x00", ACK_OK if param == MODE_ARM_BLB else ACK_I_INVALID_PARAM)
        if cmd == CMD_INTERFACE_EXIT:
            self.exit_requested = True
            return four_way_reply(cmd, address, b"\x00", ACK_OK)
        if cmd == CMD_DEVICE_INIT_FLASH:
            return self.init_flash(address, param)
        if cmd == CMD_DEVICE_RESET:
            if param >= self.esc_count:
                return four_way_reply(cmd, address, b"\x00", ACK_I_INVALID_CHANNEL)
            self.select(param)
            self.link.send(bootloader_frame([BL_RUN, 0]))    # the bootloader starts the firmware on 00 00 00 00
            self.link.restart_esc()
            self.link.receive(DEVICE_INFO_LEN, 50)
            return four_way_reply(cmd, address, b"\x00", ACK_OK)
        return self.read(address, param)     # CMD_DEVICE_READ, the last allowed command

    def select(self, channel):
        if channel != self.channel:
            self.link.select(channel)
            self.channel = channel

    def init_flash(self, address, channel):
        if channel >= self.esc_count:
            return four_way_reply(CMD_DEVICE_INIT_FLASH, address, b"\x00\x00\x00\x00", ACK_I_INVALID_CHANNEL)
        self.select(channel)
        for kind, request in (("init", BOOT_INIT), ("init21", BOOT_INIT_LONG)):    # each carries its own checksum
            self.link.send(request)
            info = self.link.receive(DEVICE_INFO_LEN, 200)
            self.log(kind, channel, address, info)
            if len(info) >= 6 and info[-1] == BL_SUCCESS:
                self.signatures[channel] = (info[4] << 8) | info[5]
                return four_way_reply(CMD_DEVICE_INIT_FLASH, address, bytes([info[5], info[4], info[3], MODE_ARM_BLB]),
                                      ACK_OK)
        return four_way_reply(CMD_DEVICE_INIT_FLASH, address, bytes([0x06, 0x33, 0x67, MODE_ARM_BLB]), ACK_D_GENERAL_ERROR)

    def read(self, address, count):
        count = count or 256
        if self.channel is None:
            return four_way_reply(CMD_DEVICE_READ, address, b"\x00", ACK_D_GENERAL_ERROR)
        self.link.send(bootloader_frame([BL_SET_ADDRESS, 0, address >> 8, address & 0xFF]))
        ack = self.link.receive(1, 200)
        if ack != bytes([BL_SUCCESS]):
            return four_way_reply(CMD_DEVICE_READ, address, b"\x00", ACK_D_GENERAL_ERROR)
        self.link.send(bootloader_frame([BL_READ_FLASH, count & 0xFF]))
        data = self.link.receive(count + 3, 500)    # data, CRC low, CRC high, ACK
        if len(data) != count + 3 or data[-1] != BL_SUCCESS or crc_bootloader(data[:-1]) != 0:
            return four_way_reply(CMD_DEVICE_READ, address, b"\x00", ACK_D_GENERAL_ERROR)
        self.log("read", self.channel, address, data[:count])
        if address == SETTINGS_ADDRESS.get(self.signatures.get(self.channel)) and count >= SETTINGS_SIZE:
            self.last_block[self.channel] = bytes(data[:SETTINGS_SIZE])
        return four_way_reply(CMD_DEVICE_READ, address, data[:count], ACK_OK)

    def write_refusal(self, address, params):
        """None when this write is allowed, else why not. The only write the buddy makes: the settings block of the one
        granted ESC, once, with the protected bytes exactly as the buddy last read them from that ESC."""
        channel = self.channel
        if self.writes_left < 1:
            return "no write left in this session"
        if channel is None or channel != self.write_channel:
            return "not the ESC this session may write"
        if address != SETTINGS_ADDRESS.get(self.signatures.get(channel)):
            return "not the settings address"
        if len(params) != SETTINGS_SIZE:
            return "not the settings block size"
        if self.restore_block is not None:
            return None if params == self.restore_block else "not the block to restore"
        last = self.last_block.get(channel)
        if last is None:
            return "settings block not read from this ESC since the last write"
        if params[:PROTECTED_HEAD] != last[:PROTECTED_HEAD] or params[-PROTECTED_TAIL:] != last[-PROTECTED_TAIL:]:
            return "protected bytes differ from the block read"
        return None

    def write(self, address, params):
        params = bytes(params)
        self.refusal = self.write_refusal(address, params)
        if self.refusal is not None:
            self.log("refused", self.channel, address, bytes([CMD_DEVICE_WRITE]) + params)
            return four_way_reply(CMD_DEVICE_WRITE, address, b"\x00", ACK_I_INVALID_PARAM)
        self.writes_left -= 1                    # one attempt per session, whatever its outcome
        self.last_block[self.channel] = None     # the block on the ESC is no longer the one read
        self.log("write", self.channel, address, params)
        size = len(params)
        self.link.send(bootloader_frame([BL_SET_ADDRESS, 0, address >> 8, address & 0xFF]))
        ok = self.link.receive(1, 200) == bytes([BL_SUCCESS])
        if ok:
            self.link.send(bootloader_frame([BL_SET_BUFFER, 0, size >> 8, size & 0xFF]))    # no ack for this one
            self.link.send(bootloader_frame(params))
            ok = self.link.receive(1, 500) == bytes([BL_SUCCESS])
        if ok:
            self.link.send(bootloader_frame([BL_PROG_FLASH, 1]))      # erases the page, then programs the block
            ok = self.link.receive(1, 2000) == bytes([BL_SUCCESS])
        return four_way_reply(CMD_DEVICE_WRITE, address, b"\x00", ACK_OK if ok else ACK_D_GENERAL_ERROR)
