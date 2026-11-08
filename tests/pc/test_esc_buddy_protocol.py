# Tests of the ESC buddy's protocol half (tools/esc_buddy/esc_buddy_protocol.py): what the AM32 configurator
# gets back, what reaches the ESC line, and above all that nothing but the allowed read commands ever does.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "esc_buddy"))

import esc_buddy_protocol as p


class FakeLink:
    """Records what would go on the wire; answers from a queue of canned ESC replies."""

    def __init__(self, replies=()):
        self.replies = list(replies)
        self.selected = []
        self.sent = []
        self.restarts = 0

    def select(self, index):
        self.selected.append(index)

    def send(self, data):
        self.sent.append(bytes(data))

    def receive(self, count, timeout_ms):
        return self.replies.pop(0) if self.replies else b""

    def restart_esc(self):
        self.restarts += 1


def four_way(cmd, address=0, params=b"\x00"):
    body = bytes([p.LOCAL_ESCAPE, cmd, address >> 8, address & 0xFF, len(params) & 0xFF]) + bytes(params)
    crc = p.crc_xmodem(body)
    return body + bytes([crc >> 8, crc & 0xFF])


def parse_reply(reply):
    """(cmd, address, params, ack) of a 4-way reply, after checking its frame and CRC."""
    assert reply[0] == p.REMOTE_ESCAPE
    count = reply[4] or 256
    assert len(reply) == 5 + count + 3
    assert p.crc_xmodem(reply[:-2]) == (reply[-2] << 8) | reply[-1]
    return reply[1], (reply[2] << 8) | reply[3], reply[5:5 + count], reply[5 + count]


def esc_data(payload):
    """What the bootloader sends back for a read: data, its CRC (low byte first), ACK."""
    crc = p.crc_bootloader(payload)
    return bytes(payload) + bytes([crc & 0xFF, crc >> 8, p.BL_SUCCESS])


DEVICE_INFO = bytes([ord("4"), ord("7"), ord("1"), 0x64, 0x1F, 0x06, 0x06, 0x03, p.BL_SUCCESS])


class ReadOnly(unittest.TestCase):
    def test_every_command_outside_the_allow_list_is_refused_without_touching_the_line(self):
        for cmd in range(256):
            if cmd in p.ALLOWED_4WAY:
                continue
            link = FakeLink()
            logged = []
            passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry))
            got_cmd, _, _, ack = parse_reply(passthrough.handle_four_way(four_way(cmd, 0x7C00, b"\x01\x02\x03")))
            self.assertEqual((got_cmd, ack), (cmd, p.ACK_I_INVALID_CMD), hex(cmd))
            self.assertEqual((link.sent, link.selected, link.restarts), ([], [], 0), hex(cmd))
            self.assertEqual(logged[0][0], "refused")

    def test_write_erase_and_eeprom_write_are_not_allowed(self):
        for cmd in (0x38, 0x39, 0x3B, 0x3C, 0x3D, 0x3E, 0x40):    # erase all, page erase, write, C2CK, eeprom r/w, verify
            self.assertNotIn(cmd, p.ALLOWED_4WAY)

    def test_no_frame_sent_to_the_esc_is_a_write_or_erase(self):
        link = FakeLink([DEVICE_INFO, bytes([p.BL_SUCCESS]), esc_data(bytes(range(48)))])
        passthrough = p.Passthrough(link, 4)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x02"))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, 0x7C00, b"\x30"))
        passthrough.handle_four_way(four_way(p.CMD_TEST_ALIVE))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_RESET, 0, b"\x02"))
        for frame in link.sent[1:]:            # [0] is the boot init
            self.assertIn(frame[0], (p.BL_SET_ADDRESS, p.BL_READ_FLASH, p.BL_KEEP_ALIVE, p.BL_RUN))
        self.assertEqual(link.sent[0], p.BOOT_INIT)

    def test_a_refused_write_is_logged_in_full(self):
        logged = []
        passthrough = p.Passthrough(FakeLink(), 4, lambda *entry: logged.append(entry))
        passthrough.handle_four_way(four_way(0x3B, 0x7C00, b"\x01\x02\x03"))
        self.assertEqual(logged, [("refused", None, 0x7C00, b";\x01\x02\x03")])

    def test_record_writes_answers_ok_but_never_touches_the_line(self):
        for cmd in p.WRITE_CLASS_4WAY:
            link = FakeLink()
            logged = []
            passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry), record_writes=True)
            _, _, _, ack = parse_reply(passthrough.handle_four_way(four_way(cmd, 0x7C00, bytes(range(184)))))
            self.assertEqual(ack, p.ACK_OK, hex(cmd))
            self.assertEqual((link.sent, link.selected, link.restarts), ([], [], 0), hex(cmd))
            self.assertEqual(logged[0][3], bytes([cmd]) + bytes(range(184)))

    def test_record_writes_still_refuses_what_is_not_a_save_command(self):
        passthrough = p.Passthrough(FakeLink(), 4, record_writes=True)
        _, _, _, ack = parse_reply(passthrough.handle_four_way(four_way(0x3C, 0, b"\x00")))    # C2CK low
        self.assertEqual(ack, p.ACK_I_INVALID_CMD)

    def test_bad_crc_is_answered_and_nothing_is_sent(self):
        frame = bytearray(four_way(p.CMD_DEVICE_READ, 0x7C00, b"\x10"))
        frame[-1] ^= 0xFF
        link = FakeLink()
        _, _, _, ack = parse_reply(p.Passthrough(link, 4).handle_four_way(bytes(frame)))
        self.assertEqual(ack, p.ACK_I_INVALID_CRC)
        self.assertEqual(link.sent, [])


class FourWay(unittest.TestCase):
    def test_init_flash_selects_the_channel_and_maps_the_device_info(self):
        link = FakeLink([DEVICE_INFO])
        logged = []
        passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry))
        cmd, address, params, ack = parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x03")))
        self.assertEqual((cmd, address, ack), (p.CMD_DEVICE_INIT_FLASH, 0, p.ACK_OK))
        self.assertEqual(params, bytes([0x06, 0x1F, 0x64, p.MODE_ARM_BLB]))
        self.assertEqual(link.selected, [3])
        self.assertEqual(logged, [("init", 3, 0, DEVICE_INFO)])

    def test_init_flash_with_no_answer_tries_both_request_forms_then_errors(self):
        link = FakeLink()
        logged = []
        passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry))
        _, _, _, ack = parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x00")))
        self.assertEqual(ack, p.ACK_D_GENERAL_ERROR)
        self.assertEqual(link.sent, [p.BOOT_INIT, p.BOOT_INIT_LONG])
        self.assertEqual([entry[0] for entry in logged], ["init", "init21"])

    def test_init_flash_accepts_an_answer_to_the_long_form(self):
        link = FakeLink([b"", DEVICE_INFO])
        _, _, _, ack = parse_reply(p.Passthrough(link, 4).handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x00")))
        self.assertEqual(ack, p.ACK_OK)
        self.assertEqual(link.sent, [p.BOOT_INIT, p.BOOT_INIT_LONG])

    def test_init_flash_on_a_channel_that_does_not_exist(self):
        link = FakeLink()
        _, _, _, ack = parse_reply(p.Passthrough(link, 4).handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x04")))
        self.assertEqual(ack, p.ACK_I_INVALID_CHANNEL)
        self.assertEqual(link.sent, [])

    def test_read_sends_set_address_then_read_and_returns_the_data(self):
        data = bytes(range(48))
        link = FakeLink([DEVICE_INFO, bytes([p.BL_SUCCESS]), esc_data(data)])
        logged = []
        passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        cmd, address, params, ack = parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, 0x7C00, b"\x30")))
        self.assertEqual((cmd, address, params, ack), (p.CMD_DEVICE_READ, 0x7C00, data, p.ACK_OK))
        self.assertEqual(link.sent[1], p.bootloader_frame([p.BL_SET_ADDRESS, 0, 0x7C, 0x00]))
        self.assertEqual(link.sent[2], p.bootloader_frame([p.BL_READ_FLASH, 48]))
        self.assertEqual(logged[-1], ("read", 1, 0x7C00, data))

    def test_read_of_256_bytes_uses_length_zero(self):
        data = bytes(i & 0xFF for i in range(256))
        link = FakeLink([DEVICE_INFO, bytes([p.BL_SUCCESS]), esc_data(data)])
        passthrough = p.Passthrough(link, 4)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x00"))
        reply = passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, 0x1000, b"\x00"))
        self.assertEqual(reply[4], 0)
        self.assertEqual(parse_reply(reply)[2], data)

    def test_read_with_a_corrupt_esc_crc_is_a_general_error(self):
        bad = bytearray(esc_data(bytes(16)))
        bad[3] ^= 1
        link = FakeLink([DEVICE_INFO, bytes([p.BL_SUCCESS]), bytes(bad)])
        passthrough = p.Passthrough(link, 4)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x00"))
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, 0, b"\x10")))[3],
                         p.ACK_D_GENERAL_ERROR)

    def test_reply_echoes_the_request_address(self):
        # the configurator discards a reply whose command or address differs from its request
        reply = p.Passthrough(FakeLink(), 4).handle_four_way(four_way(p.CMD_PROTOCOL_GET_VERSION, 0x1234))
        self.assertEqual(parse_reply(reply)[:3], (p.CMD_PROTOCOL_GET_VERSION, 0x1234, bytes([108])))

    def test_interface_name_version_mode_and_exit(self):
        passthrough = p.Passthrough(FakeLink(), 4)
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_INTERFACE_GET_NAME)))[2], b"m4wFCIntf")
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_INTERFACE_GET_VERSION)))[2], bytes([200, 5]))
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_INTERFACE_SET_MODE, 0, b"\x04")))[3], p.ACK_OK)
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_INTERFACE_SET_MODE, 0, b"\x01")))[3],
                         p.ACK_I_INVALID_PARAM)
        self.assertFalse(passthrough.exit_requested)
        passthrough.handle_four_way(four_way(p.CMD_INTERFACE_EXIT))
        self.assertTrue(passthrough.exit_requested)

    def test_reset_starts_the_firmware_and_restarts_the_line(self):
        link = FakeLink()
        passthrough = p.Passthrough(link, 4)
        self.assertEqual(parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_RESET, 0, b"\x02")))[3], p.ACK_OK)
        self.assertEqual(link.sent, [bytes([0, 0, 0, 0])])     # CMD_RUN: the bootloader jumps on 00 00 00 00
        self.assertEqual((link.selected, link.restarts), ([2], 1))


class Msp(unittest.TestCase):
    def frames(self, data):
        return p.FrameReader().feed(data)

    def request_v1(self, cmd, payload=b""):
        checksum = len(payload) ^ cmd
        for byte in payload:
            checksum ^= byte
        return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([checksum])

    def answer(self, cmd, payload=b""):
        (frame,) = self.frames(self.request_v1(cmd, payload))
        reply = p.Passthrough(FakeLink(), 4).handle_msp(*frame)
        self.assertEqual(reply[:3], b"$M>")
        size = reply[3]
        self.assertEqual(reply[4], cmd)
        checksum = size ^ cmd
        for byte in reply[5:5 + size]:
            checksum ^= byte
        self.assertEqual(reply[5 + size], checksum)
        return reply[5:5 + size]

    def test_identifies_as_betaflight(self):
        self.assertEqual(self.answer(p.MSP_FC_VARIANT), b"BTFL")
        self.assertEqual(len(self.answer(p.MSP_API_VERSION)), 3)

    def test_motor_count_is_where_the_configurator_reads_it(self):
        self.assertEqual(self.answer(p.MSP_MOTOR_CONFIG)[6], 4)
        motors = self.answer(p.MSP_MOTOR)
        self.assertEqual(sum(1 for i in range(0, len(motors), 2) if motors[i] | motors[i + 1]), 4)

    def test_battery_state_is_answered(self):
        self.assertGreaterEqual(len(self.answer(p.MSP_BATTERY_STATE)), 8)

    def test_passthrough_reports_the_esc_count(self):
        self.assertEqual(self.answer(p.MSP_SET_PASSTHROUGH), bytes([4]))

    def test_unknown_command_gets_an_error_frame(self):
        (frame,) = self.frames(self.request_v1(200))
        self.assertEqual(p.Passthrough(FakeLink(), 4).handle_msp(*frame)[:3], b"$M!")

    def test_msp_v2_request_gets_a_v2_reply(self):
        body = bytes([0, p.MSP_FC_VARIANT, 0, 0, 0])
        (frame,) = self.frames(b"$X<" + body + bytes([p.crc8_dvb_s2(body)]))
        self.assertEqual(frame[:2], ("msp2", p.MSP_FC_VARIANT))
        reply = p.Passthrough(FakeLink(), 4).handle_msp(*frame)
        self.assertEqual(reply[:3], b"$X>")
        self.assertEqual(reply[8:12], b"BTFL")
        self.assertEqual(reply[-1], p.crc8_dvb_s2(reply[3:-1]))


class Frames(unittest.TestCase):
    def test_frames_split_across_reads_and_garbage_between_them(self):
        reader = p.FrameReader()
        whole = b"\x00\xff" + four_way(p.CMD_TEST_ALIVE) + b"zz" + b"$M<\x00\x01\x01"
        frames = []
        for i in range(len(whole)):
            frames += reader.feed(whole[i:i + 1])
        self.assertEqual([f[0] for f in frames], ["4way", "msp1"])
        self.assertEqual(frames[0][1], four_way(p.CMD_TEST_ALIVE))
        self.assertEqual(frames[1][1:], (1, b"", True))

    def test_four_way_length_zero_means_256_params(self):
        frame = four_way(p.CMD_DEVICE_READ, 0, bytes(256))
        self.assertEqual(len(frame), 5 + 256 + 2)
        self.assertEqual(p.FrameReader().feed(frame), [("4way", frame)])


class Checksums(unittest.TestCase):
    def test_long_boot_init_matches_the_bootloaders_second_offset(self):
        # decodeInput(): rxBuffer[20] == 0x7d, rxBuffer[12] == 13, rxBuffer[13] == 66; same bytes as direct.ts init()
        long_form = p.BOOT_INIT_LONG
        self.assertEqual((len(long_form), long_form[20], long_form[12], long_form[13]), (21, 0x7D, 13, 66))

    def test_boot_init_matches_what_the_bootloader_looks_for(self):
        # AM32-bootloader decodeInput(): rxBuffer[16] == 0x7d, rxBuffer[8] == 13, rxBuffer[9] == 66 -> sendDeviceInfo()
        self.assertEqual((len(p.BOOT_INIT), p.BOOT_INIT[16], p.BOOT_INIT[8], p.BOOT_INIT[9]), (17, 0x7D, 13, 66))

    def test_bootloader_crc_residue_is_zero(self):
        # a frame followed by its CRC (low byte first) checks to 0, which is how a read reply is verified
        self.assertEqual(p.crc_bootloader(p.bootloader_frame(b"\xff\x00\x7c\x00")), 0)

    def test_xmodem_check_value(self):
        self.assertEqual(p.crc_xmodem(b"123456789"), 0x31C3)


if __name__ == "__main__":
    unittest.main()
