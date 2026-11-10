# Tests of the ESC buddy's firmware flash: the image file (tools/esc_buddy/esc_buddy_image.py), the Pico-side guards
# (esc_buddy_protocol.Passthrough with flash_image) and the PC tool's sequence (esc_buddy_run.fill_flash_report), against a fake
# F421 bootloader whose flash behaves like the real one: programming can only clear bits, and a page (1 KB) is erased only
# when a write starts on its first byte (AM32-bootloader's save_flash_nolib; the bench ESC's version 1 has no public source).

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "esc_buddy"))

import esc_buddy_image as image_module
import esc_buddy_protocol as p
from test_esc_buddy_protocol import DEVICE_INFO, esc_data, four_way, parse_reply
from test_esc_buddy_write import vendor_block

PAGE = 1024
SETTINGS = p.SETTINGS_ADDRESS[p.FLASH_SIGNATURE]
CHUNKS = (p.FLASH_END - p.FLASH_START) // p.FLASH_CHUNK
NAME = b"SKYSTARS_KM55_V2_F421"


def make_image(seed=1, name=NAME):
    """A firmware image of the right size: a vector table, noise, the file name where AM32 puts it."""
    image = bytearray(b"\xff" * (p.FLASH_END - p.FLASH_START))
    image[0:8] = (0x20003C00).to_bytes(4, "little") + (0x08004585).to_bytes(4, "little")
    value = seed
    for i in range(8, len(image) - 0x20, 3):
        value = (value * 1103515245 + 12345) & 0x7FFFFFFF
        image[i] = value >> 16 & 0xFF
    offset = image_module.NAME_OFFSET
    image[offset:offset + image_module.NAME_LENGTH] = name.ljust(image_module.NAME_LENGTH, b"\x00")
    return bytes(image)


def make_hex(image):
    """Intel HEX text for `image`, in 16-byte records at the F421's flash addresses."""
    def record(kind, address, body):
        raw = bytes([len(body), address >> 8, address & 0xFF, kind]) + bytes(body)
        return ":" + (raw + bytes([(-sum(raw)) & 0xFF])).hex().upper()
    lines = [record(4, 0, (0x0800).to_bytes(2, "big"))]
    for offset in range(0, len(image), 16):
        lines.append(record(0, p.FLASH_START + offset, image[offset:offset + 16]))
    lines.append(record(1, 0, b""))
    return "\n".join(lines) + "\n"


class FakeF421:
    """The ESC's bootloader on the link the Passthrough uses: 32 KB of flash, the old firmware at 0x1000.., settings at 0x7C00."""

    def __init__(self, firmware=None, block=None, signature=0x1F06, nack_address=None, corrupt_address=None, nack_settings=0):
        self.mem = bytearray(b"\xff" * 0x8000)
        self.mem[p.FLASH_START:p.FLASH_END] = firmware or make_image(seed=7)
        block = block or vendor_block()
        self.mem[SETTINGS:SETTINGS + len(block)] = block
        self.info = DEVICE_INFO[:4] + signature.to_bytes(2, "big") + DEVICE_INFO[6:]
        self.nack_address = nack_address          # the program step of this address gets no acknowledgement (nothing is written)
        self.nack_settings = nack_settings        # the first this many settings writes get no acknowledgement, nothing is written
        self.corrupt_address = corrupt_address    # the program step of this address is acknowledged but one bit is not written
        self.sent = []
        self.replies = []
        self.address = None
        self.payload_size = None
        self.pending_payload = False
        self.buffer = None
        self.programs = []
        self.selected = []

    def select(self, index):
        self.selected.append(index)

    def restart_esc(self):
        pass

    def receive(self, count, timeout_ms):
        return self.replies.pop(0) if self.replies else b""

    def send(self, data):
        data = bytes(data)
        self.sent.append(data)
        if data == p.BOOT_INIT or data == p.BOOT_INIT_LONG:
            self.replies.append(self.info)
        elif self.pending_payload:
            self.pending_payload = False
            assert len(data) == self.payload_size + 2 and p.crc_bootloader(data) == 0, "payload frame"
            self.buffer = data[:-2]
            self.replies.append(bytes([p.BL_SUCCESS]))
        else:
            assert p.crc_bootloader(data) == 0, "command frame CRC"
            cmd = data[0]
            if cmd == p.BL_SET_ADDRESS:
                self.address = (data[2] << 8) | data[3]
                self.replies.append(bytes([p.BL_SUCCESS]))
            elif cmd == p.BL_SET_BUFFER:
                self.payload_size = (data[2] << 8) | data[3]
                self.pending_payload = True
            elif cmd == p.BL_PROG_FLASH:
                self.program()
            elif cmd == p.BL_READ_FLASH:
                self.replies.append(esc_data(bytes(self.mem[self.address:self.address + (data[1] or 256)])))
            else:
                raise AssertionError("unexpected bootloader command %02x" % cmd)

    def program(self):
        assert self.address >= p.FLASH_START, "the bootloader area was written"
        self.programs.append(self.address)
        if self.address == self.nack_address:
            return
        if self.address == SETTINGS and self.nack_settings:
            self.nack_settings -= 1
            return
        if self.address % PAGE == 0:
            self.mem[self.address:self.address + PAGE] = b"\xff" * PAGE
        for i, byte in enumerate(self.buffer):
            self.mem[self.address + i] &= byte                      # programming only clears bits
        if self.address == self.corrupt_address:
            self.mem[self.address] &= 0xFE
        self.replies.append(bytes([p.BL_SUCCESS]))

    @property
    def boot_byte(self):
        return self.mem[SETTINGS]

    def firmware(self):
        return bytes(self.mem[p.FLASH_START:p.FLASH_END])


def new_session(image=None, link=None, esc=1, grant=1):
    """A Passthrough granted ESC `grant` to flash `image`, initialised on ESC `esc` and having read its settings block."""
    link = link or FakeF421()
    image = image if image is not None else make_image()
    passthrough = p.Passthrough(link, 4, write_channel=grant, flash_image=image)
    passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, bytes([esc])))
    passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, SETTINGS, b"\x00"))
    return link, passthrough, image


def send_write(passthrough, address, data):
    return parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_WRITE, address, data)))[3]


def read_chunk(passthrough, number):
    return parse_reply(passthrough.handle_four_way(four_way(
        p.CMD_DEVICE_READ, p.FLASH_START + p.FLASH_CHUNK * number, b"\x00")))[2]


def chunk_of(image, number):
    return image[number * p.FLASH_CHUNK:(number + 1) * p.FLASH_CHUNK]


def block_with_boot_byte(value, link):
    block = bytearray(link.mem[SETTINGS:SETTINGS + p.SETTINGS_SIZE])
    block[0] = value
    return bytes(block)


class ImageFile(unittest.TestCase):
    def test_a_hex_file_becomes_the_application_area_with_ff_between(self):
        image = make_image()
        self.assertEqual(image_module.image_from_hex(make_hex(image)), image)

    def test_gaps_are_filled_with_ff_as_the_configurator_does(self):
        lines = [line for line in make_hex(make_image()).splitlines()
                 if not p.FLASH_START + 0x400 <= int(line[3:7], 16) < p.FLASH_START + 0x500 or line[7:9] != "00"]
        got = image_module.image_from_hex("\n".join(lines) + "\n")
        self.assertEqual(got[0x400:0x500], b"\xff" * 0x100)
        self.assertEqual(got[:0x400], make_image()[:0x400])

    def test_the_name_is_read_from_the_image_and_from_the_32_bytes_of_an_esc(self):
        image = make_image()
        self.assertEqual(image_module.firmware_name(image), "SKYSTARS_KM55_V2_F421")
        offset = image_module.NAME_OFFSET
        self.assertEqual(image_module.firmware_name(image[offset:offset + 32]), "SKYSTARS_KM55_V2_F421")
        self.assertIsNone(image_module.firmware_name(b"\xff" * 32))

    def test_refused_files(self):
        text = make_hex(make_image())
        bad_checksum = text.replace(text.splitlines()[3], text.splitlines()[3][:-2] + "00", 1)
        cases = {
            "checksum": bad_checksum,
            "end-of-file": "\n".join(text.splitlines()[:-1]) + "\n",
            "not an Intel HEX": "hello\n",
            "holds no data": ":00000001FF\n",
        }
        for expected, content in cases.items():
            with self.assertRaises(ValueError, msg=expected) as caught:
                image_module.image_from_hex(content)
            self.assertIn(expected, str(caught.exception))

    def test_data_in_the_bootloader_or_the_settings_page_is_refused(self):
        def record(address, body):
            raw = bytes([len(body), address >> 8, address & 0xFF, 0]) + body
            return ":" + (raw + bytes([(-sum(raw)) & 0xFF])).hex().upper()
        head = ":02000004" + "0800" + "F2"
        for address in (0x0FF0, 0x7C00):
            text = "\n".join([head, record(address, b"\x00" * 16), ":00000001FF"]) + "\n"
            with self.assertRaises(ValueError) as caught:
                image_module.image_from_hex(text)
            self.assertIn("outside the application area", str(caught.exception))

    def test_something_that_is_no_firmware_is_refused(self):
        image = bytearray(make_image())
        image[0:4] = b"\x00\x00\x00\x00"
        with self.assertRaises(ValueError):
            image_module.image_from_hex(make_hex(bytes(image)))
        image = bytearray(make_image())
        image[4:8] = (0x08009001).to_bytes(4, "little")
        with self.assertRaises(ValueError):
            image_module.image_from_hex(make_hex(bytes(image)))
        with self.assertRaises(ValueError):
            image_module.image_from_hex(make_hex(make_image(name=b"lower case")))


class FlashGuards(unittest.TestCase):
    def test_a_good_flash_through_the_guards_leaves_the_image_and_the_boot_byte_set(self):
        link, passthrough, image = new_session()
        self.assertEqual(send_write(passthrough, SETTINGS, block_with_boot_byte(0, link)), p.ACK_OK)
        self.assertEqual(link.boot_byte, 0)
        for number in range(CHUNKS):
            self.assertEqual(send_write(passthrough, p.FLASH_START + 256 * number, chunk_of(image, number)), p.ACK_OK)
        for number in range(CHUNKS):
            self.assertEqual(read_chunk(passthrough, number), chunk_of(image, number))
        self.assertEqual(send_write(passthrough, SETTINGS, block_with_boot_byte(1, link)), p.ACK_OK)
        self.assertEqual(link.firmware(), image)
        self.assertEqual(link.boot_byte, 1)
        self.assertEqual(bytes(link.mem[SETTINGS + 1:SETTINGS + p.SETTINGS_SIZE]), vendor_block()[1:])

    def refused(self, passthrough, link, address, data, reason):
        before = len(link.sent)
        self.assertEqual(send_write(passthrough, address, data), p.ACK_I_INVALID_PARAM)
        self.assertEqual(len(link.sent), before, "a refused write must not touch the line")
        self.assertIn(reason, passthrough.refusal)

    def test_no_chunk_before_the_boot_byte_is_cleared(self):
        link, passthrough, image = new_session()
        self.refused(passthrough, link, p.FLASH_START, chunk_of(image, 0), "boot byte not cleared")

    def test_a_chunk_that_is_not_the_image_is_refused(self):
        link, passthrough, image = new_session()
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        wrong = bytearray(chunk_of(image, 3))
        wrong[100] ^= 1
        self.refused(passthrough, link, p.FLASH_START + 3 * 256, bytes(wrong), "not the image")
        self.refused(passthrough, link, p.FLASH_START + 4 * 256, chunk_of(image, 3), "not the image")      # right data, wrong place

    def test_only_whole_aligned_chunks_inside_the_application_area(self):
        link, passthrough, image = new_session()
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        self.refused(passthrough, link, p.FLASH_START + 0x80, chunk_of(image, 0), "whole chunk")
        self.refused(passthrough, link, p.FLASH_START, chunk_of(image, 0)[:128], "whole chunk")
        self.refused(passthrough, link, 0x0F00, chunk_of(image, 0), "not an address")           # the bootloader
        self.refused(passthrough, link, p.FLASH_END, chunk_of(image, 0), "settings block size")    # the settings page: only its block
        self.refused(passthrough, link, 0x8000, chunk_of(image, 0), "not an address")

    def test_the_boot_byte_cannot_come_back_before_the_whole_image_is_read_back_equal(self):
        link, passthrough, image = new_session()
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(1, link), "not been read back")     # nothing written yet
        for number in range(CHUNKS):
            send_write(passthrough, p.FLASH_START + 256 * number, chunk_of(image, number))
        for number in range(CHUNKS - 1):                     # one chunk short
            read_chunk(passthrough, number)
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(1, link), "not been read back")
        read_chunk(passthrough, CHUNKS - 1)
        self.assertEqual(send_write(passthrough, SETTINGS, block_with_boot_byte(1, link)), p.ACK_OK)

    def test_rewriting_a_chunk_takes_its_read_back_mark_away(self):
        link, passthrough, image = new_session()
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        for number in range(CHUNKS):
            send_write(passthrough, p.FLASH_START + 256 * number, chunk_of(image, number))
            read_chunk(passthrough, number)
        send_write(passthrough, p.FLASH_START + 5 * 256, chunk_of(image, 5))
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(1, link), "not been read back")
        read_chunk(passthrough, 5)
        self.assertEqual(send_write(passthrough, SETTINGS, block_with_boot_byte(1, link)), p.ACK_OK)

    def test_a_chunk_that_reads_back_different_loses_its_mark(self):
        link, passthrough, image = new_session()
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        for number in range(CHUNKS):
            send_write(passthrough, p.FLASH_START + 256 * number, chunk_of(image, number))
            read_chunk(passthrough, number)
        link.mem[p.FLASH_START + 9 * 256 + 17] &= 0x7F       # the ESC's flash no longer holds the image there
        read_chunk(passthrough, 9)
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(1, link), "not been read back")

    def test_the_settings_block_may_differ_only_in_the_boot_byte(self):
        link, passthrough, image = new_session()
        changed = bytearray(block_with_boot_byte(0, link))
        changed[26] = 180                                    # motor KV: a flash is not a settings change
        self.refused(passthrough, link, SETTINGS, bytes(changed), "beyond the boot byte")
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(0, link)[:100], "block size")
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(1, link), "cleared first")      # boot byte 1 before 0

    def test_clearing_the_boot_byte_may_be_repeated(self):
        link, passthrough, _ = new_session()
        for _ in range(3):
            self.assertEqual(send_write(passthrough, SETTINGS, block_with_boot_byte(0, link)), p.ACK_OK)

    def test_only_the_granted_esc(self):
        link, passthrough, image = new_session(esc=2, grant=1)          # ESC 3 selected, ESC 2 granted
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(0, link), "not the ESC")

    def test_only_an_f421_bootloader(self):
        link, passthrough, image = new_session(link=FakeF421(signature=0x3506))
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(0, link), "not an F421")

    def test_nothing_is_written_before_an_esc_is_selected_or_without_a_read_block(self):
        link = FakeF421()
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=make_image())
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(0, link), "not the ESC")
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        self.refused(passthrough, link, SETTINGS, block_with_boot_byte(0, link), "not read")

    def test_without_an_image_the_application_area_stays_closed(self):
        link = FakeF421()
        passthrough = p.Passthrough(link, 4, write_channel=1)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, SETTINGS, b"\x00"))
        self.refused(passthrough, link, p.FLASH_START, make_image()[:256], "not the settings address")

    def test_a_wrongly_sized_image_is_refused_at_construction(self):
        with self.assertRaises(ValueError):
            p.Passthrough(FakeF421(), 4, write_channel=1, flash_image=b"\xff" * 1000)

    def test_a_chunk_is_logged_by_checksum_not_by_content(self):
        logged = []
        link = FakeF421()
        image = make_image()
        passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry), write_channel=1, flash_image=image)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, SETTINGS, b"\x00"))
        send_write(passthrough, SETTINGS, block_with_boot_byte(0, link))
        send_write(passthrough, p.FLASH_START, chunk_of(image, 0))
        kind, channel, address, data = logged[-1]
        self.assertEqual((kind, channel, address, len(data)), ("chunk", 1, p.FLASH_START, 2))


try:
    import esc_buddy_run as run
except ImportError:           # the run tool needs pyserial
    run = None


class PicoOverSerial:
    """Stands in for the USB serial port: what the PC writes is framed and answered by the Passthrough, as the Pico would."""

    def __init__(self, passthrough):
        self.passthrough = passthrough
        self.reader = p.FrameReader()
        self.out = b""

    def reset_input_buffer(self):
        self.out = b""

    def write(self, data):
        for frame in self.reader.feed(data):
            if frame[0] == "4way":
                self.out += self.passthrough.handle_four_way(frame[1])

    def read(self, count):
        chunk, self.out = self.out[:count], self.out[count:]
        return chunk


class BackupFirst(FakeF421):
    """Fails the test if a write reaches the ESC before the backup file exists."""

    def __init__(self, backup, **kwargs):
        super().__init__(**kwargs)
        self.backup = backup

    def program(self):
        assert self.backup is None or self.backup.exists(), "a write reached the ESC before the backup was saved"
        super().program()


def run_tool(image, backup, **fake):
    """fill_flash_report against a fake ESC 2: (report, fake ESC)."""
    link = BackupFirst(backup, **fake)
    passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=image)
    report = {"at": "now", "esc": 2, "gpio": 7, "image": "fake.hex", "image_sha256": "0", "applied": False, "ok": False}
    run.fill_flash_report(PicoOverSerial(passthrough), report, 1, image, backup)
    return report, link


@unittest.skipIf(run is None, "needs pyserial")
class FlashTool(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.backup = Path(self.dir.name) / "tmp" / "esc2.json"
        self.backup.parent.mkdir()
        self.image = make_image()

    def test_a_read_that_fails_once_is_asked_again(self):
        class FlakyOnce(FakeF421):
            failed = False

            def send(self, data):
                super().send(data)
                if not self.failed and data[0] == p.BL_READ_FLASH and self.address == p.FLASH_START + 20 * 256:
                    self.failed = True
                    self.replies.pop()                   # the ESC's answer never arrives
        old = make_image(seed=7)
        link = FlakyOnce(firmware=old)
        passthrough = p.Passthrough(link, 4)
        stand_in = PicoOverSerial(passthrough)
        run.four_way(stand_in, p.CMD_DEVICE_INIT_FLASH, 0, b"\x01")
        chunks = run.read_chunks(stand_in)
        self.assertTrue(link.failed)
        self.assertEqual(b"".join(chunks), old)

    def test_the_flash_writes_the_image_and_leaves_the_boot_byte_set(self):
        report, link = run_tool(self.image, self.backup)
        self.assertTrue(report["ok"], report)
        self.assertEqual((report["boot_byte_cleared"], report["chunks_written"], report["readback_mismatch"],
                          report["boot_byte_restored"], report["page_after_matches"]), (True, CHUNKS, [], True, True))
        self.assertEqual(link.firmware(), self.image)
        self.assertEqual(link.boot_byte, 1)
        self.assertEqual(link.programs[0], SETTINGS)                    # the boot byte first
        self.assertEqual(link.programs[1:1 + CHUNKS], [p.FLASH_START + 256 * n for n in range(CHUNKS)])
        self.assertEqual(link.programs[-1], SETTINGS)                   # and back last
        saved = __import__("json").loads(self.backup.read_text())
        self.assertEqual(saved["settings_block_hex"], vendor_block().hex())

    def test_the_same_image_can_be_flashed_again(self):
        report, link = run_tool(self.image, self.backup, firmware=self.image)
        self.assertTrue(report["ok"], report)
        self.assertEqual(len(link.programs), CHUNKS + 2)               # every chunk is written even though it is equal

    def test_an_image_for_another_target_is_refused_before_anything_is_written(self):
        other = make_image(name=b"SOME_OTHER_TARGET")
        report, link = run_tool(other, self.backup)
        self.assertFalse(report["ok"])
        self.assertIn("target SOME_OTHER_TARGET", report["error"])
        self.assertEqual(link.programs, [])
        self.assertFalse(self.backup.exists())

    def test_another_mcu_is_refused(self):
        report, link = run_tool(self.image, self.backup, signature=0x3506)
        self.assertIn("only the F421", report["error"])
        self.assertEqual(link.programs, [])

    def test_a_chunk_that_never_gets_its_ack_stops_the_flash_with_the_boot_byte_cleared(self):
        report, link = run_tool(self.image, self.backup, nack_address=p.FLASH_START + 40 * 256)
        self.assertFalse(report["ok"])
        self.assertIn("chunk 40", report["error"])
        self.assertEqual(link.boot_byte, 0)                               # the ESC will stay in its bootloader
        self.assertEqual(link.programs.count(p.FLASH_START + 40 * 256), run.CHUNK_TRIES)
        self.assertNotIn("boot_byte_restored", report)

    def test_a_chunk_that_reads_back_wrong_keeps_the_boot_byte_cleared(self):
        report, link = run_tool(self.image, self.backup, corrupt_address=p.FLASH_START + 77 * 256)
        self.assertFalse(report["ok"])
        self.assertEqual(report["readback_mismatch"], ["0x%04X" % (p.FLASH_START + 77 * 256)])
        self.assertEqual(link.boot_byte, 0)
        self.assertNotIn("boot_byte_restored", report)

    def test_a_settings_write_whose_answer_is_lost_is_asked_again(self):
        report, link = run_tool(self.image, self.backup, nack_settings=1)          # the boot byte clear: first try unanswered
        self.assertTrue(report["ok"], report)
        self.assertEqual(report["boot_byte_cleared_acks"], [p.ACK_D_GENERAL_ERROR, p.ACK_OK])
        self.assertEqual(link.boot_byte, 1)

    def test_the_boot_byte_going_back_is_asked_again_too_and_reported_when_it_never_works(self):
        link = BackupFirst(self.backup)
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=self.image)
        report = {"at": "now", "esc": 2, "gpio": 7, "image": "fake.hex", "image_sha256": "0", "applied": False, "ok": False}
        run.fill_flash_report(PicoOverSerial(passthrough), report, 1, self.image, self.backup)
        self.assertTrue(report["ok"])
        self.assertEqual(report["boot_byte_restored_acks"], [p.ACK_OK])
        link = FakeF421()
        link.nack_settings = 3                                    # three unanswered tries
        report = {}
        link.mem[SETTINGS] = 0
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=self.image)
        stand_in = PicoOverSerial(passthrough)
        run.four_way(stand_in, p.CMD_DEVICE_INIT_FLASH, 0, b"\x01")
        run.four_way(stand_in, p.CMD_DEVICE_READ, SETTINGS, b"\x00")
        block = bytes([0]) + bytes(link.mem[SETTINGS + 1:SETTINGS + p.SETTINGS_SIZE])
        self.assertFalse(run.write_boot_byte(stand_in, SETTINGS, block, report, "x"))
        self.assertEqual(report["x_acks"], [p.ACK_D_GENERAL_ERROR] * 3)

    def test_a_flash_that_stopped_after_the_name_page_was_erased_can_be_run_again(self):
        # the file name sits in the last chunk (0x7B00), in the 1 KB page that the chunk at 0x7800 erases: a flash that stopped in between
        # leaves the name blank and the boot byte at 0
        block = bytearray(vendor_block())
        block[0] = 0
        link = BackupFirst(self.backup, block=bytes(block))
        link.mem[0x7800:0x7C00] = b"\xff" * 0x400
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=self.image)
        report = {"at": "now", "esc": 2, "gpio": 7, "image": "fake.hex", "image_sha256": "0", "applied": False, "ok": False}
        run.fill_flash_report(PicoOverSerial(passthrough), report, 1, self.image, self.backup)
        self.assertTrue(report["ok"], report)
        self.assertTrue(report["resuming_interrupted_flash"])
        self.assertEqual((link.firmware(), link.boot_byte), (self.image, 1))

    def test_a_blank_name_with_the_boot_byte_set_is_still_refused(self):
        # nothing of ours is under way (the boot byte is 1): an ESC without a readable name is not one this tool may write
        link = BackupFirst(self.backup)
        link.mem[0x7800:0x7C00] = b"\xff" * 0x400
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=self.image)
        report = {"at": "now", "esc": 2, "gpio": 7, "image": "fake.hex", "image_sha256": "0", "applied": False, "ok": False}
        run.fill_flash_report(PicoOverSerial(passthrough), report, 1, self.image, self.backup)
        self.assertFalse(report["ok"])
        self.assertIn("runs target None", report["error"])
        self.assertEqual(link.programs, [])

    def test_a_read_of_the_name_that_fails_once_is_asked_again(self):
        class FlakyName(FakeF421):
            failed = False

            def send(self, data):
                super().send(data)
                if not self.failed and data[0] == p.BL_READ_FLASH and self.address == 0x7BE0:
                    self.failed = True
                    self.replies.pop()
        link = FlakyName()
        passthrough = p.Passthrough(link, 4, write_channel=1, flash_image=self.image)
        report = {"at": "now", "esc": 2, "gpio": 7, "image": "fake.hex", "image_sha256": "0", "applied": False, "ok": False}
        run.fill_flash_report(PicoOverSerial(passthrough), report, 1, self.image, self.backup)
        self.assertTrue(link.failed)
        self.assertTrue(report["ok"], report)

    def test_a_page_with_data_beyond_the_block_stops_everything(self):
        block = vendor_block() + b"\xff" * 10 + b"\x01"
        report, link = run_tool(self.image, self.backup, block=block)
        self.assertIn("beyond the settings block", report["error"])
        self.assertEqual(link.programs, [])


@unittest.skipIf(run is None, "needs pyserial")
class FlashCommandLine(unittest.TestCase):
    def parse(self, *argv):
        return run.parse_args(list(argv))

    def test_there_is_no_plan_flash(self):
        with self.assertRaises(SystemExit):
            self.parse("--plan-flash", "a.hex", "--pins", "6", "--esc", "1")

    def test_a_flash_needs_a_backup_and_an_esc(self):
        args = self.parse("--flash", "a.hex", "--pins", "6", "--esc", "1", "--backup", "b.json")
        self.assertEqual(args.flash, "a.hex")
        with self.assertRaises(SystemExit):
            self.parse("--flash", "a.hex", "--pins", "6", "--esc", "1")
        with self.assertRaises(SystemExit):
            self.parse("--flash", "a.hex", "--pins", "6", "--backup", "b.json")

    def test_a_flash_takes_no_settings_to_set(self):
        with self.assertRaises(SystemExit):
            self.parse("--flash", "a.hex", "--pins", "6", "--esc", "1", "--backup", "b.json", "--set", "motor KV=2220")


if __name__ == "__main__":
    unittest.main()
