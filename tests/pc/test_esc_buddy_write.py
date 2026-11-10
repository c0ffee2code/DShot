# Tests of the ESC buddy's write path: the settings encoders (tools/esc_buddy/esc_buddy_settings.py) and the Pico-side
# guards and bootloader sequence (esc_buddy_protocol.Passthrough with a write grant), against a fake bootloader that
# behaves as AM32-bootloader's bootloader/main.c and Mcu/f421/Src/eeprom.c do (version 19; the bench ESC's version 1 has no public
# source): set address, set buffer without an ack, payload with CRC, program flash, page erase when the address starts a
# 1 KB page, 4-byte alignment.

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "esc_buddy"))

import esc_buddy_protocol as p
import esc_buddy_settings as settings
from test_esc_buddy_protocol import DEVICE_INFO, esc_data, four_way, parse_reply

PAGE = 1024
ADDRESS = 0x7C00


def vendor_block():
    """A settings block as the bench ESCs hold one after AM32 2.22: layout 4, bootloader 1, KV 7180, 14 poles, beeper volume 1,
    and the bytes from 48 on at 0xFF (the first 48 are what the spare board's ESCs returned, read 2026-11-09)."""
    return bytes.fromhex(
        "0104010216a001000a64006400000000000000000101011a1864b30e0000010080808032003200000f0a0aff66060000"
    ) + b"\xff" * (p.SETTINGS_SIZE - 48)


class FakeBootloader:
    """Plays the ESC's bootloader on the link the Passthrough uses; `flash` is the settings page."""

    def __init__(self, block=None, nack_after_payload=False, no_prog_ack=False):
        self.flash = bytearray(block or vendor_block()).ljust(PAGE, b"\xff")
        self.sent = []
        self.replies = []
        self.address = None
        self.payload_size = None
        self.buffer = None
        self.pending_payload = False
        self.nack_after_payload = nack_after_payload
        self.no_prog_ack = no_prog_ack
        self.selected = []
        self.erases = 0

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
            self.replies.append(DEVICE_INFO)
        elif self.pending_payload:
            self.pending_payload = False
            assert len(data) == self.payload_size + 2 and p.crc_bootloader(data) == 0, "payload frame"
            self.buffer = data[:-2]
            self.replies.append(b"" if self.nack_after_payload else bytes([p.BL_SUCCESS]))
        else:
            assert p.crc_bootloader(data) == 0, "command frame CRC"
            cmd = data[0]
            if cmd == p.BL_SET_ADDRESS:
                self.address = (data[2] << 8) | data[3]
                self.replies.append(bytes([p.BL_SUCCESS]))
            elif cmd == p.BL_SET_BUFFER:
                self.payload_size = 256 if data[2] == 1 else data[3]
                self.pending_payload = True               # no ack for this one
            elif cmd == p.BL_PROG_FLASH:
                self.program()
            elif cmd == p.BL_READ_FLASH:
                offset = self.address - ADDRESS
                self.replies.append(esc_data(bytes(self.flash[offset:offset + (data[1] or 256)])))
            else:
                raise AssertionError("unexpected bootloader command %02x" % cmd)

    def program(self):
        """save_flash_nolib: alignment check, erase the page when the address starts one, program, compare."""
        if self.address % 4 or len(self.buffer) % 4 or self.address < ADDRESS:
            self.replies.append(b"\x00")
            return
        if self.address % PAGE == 0:
            self.flash = bytearray(b"\xff" * PAGE)
            self.erases += 1
        offset = self.address - ADDRESS
        self.flash[offset:offset + len(self.buffer)] = self.buffer
        if not self.no_prog_ack:
            self.replies.append(bytes([p.BL_SUCCESS]))


def session(link=None, write_channel=1, read=True):
    """A Passthrough granted ESC 2 (index 1), already initialised on it and, unless read=False, having read its block."""
    link = link or FakeBootloader()
    passthrough = p.Passthrough(link, 4, write_channel=write_channel)
    passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
    if read:
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, ADDRESS, b"\x00"))     # 0 = 256 bytes, as the run tool reads
    return link, passthrough


def write(passthrough, block, address=ADDRESS):
    return parse_reply(passthrough.handle_four_way(four_way(p.CMD_DEVICE_WRITE, address, block)))[3]


def changed(offset, value, block=None):
    block = bytearray(block or vendor_block())
    block[offset] = value
    return bytes(block)


class Encoders(unittest.TestCase):
    def test_kv_goes_in_steps_of_40_from_20(self):
        self.assertEqual(settings.encode_kv("2220"), 55)
        self.assertEqual(settings.encode_kv("7220"), 180)
        self.assertEqual(settings.encode_kv("20"), 0)
        self.assertEqual(settings.encode_kv("10220"), 255)

    def test_kv_off_the_grid_or_out_of_range_names_the_neighbours(self):
        for text in ("7200", "19", "10260", "-20"):
            with self.assertRaises(ValueError):
                settings.encode_kv(text)
        with self.assertRaises(ValueError) as caught:
            settings.encode_kv("7200")
        self.assertIn("7180", str(caught.exception))
        self.assertIn("7220", str(caught.exception))

    def test_poles_are_even_within_limits(self):
        self.assertEqual(settings.encode_poles("12"), 12)
        self.assertEqual((settings.encode_poles("2"), settings.encode_poles("36")), (2, 36))
        for text in ("13", "0", "38", "40", "-2"):                # the configurator's range is 2 to 36
            with self.assertRaises(ValueError):
                settings.encode_poles(text)

    def test_beeper_volume_is_0_to_11(self):
        self.assertEqual([settings.encode_beeper_volume(t) for t in ("0", "1", "11")], [0, 1, 11])
        for text in ("12", "-1"):
            with self.assertRaises(ValueError):
                settings.encode_beeper_volume(text)

    def test_beeper_volume_changes_only_byte_30(self):
        new, done = settings.apply_changes(vendor_block(), {"beeper volume": "5"})
        self.assertEqual(done, [("beeper volume", 30, 1, 5)])
        self.assertEqual([i for i in range(len(new)) if new[i] != vendor_block()[i]], [30])

    def test_every_writable_field_is_a_decoded_field_outside_the_protected_bytes(self):
        named = {name: offset for offset, name, _ in settings.FIELDS}
        for name, (offset, _) in settings.WRITABLE.items():
            self.assertEqual(named[name], offset, name)
            self.assertNotIn(offset, settings.PROTECTED, name)

    def test_fields_the_buddy_must_not_change_are_not_writable(self):
        for name in ("direction reversed", "3D mode (bi-direction)", "complementary PWM", "use hall sensors", "input type",
                     "running brake level", "servo low threshold", "car type reverse braking", "boot byte"):
            self.assertNotIn(name, settings.WRITABLE)

    def test_flags_are_written_as_exactly_0_or_1(self):
        for name in ("sinusoidal startup", "auto timing advance", "stuck rotor protection", "stall protection",
                     "disable stick calibration"):
            encode = settings.WRITABLE[name][1]
            self.assertEqual([encode(t) for t in ("on", "ON", "1", "off", "0")], [1, 1, 1, 0, 0])
            for text in ("2", "yes", "", "255"):
                with self.assertRaises(ValueError):
                    encode(text)

    def test_ranges_are_those_of_the_firmware_source(self):
        # AM32 2.22 loadEEpromSettings: (field, lowest accepted, highest accepted, stored byte of the lowest, of the highest)
        cases = [("timing advance steps", 0, 32, 10, 42), ("PWM frequency", 8, 144, 8, 144), ("startup power", 50, 150, 50, 150),
                 ("sine mode range", 5, 25, 5, 25), ("sine mode power", 1, 10, 1, 10), ("brake strength", 1, 10, 1, 10),
                 ("active brake power", 0, 5, 0, 5), ("current limit P gain", 0, 255, 0, 255),
                 ("current limit I gain", 0, 255, 0, 255), ("current limit D gain", 0, 255, 0, 255),
                 ("temperature limit", 70, 140, 70, 140), ("current limit", 2, 200, 1, 100)]
        for name, low, high, raw_low, raw_high in cases:
            encode = settings.WRITABLE[name][1]
            self.assertEqual((encode(str(low)), encode(str(high))), (raw_low, raw_high), name)
            for text in (str(low - 1), str(high + 1), "x"):
                with self.assertRaises(ValueError, msg="%s %s" % (name, text)):
                    encode(text)

    def test_current_limit_goes_in_whole_amps_of_two(self):
        encode = settings.WRITABLE["current limit"][1]
        self.assertEqual(encode("40"), 20)
        for text in ("41", "0", "202", "1"):
            with self.assertRaises(ValueError):
                encode(text)

    def test_low_voltage_threshold_is_2_50_to_3_50_volts(self):
        encode = settings.WRITABLE["low voltage threshold"][1]
        self.assertEqual((encode("2.5"), encode("3.0"), encode("3.50")), (0, 50, 100))
        for text in ("2.49", "3.51", "3.005", "abc"):
            with self.assertRaises(ValueError):
                encode(text)

    def test_low_voltage_threshold_refuses_non_finite_numbers_with_a_value_error(self):
        encode = settings.WRITABLE["low voltage threshold"][1]
        for text in ("inf", "-inf", "nan", "1e400"):
            with self.assertRaises(ValueError):
                encode(text)

    def test_temperature_and_current_limit_can_be_switched_off_again(self):
        # the firmware replaces a temperature outside 70-140 by 255 (no limit) and ignores a current of 0 or above 100 (no limit);
        # the temperature goes back as the 255 the firmware itself stores, the current as 0
        for name, raw in (("temperature limit", 255), ("current limit", 0)):
            encode = settings.WRITABLE[name][1]
            self.assertEqual((encode("disabled"), encode("OFF")), (raw, raw), name)
            on, _ = settings.apply_changes(vendor_block(), {name: "100" if name == "temperature limit" else "40"})
            off, _ = settings.apply_changes(on, {name: "disabled"})
            self.assertEqual(off[settings.WRITABLE[name][0]], raw)
            self.assertEqual({n: t for _, n, _, t in settings.decode(off)}[name], "disabled")

    def test_other_fields_do_not_accept_disabled(self):
        for name in ("startup power", "PWM frequency", "timing advance steps"):
            with self.assertRaises(ValueError):
                settings.WRITABLE[name][1]("disabled")

    def test_timing_advance_decoder_agrees_with_the_encoder(self):
        encode = settings.WRITABLE["timing advance steps"][1]
        for steps in (0, 16, 32):
            self.assertEqual(settings.advance(encode(str(steps))), "%d steps (about %.1f deg)" % (steps, steps * 30 / 32))
        self.assertIn("firmware uses 16 steps", settings.advance(4))
        self.assertIn("firmware uses 16 steps", settings.advance(50))

    def test_encoded_values_decode_back_for_every_writable_field(self):
        asks = {"sinusoidal startup": "on", "PWM type": "by RPM", "timing advance steps": "24", "auto timing advance": "on",
                "PWM frequency": "48", "startup power": "120", "sine mode range": "10", "sine mode power": "4",
                "brake on stop": "active brake", "brake strength": "5", "stuck rotor protection": "off", "stall protection": "on",
                "low voltage cut off": "absolute", "low voltage threshold": "3.2", "temperature limit": "100", "current limit": "60",
                "minimum duty cycle": "10", "demag compensation": "1/32 power cut", "ramp rate": "12.5", "absolute voltage cutoff": "9.5",
                "active brake power": "3", "disable stick calibration": "on"}
        shown = {"sinusoidal startup": "on", "PWM type": "by RPM", "timing advance steps": "24 steps (about 22.5 deg)",
                 "auto timing advance": "on", "PWM frequency": "48 kHz", "startup power": "120 %", "sine mode range": "10 %",
                 "sine mode power": "4", "brake on stop": "active brake", "brake strength": "5", "stuck rotor protection": "off",
                 "stall protection": "on", "low voltage cut off": "absolute", "low voltage threshold": "3.20 V per cell",
                 "temperature limit": "100 C", "current limit": "60 A", "minimum duty cycle": "10.0 %",
                 "demag compensation": "1/32 power cut", "ramp rate": "12.5 % duty cycle per ms", "absolute voltage cutoff": "9.5 V",
                 "active brake power": "3 %", "disable stick calibration": "on"}
        new, done = settings.apply_changes(vendor_block(), asks)
        self.assertEqual(len(done), len(asks))
        decoded = {name: text for _, name, _, text in settings.decode(new)}
        for name, text in shown.items():
            self.assertEqual(decoded[name], text, name)
        changed = {i for i in range(len(new)) if new[i] != vendor_block()[i]}
        self.assertLessEqual(changed, {offset for offset, _ in (settings.WRITABLE[n] for n in asks)})

    def test_a_value_that_is_not_a_number_is_refused(self):
        for encode in (settings.encode_kv, settings.encode_poles, settings.encode_beeper_volume):
            with self.assertRaises(ValueError):
                encode("twelve")

    def test_encoded_values_decode_back_to_what_was_asked(self):
        new, _ = settings.apply_changes(vendor_block(), {"motor KV": "7220", "motor poles": "12"})
        decoded = {name: text for _, name, _, text in settings.decode(new)}
        self.assertEqual((decoded["motor KV"], decoded["motor poles"]), ("7220", "12"))

    def test_only_the_named_bytes_change(self):
        new, done = settings.apply_changes(vendor_block(), {"motor KV": "7220", "motor poles": "12"})
        differ = [i for i in range(len(new)) if new[i] != vendor_block()[i]]
        self.assertEqual(differ, [26, 27])
        self.assertEqual(done, [("motor KV", 26, 179, 180), ("motor poles", 27, 14, 12)])

    def test_nothing_else_can_be_named(self):
        for name in ("firmware name", "bootloader version", "EEPROM layout version", "boot byte", "brake on zero throttle", "motor Kv"):
            with self.assertRaises(ValueError):
                settings.apply_changes(vendor_block(), {name: "1"})

    def test_writable_fields_never_touch_protected_bytes(self):
        for name, (offset, _) in settings.WRITABLE.items():
            self.assertNotIn(offset, settings.PROTECTED, name)
            self.assertIn(name, [field for _, field, _ in settings.FIELDS])

    def test_protected_bytes_are_the_first_5_and_the_last_16(self):
        self.assertEqual(settings.PROTECTED, tuple(range(5)) + tuple(range(176, 192)))

    def test_a_field_name_that_does_not_exist_in_layout_4_is_refused(self):
        # 'timing advance' (2.16 levels of 7.5 degrees) is gone: the field is 'timing advance steps' (2.22 steps of about 0.94 degree)
        with self.assertRaises(ValueError) as caught:
            settings.apply_changes(vendor_block(), {"timing advance": "2"})
        self.assertIn("timing advance steps", str(caught.exception))          # the message lists the writable names

    def test_the_three_state_fields_take_a_name_or_a_number_and_nothing_else(self):
        for name, names in (("PWM type", settings.PWM_TYPES), ("low voltage cut off", settings.LOW_VOLTAGE_MODES),
                            ("brake on stop", settings.BRAKE_MODES), ("demag compensation", settings.DEMAG_LEVELS)):
            encode = settings.WRITABLE[name][1]
            for raw, text in names.items():
                self.assertEqual((encode(text), encode(text.upper()), encode(str(raw))), (raw, raw, raw), name)
            for bad in ("", "on", str(len(names)), "-1", "7"):
                with self.assertRaises(ValueError, msg="%s %r" % (name, bad)):
                    encode(bad)

    def test_ramp_rate_and_absolute_voltage_ranges(self):
        ramp = settings.WRITABLE["ramp rate"][1]
        self.assertEqual([ramp(t) for t in ("0.1", "1", "16", "20")], [1, 10, 160, 200])
        for text in ("0", "0.05", "20.1", "nan", "x"):
            with self.assertRaises(ValueError, msg=text):
                ramp(text)
        volts = settings.WRITABLE["absolute voltage cutoff"][1]
        self.assertEqual([volts(t) for t in ("0.5", "5", "50")], [1, 10, 100])
        for text in ("0", "0.25", "50.5", "inf"):
            with self.assertRaises(ValueError, msg=text):
                volts(text)

    def test_minimum_duty_cycle_is_percent_in_half_steps_up_to_25(self):
        encode = settings.WRITABLE["minimum duty cycle"][1]
        self.assertEqual([encode(t) for t in ("0", "0.5", "10", "25")], [0, 1, 20, 50])
        for text in ("0.25", "-0.5", "25.5", "26", "nan", "inf", "x"):
            with self.assertRaises(ValueError, msg=text):
                encode(text)


class WriteSequence(unittest.TestCase):
    def test_a_good_write_sends_the_four_steps_and_leaves_the_page_as_block_plus_blank(self):
        link, passthrough = session()
        new = changed(26, 180)
        self.assertEqual(write(passthrough, new), p.ACK_OK)
        self.assertEqual(link.sent[-4], p.bootloader_frame([p.BL_SET_ADDRESS, 0, 0x7C, 0x00]))
        self.assertEqual(link.sent[-3], p.bootloader_frame([p.BL_SET_BUFFER, 0, 0x00, 0xC0]))
        self.assertEqual(link.sent[-2], p.bootloader_frame(new))
        self.assertEqual(link.sent[-1], p.bootloader_frame([p.BL_PROG_FLASH, 1]))
        self.assertEqual(bytes(link.flash), new + b"\xff" * (PAGE - len(new)))
        self.assertEqual(link.erases, 1)

    def test_the_write_is_logged_with_its_data(self):
        logged = []
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, lambda *entry: logged.append(entry), write_channel=1)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, ADDRESS, b"\x00"))
        new = changed(27, 12)
        write(passthrough, new)
        self.assertEqual(logged[-1], ("write", 1, ADDRESS, new))

    def test_no_ack_to_the_payload_is_a_general_error_and_no_program_is_sent(self):
        link, passthrough = session(FakeBootloader(nack_after_payload=True))
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_D_GENERAL_ERROR)
        self.assertNotIn(p.bootloader_frame([p.BL_PROG_FLASH, 1]), link.sent)
        self.assertEqual(bytes(link.flash), vendor_block() + b"\xff" * (PAGE - p.SETTINGS_SIZE))

    def test_no_ack_to_the_program_step_is_a_general_error(self):
        _, passthrough = session(FakeBootloader(no_prog_ack=True))
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_D_GENERAL_ERROR)

    def test_only_one_write_per_session_even_after_a_failure(self):
        link, passthrough = session(FakeBootloader(nack_after_payload=True))
        write(passthrough, changed(26, 180))
        sent = len(link.sent)
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_I_INVALID_PARAM)
        self.assertEqual(len(link.sent), sent)
        self.assertEqual(passthrough.refusal, "no write left in this session")


class WriteGuards(unittest.TestCase):
    def refused(self, passthrough, link, block, address=ADDRESS, reason=None):
        sent = len(link.sent)
        self.assertEqual(write(passthrough, block, address), p.ACK_I_INVALID_PARAM)
        self.assertEqual(len(link.sent), sent, "a refused write must put nothing on the line")
        if reason:
            self.assertEqual(passthrough.refusal, reason)

    def test_without_a_grant_the_write_command_is_invalid_and_the_line_untouched(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, ADDRESS, b"\x00"))
        sent = len(link.sent)
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_I_INVALID_CMD)
        self.assertEqual(len(link.sent), sent)

    def test_record_writes_without_a_grant_still_sends_nothing(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, record_writes=True)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        sent = len(link.sent)
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_OK)
        self.assertEqual(len(link.sent), sent)

    def test_another_esc_than_the_granted_one(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, write_channel=1)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x02"))       # ESC 3 selected
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, ADDRESS, b"\x00"))
        self.refused(passthrough, link, changed(26, 180), reason="not the ESC this session may write")

    def test_no_esc_selected_yet(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, write_channel=1)
        self.refused(passthrough, link, changed(26, 180), reason="not the ESC this session may write")

    def test_another_address(self):
        link, passthrough = session()
        for address in (0x0000, 0x7BE0, 0x7C04, 0x7D00, 0xF800):
            self.refused(passthrough, link, changed(26, 180), address, "not the settings address")

    def test_another_length(self):
        link, passthrough = session()
        for length in (1, 183, 185, 255):
            block = (changed(26, 180) + b"\xff" * 80)[:length]
            self.refused(passthrough, link, block, reason="not the settings block size")

    def test_block_not_read_from_this_esc_first(self):
        link, passthrough = session(read=False)
        self.refused(passthrough, link, changed(26, 180), reason="settings block not read from this ESC since the last write")

    def test_a_read_of_something_else_does_not_count_as_the_block(self):
        link, passthrough = session(read=False)
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_READ, ADDRESS, bytes([32])))     # too short to be the block
        self.refused(passthrough, link, changed(26, 180), reason="settings block not read from this ESC since the last write")

    def test_each_protected_byte_is_guarded(self):
        link, passthrough = session()
        for offset in settings.PROTECTED:
            self.refused(passthrough, link, changed(offset, vendor_block()[offset] ^ 0xFF), reason="protected bytes differ from the block read")

    def test_the_configurators_own_save_frame_is_refused(self):
        link, passthrough = session()
        configurator = bytearray(changed(26, 180))
        configurator[5:17] = b"\xff" * 12            # what it sent on 2026-11-05
        configurator[176:184] = b"\xfd" * 8
        self.refused(passthrough, link, bytes(configurator), reason="protected bytes differ from the block read")

    def test_an_unknown_bootloader_signature_gets_no_write(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, write_channel=1)
        info = bytearray(DEVICE_INFO)
        info[4], info[5] = 0xAB, 0xCD
        link.replies.append(bytes(info))
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x01"))
        self.refused(passthrough, link, changed(26, 180), reason="not the settings address")

    def test_a_write_makes_the_next_one_need_a_new_read(self):
        link, passthrough = session()
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_OK)
        self.assertIsNone(passthrough.last_block[1])

    def test_read_only_session_cannot_be_turned_into_a_write_by_the_allow_list(self):
        self.assertNotIn(p.CMD_DEVICE_WRITE, p.ALLOWED_4WAY)


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


class BackupFirst(FakeBootloader):
    """Fails the test if the program step arrives before the backup file exists."""

    def __init__(self, backup, **kwargs):
        super().__init__(**kwargs)
        self.backup = backup

    def program(self):
        assert self.backup.exists(), "the write reached the ESC before the backup was saved"
        super().program()


def run_tool(flash_block, changes, backup, restore=None, grant=3, **fake):
    """fill_report against a fake ESC 4: (report, fake bootloader)."""
    link = (BackupFirst(backup, block=flash_block, **fake) if backup else FakeBootloader(flash_block, **fake))
    passthrough = p.Passthrough(link, 4, write_channel=grant, restore_block=restore)
    report = {"at": "now", "esc": 4, "gpio": 9, "applied": False, "ok": False}
    run.fill_report(PicoOverSerial(passthrough), report, 3, changes, backup, restore)
    return report, link


@unittest.skipIf(run is None, "needs pyserial")
class RunTool(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.backup = Path(self.dir.name) / "tmp" / "esc4.json"

    def test_the_report_names_each_change_with_its_old_and_new_value(self):
        self.backup.parent.mkdir()
        report, link = run_tool(None, {"motor KV": "7220", "motor poles": "12"}, self.backup)
        self.assertTrue(report["ok"], report)
        self.assertEqual([(c["field"], c["old"], c["new"]) for c in report["changes"]],
                         [("motor KV", "7180", "7220"), ("motor poles", "14", "12")])
        self.assertEqual(report["new_block_hex"], changed(27, 12, changed(26, 180)).hex())

    def test_a_write_saves_the_page_first_writes_and_confirms_with_the_read_back(self):
        self.backup.parent.mkdir()
        report, link = run_tool(None, {"motor KV": "7220"}, self.backup)
        self.assertEqual((report["write_ack"], report["readback_matches"], report["ok"], report["applied"]), (0, True, True, True))
        new = changed(26, 180)
        self.assertEqual(bytes(link.flash), new + b"\xff" * (PAGE - len(new)))
        import json
        saved = json.loads(self.backup.read_text())
        self.assertEqual(saved["settings_block_hex"], vendor_block().hex())
        self.assertEqual(saved["page_hex"], (vendor_block() + b"\xff" * (PAGE - p.SETTINGS_SIZE)).hex())
        self.assertEqual((saved["esc"], saved["gpio"], saved["address"]), (4, 9, "0x7C00"))

    def test_a_program_step_that_is_not_acknowledged_is_a_failure_with_the_read_back_reported(self):
        self.backup.parent.mkdir()
        report, _ = run_tool(None, {"motor KV": "7220"}, self.backup, no_prog_ack=True)
        self.assertFalse(report["ok"])
        self.assertEqual(report["write_ack"], p.ACK_D_GENERAL_ERROR)
        self.assertIsNotNone(report["readback_block_hex"])

    def test_nothing_to_change_is_not_written(self):
        report, link = run_tool(None, {"motor KV": "7180"}, self.backup)
        self.assertFalse(report["ok"])
        self.assertIn("nothing to change", report["error"])
        self.assertFalse(self.backup.exists())
        self.assertNotIn(p.bootloader_frame([p.BL_PROG_FLASH, 1]), link.sent)

    def test_data_beyond_the_block_in_the_page_stops_everything(self):
        flash = vendor_block() + b"\xff" * 10 + b"\x01" + b"\xff" * (PAGE - p.SETTINGS_SIZE - 11)
        report, link = run_tool(flash, {"motor KV": "7220"}, self.backup)
        self.assertFalse(report["ok"])
        self.assertIn("beyond the settings block", report["error"])
        self.assertFalse(self.backup.exists())
        self.assertNotIn(p.bootloader_frame([p.BL_PROG_FLASH, 1]), link.sent)

    def test_restore_puts_the_saved_block_back(self):
        self.backup.parent.mkdir()
        run_tool(None, {"motor KV": "7220"}, self.backup)         # the first write, as the tool does it
        edited = changed(26, 180)
        again = Path(self.dir.name) / "tmp" / "esc4_now.json"
        report, link = run_tool(edited, {}, again, restore=vendor_block())
        self.assertTrue(report["ok"], report)
        self.assertEqual([(c["field"], c["old"], c["new"]) for c in report["changes"]], [("motor KV", "7220", "7180")])
        self.assertEqual(bytes(link.flash), vendor_block() + b"\xff" * (PAGE - p.SETTINGS_SIZE))

    def test_restore_works_on_a_page_the_erase_left_blank(self):
        self.backup.parent.mkdir()
        report, link = run_tool(b"\xff" * PAGE, {}, self.backup, restore=vendor_block())
        self.assertTrue(report["ok"], report)
        self.assertEqual(bytes(link.flash), vendor_block() + b"\xff" * (PAGE - p.SETTINGS_SIZE))
        self.assertEqual(report["changes"][0]["old"], "unreadable")

    def test_the_pico_writes_nothing_but_the_block_it_was_started_to_restore(self):
        link = FakeBootloader()
        passthrough = p.Passthrough(link, 4, write_channel=3, restore_block=vendor_block())
        passthrough.handle_four_way(four_way(p.CMD_DEVICE_INIT_FLASH, 0, b"\x03"))
        sent = len(link.sent)
        self.assertEqual(write(passthrough, changed(26, 180)), p.ACK_I_INVALID_PARAM)
        self.assertEqual(passthrough.refusal, "not the block to restore")
        self.assertEqual(len(link.sent), sent)

    def test_restore_refuses_a_backup_of_another_esc_or_a_broken_file(self):
        import json
        saved = Path(self.dir.name) / "b.json"
        saved.write_text(json.dumps({"esc": 2, "gpio": 7, "settings_block_hex": vendor_block().hex()}))
        with self.assertRaises(SystemExit) as caught:
            run.restore_settings([6, 7, 8, 9], 4, str(saved), None)
        self.assertIn("not of ESC 4", str(caught.exception))
        saved.write_text("not json")
        with self.assertRaises(SystemExit):
            run.restore_settings([6, 7, 8, 9], 4, str(saved), None)
        saved.write_text(json.dumps({"esc": 4, "gpio": 9, "settings_block_hex": "ff" * p.SETTINGS_SIZE}))     # blank: does not decode
        with self.assertRaises(SystemExit):
            run.restore_settings([6, 7, 8, 9], 4, str(saved), None)


@unittest.skipIf(run is None, "needs pyserial")
class CommandLine(unittest.TestCase):
    """--write / --restore write and need --backup; --plan, --plan-restore and --apply no longer exist."""
    PINS = ["--pins", "6", "7", "8", "9"]

    def parse(self, *argv):
        import contextlib
        import io
        with contextlib.redirect_stderr(io.StringIO()) as error:
            try:
                return run.parse_args(list(argv))
            except SystemExit:
                return error.getvalue() or "refused"

    def test_the_plan_actions_are_gone(self):
        for argv in (["--plan", "--set", "motor KV=7180"], ["--plan-restore", "tmp/b.json"], ["--plan-flash", "a.hex"]):
            self.assertIsInstance(self.parse(*argv, *self.PINS, "--esc", "2"), str, argv)

    def test_a_write_needs_a_backup_and_writes(self):
        args = self.parse("--write", *self.PINS, "--esc", "2", "--set", "motor KV=7180", "--backup", "tmp/x.json")
        self.assertTrue(args.write)
        self.assertEqual(args.backup, "tmp/x.json")
        self.assertIn("--write needs --backup", self.parse("--write", *self.PINS, "--esc", "2", "--set", "motor KV=7180"))

    def test_a_restore_needs_a_backup_and_writes(self):
        args = self.parse("--restore", "tmp/b.json", *self.PINS, "--esc", "2", "--backup", "tmp/now.json")
        self.assertEqual(args.restore, "tmp/b.json")
        self.assertIn("--restore needs --backup", self.parse("--restore", "tmp/b.json", *self.PINS, "--esc", "2"))

    def test_the_old_apply_flag_is_gone(self):
        for argv in (["--write", "--apply"], ["--restore", "b.json", "--apply"]):
            self.assertIsInstance(self.parse(*argv, *self.PINS, "--esc", "2", "--set", "motor KV=7180", "--backup", "x"), str)

    def test_missing_pins_esc_or_set_is_refused(self):
        self.assertIn("--write needs --pins", self.parse("--write", "--esc", "2", "--set", "motor KV=7180", "--backup", "x"))
        self.assertIn("--write needs --esc", self.parse("--write", *self.PINS, "--set", "motor KV=7180", "--backup", "x"))
        self.assertIn("--write needs --set", self.parse("--write", *self.PINS, "--esc", "2", "--backup", "x"))

    def test_the_other_actions_take_none_of_the_write_options(self):
        for action in (["--read"], ["--passthrough"]):
            for extra in (["--esc", "2"], ["--set", "motor KV=7180"], ["--backup", "x"]):
                self.assertIsInstance(self.parse(*action, *self.PINS, *extra), str, (action, extra))
        self.assertEqual(self.parse("--debug").debug, True)
        self.assertTrue(self.parse("--read", *self.PINS).read)

    def test_only_one_action_at_a_time(self):
        self.assertIsInstance(self.parse("--read", "--write", *self.PINS, "--esc", "2", "--set", "motor KV=7180"), str)


if __name__ == "__main__":
    unittest.main()
