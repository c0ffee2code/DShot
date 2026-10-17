# driver/gcr_decode.py: decoding an ESC's GCR telemetry reply.
#
# analyze_frame() is the only entry point now - the frame receiver
# (dshot_bidir_rx_frame) already reconstructs the 21-bit frame in hardware,
# so this module just decodes it. (There used to be a second path,
# analyze_capture(), for the older sample receiver's raw oversampled
# waveform - removed 2026-10-11 along with that receiver; its own
# reconstruction math and tests moved to scripts/simulate_frame_receiver.py
# and tests/pc/test_simulate_frame_receiver.py.)
#
# Covers every 12-bit value via a synthetic reply: GCR-encoded, turned into
# a 21-bit frame the way the frame receiver would hand it over, then
# decoded. scripts/verify_gcr_decode_port.py cross-checks the same path
# against an independent PC-side reference; this file is what runs without it.

import sys
import unittest
from pathlib import Path

import fakes  # noqa: F401  puts driver/ on sys.path
import gcr_decode

# The GCR code AM32 uses (specification/AM32.md#11), written out here so the
# encoder in this file does not share a table with the decoder under test.
GCR_CODE = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
]


def crc_inverted(data12):
    plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    return plain ^ 0xF


def reply_number(data12):
    """The 16-bit number AM32 sends: 12 data bits and the inverted CRC."""
    return (data12 << 4) | crc_inverted(data12)


def nibbles16(number):
    return ((number >> 12) & 0xF, (number >> 8) & 0xF, (number >> 4) & 0xF, number & 0xF)


def frame_for(symbols):
    """A 21-bit frame (marker 0) whose differential decoding is the given 5-bit symbols."""
    decoded = 0
    for symbol in symbols:
        decoded = (decoded << 5) | symbol
    level = 0
    data = 0
    for shift in range(19, -1, -1):
        level ^= (decoded >> shift) & 1
        data = (data << 1) | level
    return data  # marker bit 0 above


class CrcTest(unittest.TestCase):
    def test_inverted_crc_is_accepted_and_returns_the_data(self):
        for data12 in (0, 1, 1330, 1881, 4095):
            kind, data = gcr_decode.check_crc(reply_number(data12))
            self.assertEqual((kind, data), ("inverted", data12))

    def test_plain_crc_is_rejected(self):
        for data12 in (0, 1330, 1881):
            plain = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
            kind, data = gcr_decode.check_crc((data12 << 4) | plain)
            self.assertIsNone(kind)
            self.assertEqual(data, data12)

    def test_a_flipped_bit_is_rejected(self):
        number = reply_number(1881)
        for bit in range(16):
            self.assertIsNone(gcr_decode.check_crc(number ^ (1 << bit))[0], "bit %d" % bit)


class DecodeTest(unittest.TestCase):
    def test_a_valid_frame_gives_the_reply_number(self):
        nibbles = (0xA, 0x5, 0x0, 0xF)
        frame = frame_for([GCR_CODE[n] for n in nibbles])
        self.assertEqual(gcr_decode.decode(frame), 0xA50F)

    def test_a_symbol_outside_the_code_gives_none(self):
        valid = GCR_CODE[3]
        for invalid in (0b00000, 0b00001, 0b01000, 0b11111):
            self.assertNotIn(invalid, GCR_CODE)
            self.assertIsNone(gcr_decode.decode(frame_for([valid, invalid, valid, valid])))

    def test_the_code_table_matches_the_protocol_code(self):
        self.assertEqual(gcr_decode.GCR_ENCODE_TABLE, GCR_CODE)
        for nibble, symbol in enumerate(GCR_CODE):
            self.assertEqual(gcr_decode.GCR_DECODE_TABLE[symbol], nibble)


class AnalyzeFrameTest(unittest.TestCase):
    """gcr_decode.analyze_frame(): the frame receiver (dshot_bidir_rx_frame)
    already reconstructed the frame in hardware, so this takes the 21-bit
    frame directly and has no period to report. Cross-checked against the
    raw-sample reconstruction path on a real reply in
    tests/pc/test_simulate_frame_receiver.py (they must agree, since both
    ultimately decode the same wire format)."""

    def test_period_fields_are_none(self):
        frame = frame_for([GCR_CODE[n] for n in (0xA, 0x5, 0x0, 0xF)])
        result = gcr_decode.analyze_frame(frame)
        self.assertIsNone(result["period_cycles"])
        self.assertIsNone(result["period_us"])
        self.assertIsNone(result["bitrate_bps"])

    def test_every_12_bit_value_round_trips(self):
        for data12 in range(4096):
            frame = frame_for([GCR_CODE[n] for n in nibbles16(reply_number(data12))])
            result = gcr_decode.analyze_frame(frame)
            self.assertTrue(result["crc_ok"], data12)
            self.assertEqual(result["data12"], data12)
            self.assertEqual(result["not_running"], data12 == gcr_decode.AM32_NOT_RUNNING_DATA12, data12)

    def test_not_running_is_none_without_a_crc_valid_reply(self):
        valid = GCR_CODE[3]
        frame = frame_for([valid, 0b00001, valid, valid])  # invalid symbol -> no full number
        result = gcr_decode.analyze_frame(frame)
        self.assertIsNone(result["not_running"])

    def test_a_false_marker_bit_is_reported(self):
        frame = frame_for([GCR_CODE[n] for n in (0xA, 0x5, 0x0, 0xF)])
        frame |= 1 << 20  # force the marker bit high
        result = gcr_decode.analyze_frame(frame)
        self.assertFalse(result["marker_ok"])

    def test_a_symbol_outside_the_code_gives_no_full_number(self):
        valid = GCR_CODE[3]
        frame = frame_for([valid, 0b00001, valid, valid])
        result = gcr_decode.analyze_frame(frame)
        self.assertIsNone(result["full"])
        self.assertFalse(result["crc_ok"])


class NotRunningFrameTest(unittest.TestCase):
    def test_it_is_what_am32_sends_for_a_stopped_motor(self):
        # AM32's make_dshot_package() substitutes com_time 65535 when not running;
        # scripts/verify_am32_reply.py ports it
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
        from verify_am32_reply import am32_make_dshot_package, expected_frame
        payload, levels = am32_make_dshot_package(65535)
        self.assertEqual(payload, gcr_decode.AM32_NOT_RUNNING_DATA12)
        self.assertEqual(expected_frame(levels), gcr_decode.AM32_NOT_RUNNING_FRAME)

    def test_it_decodes_as_not_running(self):
        result = gcr_decode.analyze_frame(gcr_decode.AM32_NOT_RUNNING_FRAME)
        self.assertTrue(result["crc_ok"])
        self.assertTrue(result["not_running"])


if __name__ == "__main__":
    unittest.main()
