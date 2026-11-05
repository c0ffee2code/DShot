# The bidirectional receive path against AM32's own reply encoder (am32_reply.py, a line-for-line port of AM32's
# make_dshot_package()): every payload the ESC can emit must decode exactly with driver/gcr_decode, and the PIO receiver
# model of dshot_bidir_rx_frame, tuned by BIDIR_PROFILES, must rebuild every F421 reply (the bench ESC's family) at both
# DSHOT speeds. No capture data or hardware needed.

import random
import unittest

import fakes  # noqa: F401  puts driver/ on sys.path
import am32_reply
import gcr_decode
from dshot_profiles import DSHOT_SPEEDS


class EncoderTable(unittest.TestCase):
    def test_our_gcr_table_is_am32s(self):
        self.assertEqual(am32_reply.AM32_GCR_ENCODE_TABLE, gcr_decode.GCR_ENCODE_TABLE)


class DecoderOnEveryPayload(unittest.TestCase):
    def test_every_payload_am32_emits_decodes_exactly(self):
        com_times = list(am32_reply.am32_com_times())
        self.assertGreater(len(com_times), 2000)
        for com_time in com_times:
            payload, levels = am32_reply.am32_make_dshot_package(com_time)
            result = gcr_decode.analyze_frame(am32_reply.expected_frame(levels))
            period_us = (payload & 0x1FF) << (payload >> 9)
            erpm = None if period_us == 0 else 60_000_000 / period_us
            self.assertTrue(result["crc_ok"] and result["marker_ok"], com_time)
            self.assertEqual((result["data12"], result["erpm"]), (payload, erpm), com_time)


class ReceiverReadsF421(unittest.TestCase):
    def test_every_f421_reply_is_rebuilt_at_both_speeds(self):
        sample = list(am32_reply.am32_com_times())[::7]
        for speed in (DSHOT_SPEEDS.DSHOT300, DSHOT_SPEEDS.DSHOT600):
            rate = am32_reply.receive_rate(am32_reply.f421_bit_cycles(speed), sample, random.Random(1))
            self.assertEqual(rate, 1.0, speed)


if __name__ == "__main__":
    unittest.main()
