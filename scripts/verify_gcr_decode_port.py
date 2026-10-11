"""
verify_gcr_decode_port.py - checks driver/gcr_decode.py's analyze_frame()
(the on-device port, the only decode path BidirectionalDShot has) against
scripts/dshot_bidir_decode.py's own analyze_frame() (the PC-side reference),
over all 4096 possible 12-bit eRPM payloads, synthetically GCR-encoded with
the inverted CRC into the 21-bit frame dshot_bidir_rx_frame would push.
Catches a mechanical porting bug for free, before MicroPython ever enters
the picture.

Not a one-off: this is a permanent regression check, kept alongside both
files - re-run it whenever either changes.

Run from project root:
  python scripts/verify_gcr_decode_port.py

History: this used to also diff both files' analyze_capture() (the old
sample receiver's raw-sample reconstruction path) against real stored
capture sessions under captures/. That half was removed 2026-10-11 along
with analyze_capture() itself - driver/gcr_decode.py no longer has an
on-device "port" of that path to check against a reference (the reference's
own analyze_capture() is now only compared against scripts/
simulate_frame_receiver.py's independent PIO-instruction-level model, a
different and still-live check - see that script).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "driver"))

from dshot_bidir_decode import analyze_frame as reference_analyze_frame, GCR_ENCODE_TABLE, crc_inverted
from gcr_decode import analyze_frame as port_analyze_frame


def verify_frame_parity():
    """
    Synthetic parity check between driver/gcr_decode.py's analyze_frame() and
    this script's own, for the frame receiver's path: synthesizes every
    12-bit value, GCR-encodes it with the inverted CRC into the 21-bit frame
    dshot_bidir_rx_frame would push, and checks both implementations agree
    on the whole result dict.

    Returns a list of (data12, field, reference_value, port_value) mismatches.
    """
    mismatches = []
    for data12 in range(4096):
        number = (data12 << 4) | crc_inverted(data12)
        code = 0
        for shift in (12, 8, 4, 0):
            code = (code << 5) | GCR_ENCODE_TABLE[(number >> shift) & 0xF]
        level = 0
        frame = 0
        for shift in range(19, -1, -1):
            level ^= (code >> shift) & 1
            frame = (frame << 1) | level

        ref = reference_analyze_frame(frame)
        port = port_analyze_frame(frame)
        for field in ("full", "marker_ok", "crc_ok", "crc_kind", "data12", "erpm"):
            if ref[field] != port[field]:
                mismatches.append((data12, field, ref[field], port[field]))
                break

    return mismatches


def main():
    mismatches = verify_frame_parity()
    print(f"Frame receiver parity: 4096 synthetic values checked, {len(mismatches)} mismatches")
    for data12, field, ref_value, port_value in mismatches[:5]:
        print(f"    mismatch: data12={data12} field={field} reference={ref_value!r} port={port_value!r}")
    if mismatches:
        sys.exit(1)


if __name__ == "__main__":
    main()
