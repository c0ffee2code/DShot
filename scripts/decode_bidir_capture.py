"""
Throwaway offline analysis - NOT part of the driver or its test suite.

Decodes a handful of pasted raw captures using the shared decode algorithm in
dshot_bidir_decode.py. See that module's docstring for the full method and
its hardware-verified history.
"""

from dshot_bidir_decode import analyze_capture, crc_plain, crc_inverted, MOTOR_POLES

RX_CLOCK_HZ = 4_000_000  # see dshot_pio.py's DShotPIO.__init__

CAPTURES = [
    (100, [0x0ffc1f03, 0xfe007ff0, 0x07fe007e, 0x1fffffff]),
    (100, [0x0ffc1f83, 0xfe007ff0, 0x07c1ff81, 0xffffffff]),
    (100, [0x0ffc1f83, 0xfe007ff0, 0x07fe007e, 0x1fffffff]),
    (100, [0x0ffc1f07, 0xfe007fe0, 0x0fc1ff83, 0xffffffff]),
    (100, [0x0ffc1f03, 0xfe007fe0, 0x07c1ff83, 0xffffffff]),
    (200, [0x0ffc1f07, 0xfe007fe0, 0x07c1ff83, 0xffffffff]),
    (200, [0x0ffc007c, 0x000f83ff, 0xf8000ffe, 0x1fffffff]),
    (200, [0x0ffc00f8, 0x000f83ff, 0xf83ff003, 0xffffffff]),
    (200, [0x0ffc007c, 0x000f83ff, 0x003e0f81, 0xffffffff]),
    (200, [0x0ffc00fc, 0x000f83ff, 0xf8000ffc, 0x1fffffff]),
    (200, [0x0ffc00fc, 0x000f83e0, 0xffc00f80, 0x3fffffff]),
    (300, [0x0ffc007c, 0x000f83ff, 0xf83ff001, 0xffffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xffc00f80, 0x1fffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
    (300, [0x0fffe0f8, 0x3ff0001f, 0xf83ff07c, 0x3fffffff]),
    (300, [0x0fffe0fc, 0x3ff0001f, 0xf8000f83, 0xffffffff]),
]


def main():
    if not CAPTURES:
        print("No captures pasted in yet - see module docstring. Nothing to analyze.")
        return

    total = 0
    symbol_valid = 0
    crc_valid = 0
    for throttle, words in CAPTURES:
        total += 1
        result = analyze_capture(words, RX_CLOCK_HZ)
        if result is None:
            print(f"throttle={throttle:4d}: could not analyze (no edges)")
            continue
        print(f"throttle={throttle:4d} period={result['period_cycles']:.2f} cycles "
              f"({result['period_us']:.2f}us, {result['bitrate_bps']:,} bps)", end="  ")
        full = result["full"]
        if full is None:
            print("invalid GCR symbols")
            continue
        symbol_valid += 1
        data12 = result["data12"]
        if result["crc_ok"]:
            crc_valid += 1
            mantissa = data12 & 0x1FF
            exponent = (data12 >> 9) & 0x7
            erpm = result["erpm"]
            rpm = None if erpm is None else erpm / (MOTOR_POLES / 2)
            erpm_s = "n/a" if erpm is None else f"{erpm:.0f}"
            rpm_s = "n/a" if rpm is None else f"{rpm:.0f}"
            print(f"CRC-OK({result['crc_kind']:8s}) mantissa={mantissa:4d} exponent={exponent} "
                  f"eRPM={erpm_s:>7s} RPM={rpm_s:>7s}")
        else:
            print(f"symbols valid, CRC FAIL (got 0x{full & 0xF:x}, "
                  f"plain=0x{crc_plain(data12):x}, inverted=0x{crc_inverted(data12):x})")

    print()
    print(f"{symbol_valid}/{total} symbol-valid, {crc_valid}/{total} CRC-valid")


if __name__ == "__main__":
    main()
