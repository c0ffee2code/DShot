"""
esc_buddy_run.py - the PC side of the ESC buddy: read the ESCs' AM32 configuration through the main Pico, hand them
to the AM32 web configurator (passthrough), or show the buddy's record of its last session.

Exactly one action is required; all of them are read-only:

  python tools/esc_buddy/esc_buddy_run.py --read --pins 6 7 8 9 > esc.json   # read every ESC's settings, JSON on stdout
  python tools/esc_buddy/esc_buddy_run.py --passthrough --pins 6 7 8 9       # hand the ESCs to the browser configurator
  python tools/esc_buddy/esc_buddy_run.py --debug                            # the buddy's record of its last session

--read writes nothing to disk: stdout carries exactly one JSON report (progress goes to stderr), and the caller decides
what to keep - pipe it, redirect it, or store it as a session's metadata. Exit status 1 when no ESC could be read.

--pins: the GPIO numbers of the ESC signal lines, in order = ESC 1..n (in the configurator and in the report). The
caller knows the wiring; nothing about it is written here. The main Pico is found by its unique ID, never by COM number.

Read-only: the buddy refuses every write, erase and settings-write command (esc_buddy_protocol.ALLOWED_4WAY).
It puts signals on the ESC lines but no throttle and no writes: no permission needed (CLAUDE.md), never during a
motor run.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "bench" / "wingman" / "tools"))
import wingman_board  # noqa: E402  (MAIN_ID, mpremote())
sys.path.insert(0, str(ROOT / "tests" / "bench" / "wingman" / "main_pico"))
import start_line  # noqa: E402  (START_PIN: the line the wingman's spy records by)
import esc_buddy_protocol as protocol  # noqa: E402  (this folder: the 4-way frame format and CRC)
import esc_buddy_settings  # noqa: E402

BOARD_FILES = [("tools/esc_buddy/esc_buddy_protocol.py", "esc_buddy_protocol.py"),
               ("tools/esc_buddy/esc_buddy_board.py", "esc_buddy_board.py")]
LOG_NAME = "esc_buddy_log.txt"
# Where the settings block sits, per bootloader signature, for bootloaders without the v3 devinfo block (am32-configurator
# src/mcu.ts variants). Only what this bench has met is listed; an unknown signature is refused, not guessed.
SETTINGS_ADDRESS = {0x1F06: 0x7C00, 0x3506: 0xF800}
SETTINGS_SIZE = 0xB8          # Mcu.LAYOUT_SIZE


def check_pins(pins):
    """The ESC pins as given, or exit: 1-8 distinct GPIO numbers (RP2350: 0-47), not the start line."""
    if not 1 <= len(pins) <= 8:
        sys.exit("give 1 to 8 ESC pins, got %d" % len(pins))
    if len(set(pins)) != len(pins):
        sys.exit("a pin appears twice in %s" % pins)
    for pin in pins:
        if not 0 <= pin <= 47:
            sys.exit("GPIO %d does not exist" % pin)
        if pin == start_line.START_PIN:
            sys.exit("GPIO %d is the bench start line, not an ESC line" % pin)
    return pins


def find_main_port():
    for port in list_ports.comports():
        if (port.serial_number or "").lower() == wingman_board.MAIN_ID:
            return port.device
    sys.exit("main Pico (ID %s) not found on any COM port" % wingman_board.MAIN_ID)


def check_main(port):
    result = wingman_board.mpremote(port, "exec", "import machine;print(machine.unique_id().hex())")
    found = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ""
    if found != wingman_board.MAIN_ID:
        sys.exit("%s reports board ID %r, expected the main Pico %s. %s" % (port, found, wingman_board.MAIN_ID,
                                                                          result.stderr.strip()))


def stop_running(port):
    """End an passthrough that is still running (it ignores Ctrl-C) with the configurator's own 4-way exit command; a
    board at the REPL just sees a few stray characters, which the next Ctrl-C clears."""
    body = bytes([0x2F, 0x34, 0, 0, 1, 0])
    crc = 0
    for byte in body:                      # CRC16-XMODEM, as esc_buddy_protocol.crc_xmodem
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    link = serial.Serial(port, 115200, timeout=0.5)
    try:
        link.write(body + bytes([crc >> 8, crc & 0xFF]))
        time.sleep(0.5)
        link.read_all()
    finally:
        link.close()


def start_detached(port, code):
    """Hand `code` to the raw REPL, wait for its OK, and let go of the port so the browser can open it."""
    link = serial.Serial(port, 115200, timeout=0.5)
    try:
        link.write(b"\x03\x03")
        time.sleep(0.3)
        link.reset_input_buffer()
        link.write(b"\x01")
        time.sleep(0.2)
        link.read_all()
        link.write(code.encode() + b"\x04")
        reply = link.read(2)
        if reply != b"OK":
            sys.exit("the Pico did not accept the start command (got %r)" % reply)
    finally:
        link.close()


def start(pins, quiet=False):
    pins = check_pins(pins)
    port = find_main_port()
    stop_running(port)
    check_main(port)
    for local, remote in BOARD_FILES:
        result = wingman_board.mpremote(port, "cp", str(ROOT / local), ":" + remote)
        if result.returncode != 0:
            sys.exit("upload of %s failed: %s" % (local, result.stderr.strip()))
    wingman_board.mpremote(port, "reset")
    time.sleep(3)
    start_detached(find_main_port(), "import esc_buddy_board\nesc_buddy_board.main(%r, %d)\n" % (pins, start_line.START_PIN))
    if quiet:
        return pins
    print("ESC buddy running on the main Pico (%s), pins %s = ESC 1..%d." % (port, pins, len(pins)))
    print("Read-only: writes, erases and settings writes are refused.")
    print("Start line GPIO %d is high for the session (the wingman's spy records while it is)." % start_line.START_PIN)
    print("Now: AM32 configurator -> select this port -> Connect -> read the ESCs -> Disconnect.")
    print("Connect within 30 min. It returns to the REPL on Disconnect, or after 5 min of silence once connected.")
    print("Afterwards, the buddy's own record: python tools/esc_buddy/esc_buddy_run.py --debug")


def four_way(link, cmd, address=0, params=b"\x00", timeout_s=3.0):
    """One 4-way exchange as the configurator makes it; (ack, params) of the reply, or None on silence/corruption."""
    body = bytes([protocol.LOCAL_ESCAPE, cmd, address >> 8, address & 0xFF, len(params) & 0xFF]) + bytes(params)
    crc = protocol.crc_xmodem(body)
    link.reset_input_buffer()
    link.write(body + bytes([crc >> 8, crc & 0xFF]))
    reply = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        reply += link.read(512)
        if len(reply) >= 5 and len(reply) >= 5 + (reply[4] or 256) + 3:
            break
    if len(reply) < 8 or reply[0] != protocol.REMOTE_ESCAPE:
        return None
    count = reply[4] or 256
    reply = reply[:5 + count + 3]
    if (reply[1], (reply[2] << 8) | reply[3]) != (cmd, address) or protocol.crc_xmodem(reply[:-2]) != (
            (reply[-2] << 8) | reply[-1]):
        return None
    return reply[5 + count], reply[5:5 + count]


def note(text):
    """Progress for a person; stderr, so stdout carries only the JSON report."""
    print(text, file=sys.stderr)


def read_settings(pins):
    """Play the configurator's read: init flash access, firmware file name, settings block - per ESC - then exit.
    Every command used is on the buddy's read-only allow-list."""
    pins = start(pins, quiet=True)
    note("ESC buddy started, pins %s. Reading %d ESC(s) - each needs ~2 s without DShot to reach its bootloader."
         % (pins, len(pins)))
    report = {"read_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "pins": pins, "escs": []}
    blocks = []
    link = serial.Serial(find_main_port(), 115200, timeout=0.2)
    try:
        for esc in range(len(pins)):
            entry = {"esc": esc + 1, "gpio": pins[esc]}
            report["escs"].append(entry)
            info = None
            for attempt in range(10):
                answer = four_way(link, protocol.CMD_DEVICE_INIT_FLASH, 0, bytes([esc]))
                if answer and answer[0] == protocol.ACK_OK:
                    info = answer[1]
                    break
                time.sleep(1)
            if info is None:
                entry["error"] = "no answer to init flash access after 10 tries"
                note("ESC %d (GPIO %d): %s" % (esc + 1, pins[esc], entry["error"]))
                continue
            signature, pin_code = (info[1] << 8) | info[0], info[2]
            entry["bootloader"] = {"signature": "0x%04X" % signature, "input_pin": "P%s%d" % (
                "ABCDEF"[pin_code >> 4] if pin_code >> 4 < 6 else "?", pin_code & 15)}
            note("ESC %d (GPIO %d): bootloader signature %s, input pin %s" % (
                esc + 1, pins[esc], entry["bootloader"]["signature"], entry["bootloader"]["input_pin"]))
            address = SETTINGS_ADDRESS.get(signature)
            if address is None:
                entry["error"] = "settings address unknown for this bootloader signature"
                continue
            name = four_way(link, protocol.CMD_DEVICE_READ, address - 32, bytes([32]))
            if name and name[0] == protocol.ACK_OK:
                entry["firmware_file"] = name[1].split(b"\x00")[0].decode("ascii", "replace")
            block = four_way(link, protocol.CMD_DEVICE_READ, address, bytes([SETTINGS_SIZE]))
            if not block or block[0] != protocol.ACK_OK:
                entry["error"] = "settings read failed"
                continue
            blocks.append(block[1])
            try:
                entry.update(esc_buddy_settings.as_dict(block[1]))
            except ValueError as error:
                entry["raw_hex"] = block[1].hex()
                entry["error"] = "settings read but not decoded: %s" % error
        four_way(link, protocol.CMD_INTERFACE_EXIT, timeout_s=1.0)
    finally:
        link.close()
    report["identical"] = len(blocks) == len(pins) and len(set(blocks)) == 1
    print(json.dumps(report, indent=1))
    if not blocks:
        sys.exit(1)


def debug():
    """The buddy's record of its last session (esc_buddy_log.txt on the Pico), as plain text on stdout; ends a buddy that
    is still running first. Nothing is written on the PC."""
    port = find_main_port()
    stop_running(port)
    result = wingman_board.mpremote(port, "cat", ":" + LOG_NAME)
    if result.returncode != 0:
        sys.exit("could not read %s from the Pico: %s" % (LOG_NAME, result.stderr.strip()))
    sys.stdout.write(result.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--read", action="store_true", help="read every ESC's settings; one JSON report on stdout")
    action.add_argument("--passthrough", action="store_true",
                        help="hand the ESCs to the AM32 configurator (browser) until it disconnects")
    action.add_argument("--debug", action="store_true",
                        help="print the buddy's record of its last session (plain text, stdout)")
    parser.add_argument("--pins", type=int, nargs="+", metavar="GPIO",
                        help="ESC signal lines in order = ESC 1..n (needed by --read and --passthrough)")
    args = parser.parse_args()
    if (args.read or args.passthrough) and not args.pins:
        parser.error("--%s needs --pins" % ("read" if args.read else "passthrough"))
    if args.read:
        read_settings(args.pins)
    elif args.passthrough:
        start(args.pins)
    else:
        debug()


if __name__ == "__main__":
    main()
