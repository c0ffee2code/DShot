"""
esc_buddy_run.py - the PC side of the ESC buddy: read the ESCs' AM32 configuration through the main Pico, hand them
to the AM32 web configurator (passthrough), or show the buddy's record of its last session.

Exactly one action is required. --read, --passthrough, --debug, --plan and --plan-restore cannot write; --write and --restore
change settings on ONE ESC (a plan first, then the same command with --write / --restore):

  python tools/esc_buddy/esc_buddy_run.py --read --pins 6 7 8 9 > esc.json   # read every ESC's settings, JSON on stdout
  python tools/esc_buddy/esc_buddy_run.py --passthrough --pins 6 7 8 9       # hand the ESCs to the browser configurator
  python tools/esc_buddy/esc_buddy_run.py --debug                            # the buddy's record of its last session
  python tools/esc_buddy/esc_buddy_run.py --plan --pins 6 7 8 9 --esc 4 --set "motor KV=2220"                     # what would change
  python tools/esc_buddy/esc_buddy_run.py --write --pins 6 7 8 9 --esc 4 --set ... --backup tmp/esc4_before.json   # change it
  python tools/esc_buddy/esc_buddy_run.py --plan-restore BACKUP | --restore BACKUP --backup ... (with --pins, --esc)  # put a backup back

--read writes nothing to disk: stdout carries exactly one JSON report (progress goes to stderr), and the caller decides
what to keep - pipe it, redirect it, or store it as a session's metadata. Exit status 1 when no ESC could be read.

--pins: the GPIO numbers of the ESC signal lines, in order = ESC 1..n (in the configurator and in the report). The
caller knows the wiring; nothing about it is written here. The main Pico is found by its unique ID, never by COM number.

--read, --passthrough, --debug, --plan and --plan-restore are read-only: the buddy refuses every write, erase and settings-write
command (esc_buddy_protocol.ALLOWED_4WAY). --write and --restore write the settings block of one ESC under the guards in
esc_buddy_protocol.Passthrough.write_refusal, need --backup, and need the owner's go (README, "Permissions and safety rules"). Never during a motor run.
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
SETTINGS_ADDRESS = protocol.SETTINGS_ADDRESS
SETTINGS_SIZE = protocol.SETTINGS_SIZE


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


def start(pins, quiet=False, record_writes=False, write_channel=None, restore_block=None):
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
    start_detached(find_main_port(), "import micropython\nmicropython.kbd_intr(-1)\nimport esc_buddy_board\nesc_buddy_board.main(%r, %d, %r, %r, %r)\n" % (
        pins, start_line.START_PIN, record_writes, write_channel, restore_block))
    if quiet:
        return pins
    print("ESC buddy running on the main Pico (%s), pins %s = ESC 1..%d." % (port, pins, len(pins)))
    if record_writes:
        print("Recording writes: the configurator's save sequence is answered OK and logged in full; nothing reaches the ESCs.")
        print("Its read-back will show the old settings, so the configurator may report a failed save - expected.")
    else:
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
        wait_until_ready(link)
        for esc in range(len(pins)):
            entry = {"esc": esc + 1, "gpio": pins[esc]}
            report["escs"].append(entry)
            connected = connect_esc(link, esc)
            if connected is None:
                entry["error"] = "no answer to init flash access after 10 tries"
                note("ESC %d (GPIO %d): %s" % (esc + 1, pins[esc], entry["error"]))
                continue
            signature, pin_code = connected
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


def wait_until_ready(link, tries=40):
    """Ask the buddy for its interface name until it answers (it needs a moment after the start command); a harmless request
    that touches no ESC. Exits if it never does."""
    for _ in range(tries):
        if four_way(link, protocol.CMD_INTERFACE_GET_NAME, 0, b"\x00", timeout_s=0.5):
            return
    sys.exit("the ESC buddy on the main Pico does not answer")


def connect_esc(link, esc, tries=10):
    """Init flash access on ESC `esc` (index from 0), retried while it reboots into its bootloader: (signature, input pin
    code), or None."""
    for _ in range(tries):
        answer = four_way(link, protocol.CMD_DEVICE_INIT_FLASH, 0, bytes([esc]))
        if answer and answer[0] == protocol.ACK_OK:
            return (answer[1][1] << 8) | answer[1][0], answer[1][2]
        time.sleep(1)
    return None


def parse_sets(items):
    """--set NAME=VALUE ... -> {name: value text}; exits on a malformed or repeated one."""
    changes = {}
    for item in items:
        name, separator, value = item.partition("=")
        name = name.strip()
        if not separator or not name or not value.strip():
            sys.exit("--set needs NAME=VALUE (for example 'motor KV=2220'), got %r" % item)
        if name in changes:
            sys.exit("%r is set twice" % name)
        changes[name] = value.strip()
    return changes


def write_settings(pins, esc, sets, apply, backup):
    """Change named settings on ONE ESC (esc = 1..n). Without apply it is a plan (--plan): the ESC is read and the change shown,
    nothing is written. With apply (--write): the page is saved to `backup` first (never overwritten), the Pico writes the block - it checks the
    protected bytes itself - and the page is read back and compared. Prints one JSON report on stdout and returns it."""
    changes = parse_sets(sets)
    try:
        esc_buddy_settings.apply_changes(bytes(SETTINGS_SIZE), changes)      # names and values checked before anything starts
    except ValueError as error:
        sys.exit(str(error))
    return run_write(pins, esc, changes, None, apply, backup)


def restore_settings(pins, esc, saved, apply, backup):
    """Put the settings block saved by an earlier --write back on ONE ESC (`saved` = that backup file). Plan unless apply; the
    backup must be of this ESC (same number and GPIO) and hold a block that decodes. `backup` is where the page as it is NOW
    is saved first, like any write."""
    try:
        with open(saved) as handle:
            data = json.load(handle)
        block = bytes.fromhex(data["settings_block_hex"])
        esc_buddy_settings.decode(block)
    except (OSError, ValueError, KeyError) as error:
        sys.exit("cannot restore from %s: %s" % (saved, error))
    if len(block) != SETTINGS_SIZE:
        sys.exit("%s holds %d bytes, not a settings block of %d" % (saved, len(block), SETTINGS_SIZE))
    if not 1 <= esc <= len(pins):
        sys.exit("--esc %d: there are %d ESCs in --pins" % (esc, len(pins)))
    if data.get("esc") != esc or data.get("gpio") != pins[esc - 1]:
        sys.exit("%s is a backup of ESC %s (GPIO %s), not of ESC %d (GPIO %d)" % (
            saved, data.get("esc"), data.get("gpio"), esc, pins[esc - 1]))
    return run_write(pins, esc, {}, block, apply, backup)


def run_write(pins, esc, changes, restore, apply, backup):
    """The common part of write_settings and restore_settings. restore: the exact block to put back, or None."""
    if not 1 <= esc <= len(pins):
        sys.exit("--esc %d: there are %d ESCs in --pins" % (esc, len(pins)))
    if apply:
        if not backup:
            sys.exit("--write and --restore need --backup <path>: the old settings page is saved there before anything is written")
        backup = Path(backup)
        if backup.exists():
            sys.exit("%s exists; the backup is never overwritten" % backup)
        backup.parent.mkdir(parents=True, exist_ok=True)
    index = esc - 1
    pins = start(pins, quiet=True, write_channel=index if apply else None, restore_block=restore if apply else None)
    report = {"at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "esc": esc, "gpio": pins[index], "applied": False, "ok": False}
    link = serial.Serial(find_main_port(), 115200, timeout=0.2)
    try:
        wait_until_ready(link)
        fill_report(link, report, index, changes, apply, backup, restore)
    finally:
        four_way(link, protocol.CMD_INTERFACE_EXIT, timeout_s=1.0)
        link.close()
    print(json.dumps(report, indent=1))
    return report


def read_page(link, address):
    """The whole 1 KB settings page (four reads of 256), or None. The bootloader erases all of it when a write starts at its
    first byte, so it is what is saved, checked for blankness and compared after a write."""
    page = b""
    for part in range(4):
        answer = four_way(link, protocol.CMD_DEVICE_READ, address + 256 * part, bytes([0]))
        if not answer or answer[0] != protocol.ACK_OK or len(answer[1]) != 256:
            return None
        page += answer[1]
    return page


def field_texts(block):
    """{byte offset: decoded text} of a settings block, or {} when it does not decode (blank, other layout)."""
    try:
        return {offset: text for offset, _, _, text in esc_buddy_settings.decode(block)}
    except ValueError:
        return {}


def fill_report(link, report, index, changes, apply, backup, restore=None):
    """The body of run_write: sets report["error"] and returns at the first thing that stops it."""
    found = connect_esc(link, index)
    if found is None:
        report["error"] = "no answer to init flash access after 10 tries"
        return
    address = SETTINGS_ADDRESS.get(found[0])
    if address is None:
        report["error"] = "settings address unknown for bootloader signature 0x%04X" % found[0]
        return
    page = read_page(link, address)
    if page is None:
        report["error"] = "reading the settings page failed"
        return
    old = page[:SETTINGS_SIZE]
    blank = b"\xff" * (len(page) - SETTINGS_SIZE)
    report["page_beyond_block_blank"] = page[SETTINGS_SIZE:] == blank
    if restore is None:
        try:
            new, done = esc_buddy_settings.apply_changes(old, changes)
            esc_buddy_settings.decode(old)
        except ValueError as error:
            report["error"] = str(error)
            return
    else:
        new = restore
        names = {offset: name for offset, name, _, _ in esc_buddy_settings.decode(new)}
        done = [(names[offset], offset, old[offset], new[offset]) for offset in names if old[offset] != new[offset]]
    before, after = field_texts(old), field_texts(new)
    report["changes"] = [{"field": name, "byte": offset, "old_raw": old_raw, "new_raw": new_raw,
                          "old": before.get(offset, "unreadable"), "new": after.get(offset, "unreadable")}
                         for name, offset, old_raw, new_raw in done]
    report["old_block_hex"], report["new_block_hex"] = old.hex(), new.hex()
    if not report["page_beyond_block_blank"]:
        report["error"] = "the page holds data beyond the settings block; a write would erase it - not doing that"
        return
    if new == old:
        report["error"] = "nothing to change: the ESC already holds these values"
        return
    if not apply:
        report["ok"] = True
        return
    with open(backup, "x") as saved:                   # before anything is sent to the ESC
        json.dump({"saved_at": report["at"], "esc": report["esc"], "gpio": report["gpio"], "address": "0x%04X" % address,
                   "page_hex": page.hex(), "settings_block_hex": old.hex()}, saved, indent=1)
    report["backup"] = str(backup)
    answer = four_way(link, protocol.CMD_DEVICE_WRITE, address, new, timeout_s=10.0)
    report["applied"] = True
    report["write_ack"] = None if answer is None else answer[0]
    back = read_page(link, address)
    report["readback_block_hex"] = None if back is None else back[:SETTINGS_SIZE].hex()
    report["readback_matches"] = back == new + blank       # the block as written and nothing left in the rest of the page
    report["ok"] = report["write_ack"] == protocol.ACK_OK and report["readback_matches"]


def debug():
    """The buddy's record of its last session (esc_buddy_log.txt on the Pico), as plain text on stdout; ends a buddy that
    is still running first. Nothing is written on the PC."""
    port = find_main_port()
    stop_running(port)
    result = wingman_board.mpremote(port, "cat", ":" + LOG_NAME)
    if result.returncode != 0:
        sys.exit("could not read %s from the Pico: %s" % (LOG_NAME, result.stderr.strip()))
    sys.stdout.write(result.stdout)


def parse_args(argv=None):
    """The command line, checked. Exactly one action; --plan / --write and --plan-restore / --restore are pairs: the first of
    each pair only reads and shows what would change, the second writes (and then needs --backup)."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--read", action="store_true", help="read every ESC's settings; one JSON report on stdout")
    action.add_argument("--passthrough", action="store_true",
                        help="hand the ESCs to the AM32 configurator (browser) until it disconnects")
    action.add_argument("--plan", action="store_true",
                        help="show what --write would change on ONE ESC (--esc, --set); reads, writes nothing")
    action.add_argument("--write", action="store_true",
                        help="change named settings on ONE ESC (--esc, --set); needs --backup and the owner's go")
    action.add_argument("--plan-restore", metavar="BACKUP",
                        help="show what --restore would change on ONE ESC (--esc); reads, writes nothing")
    action.add_argument("--restore", metavar="BACKUP",
                        help="put the settings block saved in BACKUP (by an earlier --write) back on ONE ESC (--esc); "
                             "needs --backup for the page as it is now, and the owner's go")
    action.add_argument("--debug", action="store_true",
                        help="print the buddy's record of its last session (plain text, stdout)")
    parser.add_argument("--record-writes", action="store_true",
                        help="with --passthrough: answer the configurator's save commands OK without sending them to "
                             "the ESC, and log them in full (--debug shows them)")
    parser.add_argument("--esc", type=int,
                        help="with --plan, --write, --plan-restore or --restore: which ESC, 1..n in the order of --pins")
    parser.add_argument("--set", nargs="+", metavar="NAME=VALUE",
                        help="with --plan or --write: settings to change, e.g. \"motor KV=2220\" \"motor poles=12\"")
    parser.add_argument("--backup", metavar="PATH",
                        help="with --write or --restore: where the settings page as it is now is saved first (never "
                             "overwritten; created with its folder if missing)")
    parser.add_argument("--pins", type=int, nargs="+", metavar="GPIO",
                        help="ESC signal lines in order = ESC 1..n (needed by every action except --debug)")
    args = parser.parse_args(argv)
    args.saved = args.restore if args.restore is not None else args.plan_restore       # the backup file to put back
    args.apply = bool(args.write or args.restore is not None)                           # the actions that write
    changing = bool(args.plan or args.write or args.saved is not None)
    chosen = next(name for name in ("read", "passthrough", "plan", "write", "plan_restore", "restore", "debug")
                  if getattr(args, name) not in (False, None))
    shown = "--" + chosen.replace("_", "-")
    if chosen != "debug" and not args.pins:
        parser.error("%s needs --pins" % shown)
    if changing and args.esc is None:
        parser.error("%s needs --esc" % shown)
    if (args.plan or args.write) and not args.set:
        parser.error("%s needs --set" % shown)
    if args.set and not (args.plan or args.write):
        parser.error("--set only goes with --plan or --write")
    if args.esc is not None and not changing:
        parser.error("--esc only goes with --plan, --write, --plan-restore or --restore")
    if args.apply and not args.backup:
        parser.error("%s needs --backup <path>: the old settings page is saved there before anything is written" % shown)
    if args.backup and not args.apply:
        parser.error("--backup only goes with --write or --restore (a plan writes nothing, so there is nothing to back up)")
    if args.record_writes and not args.passthrough:
        parser.error("--record-writes only goes with --passthrough")
    return args


def main():
    args = parse_args()
    if args.plan or args.write:
        report = write_settings(args.pins, args.esc, args.set, args.apply, args.backup)
        sys.exit(0 if report["ok"] else 1)
    if args.saved is not None:
        report = restore_settings(args.pins, args.esc, args.saved, args.apply, args.backup)
        sys.exit(0 if report["ok"] else 1)
    if args.read:
        read_settings(args.pins)
    elif args.passthrough:
        start(args.pins, record_writes=args.record_writes)
    else:
        debug()


if __name__ == "__main__":
    main()
