"""
wingman_board.py - reaching the wingman Pico (the second board) from the PC, and never the main Pico.

The wingman is found by its unique ID, never by COM number (the ports move), and a board with any other ID is refused: the
main Pico (the one with the motors) must never be touched from the wingman's tools. Used by wingman_run.py and
scripts/run_session.py.
"""

import subprocess
import sys
from pathlib import Path

from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[4]
WINGMAN_ID = "5d703f56522b50fd"       # machine.unique_id() of the wingman Pico
MAIN_ID = "31b4d922f5d5f436"          # the Pico that drives the motors - never used here


def find_port():
    """The wingman's COM port, or None when it is not connected."""
    for port in list_ports.comports():
        serial_number = (port.serial_number or "").lower()
        if serial_number == MAIN_ID:
            continue
        if serial_number == WINGMAN_ID:
            return port.device
    return None


def mpremote(port, *args, timeout=30):
    return subprocess.run([sys.executable, "-m", "mpremote", "connect", port, *args],
                          capture_output=True, text=True, timeout=timeout)


def check_board(port):
    """Exit unless the board on `port` reports the wingman's ID (the main Pico is named in the refusal)."""
    result = mpremote(port, "exec", "import machine;print(machine.unique_id().hex())")
    found = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ""
    if found == MAIN_ID:
        sys.exit("%s is the MAIN Pico (%s). Refusing: these tools only drive the wingman." % (port, found))
    if found != WINGMAN_ID:
        sys.exit("%s reports board ID %r, expected the wingman %s. %s" % (port, found, WINGMAN_ID, result.stderr.strip()))


def upload(port, files):
    """files: [(path relative to the repository root, name on the board)]."""
    for local, remote in files:
        result = mpremote(port, "cp", str(ROOT / local), ":" + remote)
        if result.returncode != 0:
            sys.exit("upload of %s failed: %s" % (local, result.stderr.strip()))
        print("  uploaded %s" % remote)


def reset(port):
    mpremote(port, "reset")
