"""
pull_captures.py — move new bidir capture sessions from the PicoBell's SD
card to captures/ on the PC. A session is deleted from the SD card once every
one of its files has arrived on the PC with the size the card reported; a
session that arrived incompletely stays on the card.

Run from project root:
  python scripts/pull_captures.py

Pico must be connected on COM10. mpremote interrupts any running script on
connect - don't run this during a live capture session.

Same base64-over-exec transfer technique as the sister test rig's
pull_flights.py (Flight-Benchy, pipelines/flight-runner/scripts/pull_flights.py):
the SD card is only mounted transiently by a script we push over, not exposed
as a normal mpremote ':' path, so files have to be read and streamed out
through print() rather than fetched with a plain 'mpremote cp'.
"""

import base64
import os
import subprocess
import sys
import tempfile
from pathlib import Path

PYTHON = sys.executable
COM_PORT = "COM10"
REMOTE_DIR = "/sd/dshot_captures"
LOCAL_DIR = Path("captures")

# Sessions moved per transfer. Each batch is downloaded, checked and deleted from the
# SD card before the next starts, so an interrupted pull loses one batch of progress,
# not all of it.
BATCH_SIZE = 3

# PicoBell Adalogger for Pico SD pins - see tests/harness/bidir_capture_sink.py
_SD_MOUNT = """\
import os, time
from machine import SPI, Pin
import sdcard
_cs  = Pin(17, Pin.OUT, value=1)
_spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
           sck=Pin(18), mosi=Pin(19), miso=Pin(16))
time.sleep_ms(250)
_sd  = sdcard.SDCard(_spi, _cs, baudrate=25_000_000)
os.mount(os.VfsFat(_sd), '/sd')
"""

_LIST_SCRIPT = _SD_MOUNT + f"""\
try:
    for name in sorted(os.listdir('{REMOTE_DIR}')):
        print(name)
finally:
    os.umount('/sd')
"""


def _transfer_script(session_ids):
    return _SD_MOUNT + f"""\
import ubinascii
try:
    for sid in {session_ids!r}:
        for fname in sorted(os.listdir('{REMOTE_DIR}/' + sid)):
            path = '{REMOTE_DIR}/' + sid + '/' + fname
            try:
                size = os.stat(path)[6]
                print('BEGIN_FILE ' + sid + '/' + fname + ' ' + str(size))
                with open(path, 'rb') as f:
                    while True:
                        chunk = f.read(192)
                        if not chunk:
                            break
                        print(ubinascii.b2a_base64(chunk).decode(), end='')
                print('END_FILE ' + sid + '/' + fname)
            except OSError as e:
                print('PICO_ERROR ' + sid + '/' + fname + ' ' + str(e))
finally:
    os.umount('/sd')
"""


def _run_on_pico(code, timeout=120):
    with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False,
                                      encoding='utf-8') as tf:
        tf.write(code)
        tf_path = tf.name
    try:
        result = subprocess.run(
            [PYTHON, '-m', 'mpremote', 'connect', COM_PORT, 'run', tf_path],
            capture_output=True, text=True, timeout=timeout,
        )
    finally:
        os.unlink(tf_path)
    if result.returncode != 0:
        sys.exit(f"mpremote failed:\n{result.stderr.strip()}")
    return result.stdout


def _parse_transfer(output):
    files = {}
    expected_sizes = {}
    current_key = None
    b64_lines = []

    for line in output.splitlines():
        if line.startswith('BEGIN_FILE '):
            _, key, size = line.split(' ', 2)
            current_key = key
            expected_sizes[key] = int(size)
            b64_lines = []
        elif line.startswith('END_FILE '):
            if current_key:
                files[current_key] = base64.b64decode(''.join(b64_lines))
            current_key = None
            b64_lines = []
        elif line.startswith('PICO_ERROR '):
            print(f"  {line}")
            # a file the card could not read must count against its session, even
            # when the error came before its BEGIN_FILE line
            expected_sizes[line.split(' ', 2)[1]] = -1
        elif current_key is not None:
            b64_lines.append(line.strip())

    return files, expected_sizes


def list_remote():
    out = _run_on_pico(_LIST_SCRIPT)
    return sorted(line.strip() for line in out.splitlines() if line.strip())


def list_local():
    if not LOCAL_DIR.exists():
        return set()
    return {p.name for p in LOCAL_DIR.iterdir() if p.is_dir()}


def _delete_script(session_ids):
    return _SD_MOUNT + f"""try:
    for sid in {session_ids!r}:
        d = '{REMOTE_DIR}/' + sid
        for fname in os.listdir(d):
            os.remove(d + '/' + fname)
        os.rmdir(d)
        print('DELETED ' + sid)
finally:
    os.umount('/sd')
"""


def fetch(new_ids, transfer_timeout=120):
    """Download the sessions and write the complete ones to captures/.

    Returns (ok, failed) session ids. A session is complete when the card listed
    at least one file for it and every listed file arrived with its listed size.
    """
    output = _run_on_pico(_transfer_script(new_ids), timeout=transfer_timeout)
    files, expected = _parse_transfer(output)

    ok, failed = [], []
    for sid in new_ids:
        keys = [k for k in expected if k.startswith(sid + "/")]
        problems = [k for k in keys if k not in files or len(files[k]) != expected[k]]
        if keys and not problems:
            dest = LOCAL_DIR / sid
            dest.mkdir(parents=True, exist_ok=True)
            for key in keys:
                (dest / key.split("/", 1)[1]).write_bytes(files[key])
            ok.append(sid)
            print(f"  OK   {sid}")
        else:
            failed.append(sid)
            print(f"  FAIL {sid}")
            for key in problems:
                print(f"         {key}: missing or wrong size")

    return ok, failed


def delete_remote(session_ids):
    out = _run_on_pico(_delete_script(session_ids))
    return [line.split(" ", 1)[1].strip() for line in out.splitlines() if line.startswith("DELETED ")]


def main():
    print(f"Connecting to Pico on {COM_PORT} -- listing SD card...")
    remote = list_remote()
    local = list_local()
    new_ids = sorted(set(remote) - local)

    print(f"  SD card : {len(remote)} session(s)  |  local: {len(local)} session(s)")

    if not new_ids:
        print("Nothing new to pull.")
        return

    print(f"\nNew ({len(new_ids)}):")
    for sid in new_ids:
        print(f"  {sid}")

    # Base64 over serial is slow (a 60-second scenario is ~2MB and takes minutes), so
    # the timeout is sized per batch and generous: waiting longer costs nothing.
    all_ok, all_failed, deleted = [], [], []
    for start in range(0, len(new_ids), BATCH_SIZE):
        batch = new_ids[start:start + BATCH_SIZE]
        print(f"\nBatch {start // BATCH_SIZE + 1}: {len(batch)} session(s)...", flush=True)
        ok_ids, failed_ids = fetch(batch, transfer_timeout=max(120, len(batch) * 240))
        all_ok += ok_ids
        all_failed += failed_ids
        if ok_ids:
            gone = delete_remote(ok_ids)
            deleted += gone
            print(f"  deleted from the SD card: {len(gone)}", flush=True)

    print(f"\nDone: {len(all_ok)} pulled, {len(all_failed)} failed, {len(deleted)} deleted from the SD card.")
    if all_failed:
        print("Failed sessions remain on the SD card:")
        for sid in all_failed:
            print(f"  {sid}")


if __name__ == '__main__':
    main()
