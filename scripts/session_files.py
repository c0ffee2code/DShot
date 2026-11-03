"""
session_files.py - read and write what a session folder holds about the Pico's side of the run.

The Pico's session recorder writes its record of a run (outcome, counters, the motors' hardware, the capture
record format) as key=value lines in meta.txt on the SD card. On the PC that record lives in the "device" section of
session.json and nowhere else: scripts/pull_captures.py folds it in right after pulling the session (fold_device_meta)
and deletes the pulled meta.txt. There is no fallback to a meta.txt in a session folder: sessions from before
2026-10-30 were migrated once with fold_device_meta. Every analysis script uses this one reader. Values stay strings,
exactly as the Pico wrote them.
"""

import json
from pathlib import Path

DEVICE_META_FILE = "meta.txt"
MANIFEST_FILE = "session.json"


def parse_meta(text):
    """key=value lines -> {key: value} (both stripped; lines without '=' are ignored)."""
    fields = {}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            fields[key.strip()] = value.strip()
    return fields


def read_manifest(session_dir):
    path = Path(session_dir) / MANIFEST_FILE
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def load_device_meta(session_dir):
    """The Pico's record of the run as a dict of strings: session.json's "device" section, or {} when there is none."""
    manifest = read_manifest(session_dir)
    if manifest and isinstance(manifest.get("device"), dict):
        return dict(manifest["device"])
    return {}


def is_session(session_dir):
    """True for a folder whose session.json holds the Pico's record of the run (a session pulled from the SD card)."""
    session_dir = Path(session_dir)
    if not session_dir.is_dir():
        return False
    manifest = read_manifest(session_dir)
    return bool(manifest) and isinstance(manifest.get("device"), dict)


def is_evidence(session_dir):
    """False for a session whose replies were played by the wingman's ESC mock (session.json carries "esc_mock"): it
    shows that our side of the link works, nothing about an ESC, a motor or arming. Every tool that counts or compares
    sessions (arming rate, start quality, statistics over a folder) must skip a session for which this is False."""
    manifest = read_manifest(session_dir)
    return not (manifest and manifest.get("esc_mock"))


def fold_device_meta(session_dir):
    """Move a pulled meta.txt into session.json's "device" section (creating session.json when there is none) and
    delete meta.txt. Returns True when it folded something, False when there was no meta.txt."""
    session_dir = Path(session_dir)
    path = session_dir / DEVICE_META_FILE
    if not path.is_file():
        return False
    device = parse_meta(path.read_text(encoding="utf-8", errors="replace"))
    manifest = read_manifest(session_dir) or {"session": session_dir.name}
    manifest["device"] = device
    (session_dir / MANIFEST_FILE).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    path.unlink()
    return True
