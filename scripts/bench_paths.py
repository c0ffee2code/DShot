"""
bench_paths.py - put the folders the PC scripts import from on sys.path, in one place.

The session scripts (run_session, session_summary, session_timeline) and the unit tests use the wingman's PC tools and the pure modules in
driver/ and tests/bench/ (and the wingman's shared and main_pico modules). Importing this module (after putting scripts/ itself on the path) replaces a sys.path line in each of them:

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FOLDERS = (ROOT / "driver", ROOT / "tests" / "bench", ROOT / "tests" / "bench" / "wingman" / "tools", ROOT / "tests" / "bench" / "wingman" / "shared",
           ROOT / "tests" / "bench" / "wingman" / "main_pico")

for folder in FOLDERS:
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))
