"""
arming_bound.py - the published arming bound, in one place.

CLEAN_ARM_THRESHOLD_MS is read by scripts/run_scenarios.py (the per-run arming verdict) and by scripts/session_summary.py (the
"armed within the bound" proof of a session's verdict). The two must never disagree, so neither holds a copy; README.md and CLAUDE.md quote the
same number and must be kept in agreement with it. Importing this module has no side effects.
"""

# This IS the KPI's duration bound (BUG-002's stable-arming KPI: in >=99% of
# attempts, every configured motor reaches ARMED within this many ms of
# arm() - not a classification convenience). It also happens to sit in the
# gap between the "no reboot" duration cluster (~2.1-2.6s) and the "one
# reboot" cluster (~4.6s, one ~2.46s reboot cycle added) - reconfirmed
# archive-wide 2026-10-04 (698 sessions): clean arms 2078-2593ms, reboot arms
# >=4057ms, a clean ~1.4s gap with zero occupancy either side. Recheck this
# constant if a session is ever seen landing between the clusters, or if
# MotorGroup's own arm_duration_ms/READY_SPAN_MS defaults change (this number
# is derived from those, not independent of them). Set to 3000 on 2026-10-13 when
# the 3.0 s promise was published in README/CLAUDE.md (it was 3300 before);
# 3000 still sits inside the clean/reboot gap above, so no archived session
# changes category.
CLEAN_ARM_THRESHOLD_MS = 3000

# How the scenario runner words an arming that did not complete (tests/bench/dshot_protocol/run_scenario.py, arming_reason()).
# The batch runner classifies a run as refused by it and the session verdict fails the arming proof by it.
ARMING_REFUSED_TEXT = "arming did not complete within"
