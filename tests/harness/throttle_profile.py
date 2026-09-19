# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Compiles a scenario JSON motor "profile" (a list of hold/ramp/repeat
# segments - see scenario.py and the scenario JSON files under
# tests/harness/scenarios/) into a flat, monotonic list of
# (start_ms, throttle) waypoints at load time, so there is no segment-type
# branching left to do once a scenario is actually running.
#
# Every rule below raises ValueError at compile time rather than clamping,
# padding, or silently truncating - this is a QA harness, so "the scenario
# doesn't do what its JSON says" must fail loudly before anything is armed,
# not degrade into a slightly-different run nobody asked for.
#
# Pure Python, no MicroPython-only APIs - reused as-is by the PC-side
# scripts/check_scenario.py smoke test, not just on-device.


def compile_segments(segments, cursor, throttle):
    """
    Compile one segment list (a motor's top-level profile, or a repeat's
    inner segments) starting at absolute time `cursor` with the throttle
    already in effect being `throttle`. Returns (waypoints, cursor, throttle)
    after consuming every segment.
    """
    waypoints = []
    for segment in segments:
        kind = segment.get("type")

        if kind == "hold":
            value = segment["throttle"]
            duration_ms = segment["duration_ms"]
            waypoints.append((cursor, value))
            throttle = value
            cursor += duration_ms

        elif kind == "ramp":
            to = segment["to"]
            step = segment["step"]
            duration_ms = segment["duration_ms"]
            if step <= 0:
                raise ValueError("ramp step must be positive, got " + str(step))

            delta = to - throttle
            if delta == 0:
                num_steps = 0
            else:
                if abs(delta) % step != 0:
                    raise ValueError(
                        "ramp from " + str(throttle) + " to " + str(to) +
                        " with step " + str(step) + " does not divide evenly"
                    )
                num_steps = abs(delta) // step

            if num_steps > 0:
                if duration_ms % num_steps != 0:
                    raise ValueError(
                        "ramp duration_ms=" + str(duration_ms) +
                        " does not divide evenly into " + str(num_steps) + " steps"
                    )
                interval_ms = duration_ms // num_steps
                sign = 1 if delta > 0 else -1
                from_value = throttle
                for k in range(1, num_steps + 1):
                    waypoints.append((cursor + k * interval_ms, from_value + sign * step * k))

            throttle = to
            cursor += duration_ms

        elif kind == "repeat":
            inner = segment["segments"]
            total_ms = segment["duration_ms"]
            start_cursor = cursor
            iterations = 0
            while cursor - start_cursor < total_ms:
                before = cursor
                inner_waypoints, cursor, throttle = compile_segments(inner, cursor, throttle)
                waypoints.extend(inner_waypoints)
                iterations += 1
                if cursor == before:
                    raise ValueError("repeat's inner segments consume zero duration")
            consumed = cursor - start_cursor
            if consumed != total_ms:
                raise ValueError(
                    "repeat duration_ms=" + str(total_ms) +
                    " is not an exact multiple of its inner segments' duration "
                    "(" + str(iterations) + " iteration(s) consumed " + str(consumed) + "ms)"
                )

        else:
            raise ValueError("unknown segment type: " + str(kind))

    return waypoints, cursor, throttle


class ThrottleProfile:
    """
    A compiled, per-motor throttle-vs-time schedule.

    Usage:
        profile = ThrottleProfile(segments)   # segments: parsed JSON list
        profile.total_duration_ms             # sum of every segment's duration
        profile.throttle_at(elapsed_ms)        # forward-only, call with
                                                # non-decreasing elapsed_ms
    """

    def __init__(self, segments):
        waypoints, total_duration_ms, _ = compile_segments(segments, 0, 0)
        if not waypoints or waypoints[0][0] != 0:
            waypoints.insert(0, (0, 0))
        self.waypoints = waypoints
        self.total_duration_ms = total_duration_ms
        self.cursor = 0

    def throttle_at(self, elapsed_ms):
        """
        Throttle value in effect at `elapsed_ms` since the profile started.

        `elapsed_ms` must be non-decreasing across calls (true for a running
        scenario's wall-clock elapsed time) - this is a forward-only cursor
        scan, O(1) amortized, not a fresh search from the start each time.
        """
        waypoints = self.waypoints
        idx = self.cursor
        while idx + 1 < len(waypoints) and waypoints[idx + 1][0] <= elapsed_ms:
            idx += 1
        self.cursor = idx
        return waypoints[idx][1]
