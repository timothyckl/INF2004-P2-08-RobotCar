#!/usr/bin/env python3
"""Summarise an S4_RAW_LOG=1 serial capture for S4-H03, S4-H07 and S4-H08.

Usage:
    python3 testing/subsystem4/s4_raw_stats.py LOG [--track-mm 119]
                                              [--wheelbase-mm 88]

Reads the "s4 raw" lines printed by task_imu.c (one per IMU sample, after
processing) and recomputes the hump height estimate of terrain_logic.c
(s4_hump_height_um) from the filtered tilts and the pitch gate, so the noise
can be compared with the rise (4 mm) and level (2 mm) thresholds in mm.

Samples are split into "still" and "moving" by the subsystem 2 markers:
"GP20: START requested" starts moving; "CAL hump pass ... finished" or
"GP20: STOP requested" ends it. Calibrating and fault samples are skipped.

The constants below must match app_program/subsystem4/task_imu.c. This is a
diagnostic aid: its output is evidence for the test log, not a pass verdict.
"""

import argparse
import re
import statistics
import sys

RISE_MM = 4.0      # S4_RISE_UM
LEVEL_MM = 2.0     # S4_LEVEL_UM
STATE_MONITORING = 1
STATE_ON_HUMP = 2

RAW_RE = re.compile(
    r"s4 raw (?P<ms>\d+): fwd=(?P<fwd>-?\d+) left=(?P<left>-?\d+) "
    r"up=(?P<up>-?\d+) mg \| tilt fwd=(?P<tfwd>-?\d+) left=(?P<tleft>-?\d+) "
    r"mg \| pitch (?P<pitch>used|ignored) \| state (?P<state>\d+)")
START_RE = re.compile(r"GP20: START requested")
END_RE = re.compile(r"CAL hump pass, forward \d+ mm finished|GP20: STOP requested")
HUMP_RE = re.compile(r"s4: hump (\d+) mm under the (.+?) \(")
IMPACT_RE = re.compile(r"s4: candidate impact detected")


def hump_height_mm(tilt_fwd_mg, tilt_left_mg, pitch_used, track_mm, wheelbase_mm):
    """Same plane-through-contacts estimate as s4_hump_height_um (in mm)."""
    left = tilt_left_mg * track_mm / 2.0 / 1000.0
    right = -left
    lower = min(left, right)
    upper = max(left, right)
    caster = tilt_fwd_mg * wheelbase_mm / 1000.0 if pitch_used else lower
    return max(caster, upper) - min(caster, lower)


def describe(values):
    if not values:
        return "n/a"
    sd = statistics.pstdev(values) if len(values) > 1 else 0.0
    return "mean %7.1f  sd %6.1f  min %6d  max %6d" % (
        statistics.fmean(values), sd, min(values), max(values))


def summarise(name, samples, args):
    print("== %s: %d samples" % (name, len(samples)))
    if not samples:
        return
    for key, label in (("fwd", "accel fwd  mg"), ("left", "accel left mg"),
                       ("up", "accel up   mg"), ("tfwd", "tilt fwd   mg"),
                       ("tleft", "tilt left  mg")):
        print("  %s  %s" % (label, describe([s[key] for s in samples])))

    heights = [hump_height_mm(s["tfwd"], s["tleft"], s["pitch"] == "used",
                              args.track_mm, args.wheelbase_mm)
               for s in samples]
    n = len(heights)
    print("  height     mm  mean %7.2f  sd %6.2f  max %6.2f  p99 %6.2f" % (
        statistics.fmean(heights),
        statistics.pstdev(heights) if n > 1 else 0.0,
        max(heights),
        sorted(heights)[min(n - 1, int(0.99 * n))]))
    print("  height >= level %.0f mm: %d (%.1f %%); >= rise %.0f mm: %d (%.1f %%)" % (
        LEVEL_MM, sum(h >= LEVEL_MM for h in heights),
        100.0 * sum(h >= LEVEL_MM for h in heights) / n,
        RISE_MM, sum(h >= RISE_MM for h in heights),
        100.0 * sum(h >= RISE_MM for h in heights) / n))
    ignored = sum(s["pitch"] == "ignored" for s in samples)
    print("  pitch ignored: %d (%.1f %%)" % (ignored, 100.0 * ignored / n))

    gaps = [b["ms"] - a["ms"] for a, b in zip(samples, samples[1:])
            if 0 < b["ms"] - a["ms"] < 1000]
    if gaps:
        print("  sample gap ms  mean %.1f  max %d (gaps over 1 s, between runs, skipped)"
              % (statistics.fmean(gaps), max(gaps)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log")
    parser.add_argument("--track-mm", type=float, default=119.0)
    parser.add_argument("--wheelbase-mm", type=float, default=88.0)
    args = parser.parse_args()

    still, moving, humps = [], [], []
    impacts = 0
    b_moving = False
    b_seen_raw = False
    with open(args.log, encoding="utf-8", errors="replace") as log:
        for line in log:
            if START_RE.search(line):
                b_moving = True
            elif END_RE.search(line):
                b_moving = False
            match = HUMP_RE.search(line)
            if match:
                humps.append((int(match.group(1)), match.group(2),
                              "moving" if b_moving else "still"))
            if IMPACT_RE.search(line):
                impacts += 1
            match = RAW_RE.search(line)
            if not match:
                continue
            b_seen_raw = True
            sample = {k: (v if k == "pitch" else int(v))
                      for k, v in match.groupdict().items()}
            if sample["state"] not in (STATE_MONITORING, STATE_ON_HUMP):
                continue
            (moving if b_moving else still).append(sample)

    if not b_seen_raw:
        sys.exit("no 's4 raw <ms>: ...' lines found; build with S4_RAW_LOG=1")

    summarise("still", still, args)
    summarise("moving", moving, args)
    print("== events: %d hump(s), %d impact(s)" % (len(humps), impacts))
    for height, contact, phase in humps:
        print("  hump %3d mm  %-12s while %s" % (height, contact, phase))


if __name__ == "__main__":
    main()
