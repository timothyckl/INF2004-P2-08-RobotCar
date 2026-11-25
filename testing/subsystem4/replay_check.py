#!/usr/bin/env python3
"""S4-U44: replay every logged drive through the current terrain logic.

    python3 testing/subsystem4/replay_check.py <replay_raw binary>

Run by `make -C testing/host s4`. For each console log recorded with
S4_RAW_LOG and the detection gate (every IMU sample the car processed), the
replay (replay_raw.c) feeds the samples through terrain_logic.c with the
mission settings (s4_config_mission()) and the mission's FOLLOW-only gate.
It must report exactly the humps, in order and in mm, that the car itself
reported in that log: four crossings of the 10 mm hump read 11, 10, 10 and
9 mm, two more 9 and 9 mm, and none anywhere else. A change to the terrain
logic or its settings that alters a reading or adds a false hump on this
real data fails here and must be reviewed. Exit status 0 only if every log
matches.
"""
import subprocess
import sys

# Logs with raw samples and the detection gate, and the humps the car
# reported in each (S4 decision 17).
LOGS = [
    ("testing/subsystem4/logs/2026-11-13_s4-h08-gated.txt", [11, 10, 10, 9]),
    ("testing/subsystem4/logs/2026-11-13_s4-if18.txt", [9, 9]),
    ("testing/subsystem3/logs/2026-11-17_s3-nav-drive-2.txt", []),
    ("testing/subsystem3/logs/2026-11-17_s3-nav-drive-3.txt", []),
    ("testing/subsystem3/logs/2026-11-17_s3-nav-drive-4.txt", []),
]


def replay(binary, log):
    """Hump readings (mm) the replay reports with the mission gate."""
    out = subprocess.run([binary, log, "-", "-", "-", "follow"],
                         check=True, capture_output=True, text=True).stdout
    readings = [int(line.split()[2]) for line in out.splitlines()
                if line.startswith("replay hump")]
    summary = [line for line in out.splitlines() if line.startswith("# samples")]
    return readings, summary[0] if summary else ""


def main():
    binary = sys.argv[1]
    failed = 0
    for log, expected in LOGS:
        readings, summary = replay(binary, log)
        ok = readings == expected
        failed += 0 if ok else 1
        print("[TEST] S4-U44 %s  %s: car %s, replay %s"
              % ("PASS" if ok else "FAIL", log.rsplit("/", 1)[-1],
                 expected, readings))
        print("       " + summary)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
