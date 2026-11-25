#!/usr/bin/env python3
"""Fit the hump calibration (S4 decision 17) from logged crossings.

    python3 testing/subsystem4/fit_hump_calibration.py REPLAY \\
        HEIGHT_MM:LOG [HEIGHT_MM:LOG ...] [--flat LOG ...]

REPLAY is the replay_raw binary (built by `make -C testing/host s4`, or by
hand, see testing/subsystem4/README.md). Each HEIGHT_MM:LOG is a console log
recorded with S4_RAW_LOG=1 while the car crossed a reference hump of that
height, measured with a ruler, under the right wheel; every crossing in it
counts. Each --flat LOG is a run on the same floor with no hump.

The replay gives each crossing's uncalibrated peak (scale 1000, offset 0,
the mission rise and FOLLOW-only gate). A least-squares line
    peak_mm = gain * true_mm + offset_mm
over all crossings gives the firmware constants
    S4_HEIGHT_SCALE_PERMILLE = 1000 / gain
    S4_HEIGHT_OFFSET_UM      = 1000 * offset_mm / gain
since the reported height is peak * scale / 1000 - offset. Every crossing
is then re-read with those constants and checked against +/-2 mm (the
team's SYS-04 criterion). The flat runs give the largest uncalibrated bump
while following, to check the rise (S4_RISE_UM) keeps its 2 mm margin.

With one height only, an offset cannot be told from a gain: the tool then
prints the gain-only scale and refuses to fit an offset. It never writes the
constants; a person reviews the numbers and changes s4_config.h.
"""
import subprocess
import sys

MARGIN_MM = 2.0     # decision 11: rise at least 2 mm above the floor
TOLERANCE_MM = 2.0  # SYS-04 engineering criterion


def peaks(replay, log, rise="-"):
    """Uncalibrated peaks (mm) of every completed hump in a log."""
    out = subprocess.run([replay, log, "1000", "0", rise, "follow"],
                         check=True, capture_output=True, text=True).stdout
    return [int(line.split()[5]) / 1000.0 for line in out.splitlines()
            if line.startswith("replay hump")]


def fit(points):
    """Least-squares gain and offset for (true, peak) pairs."""
    count = len(points)
    mean_x = sum(x for x, _ in points) / count
    mean_y = sum(y for _, y in points) / count
    sxx = sum((x - mean_x) ** 2 for x, _ in points)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in points)
    gain = sxy / sxx
    return gain, mean_y - gain * mean_x


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    replay = argv[1]
    points = []
    flats = []
    flat_mode = False
    for arg in argv[2:]:
        if arg == "--flat":
            flat_mode = True
        elif flat_mode:
            flats.append(arg)
        else:
            height, log = arg.split(":", 1)
            for peak in peaks(replay, log):
                points.append((float(height), peak))
                print("crossing  %5.1f mm reference  peak %6.2f mm  (%s)"
                      % (float(height), peak, log.rsplit("/", 1)[-1]))

    heights = sorted({x for x, _ in points})
    if not points:
        print("no crossings found")
        return 1
    if len(heights) < 2:
        mean = sum(y for _, y in points) / len(points)
        print("one reference height (%.1f mm, %d crossings, mean peak "
              "%.2f mm): gain-only scale %d per mille. An offset needs a "
              "second height; none fitted."
              % (heights[0], len(points), mean,
                 round(1000.0 * heights[0] / mean)))
    else:
        gain, offset = fit(points)
        scale = round(1000.0 / gain)
        offset_um = round(1000.0 * offset / gain)
        print("fit over %d crossings at %s mm: peak = %.3f x true + %.2f mm"
              % (len(points), heights, gain, offset))
        if offset_um < 0:
            print("negative offset (%d um): the firmware offset is "
                  "unsigned; use the gain-only scale or re-measure"
                  % offset_um)
        print("S4_HEIGHT_SCALE_PERMILLE %d, S4_HEIGHT_OFFSET_UM %d"
              % (scale, max(offset_um, 0)))
        worst = 0.0
        for true, peak in points:
            reading = round(max(peak * scale / 1000.0
                                - max(offset_um, 0) / 1000.0, 0.0))
            worst = max(worst, abs(reading - true))
        print("worst reading error with these constants: %.0f mm (%s)"
              % (worst, "within" if worst <= TOLERANCE_MM else "OUTSIDE"))

    for log in flats:
        bumps = peaks(replay, log, rise="2001")
        largest = max(bumps) if bumps else 0.0
        print("flat run %s: largest bump while following %.2f mm; the rise "
              "should be at least %.1f mm"
              % (log.rsplit("/", 1)[-1], largest, largest + MARGIN_MM))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
