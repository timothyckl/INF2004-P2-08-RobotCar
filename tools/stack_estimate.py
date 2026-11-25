#!/usr/bin/env python3
"""Static stack estimate of every application task (review F-24).

    python3 tools/stack_estimate.py            from the repository root

Recompiles every app_program .c file with the mission image's exact
compiler and flags (taken from `make -n`, as target_static_check.py does)
plus -fstack-usage and -fcallgraph-info=su, into a scratch folder (the
build and the image are not touched). For each task entry it prints the
deepest chain of application frames. Calls into the kernel, the SDK,
tm_printf, memcpy and memset are NOT counted, nor interrupt frames, so
the real use is higher; the run-time high-water marks printed by
app_stack_report() on the car are the measurement.
"""
import glob
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "testing", "host"))
import target_static_check  # noqa: E402  (same flags as the static check)

TASKS = ["s3_follow_run", "controller_task", "motion_task", "s4_imu_task",
         "obstacle_scan_task"]
NODE = re.compile(r'node: \{ title: "([^"]+)" label: "([^"]*)"')
EDGE = re.compile(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"')
BYTES = re.compile(r"\\n(\d+) bytes \((static|dynamic|bounded)")


def main():
    compiler, flags = target_static_check.target_flags()
    sources = sorted(glob.glob("app_program/**/*.c", recursive=True))
    frame, calls = {}, {}
    with tempfile.TemporaryDirectory() as out:
        for index, path in enumerate(sources):
            obj = os.path.join(out, f"{index}.o")
            subprocess.run([compiler, *flags, "-fstack-usage",
                            "-fcallgraph-info=su", "-c",
                            os.path.join("..", path), "-o", obj],
                           cwd=target_static_check.BUILD, check=True,
                           capture_output=True)
        for ci in glob.glob(os.path.join(out, "*.ci")):
            text = open(ci, errors="replace").read()
            for name, label in NODE.findall(text):
                size = BYTES.search(label)
                if size:
                    frame[name] = max(frame.get(name, 0), int(size.group(1)))
                calls.setdefault(name, set())
            for src, dst in EDGE.findall(text):
                calls.setdefault(src, set()).add(dst)

    def deepest(name, seen):
        """(bytes, chain) of the deepest application chain from name."""
        best = (0, [])
        for callee in calls.get(name, ()):
            if callee in seen or callee not in frame:
                continue
            depth = deepest(callee, seen | {callee})
            best = max(best, depth, key=lambda item: item[0])
        return (frame.get(name, 0) + best[0], [name] + best[1])

    print("# Static stack estimate (application frames only; kernel, SDK,")
    print("# tm_printf, memcpy, memset and interrupt frames not counted)")

    def short(name):
        """A static function's node is "path:name"; print the name."""
        return name.rsplit(":", 1)[-1]

    for task in TASKS:
        # A static task entry is qualified by its file in the graph.
        node = next((n for n in frame if short(n) == task), task)
        total, chain = deepest(node, {node})
        print(f"{task}: {total} bytes via "
              + " > ".join(f"{short(n)} {frame.get(n, 0)}" for n in chain))
    print("# largest single frames")
    for name, size in sorted(frame.items(), key=lambda kv: -kv[1])[:10]:
        print(f"{size:6}  {short(name)}")


if __name__ == "__main__":
    main()
