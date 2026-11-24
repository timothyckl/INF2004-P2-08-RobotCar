#!/usr/bin/env python3
"""Strict warnings and GCC static analysis of the application code, using
the exact flags of the mission image.

    python3 testing/host/target_static_check.py      from the repository root

The mission build (build_make/makefile) passes no -W flags, so this script
re-runs every application .c file with the target compiler, defines and
include paths, plus a strict warning set (-fsyntax-only) and then
-fanalyzer. It does not change the firmware or the build. Kernel and SDK
headers are third-party code: their warnings are counted separately.
"""
import collections
import glob
import os
import re
import shlex
import subprocess
import sys

BUILD = "build_make"
MISSION = ["NET=lwip", "MQTT=1", "S3_NAV_F=0"]
STRICT = ["-Wall", "-Wextra", "-Wshadow", "-Wconversion", "-Wsign-conversion",
          "-Wcast-qual", "-Wpointer-arith", "-Wstrict-prototypes",
          "-Wmissing-prototypes", "-Wundef", "-Wswitch-default",
          "-Wdouble-promotion", "-Wnull-dereference", "-Wduplicated-cond",
          "-Wlogical-op", "-Wjump-misses-init", "-Wformat=2", "-Wunused"]
WARNING = re.compile(r"^(\S+?):(\d+):\d+: warning: (.*?)(?: \[(-W[^\]]+)\])?$",
                     re.M)


def target_flags():
    """Compiler and flags of one application file in the mission build."""
    dry = subprocess.run(["make", "-n", "-B", *MISSION], cwd=BUILD,
                         capture_output=True, text=True, check=True).stdout
    for line in dry.splitlines():
        if "gcc" in line and "robot_state.c" in line and " -c " in line:
            args = shlex.split(line)
            keep, skip = [], False
            for arg in args[1:]:
                if skip:
                    skip = False
                elif arg in ("-o", "-MF", "-MT"):
                    skip = True
                elif not (arg.startswith(("-M", "-c"))
                          or arg.endswith((".c", ".o"))):
                    keep.append(arg)
            return args[0], keep
    sys.exit("could not find the target compile command")


def main():
    compiler, flags = target_flags()
    # Paths relative to build_make, where the compiler runs.
    files = sorted(os.path.relpath(f, BUILD)
                   for f in glob.glob("app_program/**/*.c", recursive=True)
                   if "test" not in os.path.basename(f))
    print(f"compiler: {compiler}")
    print(f"files: {len(files)} application .c files (test files excluded)")
    for label, extra in (("strict warnings", STRICT + ["-fsyntax-only"]),
                         ("gcc -fanalyzer", ["-fanalyzer", "-c", "-o",
                                             os.devnull])):
        app, third_party = collections.Counter(), collections.Counter()
        for path in files:
            err = subprocess.run([compiler, *flags, *extra, path], cwd=BUILD,
                                 capture_output=True, text=True).stderr
            for match in WARNING.finditer(err):
                where = os.path.normpath(os.path.join(BUILD, match.group(1)))
                key = (where, match.group(2), match.group(3))
                (app if where.startswith("app_program") else
                 third_party)[key] += 1
        print(f"\n== {label}: {len(app)} in application code, "
              f"{sum(third_party.values())} in third-party headers")
        for (where, line, text), _ in sorted(app.items()):
            print(f"  {where}:{line}: {text}")


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)) + "/../..")
    main()
