#!/bin/sh
# Line and branch coverage of the host suites (clang source-based coverage).
#
#   sh testing/host/coverage.sh            from the repository root
#
# Needs Apple clang or LLVM clang with llvm-profdata and llvm-cov (on macOS
# they come with the Xcode command line tools, run through xcrun). Uses the
# existing Makefile unchanged: only CC is overridden. Test files are left out
# of the report, so the figures cover application code only. Hardware drivers
# and RTOS task files are not built on the host and are not in the report.
set -e

OUT=$(mktemp -d)
LLVM_PROFILE_FILE="$OUT/%p.profraw" make -C testing/host OUT="$OUT" \
    CC="cc -fprofile-instr-generate -fcoverage-mapping -O0"

xcrun llvm-profdata merge -o "$OUT/all.profdata" "$OUT"/*.profraw

OBJECTS=""
for suite in s1_logic s2_logic s3_logic s4_logic s5_logic s5_view s2_int; do
    OBJECTS="$OBJECTS -object $OUT/$suite"
done

# shellcheck disable=SC2086
xcrun llvm-cov report $OBJECTS -instr-profile="$OUT/all.profdata" \
    | grep -E '^(Filename|app_program/.*\.c )' \
    | grep -vE '/test_|_test\.c|comms_test|obstacle_test|scan_test_adapter|test_interface'

echo "raw profiles: $OUT"
