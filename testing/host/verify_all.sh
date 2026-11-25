#!/bin/sh
# Full software verification of the current tree, as used for every
# post-demo change: host suites, dashboard tests, every build option of the
# target image with its warning count, static analysis, BARR-C checkers and
# the integrity of the demo evidence.
#
#   sh testing/host/verify_all.sh            from the repository root
#
# Exits non-zero if any host test, dashboard test or build fails, or if a
# build prints a warning. The static-analysis and BARR-C sections are
# reports; read them. Nothing here runs on the car.

ROOT=$(pwd)
FAILED=0
TMP=$(mktemp -d)

note_status() {
    echo "(exit status $1)"
    if [ "$1" -ne 0 ]; then
        FAILED=1
    fi
}

echo "# Verification, $(date '+%Y-%m-%d %H:%M %z'), $(git rev-parse --abbrev-ref HEAD) at $(git rev-parse --short HEAD)$(git diff --quiet || echo ' plus uncommitted changes')"
echo "# host: $(cc --version | head -1); target: $(arm-none-eabi-gcc --version | head -1)"

echo
echo "## 1. make -C testing/host"
make -C testing/host > "$TMP/host.txt" 2>&1
status=$?
grep -E "^S1-U|SUMMARY|checks,|KNOWN|noise blip|damaged end|BLOCKED|FAIL|all suites" "$TMP/host.txt"
note_status $status

echo
echo "## 2. dashboard unit tests"
# To a file, not a pipe, so the status is the tests' own (F-19).
python3 -m unittest discover -s testing/subsystem1/dashboard/tests \
    > "$TMP/dashboard.txt" 2>&1
status=$?
tail -3 "$TMP/dashboard.txt"
note_status $status

echo
echo "## 3. builds (make -B NET=lwip MQTT=1 S3_NAV_F=0 <option>)"
for option in "TEST=1" "TEST=1 S3_HW_TEST=1" "TEST=1 S3_LINE_TEST=1" \
              "TEST=1 S2_CAL_TEST=1" "TEST=1 S2_HW_TEST=1" \
              "TEST=1 S4_HW_TEST=1" "S2_MOTION_LOG=1" "S5_RANGE_DIAG=1" \
              "S4_RAW_LOG=1" ""; do
    # shellcheck disable=SC2086
    (cd build_make && make -B -j8 NET=lwip MQTT=1 S3_NAV_F=0 $option \
        > "$TMP/build.txt" 2>&1)
    status=$?
    warnings=$(grep -c "warning:" "$TMP/build.txt")
    echo "${option:-mission}: exit status $status, warnings $warnings"
    if [ "$status" -ne 0 ] || [ "$warnings" -ne 0 ]; then
        grep -E "warning:|error:" "$TMP/build.txt" | head -20
        FAILED=1
    fi
done
ELF=build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.elf
echo "size (text data bss dec hex file): $(arm-none-eabi-size "$ELF" | tail -1)"
echo "mission UF2 SHA-256: $(shasum -a 256 "${ELF%.elf}.uf2" | cut -d' ' -f1)"

echo
echo "## 4. python3 testing/host/target_static_check.py"
python3 testing/host/target_static_check.py 2>&1 | tail -6

echo
echo "## 5. BARR-C checkers"
python3 tools/barr_c_report.py 2>&1 | grep -E "^ +[0-9]+ "
python3 tools/check_subsystem5_barr_c.py 2>&1 | tail -1

echo
echo "## 6. evidence integrity"
shasum -a 256 testing/system/logs/2026-11-24_demo-dashboard-1.jsonl \
              testing/system/logs/2026-11-24_demo-dashboard-2.jsonl
echo "recorded: 2d845624...cf5c (dashboard-1), 4e450086...3441 (dashboard-2)"
echo "tags: demo-2026-11-24 -> $(git rev-parse --short 'demo-2026-11-24^{}'), demo-2026-11-24-flashed -> $(git rev-parse --short 'demo-2026-11-24-flashed^{}')"

cd "$ROOT" || exit 1
rm -rf "$TMP"
echo
if [ "$FAILED" -eq 0 ]; then
    echo "VERIFY: all tests and builds passed with 0 warnings"
else
    echo "VERIFY: FAILED"
fi
exit "$FAILED"
