# Subsystem 3 foundation verification

Run date: 2026-10-27 (Asia/Singapore).

Command from project root:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/run_tests.ps1
```

Observed result: **656 checks, 0 failures**.

Host compilation used GCC with C99, `-Wall -Wextra -Werror -Wconversion
-Wshadow -pedantic`. Compilation completed without diagnostics.

Verified with synthetic inputs:

- Every three-sensor polarity combination and normalized input pattern.
- Failed/absent calibration, stuck sensor observation and invalid bits.
- Settling, glitch rejection, invalid filter configuration and saturation.
- All eight configurable line-table entries and invalid lookup inputs.
- Barcode edge capture, repeated levels, zero-duration rejection, timer wrap,
  no-edge timeout, bounded storage and overflow rejection.
- Prototype single-symbol frame decoding, invalid sentinels, ambiguous alphabet,
  width ambiguity, invalid thresholds/gaps and truncated frames.
- A-D example command mapping, invalid input, pending-command protection,
  junction/authority gating, duplicate IDs, STOP cancellation, expiry and wrap.
- Complete synthetic barcode capture -> decode -> stage -> junction request.

The repeated steady-state assertions are included in the check count; it is not
a count of independent scenarios. Test patterns and timing are fixtures, not
measured/calibrated values or evidence of course barcode compatibility.

New C/header files were checked for tabs and lines longer than 80 characters;
none were found. Existing starter code and other subsystems were not edited.

Not verified: Arm/Pico firmware build (no `arm-none-eabi-gcc` found on PATH),
RTOS integration, actual sensors/wiring, printed alphabet/framing, reverse reads,
sampling performance, junction recognition, motor behaviour or original-line
reacquisition. No Git branch/commit/push: this workspace is a downloaded tree
without Git metadata.
