# Subsystem 3 host-test baseline

Run date: 2026-11-04 (Asia/Singapore).

Command from the project root:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/run_tests.ps1
```

Observed result: **552 checks, 0 failures**.

Host compilation used GCC with C99, `-Wall -Wextra -Werror -Wconversion`,
`-Wshadow`, and `-pedantic`. Compilation completed without diagnostics.

The checks cover the two-front-sensor calibration/polarity combinations, all
four normalized input patterns, filtering, synthetic barcode capture/decoding,
and navigation request staging. Repeated steady-state assertions contribute to
the check count; this is not a count of independent scenarios.

Not verified by this run: the Arm/Pico firmware build, RTOS integration, real
sensor wiring or polarity, physical white/black detection, course barcodes,
junction recognition, line following, motor behaviour, or track performance.
The Arm cross-compiler was not available on `PATH` in this environment.
