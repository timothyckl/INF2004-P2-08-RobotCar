# Subsystem 3: Edmund's robot foundation

This is host-tested logic, not hardware-tested robot firmware.

**Integration status (2026-11-11).** `task_line.c` runs a 10 ms Line Sensor
task. It reads front-left AOUT through ADC0/GP26 and front-right AOUT through
ADC1/GP27, classifies them with independent adaptive white baselines and
hysteresis. It reads side-barcode AOUT through ADC2/GP28 and classifies it
independently; the barcode never steers the robot. It publishes IF-06
(`subsystem3.h`) to the Vehicle Controller, which remains the movement
authority. All three DOUT wires are disconnected. Thresholds, geometry and the printed
symbology still require S3-H01..H04 (`docs/integration.md`).

## Modules

- `s3_ir`: bounded 12-bit adaptive classification for each front analogue
  channel, with hysteresis and a 0..100 side contrast strength. It also keeps
  the consecutive-sample filter and line decision table used by `s3_track`.
- `s3_ir_gpio`: opens the kernel ADC device and reads ADC0/ADC1/ADC2
  synchronously. ADC read failure is latched as an S3 fault
  until the line task restarts.
- `s3_track`: both-white is STRAIGHT, one black sensor selects that side and
  both-black is a junction. Because centred and missing line are both-white,
  this geometry cannot infer line loss from those readings alone.
- `s3_bar` and `s3_scan`: bounded timestamped edge capture, explicit timeout
  polling, adaptive width classification and forward/reverse decoding. Normal
  decoding requires sentinel/gap/payload/gap/sentinel (29 runs). A bounded
  24--28 run fallback requires one intact sentinel, both gaps and the complete
  payload; it reconstructs only the damaged terminal sentinel. Ambiguous
  widths, invalid gaps, ambiguous dictionary matches, overflow and timeout are
  rejected. The shared lookup table contains the standard Code 39 A--Z
  alphabet; every letter is decoded in either travel direction. Printed
  hardware repeatability remains subject to validation.
- `s3_nav`: example A/B/C/D mapping to left/right/straight/U-turn, one pending
  request, duplicate scan suppression, age expiry and STOP cancellation. A request
  is held until an authorised junction. No turn angles or motor actions are set.
- `barcode_test`: test-only ADC2/GP28 diagnostic, enabled with
  `TEST=1 S3_BAR_DIAG=1`. It first gathers 50 samples each from white,
  solid black and faded black at the final mounted height. It rejects
  overlapping ranges and calculates a provisional runtime threshold before
  logging classified transitions, observed run durations, decoded letters and
  errors. The startup path does not create any robot subsystem or motor task.
  The isolated and moving diagnostics use the same standard A--Z lookup table
  as the normal Line Sensor task.
- `barcode_drive_test`: moving diagnostic enabled with `S3_BAR_DRIVE=1`.
  GP20 submits START/STOP/RESET requests to the Vehicle Controller, and a
  low-priority observer reports wheel speed and barcode results. It also
  prints a bounded duration list and narrow/wide pattern after the measured
  quiet period. It never writes motor PWM.

## Simulated-input tests (Windows PowerShell)

From the project root, with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/run_tests.ps1
```

For the separate on-target barcode bench diagnostic, build from `build_make`:

```sh
make SMP=0 WIFI=none TEST=1 S3_BAR_DIAG=1 CRASH_REPORT=0 -j8
```

After stationary colour detection passes, build the controlled moving test:

```sh
make SMP=0 WIFI=none S3_BAR_DRIVE=1 CRASH_REPORT=0 -j8
```

The runner builds in the system temporary directory and removes its executable.
It checks calibration/polarity, filtering, all configured line patterns, barcode
framing and failures, timer wrap, command staging, duplicates, expiry and STOP.
These tests do not establish barcode read success, line-tracking accuracy,
sampling rates, voltage compatibility or RTOS timing on the robot.

## Integration sequence

1. Keep the wiring aligned with `AGENTS.md` and `TEAM.md`. Do not connect a
   sensor output to another claimed GPIO.
2. Record raw ADC values for all three sensors on white and black, then validate
   the provisional limits and margins in `s3_config.h` under course lighting.
3. Confirm the straddle spacing and truth table. Both-white must remain centred
   through every curve; both-black must be a real junction before the bounded
   crossing logic is accepted.
4. Capture barcode ADC2 level changes in the owning Line Sensor task. Record
   stationary white, printed-black and faded-black values at final mounting
   height before accepting the adaptive threshold. Poll timeout without level
   changes and verify the 10 ms sampling period, quiet zones and framing.
5. Validate every supplied A--Z pattern at stated speeds and lighting
   conditions. Record repeated moving passes before accepting S3-H04.
6. Agree the controller message format, queue ownership, junction event and
   acknowledgement with Dalton. Stage each validated scan once with a unique ID.
   Call `s3_nav_take` each iteration with the controller's latched STOP state.
   The controller must reject stale/unauthorised requests and remains the final
   movement authority. Stop or reset also clears in-progress barcode capture.
7. Agree original-line reacquisition evidence with Timothy. This base does not
   claim to recognise the original line after a bypass.

All state is caller-owned, zero-initialised and single-task-owned. A non-null
pointer must refer to a valid object/array of the declared size. Time differences
must be less than one 32-bit wrap (about 71 minutes in microseconds); select
timeouts far below that. Scan IDs identify scans, not symbols: seeing the same
letter at a later barcode is legitimate. Only the last accepted ID is remembered;
the adapter must not replay older scans. Do not stage while STOP is latched.

## Git handoff

Run the host suites and a Pico W build before committing. Record hardware
measurements in `testing/subsystem3/TEST_LOG.md`; do not replace the historical
digital-comparator rows because they remain evidence for the former mapping.

IR means infrared; GPIO means general-purpose input/output; RTOS means real-time
operating system; PWM means pulse-width modulation. New C uses C99, fixed-width
data, explicit bounds, module prefixes, guarded headers and BARR-C brace style.
