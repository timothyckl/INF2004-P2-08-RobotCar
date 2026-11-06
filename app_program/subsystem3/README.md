# Subsystem 3: Edmund's robot foundation

This is host-tested logic, not hardware-tested robot firmware.

**Integration status (2026-11-06).** `task_line.c` runs a 10 ms Line Sensor
task. It reads front-left AOUT through ADC0/GP26 and front-right AOUT through
ADC1/GP27, classifies them with independent adaptive white baselines and
hysteresis, and reads side-barcode DOUT on GP16 through subsystem 2's shared
GPIO dispatcher. It publishes IF-06 (`subsystem3.h`) to the Vehicle Controller,
which remains the movement authority. Both front DOUT wires and barcode AOUT
are disconnected in this mapping. Thresholds, geometry and the printed
symbology still require S3-H01..H04 (`docs/integration.md`).

## Modules

- `s3_ir`: bounded 12-bit adaptive classification for each front analogue
  channel, with hysteresis and a 0..100 side contrast strength. It also keeps
  the consecutive-sample filter and line decision table used by `s3_track`.
- `s3_ir_gpio`: opens the kernel ADC device, reads ADC0/ADC1 synchronously and
  reads the GP16 barcode input. ADC read failure is latched as an S3 fault
  until the line task restarts.
- `s3_track`: both-white is STRAIGHT, one black sensor selects that side and
  both-black is a junction. Because centred and missing line are both-white,
  this geometry cannot infer line loss from those readings alone.
- `s3_bar`: bounded timestamped edge capture, explicit timeout polling and
  a configurable nine-element, one-payload-symbol framed decoder prototype.
  It requires sentinel/gap/payload/gap/sentinel (29 runs). Width windows,
  sentinel and alphabet are supplied by the caller; NONE are calibrated defaults.
  Input is normalized black/white, starting at the first black run. Begin/finish
  boundaries and surrounding quiet zones must be verified by the future scanner.
  Ambiguous widths, invalid gaps/framing, duplicate dictionary matches,
  overflow and timeout are rejected. Reverse travel, multiple payload characters,
  automatic framing, adaptive speed compensation and checksums are not implemented.
  The prototype format must be checked against the printed course barcode before
  selecting an alphabet. Tests deliberately use synthetic patterns.
- `s3_nav`: example A/B/C/D mapping to left/right/straight/U-turn, one pending
  request, duplicate scan suppression, age expiry and STOP cancellation. A request
  is held until an authorised junction. No turn angles or motor actions are set.

## Simulated-input tests (Windows PowerShell)

From the project root, with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/run_tests.ps1
```

The runner builds in the system temporary directory and removes its executable.
It checks calibration/polarity, filtering, all configured line patterns, barcode
framing and failures, timer wrap, command staging, duplicates, expiry and STOP.
These tests do not establish barcode read success, line-tracking accuracy,
sampling rates, voltage compatibility or RTOS timing on the robot.

## Integration sequence

1. Keep the wiring aligned with `AGENTS.md` and `TEAM.md`. Do not connect a
   sensor output to another claimed GPIO.
2. Record raw ADC values for each front sensor on white and black, then validate
   the provisional limits and margins in `s3_config.h` under course lighting.
3. Confirm the straddle spacing and truth table. Both-white must remain centred
   through every curve; both-black must be a real junction before the bounded
   crossing logic is accepted.
4. Capture barcode edges through the team's shared interrupt dispatcher. Keep
   decoding in the owning task; do not share capture state with an ISR/core
   without an agreed bounded queue. Feed raw edges, not line-filtered samples.
   Poll timeout without edges. Verify quiet zones/framing before finishing.
5. Confirm symbology and A-D mapping using the course materials. Supply measured
   width windows/alphabet; test at stated speeds and lighting conditions. Replace
   this decoder prototype if the printed format differs.
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
