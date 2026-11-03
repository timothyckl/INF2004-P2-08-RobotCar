# Subsystem 3: Edmund's robot foundation

This is host-tested base logic, not hardware-tested robot firmware.
The existing consumer example is preserved because Subsystems 2 and 4 still
depend on its mailbox/statistics interfaces. The new modules compile through
the existing flat source wildcard, but no robot task calls them yet. No GPIO,
interrupt, motor PWM, shared configuration or teammate code is changed.

## Modules

- `s3_ir`: digital calibration from stable white/black observations, polarity
  normalization, consecutive-sample filtering and an eight-entry line table.
  Bits identify sensor channels; the physical role of each must be confirmed.
  A failed calibration invalidates the result. An unsettled filter must produce
  UNKNOWN to the caller, not reuse a previous steering decision.
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

1. Read the IR module, comparator, Pico W and carrier datasheets before GPIO work.
   Reconcile AGENTS.md's agreed pins with TEAM.md's still-unclaimed starter rows.
   Confirm sensor models, digital output levels, roles, mounting and polarity.
   Digital comparator calibration is not ADC threshold calibration.
2. Add a GPIO adapter and sensor task only after those checks. Feed stable known
   white/black observations into `s3_ir_calibrate`; do not guess polarity.
3. Fill the eight-entry line table from the actual track/sensor geometry. Keep
   ambiguous patterns UNKNOWN. A table entry alone is not validated junction
   detection: agree junction recognition/debounce and controller handshaking.
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

## GitHub handoff

This downloaded workspace has no Git metadata. No branch, commit or push has
been made. In an actual clone, create `feature/subsystem3-ir-line-barcode`, copy
these additions, run the host tests and firmware build, then commit only the
Subsystem 3 additions. Shared startup/task wiring requires team agreement.

IR means infrared; GPIO means general-purpose input/output; RTOS means real-time
operating system; PWM means pulse-width modulation. New C uses C99, fixed-width
data, explicit bounds, module prefixes, guarded headers and BARR-C brace style.
