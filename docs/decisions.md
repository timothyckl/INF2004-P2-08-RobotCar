# Decision Resolutions

## Subsystem 1 — WiFi/MQTT (Hong Yih)

_No decisions recorded yet._

## Subsystem 2 — Vehicle and Motion Control (Dalton)

_No decisions recorded yet._

## Subsystem 3 — Line Following and Barcodes (Edmund)

_No decisions recorded yet._

## Subsystem 4 — IMU and Terrain Monitoring (Khang)

### 1. Standalone ownership and wiring

Subsystem 4 owns the GY-511 / LSM303DLHC and I2C0. Use GP0 for SDA and GP1
for SCL, power the module from 3V3, and use a common ground. UART0 and its
crash-report mirror must remain disabled or be relocated while this wiring is
present. Only `s4_imu_task` accesses I2C0; the standalone implementation has
no dependency on motor, encoder, controller, or network subsystems.

### 2. Sensor mode and calibration

Configure the accelerometer for 100 Hz, all axes enabled, +/-2 g,
high-resolution mode, and block-data update. Verify the magnetometer identity
bytes and read back both accelerometer control registers before accepting the
device. Require fresh stationary calibration at startup and after local sensor
recovery. Assume the installed sensor X axis points forward and Z points up;
physical orientation must be recorded before validation.

### 3. Terrain and fault processing

Use integer milligravity and millimetre units. Confirm ascent, descent, and
return-to-level over consecutive samples before reporting a hump. A direct
ascent-to-level transition is incomplete. Treat repeated invalid samples,
stale timestamps, unstable calibration, I2C failures, and data overruns as
faults. Local recovery closes and revalidates the sensor and recalibrates,
while retaining the run maximum; only an explicit new-run reset clears it.

### 4. Experimental measurements

Candidate height is the accumulated forward distance multiplied by the
small-angle gravity projection. It is valid only when distance increments are
provided for the active ascent. The standalone task has no encoder input, so
its height result remains invalid even when a hump is detected. The
LSM303DLHC has no gyroscope; no angular-rate result may be claimed.

All current calibration spans, tilt thresholds, impact thresholds, and sample
confirmation counts are provisional engineering estimates. They are not
acceptance criteria and must be replaced or approved using recorded physical
evidence.

### 5. Test evidence

Run synthetic logic and raw-decode cases in the Pico W `TEST=1` image. Enable
the live startup check only with `TEST=1 S4_HW_TEST=1` and connected hardware.
A successful cross-build is not a passed on-board or physical test. Record the
commit, wiring, mounting, reference instruments, serial log, raw observations,
and repeated measurements before marking a hardware case passed.

## Subsystem 5 — Obstacle Scanning and Bypass Planning (Timothy)

### 1. Test-first approach

Write specifications and executable C tests for the Pico W before production code. Build with `TEST=1`, flash the firmware and collect results from the Pico W serial console. Run logic cases on the Pico W using controlled timestamps and injected inputs, followed by real RTOS/ISR, sensor, servo and integrated bypass tests. A computer builds the firmware and saves logs; it does not execute the test cases.

### 2. Hardware test setup

Use TRIG GP2, ECHO GP3 and servo GP15. Verify the assembled wiring, ECHO level shifting, supply voltages and actual servo model before powered tests. Obtain the matching servo datasheet before choosing PWM limits. Use a ruler, angle reference, logic analyser and voltage measurement for calibration.

### 3. Ranging rules

- Use integer millimetres and microsecond echo timing, with the datasheet's 340 m/s reference for conversion tests.
- Accept nominal distances of 20–4000 mm; establish usable course coverage through physical tests.
- Use a trigger pulse of at least 10 microseconds, at least 65 ms between triggers and a 30 ms echo-completion deadline. Never retrigger while ECHO is high.
- Monitor forward every 65 ms; reject data older than 150 ms.
- An unsafe, invalid or stale forward reading requests a stop immediately. Retry invalid readings only while stationary; three consecutive invalid attempts or stuck ECHO causes FAULT.

### 4. Scan parameters

Scan −60° to +60° in 15° coarse steps, then use 5° fine steps around detected edges and candidate gaps. Define 0° as forward and positive angles as left. Use the median of three valid readings per angle.

Calibrate pulse-to-angle mapping and wait the worst measured settling time plus 25% before sampling. Block this configuration if the servo cannot safely reach the required angles. Limit a complete scan to 15 s. Return forward and obtain fresh clearance before bypass movement.

### 5. Profile and bypass acceptance

Convert samples to vehicle-relative points and estimate width from fine-scan edges. Treat incomplete edges and unobserved space as unknown.

Evaluate left/right routes against the whole vehicle's swept footprint. Choose the shorter valid route, left on a tie. Require 50 mm physical clearance plus planning margins for measured sensing and motion errors. No safe route causes FAULT.

Set the hazard threshold from measured sensor offset, worst braking distance, travel during 65 ms sampling plus 100 ms response, 50 mm clearance and measurement uncertainty. Reject operating speeds beyond validated detection coverage.

### 6. Stop, cancellation and recovery contract

Match scan authorisation and stop confirmation to the active request ID. Confirm both wheels stationary for 100 ms using calibrated encoder thresholds; allow at most 2 s for confirmation.

Distinguish existing motor inhibition from a new STOP event. A new STOP, cancellation, vehicle movement or withdrawn authorisation aborts the scan and invalidates its echo generation, profile and plan. Acknowledge cancellation and reject late results. Recovery requires controller-authorised RESET and fresh sensor validation.

### 7. Messages and compact formats

Allow one active scan request and one pending result. Reject busy, duplicate and expired requests. Unstarted requests expire after 2 s; unaccepted plans expire after 2 s or a reference-pose change.

Include message type, request ID, capture time and validity. Plans additionally contain reference pose, selected side, move/turn segments, minimum clearance and failure reason. Keep continuous clearance in protected shared state and discrete results in queues. Use non-blocking sends; an undeliverable result is invalidated and raises a delivery fault.

Use explicit masks/shifts, little-endian multi-byte fields and separate validity flags. Derive widths from valid ranges and precision; document the field table and golden messages before implementation. Telemetry contains obstacle status, essential measurements and bypass outcome; detailed scans stay local.

### 8. ISR/task handoff

Use the coordinated GPIO dispatcher. Capture native timestamps in the ISR; snapshot, validate and pack them in the scan task. Protect the handoff against both interrupts and cross-core access. Clear old completion signals before arming and associate accepted edges with the active trigger generation.

Give each hazard/fault its own event bit. Clear only handled bits. Test timestamp wraparound, stale completion signals, simultaneous events and cancellation during handoff; no lost or torn updates are acceptable.

### 9. Original-line confirmation

Combine line validity with encoder pose and the recorded original line corridor and direction. Require an unambiguous corridor match, heading within 15° of the original travel direction and stable valid line detection for 100 ms. Calibrate corridor tolerance from track width and odometry error.

Bound the controller's search to 5 s or 500 mm, whichever comes first. Ambiguous branches or exhausted bounds cause a stopped fault.

### 10. Acceptance and evidence

Repeat each physical measurement condition 10 times. Target distance error no greater than the larger of 10 mm or 2%, and width error no greater than the larger of 20 mm or 5%. Require 10/10 safe returns for each passable layout and safe stopped outcomes for every blocked, invalid or cancelled case.

Test under concurrent encoder, IR, ECHO, servo and WiFi load. Require control cycles ≤20 ms, stop/slowing initiation ≤100 ms after unsafe confirmation, and STOP-flag-to-motor-off ≤20 ms. Measure STOP reception-to-flag and physical braking separately.

Compare actual RAM, queue/payload sizes and timing against an unpacked reference using identical data and workload. Record calibration, tested commit, inputs, results and saved evidence. Numerical targets remain unvalidated until tested.
