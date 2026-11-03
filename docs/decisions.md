# Decision Resolutions

## Subsystem 1 — WiFi/MQTT (Hong Yih)

_No decisions recorded yet._

## Subsystem 2 — Vehicle and Motion Control (Dalton)

### 1. Encoder and IMU pins

The left wheel encoder uses GP7 (A) / GP28 (B) and the right wheel encoder
GP0 (A) / GP1 (B), agreed with Khang (subsystem 4) on 2026-11-02 to match the
robot's wiring. The team pin mapping of 2026-11-03 then put the IMU on I2C1 at
GP2 (SDA1) / GP3 (SCL1) and confirmed the wheels: motor 1 (GP8/GP9) drives the
right wheel and motor 2 (GP10/GP11) the left. Consequence: `CRASH_REPORT=1`
(UART0 on GP0/GP1) cannot be used.

### 2. PWM block reset

Motion Control releases the PWM block from reset in its own driver instead of
setting `USE_PTMR`, which would make the kernel use PWM as its physical timer.
It only clears that reset and never asserts it, because the servo (subsystem
5) shares the PWM block.

### 3. Motor protection

The RP2040 hardware watchdog is kicked only by Motion Control, once per control
period, and is armed only outside `TEST=1` and bench builds; a hung or starved
Motion Control task resets the chip, which stops the motors. Motion Control
ramps duty increases (full scale in 200 ms, provisional); decreases, STOP and
Fault are never ramped.

## Subsystem 3 — Line Following and Barcodes (Edmund)

_No decisions recorded yet._

## Subsystem 4 — IMU and Terrain Monitoring (Khang)

_No decisions recorded yet._

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
