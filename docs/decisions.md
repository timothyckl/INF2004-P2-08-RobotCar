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

### 1. Standalone ownership and wiring

Subsystem 4 owns the GY-511 / LSM303DLHC and I2C1. Use GP2 for SDA and GP3
for SCL, power the module from 3V3, and use a common ground. Only
`s4_imu_task` accesses I2C1; the standalone implementation has
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
stale timestamps, unstable calibration, and I2C failures as faults; a data
overrun is not a fault (decision 6). Local recovery closes and revalidates the sensor and recalibrates,
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

### 6. Overrun, turn-on and sample period (2026-11-04)

Agreed by the subsystem 4 owner after the first on-target S4-H01 runs
(`testing/subsystem4/TEST_LOG.md`, 2026-11-04). Revises the overrun clause
of decision 3.

- A data overrun (STATUS_REG_A ZYXOR) is not an error. The datasheet
  (7.1.8) defines it as new data overwriting unread data, so the driver
  reads the newest sample. Treating it as a fault had made every restart
  fault on its first read, because the sensor keeps sampling while closed.
- A failed read is one invalid sample. The fault comes only after
  `S4_INVALID_SAMPLE_LIMIT` consecutive invalid samples.
- After each open, the first `S4_TURN_ON_SAMPLES` (7) samples are discarded:
  turn-on time is 7/ODR in high-resolution mode (datasheet Table 8).
- A cyclic handler wakes the IMU task every `S4_SAMPLE_PERIOD_MS`, as in
  subsystems 3 and 5. `tk_dly_tsk()` adds one timer tick, so the previous
  10 ms delay sampled at about 50 Hz.
- A calibration fault prints the measured span per axis, as evidence for
  S4-H03.

## Subsystem 5 — Obstacle Scanning and Bypass Planning (Timothy)

### 1. Test-first approach

Write specifications and executable C tests for the Pico W before production code. Build with `TEST=1`, flash the firmware and collect results from the Pico W serial console. Run logic cases on the Pico W using controlled timestamps and injected inputs, followed by real RTOS/ISR, sensor, servo and integrated bypass tests. A computer builds the firmware and saves logs; it does not execute the test cases.

### 2. Hardware test setup

Use TRIG GP4, ECHO GP5 and servo GP12. Verify the assembled wiring, ECHO level
shifting, supply voltages and actual servo model before powered tests. The local
HC-SR04 datasheet specifies a 5 V supply, so the present 3.3 V test wiring is not
accepted as validated operation. Obtain the matching servo datasheet before
choosing PWM limits. Use a ruler, angle reference, logic analyser and voltage
measurement for calibration.

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

Distinguish existing motor inhibition from a new STOP event. A new STOP, cancellation, vehicle movement or withdrawn authorisation aborts the scan and invalidates its echo generation, profile and plan. Acknowledge cancellation and reject late results. A rejected initial servo command prevents scan entry. A rejected trigger command aborts the active scan and requests a best-effort return to forward; the request remains invalid even if that return command is also rejected. Do not recursively publish a fault through a failed output path. Recovery requires controller-authorised RESET and fresh sensor validation.

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

### 11. C module templates

Use these maintained templates for new subsystem 5 modules. Replace `module`
consistently and choose a root name that is unique in its first eight characters.

Header template:

```c
/**
 * @file module.h
 * @brief Describe the single entity exposed by this module.
 */
#ifndef MODULE_H
#define MODULE_H

/** Perform the documented module operation. */
void module_run (void);

#endif /* MODULE_H */

/* End of file. */
```

Source template:

```c
/**
 * @file module.c
 * @brief Implement the single entity exposed by this module.
 */
#include "../include/module.h"

void
module_run (void)
{
    /* Implement the documented operation. */
}

/* End of file. */
```

## Integration — all subsystems (integration owner: Dalton)

Agreed in the Phase 3 integration brief of 2026-11-03 (decisions D1 to
D4). Interface details built on these, and every provisional value, are in
`docs/integration.md`; they still need review by each subsystem owner.

### 1. Continuous motion for line following (D1)

Motion Control keeps sole ownership of motor PWM. Line following uses its
existing Speed command, exposed as `motion_drive(left, right)` and bounded by
`MC_SPEED_MAX_MM_S`. Each Speed command is a lease of `MC_SPEED_LEASE_MS`: the
Vehicle Controller renews it every cycle, and without renewal Motion Control
brakes by itself (STOPPED, reason `MOTION_STOP_LEASE`). Latched STOP rejects it
like every other motion command.

### 2. Line Sensor to Vehicle Controller (D2, IF-06)

Subsystem 3 publishes one snapshot (`s3_line_get()`, its own mutex, read
without waiting) with steering, line state, wrapping event counters and the
offered navigation command with a unique `nav_id`. The controller executes
each `nav_id` at most once. Subsystem 3 offers a command only at a junction,
while robot_state authorises navigation and STOP is clear. The side barcode
sensor is never a line input.

### 3. HC-SR04 supply (D3)

The documented 3V3 wiring is kept: it holds ECHO within the Pico's input
level, but the datasheet specifies 5 V and ranging at 3V3 is unvalidated.
Firmware applies the agreed timeout, invalid-reading and fault rules
(subsystem 5 decision 3) whatever the supply. Any wiring change needs team
review first.

### 4. UART0 on the encoder pins (D4)

Builds that would give GP0/GP1 to UART0 are refused: the makefile rejects
`CRASH_REPORT=1` and `CONSOLE=uart`, and `hw_setting.c` and
`config_device.h` fail to compile if UART0 would claim the pins. I2C0, which
is never started, has placeholder pins GP20/GP21 with a compile-time check
against the claimed pins.

### 5. Network hook in the shared MQTT client

Subsystem 1's service bridge runs inside `cyw43_utk_task`, called from
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` (a shared-library change made
at the integration owner's direction; TEAM.md section 3 still asks for
instructor agreement). Command receipt times use the controller's clock
(`s2_clock_ms()`). A WiFi or broker loss produces no controller input.
