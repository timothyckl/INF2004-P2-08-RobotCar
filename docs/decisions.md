# Decision Resolutions

## Subsystem 1 — WiFi/MQTT (Hong Yih)

_No decisions recorded yet._

## Subsystem 2 — Vehicle and Motion Control (Dalton)

### 1. Encoder and IMU pins

The wheel encoders originally used GP7 (A) / GP28 (B) and GP0 (A) / GP1 (B), agreed with Khang (subsystem 4) on 2026-11-02 to match the
robot's wiring. The team pin mapping of 2026-11-03 then put the IMU on I2C1 at
GP2 (SDA1) / GP3 (SCL1) and confirmed the wheels: motor 1 (GP8/GP9) drives the
right wheel and motor 2 (GP10/GP11) the left. Which encoder is on which
wheel is decision 4. Consequence: `CRASH_REPORT=1` (UART0 on GP0/GP1)
cannot be used. Subsystem 3 decision 5 supersedes encoder 1's pins with
GP16/GP17; encoder 2 remains on GP0/GP1.

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

### 4. Encoder sides, signs and counts per revolution (2026-11-04)

Measured on the bench (S2-H01 and S2-H03, `testing/subsystem2/TEST_LOG.md`)
and agreed by the subsystem 2 owner. Revises the encoder sides in
decision 1 and the pin mapping of 2026-11-03.

- Motor 1 drives encoder 1 and motor 2 drives encoder 2. With motor 1 on
  the right wheel, encoder 1 is the right encoder and encoder 2
  (GP0/GP1) the left. The wiring is unchanged; `motion_config.h`,
  `AGENTS.md` and `TEAM.md` were corrected. Before this, each wheel's speed
  loop read the other wheel's encoder.
- Going forward, the right encoder counts up and the left counts down
  (`MC_LEFT_ENC_INVERT 1`, `MC_RIGHT_ENC_INVERT 0`).
- `MC_COUNTS_PER_REV` is 1330 (x2 decode), provisional: the median of single
  hand-turned revolutions (1231 to 1450). The earlier 40 was a guess. A
  10-revolution count replaces it.

### 5. Encoder decode errors at full speed (2026-11-04)

Agreed by the subsystem 2 owner after S2-H04 on `de808dc` and `aa9979f`
(`testing/subsystem2/TEST_LOG.md`). Revises the S2-H04 pass criterion, which
was zero decode errors.

- A decode error is a rise and a fall of A pending in one interrupt
  (`drv_encoder.c`): the interrupt was serviced more than half an A period
  late, about 190 us at 5k counts/s. It loses 2 counts and moves nothing
  else. Errors rose with both motors running and at the brake step.
- S2-H04 passes if every step prints "done" and brakes, the lost counts
  (2 per error) stay at or below 0.1% of each step's counts, the step
  including its 2 s braked print, and no 100 ms print shows more than
  `MC_ENC_ERRORS_MAX` new errors. 0.1% is 0.5 mm on a 500 mm move,
  against the 20 mm accuracy target.
- Observed on `aa9979f`: worst step H04c, 8 errors on encoder 2 in about
  25,600 counts (0.06% lost); never more than 1 per 100 ms print.
- Re-check under MQTT and full-system load. If the limit is exceeded,
  find the interrupt latency source instead of relaxing it again.

## Subsystem 3 — Line Following and Barcodes (Edmund)

### 1. Analogue front sensors and original barcode remap (2026-11-06)

The team selected the wiring contained in Edmund's supplied ZIP for the
integrated robot: front-left AOUT uses ADC0/GP26, front-right AOUT uses
ADC1/GP27, and the side barcode DOUT uses GP16. Both front DOUT wires and the
barcode AOUT wire are disconnected. ADC2/GP28 remains disabled for the right
encoder and ADC3/GP29 remains disabled for the Pico W radio.

Decision 5 supersedes only the barcode and encoder-1 pins in this paragraph;
the front analogue mapping and line-following policy below remain current.

The Line Sensor task filters both 12-bit channels independently, adapts only
their white baselines, applies hysteresis, and publishes a 0..100 side
contrast strength. With the selected straddle geometry, both-white is the
centred observation and does not time out as LOST; physical removal from the
line is therefore not distinguishable from a centred line using these two
front readings alone. The Vehicle Controller retains movement authority. It
uses the ZIP's provisional 40 mm/s straight profile, strength-scaled gentle
corrections, 5/35 mm/s sustained curves, and bounded crossing/exit state
machine. All thresholds, speeds and times are bench-derived starting values;
they are not validated course acceptance results.

### 2. Separate barcode bench diagnostic (2026-11-11)

Barcode hardware testing is separate from front-sensor line diagnostics.
`barcode_test.c`, enabled only by `TEST=1 S3_BAR_DIAG=1`, is entered before
any robot subsystem task is created. Following decision 5, it samples
side-sensor AOUT on ADC2/GP28, logs raw/filtered values, the adaptive threshold,
classified transitions and polling-based run durations, and runs `s3_scan`
locally. It does not start the line task, motors, IMU, obstacle scanner, or
application network tasks, and does not change barcode framing, timing windows,
or the active A/F/Z alphabet.

### 3. Controlled barcode-drive diagnostic (2026-11-11)

After the stationary ADC2/GP28 colour test passes, `S3_BAR_DRIVE=1` starts only
subsystems 2 and 3 in a single-core, no-WiFi image. Robo Pico button 1 on GP20
is an active-low diagnostic input owned by subsystem 3; it submits START,
STOP, or RESET requests to the Vehicle Controller. It does not access PWM.
The controller retains the provisional 40 mm/s line-follow profile and Motion
Control remains the only motor writer. A low-priority observer reports actual
wheel speeds and production barcode decoder results without changing barcode
timing or symbols.

### 4. Provisional barcode timing from controlled drive (2026-11-11)

One controlled pass at the provisional 40 mm/s follow speed observed narrow
runs of approximately 120 to 220 ms and wide runs of approximately 420 to
540 ms. The earlier 250 ms quiet period split valid wide white runs, while
the 400 ms run limit rejected them. The team therefore selected provisional
700 ms quiet and 800 ms maximum-run bounds for the next S3-H04 capture. These
values are evidence-based test settings, not validated acceptance limits;
repeat every printed symbol under course lighting before finalising them.

### 5. Grove 4/7 swap and analogue barcode input (2026-11-11)

The team swapped the Grove 4 and Grove 7 connections. Encoder 1 on the right
wheel now uses GP16/GP17 through Grove 4. The side barcode sensor now uses its
AOUT on ADC2/GP28 through Grove 7; its DOUT on GP7 is disconnected. The GP28
battery-sense jumper remains open. The two front AOUT signals remain on
ADC0/GP26 and ADC1/GP27.

The Line Sensor task is the sole ADC owner and samples all three channels every
10 ms. Only ADC0 and ADC1 affect steering. ADC2 is independently filtered and
classified for barcode capture; the raw sample and adaptive threshold are
published for calibration. The initial ADC2 threshold and the 10 ms barcode
sampling rate are provisional until S3-H04 records stationary white, printed
black and faded-black values at final height and repeated driven passes.
The moving diagnostic prints a bounded full-frame duration and N/W pattern
after the quiet period so the assumed Code 39 framing can be checked.

### 6. Calibrated barcode threshold and A/F/Z alphabet (2026-11-11)

At the final mounted height, 50 ADC2 samples per surface measured white at
154..184, solid printed black at 1679..1709 and faded printed black at
1637..1672. Subsystem 3 therefore uses an initial white baseline of 169, a
black margin of 741 and a white-release margin of 378 for the next moving
test. These values remain provisional until repeated driven passes complete
S3-H04.

The active course alphabet is A, F and Z in mission and diagnostic builds.
Patterns measured from the supplied artwork are A=`WNNNNWNNW`,
F=`NNWNWWNNN` and Z=`WWWNNNNNN`; wide elements are encoded as one from bit 8
to bit 0. The observed Z capture matched the supplied Z payload exactly.
This decision does not assign navigation meanings. The older prototype still
maps A to LEFT; F and Z do not stage a navigation request. That separate
mapping remains unvalidated and must not be treated as a course requirement.

### 7. Thin-bar capture and bounded decode tolerance (2026-11-11)

A lower-sensor pass over the small Z captured 27 of the required 29 runs,
including a thin black transition at ADC 682 while steady white remained about
156..233. The provisional barcode black margin is therefore reduced from 741
to 400 ADC counts (initial threshold 569), with a 340-count white-release
margin. This threshold change must be rechecked at the fixed test height; it is
not a validated calibration result.

The decoder still requires a complete 29-run frame, exact start and stop
sentinels, and exact narrow inter-character gaps. Within the nine-element
payload only, it may accept one narrow/wide error (8/9 correct) when exactly
one of A, F or Z is the uniquely closest pattern. Incomplete frames and a tie
between symbols remain decode errors. A generic 70% match is not accepted
because it can turn missed transitions or ambiguous payloads into a false
letter.

A subsequent pass still captured 27 runs. Its shortest detected bars were
20..40 ms, comparable with the delay of the two-sample low-pass filter used by
the front line sensors. Barcode ADC2 classification therefore bypasses that
low-pass filter while retaining threshold hysteresis and white-baseline
adaptation. The front ADC0/ADC1 line sensors keep their existing filtering.

### 8. Smaller Z observed variant (2026-11-12)

Three normal-direction passes over the smaller Z were reviewed. One complete
29-run observer frame had exact `NWNNWNWNN` start and stop sentinels, narrow
inter-character gaps, and payload `NWWNWNNNN` (`0x0D0`). The other two
observer frames contained 27 runs and remain rejected as incomplete; they are
not stored as symbol variants.

The previously accepted larger-Z pattern `WWWNNNNNN` (`0x1C0`) remains in the
alphabet. The complete smaller-Z payload is added as a second dictionary entry
for the same `Z` symbol. The decoder's existing whole-frame reversal is applied
before dictionary matching, so both Z entries are accepted in either travel
direction. Exact sentinels, exact gaps, a complete 29-run frame, and a unique
closest dictionary entry are still required. A 27-run observation cannot be
repaired or decoded.

The moving diagnostic reconstructs its duration list from published line-task
snapshots. Its observed run count is evidence for diagnosis, but the next
hardware pass must confirm the production decoder result through
`[S3 BAR RESULT] DECIPHERED=Z` in both directions.

### 9. Bounded partial-frame decode (2026-11-12)

Three opposite-direction smaller-Z passes produced complete 29-run frames and
decoded Z. A separate normal-direction failure produced 25 runs. Its first 20
runs were an exact sentinel, gap, `NWWNWNNNN` payload and gap; only the final
sentinel had collapsed from nine runs to five.

The decoder therefore does not use a generic 80% pattern match. It accepts a
24--28 run capture only when an intact end contains a complete 20-run core:
sentinel, gap, payload and gap, or gap, payload, gap and sentinel. It rebuilds
only the damaged terminal sentinel and then applies the normal exact sentinel,
gap, unique-nearest-payload and forward/reverse checks. If both ends decode to
different symbols, or fewer than 24 runs were captured, the frame is rejected.

This supersedes decision 8 only for a damaged terminal sentinel. Missing or
ambiguous payload elements are not inferred, and a general percentage match
remains prohibited.

### 10. Smallest-Z barcode-only ADC threshold (2026-11-12)

Three controlled passes over the smallest Z produced repeatable 7-run and
3-run fragments instead of the required 29 runs. Detected transitions were
120..160 ms and 520 ms, so the 10 ms sampling period is not the immediate
limit. The sensor reliably saw the wide printed bars, but missed most thin
black bars. Samples still classified as white rose to ADC 491..523 over the
print while ordinary moving white readings remained at or below 352; the
barcode threshold was 554..584.

The ADC2 barcode-only black margin is reduced from 400 to 250 and its release
margin from 340 to 180. With the bounded white baseline this gives a black
threshold of 404..434 and a release level of 334..364. Front ADC0/ADC1 sensor
thresholds, the 10 ms task period, decoder patterns and framing rules are not
changed. Repeated hardware passes must confirm 29-run frames without false
edges on white paper before this calibration is accepted.

### 11. Full standard Code 39 letter table (2026-11-12)

Testing is reported to select an arbitrary letter from A through Z, rather
than only the example artwork reviewed so far. The local project notes did not
contain a definitive A--Z statement, so this remains subject to confirmation
against the assessment brief; accepting the complete standard table is the
safer implementation and does not weaken frame validation.

Mission and diagnostic builds now share one standard Code 39 A--Z lookup table
in `s3_code39.c`. Host tests feed all 26 letters forwards and backwards. This
supersedes the restricted-alphabet parts of decisions 6 through 8. In
particular, `0x1C0` is the standard Code 39 pattern for W and `0x0D0` is Z;
the former is no longer retained as a non-standard Z alias. Framing, sentinel,
gap, contrast and ambiguity checks are unchanged.

Decoded E--Z letters are reported through IF-06 but do not automatically
command a turn. The existing separate navigation policy maps only A--D to
left, right, straight and U-turn. Any different letter-to-manoeuvre mapping
requires an explicit course requirement.

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

2026-11-04: the team keeps the HC-SR04 on 3V3 and will not move it to 5 V.
Reason given: other groups run the same module at 3V3. This is a wiring
decision, not a measurement; ranging at 3V3 on this robot remains
unvalidated until S5-U48 (known distances, 10 repetitions) passes. The
S3-H03 runs of 2026-11-04 stopped in a way consistent with the forward
monitor seeing no echo, so S5-U48 is needed before line-following
results can be trusted.

### 4. UART0 on the encoder pins (D4)

Builds that would give GP0/GP1 to UART0 are refused: the makefile rejects
`CRASH_REPORT=1` and `CONSOLE=uart`, and `hw_setting.c` and
`config_device.h` fail to compile if UART0 would claim the pins. I2C0 is never
started. Its dormant GP20/GP21 placeholders are not configured; GP20 is now
claimed by the subsystem 3 barcode-drive diagnostic and I2C0 must be remapped
before anyone enables unit 0.

### 5. Network hook in the shared MQTT client

Subsystem 1's service bridge runs inside `cyw43_utk_task`, called from
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` (a shared-library change made
at the integration owner's direction; TEAM.md section 3 still asks for
instructor agreement). Command receipt times use the controller's clock
(`s2_clock_ms()`). A WiFi or broker loss produces no controller input.
