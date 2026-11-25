# Requirements traceability

This document links every requirement to its design decision, the code
that implements it, the tests that check it, the result and the evidence.
It follows the Lecture 6 chain:

**Requirement → design → source → test condition → acceptance criterion →
actual result → evidence → status**

A requirement is marked **Pass** only when a recorded test met its
acceptance criterion on the car. Code that exists, or a host test that
passes, is not enough on its own.

## 1. Sources

| Source | What it gives | Where |
|---|---|---|
| Project brief | Mission requirements MR1–MR10, expected behaviour, buddy tasks, assessment points | [Project Description.pdf](Project%20Description.pdf) |
| Design review | Initial architecture and ownership | [AAI08_Design_Review.pdf](AAI08_Design_Review.pdf) |
| Tribunal 1 | Data widths and later design changes | [AAI08_PostTri1.pdf](AAI08_PostTri1.pdf), [data_widths.md](data_widths.md) |
| Pre-Tribunal 2 submission | Final functional requirements FR1–FR12 and non-functional requirements NFR1–NFR6, each with a test ID. These refine MR1–MR10 into measurable form and are the requirements this matrix uses | Team submission of 2026-11 (quoted in section 3) |
| Tribunal 2 | Injected requirement: respond safely and predictably to any failure or invalid data; 22 analysed entries | [AAI08_ImmediateTri2.pdf](AAI08_ImmediateTri2.pdf), [tribunal_closure.md](tribunal_closure.md) |
| Agreed decisions | Every design change with its reason and approval | [decisions.md](decisions.md) (cited as "S2 d26" = subsystem 2, decision 26) |
| Test logs | Test definitions and every recorded run, including failures | `testing/*/TEST_LOG.md` |

## 2. Mission requirements to final requirements

| MR | Brief wording | Final requirement(s) | System test | Status (section 4) |
|---|---|---|---|---|
| MR1 | Follow a line track autonomously | FR1, FR2 | SYS-01, SYS-09 | Pass on the track (SYS-01); FR2's 10 ms sampling not met; full course not passed |
| MR2 | Detect and decode barcodes | FR3 | SYS-02 | Partial |
| MR3 | Execute barcode commands (left, right, straight, U-turn) | FR4 | SYS-03 | Partial |
| MR4 | Detect humps | FR5 | SYS-04 | Pass (detection) |
| MR5 | Measure and report the highest hump | FR5, FR10 | SYS-04 | Partial (accuracy within ±2 mm; flat lap not run) |
| MR6 | Detect obstacles on or near the line | FR6 | SYS-05 | Pass |
| MR7 | Profile obstacle shape and position | FR7 | SYS-06 | Fail (width), Pass (nearest point, side) |
| MR8 | Navigate around obstacles without collision | FR8 | SYS-07 | Pass |
| MR9 | Reacquire the original line | FR8 | SYS-07 | Pass |
| MR10 | Report telemetry over WiFi | FR10, NFR2 | SYS-08 | Pass |

## 3. Status legend and verification methods

| Status | Meaning |
|---|---|
| **Pass** | A recorded run on the car met the acceptance criterion |
| **Partial** | Some of the criterion met on the car, or met on fewer runs than required |
| **Fail** | The latest recorded run did not meet the criterion |
| **Host only** | Logic checked by host or on-target unit tests; no car run against this criterion |
| **Not tested** | No recorded run |

| Method | Meaning |
|---|---|
| Unit (host) | Pure logic compiled with `cc` and run by `make -C testing/host`; synthetic input |
| Unit (target) | Same logic in the `TEST=1` image on the Pico W |
| Hardware (H) | A person drives the hardware and records the result |
| Integration (INT) | Two subsystems exchange real messages on the car |
| System (SYS) | The whole car on a track |
| Analysis | Calculation or code review |

## 4. Traceability matrix

Latest results only; every earlier run, including failures, stays in the
test logs. "Demo" means the assessed runs of 2026-11-24
([demo report](DEMO_REPORT_2026-11-24.md)).

### FR1 — complete the course autonomously after one START

| Item | Detail |
|---|---|
| Requirement | Full course after one START (GP20 or MQTT), no touching until the final lift; START only in READY with Motion Control healthy and both wheels still |
| Owner / design | All; Vehicle Controller statechart (S2 d9, d26, d28) |
| Source | `subsystem2/controller_logic.c` `ctrl_tick()`, `ctrl_remote()`; `subsystem2/task_controller.c` `controller_task()`, `button_step()` |
| Tests | S2-U41, U43, U48 to U52 (host); SYS-09 |
| Criterion | Every course element handled in each of 3 runs, no STOP before the lift |
| Result | START gating passes on the host. SYS-09 **Fail** on the demo: the barcode turn did not happen ([system test log](../testing/system/TEST_LOG.md)) |
| Status | **Fail** (SYS-09); START rules **Host only** |
| Limitations | No full-course run was ever recorded, before or during the demo |

### FR2 — follow the 18 mm line

| Item | Detail |
|---|---|
| Requirement | Follow the 18 mm black line through curves, crossings and humps; two analogue IR sensors with adaptive thresholds sampled every 10 ms; a line report every 30 ms |
| Owner / design | S3, S2; S3 d3, d4, d8; S2 d8, d25 |
| Source | `subsystem3/s3_follow.c` `s3_follow_run()`; `subsystem3/s3_ir.c` `s3_ir_adaptive_update()`, `s3_ir_line()`; `subsystem2/line_control.c` `line_control_step()`; `subsystem2/task_controller.c` `line_step()` |
| Tests | S2-U57 to U59, U73 (host); S3 host checks (765); S3-H01, H02, H04; SYS-01 |
| Criterion | 3 laps, no wheel leaves the track, no STOP before the lift |
| Result | SYS-01 **Pass** on 2026-11-20 (3 laps of 2 min 8 s to 2 min 15 s). Demo: followed the line in every attempt |
| Status | **Partial**: line following passed on the car (SYS-01), but the requirement's 10 ms sampling is not met: the loop runs every 20 ms (F-01, proposal P1) |
| Limitations | The requirement says sensors are read every 10 ms; the loop actually runs every 20 ms because of the 10 ms kernel tick ([review](software_quality_review.md) F-01). A line report still goes out at once when the position changes. Lap telemetry covers lap 1 and part of lap 2 (hotspot carried out of range). Lighting was not recorded. The left sensor needed re-adjustment twice on 2026-11-20 (S3-H01) |

### FR3 — decode Code 39 barcodes

| Item | Detail |
|---|---|
| Requirement | Decode single-letter Code 39 in either direction while following; reject anything that is not exactly one letter; at 40 mm/s, at least 4 of 5 passes per course barcode and never a wrong letter |
| Owner / design | S3; S2 d25 (soften curves while reading) |
| Source | `subsystem3/s3_barcode.c` `s3_barcode_update()`; `subsystem3/s3_scan.c` `s3_scan_poll()`; `subsystem3/s3_bar.c` `s3_bar_decode()`; `subsystem3/s3_code39.c` |
| Tests | S3 host checks including S3-U02 and S3-U03 (post-demo); S3 bar-drive runs 2026-11-17; SYS-02 |
| Criterion | ≥ 4 of 5 passes per barcode, no wrong letter |
| Result | 2026-11-17: 4/4 and 2/2 F reads. Demo: 1 of 3 attempts decoded; no wrong letter. Host: every single misread element corrected or rejected; a steady speed change of 0.4 to 2.5 times across one code still decodes; a single 20 ms blip inside the code gave a wrong letter in 25 of 234 synthetic cases (S3-U03, F-18), and 0 of 234 with the P2 prototype, which is not in the firmware |
| Status | **Partial**: decoding works and never gave a wrong letter on the car, but did not reach 4 of 5; on the host a 20 ms noise blip inside the code can give a wrong letter (F-18, known defect, proposal P2) |
| Limitations | Bar timings have 20 ms resolution (F-01), coarse for small prints. Small prints need the sensor close to the floor (demo day). Sensor height, print size and paper condition were not recorded for most runs |

### FR4 — carry out the barcode command at the next junction

| Item | Detail |
|---|---|
| Requirement | Hold LEFT, RIGHT, STRAIGHT or U-TURN for up to 60 s; act at the next confirmed junction with encoder-based turns; an unmapped letter never turns |
| Owner / design | S2, S3; S2 d26, d27; S1 d6 (letters set over MQTT) |
| Source | `subsystem2/nav_control.c` `nav_control_stage()`, `nav_control_step()`; `subsystem2/task_controller.c` `nav_receive()`, `nav_update()`; `subsystem2/line_control.c` (junction confirmation); `subsystem3/s3_nav.c` `s3_nav_command()`, `s3_nav_set_letter()` |
| Tests | S2-U58, U74 to U77, U79, U80 (host); INT (letters over MQTT, 2026-11-23); SYS-03 |
| Criterion | Commanded branch taken in all 12 runs (3 per command) |
| Result | 2026-11-17 (`85bdf28`): one physical right turn. 2026-11-23: F mapped to LEFT, NAVIGATE then FOLLOW. Demo practice: U-turn after barcode F. Demo assessed runs: RIGHT decoded but no NAVIGATE |
| Status | **Partial**: each turn type except STRAIGHT has worked at least once; the 12-run criterion was never run |
| Limitations | Junction confirmation needs both front sensors on black for 100 ms, so a branch that leaves on one side only reads as a curve (see [software review](software_quality_review.md)). Junction and hold messages are console-only |

### FR5 — detect humps and keep the highest

| Item | Detail |
|---|---|
| Requirement | After 32 settling samples, detect humps in FOLLOW and NAVIGATE; keep the highest valid height for the run; START clears it; reported maximum within ±2 mm of the tallest hump; no hump on a flat lap |
| Owner / design | S4; S4 d8, d10, d11, d12, d13, d16; S1 d7 |
| Source | `subsystem4/terrain_logic.c` `s4_terrain_process()`, `s4_hump_update()`, `s4_hump_complete()`; `subsystem4/task_imu.c`; `subsystem2/robot_state.c` `s2_telemetry_add_hump()` |
| Tests | S4-U05, U06, U12 to U14, U21, U23 to U33, U40, U41 (host); S4-H07, H08; SYS-04 |
| Criterion | Highest hump within ±2 mm; every hump detected; no hump on a flat lap |
| Result | S4-H08 (2026-11-13): 11, 10, 10, 9 mm for a 10 mm hump. Demo: 6, 6, 7 mm for a 5 mm hump (+1, +1, +2 mm). S4-U41: no hump can read below 6 mm |
| Status | **Partial**: height within ±2 mm on every recorded run; flat lap not run in SYS-04 |
| Limitations | Minimum reportable height 6 mm. Single-point calibration (10 mm) cannot correct an offset; small humps read high |

### FR6 — forward range and stop before contact

| Item | Detail |
|---|---|
| Requirement | Range in FOLLOW, BYPASS and REACQUIRE, ≥ 65 ms between triggers; a valid range below 250 mm stops the car; a missing echo never counts as clear |
| Owner / design | S5; S5 d3, d12, d13; S2 d9, d10, d15 |
| Source | `subsystem5/src/drv_ultrasonic.c`; `subsystem5/src/range_measurement.c` `range_measurement_from_echo()`; `subsystem5/src/forward_monitor.c` `forward_monitor_check()`; `subsystem5/src/scan_task.c` |
| Tests | S5-U03 to U08, U19 to U21, U24, U25, U55 (host); S5-U48; SYS-05 |
| Criterion | Stop in SCAN with no contact; forward range within 20 mm of the ruler |
| Result | SYS-05 **Pass** 3/3 on 2026-11-20 (230 to 240 mm, errors +2 to +20 mm). Demo: stopped and scanned in all three attempts. 2026-11-17: 200 and 500 mm within 20 mm at 3V3 |
| Status | **Pass** |
| Limitations | No fault-injected echo on the car (Tribunal 2 Entry 7) |

### FR7 — profile the obstacle

| Item | Detail |
|---|---|
| Requirement | After the wheels are still, one result (CLEAR, PLAN, NO_ROUTE, FAULT, CANCELLED); width within ±20 mm or ±5% and centre offset only when both edges are seen; nearest point within ±20 mm |
| Owner / design | S5; S5 d4, d5; S2 d11, d17, d18, d22 |
| Source | `subsystem5/src/scan_engine.c` `scan_engine_process()`; `subsystem5/src/scan_profile.c` `scan_profile_make()`; `subsystem5/src/bypass_plan.c` `bypass_plan_make_fine()` |
| Tests | S5-U15, U23, U26, U59 to U63, U67, U72 (host); S5-U47 to U49; SYS-06 |
| Criterion | As the requirement |
| Result | Nearest point within 20 mm on 3/3 (SYS-05 runs). Width: shape never valid on 2026-11-20; demo practice widths 219 to 327 mm and assessed raw widths 166 to 332 mm for a 150 or 200 mm box |
| Status | **Fail** (width, centre offset); nearest point and side **Pass** |
| Limitations | The effective half-beam is about 21° (S2 d18), so the box is "seen" past its edges and the width is overstated; with the mission's 10° allowance a 150 mm box reads 246 mm in synthetic geometry (S5-U72). Even ideal rays are limited to about ±23 mm per edge by the 5° spacing at 240 mm. Planning does not use the width (proposal P6) |

### FR8 — bypass without contact and rejoin the original line

| Item | Detail |
|---|---|
| Requirement | Drive round a supported box (up to 150 mm across, 300 mm deep), find the original line and follow it; aim for 50 mm body clearance; latch STOP if there is no safe route or the bypass fails |
| Owner / design | S5, S2; S2 d14, d16, d17, d19 to d22; S4 d16 |
| Source | `subsystem5/src/bypass_plan.c`; `subsystem2/bypass_control.c` `bypass_control_step()`; `subsystem2/scan_control.c`; `subsystem2/task_controller.c` `scan_update()` |
| Tests | S2-U60 to U70 (host); S5-U36, U37, U53, U64 to U66 (host); SYS-07 |
| Criterion | 3 of 3: no contact; SCAN, BYPASS, REACQUIRE, then FOLLOW on the original line |
| Result | SYS-07 **Pass** 3/3 on 2026-11-20. Demo: attempt 3 passed (three bypasses, no contact); attempt 1 ended in STOP (bypass), attempt 2 in a scan fault |
| Status | **Pass** on the formal test; inconsistent on the demo |
| Limitations | A 230 mm-wide box gave NO_ROUTE first time on every approach (outside the supported size) |

### FR9 — local and remote STOP

| Item | Detail |
|---|---|
| Requirement | GP20 or MQTT STOP latches STOP; MQTT STOP uses an urgent event flag; local stops do not depend on WiFi; a stale command or reconnection never restarts motion |
| Owner / design | S2, S1; S1 d2, d3 |
| Source | `subsystem1/service_bridge.c`; `subsystem2/controller_logic.c` `ctrl_flags()`, `ctrl_remote()`; `subsystem2/task_controller.c` `button_step()` |
| Tests | S2-U45, U46, U50 to U52 (host); S1-U07; INT-04; S1-H01, H03; S2-H11 |
| Criterion | STOP latched by each source; motors off within 20 ms of the latch |
| Result | INT-04 **Pass**; S1-H03 **Pass** (WiFi loss does not move the car); S2-H11 **Pass** (7 runs, worst 952 µs) |
| Status | **Pass** |
| Limitations | S2-H11 closed on 7 runs instead of 10 (S2 d32) |

### FR10 — telemetry with validity flags

| Item | Detail |
|---|---|
| Requirement | Publish run state, motion, line, barcode, highest hump, obstacle data, fault and STOP cause at the NFR2 rate; each measurement has a validity flag and the dashboard never shows an invalid field as a reading |
| Owner / design | S1; S1 d1, d4, d5, d7 |
| Source | `subsystem1/telemetry_encoder.c` `telemetry_encoder_encode()`; `subsystem2/robot_state.c` (snapshot); `subsystem1/service_bridge.c` `service_bridge_telemetry()`; `testing/subsystem1/dashboard/` |
| Tests | S1-U05 (host); S2-U37, U53, U71, U72; INT-06, INT-07; SYS-08; dashboard unit tests (20) |
| Criterion | Version 4 frames about 1 s apart; ≥ 95% of periods; fields match the run |
| Result | Demo assessed session, from the last restart (14:32:25 to 15:03:50): 1863 frames received of 1890 periods (98.6%); the car counted 1867 sent and 23 refused |
| Status | **Pass** |
| Limitations | Only one fault code is sent; the motion fault detail is console-only |

### FR11 — latched STOP and recovery

| Item | Detail |
|---|---|
| Requirement | Line loss, motor or encoder fault, motion timeout, scan fault or no route latch STOP; no self-restart; RESET with wheels still; FAULT → INITIALISING → READY only when Motion Control is healthy; a new START is needed |
| Owner / design | S2; S2 d27 |
| Source | `subsystem2/controller_logic.c`; `subsystem2/motion_logic.c` `motion_step()`, `motion_fault()` |
| Tests | S2-U25 to U32, U41, U42, U47 to U49, U78 (host); INT-05 |
| Criterion | Each fault latches STOP; recovery needs RESET then START |
| Result | Host tests pass. INT-05 **Pass**. Demo: four motion faults and one scan fault each latched STOP; each cleared only after a RESET from the dashboard or a GP20 press (dashboard logs) |
| Status | **Partial**: the fault paths were observed on the car, but line loss is not fully detected (F-17); fault injection on the car not done |
| Limitations | The fault detail is not in telemetry. A car that runs fully off the line reads as centred, so "persistent line loss" is caught only as a stale report or an over-long crossing or exit ([review](software_quality_review.md) F-17) |

### FR12 — stop when lifted

| Item | Detail |
|---|---|
| Requirement | During a run, detect lift or overturn; signal the controller, which latches STOP; motors off (NFR1) |
| Owner / design | S4, S2; S4 d15, d16 |
| Source | `subsystem4/terrain_logic.c` (lift rules); `subsystem4/task_imu.c`; `subsystem2/controller_logic.c` (`CTRL_FLG_PICKUP`) |
| Tests | S4-U35 to U39 (host); S4-H11; SYS-10 |
| Criterion | 5 of 5 lifts give `STOP latched: pick-up`; none during laps or over humps |
| Result | SYS-10 **Pass** 5/5 on 2026-11-20 (`29bceff`); single-core check 1 lift on 2026-11-23; demo: 15 pick-up stops in the two sessions |
| Status | **Pass** |
| Limitations | Lift detection is off in BYPASS and REACQUIRE (S4 d16). Overturn is not tested separately |

### NFR1 — Motion Control timing and PWM ownership

| Item | Detail |
|---|---|
| Requirement | Motion Control every 10 ms and the only task that writes motor PWM; urgent STOP through event flags; motors off within 20 ms of the STOP latch |
| Source | `subsystem2/task_motion.c` `motion_task()`; `subsystem2/drv_motor.c`; `subsystem2/motion_logic.c` |
| Tests | S2-H11; code review: in the mission image only `task_motion.c` calls `drv_motor_set()` (the other callers are hardware tests in the `TEST=1` image) |
| Result | Worst 952 µs over 7 runs (about 5% of the limit) |
| Status | **Pass** |

### NFR2 — offline operation, frame and heartbeat rate

| Item | Detail |
|---|---|
| Requirement | Full course without WiFi; a frame every 1 s and a heartbeat every 5 s while connected; ≥ 95% of frames received |
| Source | `subsystem1/comms_timer.c`; `subsystem1/service_bridge.c`; kernel WiFi task |
| Tests | S1-U06, S1-H02, S1-H03; SYS-01 (driving continued without WiFi); SYS-08 |
| Result | 98.6% of frames on the demo; heartbeat count matched 5 s spacing; SYS-01 lap continued after the link was lost |
| Status | **Pass** |

### NFR3 — minimum bit widths, compact binary, fault bitmask

| Item | Detail |
|---|---|
| Requirement | Minimum bits for every field; packed where appropriate; compact binary MQTT; validity flags; independent faults as one bit each in a shared bitmask; race-free shared data |
| Source | `subsystem1/telemetry_encoder.c`; interface records in `subsystem2/subsystem2.h`; [data_widths.md](data_widths.md) |
| Tests | S1-U05; S2-U39 (record sizes); analysis in [data_widths.md](data_widths.md) |
| Result | 31-byte binary frame with validity flags and sized fields. The independent fault bitmask is **not implemented**: one 4-bit fault code is sent (Tribunal 2 Entries 11, 19) |
| Status | **Partial** |

### NFR4 — motion accuracy

| Item | Detail |
|---|---|
| Requirement | 500 mm straight within ±20 mm; 90° within ±5°; 180° within ±10° |
| Source | `subsystem2/odometry.c`; `subsystem2/motion_logic.c`; `subsystem2/motion_config.h` (S2 d31) |
| Tests | S2-H07, H08, H10 |
| Result | Straight: 5 runs, worst +10 mm. Left 90: 2 runs; right 90: 1 run; U-turn: 1 run; all within limits, judged by eye (S2 d32) |
| Status | **Pass** on a limited sample |

### NFR5 — cold boot to READY

| Item | Detail |
|---|---|
| Requirement | READY with no fault on 5 of 5 cold power-bank boots; still until START |
| Source | `subsystem2/controller_logic.c` (INITIALISING with STOP latched); S5 d12 (0.5 s ultrasonic allowance) |
| Tests | SYS-11 |
| Result | 2026-11-20: first set 4/5 (one boot never connected, cause unknown); repeat 5/5 |
| Status | **Pass** (repeat); one unexplained failed boot stays open |

### NFR6 — platform, language and BARR-C

| Item | Detail |
|---|---|
| Requirement | Pico W with Robo Pico, C with the Pico SDK and µT-Kernel 3.0, BARR-C with every deviation recorded |
| Tests | Code review; [barr_c_compliance.md](barr_c_compliance.md); `testing/host/target_static_check.py` |
| Result | Builds with `arm-none-eabi-gcc` 15.3.1 for the RP2040. Zero warnings in application code under 18 strict flags. Deviations recorded in S2 d30 and the BARR-C report |
| Status | **Partial**: every cast now has a range comment and the naming rules are met (2026-11-25, mission UF2 unchanged); open items are 13 long functions (6.2a, an agreed deviation), short names in test code and four structure members (7.1e) and four rules met only in part (1.8b, 2.2, 6.3, 6.4a) |

## 5. Buddy tasks from the brief

The brief lists tasks per buddy. Most are covered by the FRs above; this
table shows the rest.

| Buddy | Brief task | Implemented | Where / evidence | Status |
|---|---|---|---|---|
| 1 | Connection recovery | Yes: WiFi rejoin every 5 s, MQTT retry every 3 s | kernel WiFi task, `mqtt_utk.c`; SYS-11, S1-H03 | Pass |
| 1 | Command subscription | Yes: START, STOP, RESET, letter table | `command_parser.c`; S1-U04; INT-04, INT-05 | Pass |
| 2 | Motion API (forward, backward, turn, stop) | Yes | `motion_api.c`; S2-U35 | Host only (unit), used on the car by bypass and turns |
| 2 | PID speed control and anti-windup | Yes | `pid.c`; S2-U03 to U09 (target), S2-H09 | Pass at 32 to 40 mm/s |
| 2 | PID tuning report, motion accuracy evaluation | Yes | S2-H09, S2-H10 rows in the [S2 test log](../testing/subsystem2/TEST_LOG.md) | Pass (limited sample) |
| 3 | Junction detection | Yes: both sensors black for 100 ms | `line_control.c`; S2-U58 | Partial (see FR4 limits) |
| 3 | Sensor calibration | Adaptive white baseline, fixed margins | `s3_ir.c`, `s3_barcode.c` | Barcode levels not re-measured after the sensor was lowered |
| 4 | Tilt, hump, peak, collision (impact), pick-up | Yes | `terrain_logic.c`; S4 host cases | See FR5, FR12 |
| 4 | Motion classification (stationary, accelerating, turning, climbing, descending) | Partly: stationary calibration, pitch ignored while accelerating, hump rise and level | `terrain_logic.c` | No separate classification output |
| 4 | Turn-rate monitoring | **No**. The LSM303DLHC has an accelerometer and a magnetometer but no gyroscope. The magnetometer is configured and its identity checked (S4-H01) but not used for heading; its interference test S4-H10 was not run. Turn angles come from the wheel encoders | — | Not implemented |
| 5 | Coarse scan, fine scan, profile, planning, line recovery | Yes: coarse 15° steps then 5° fine readings round each edge | `scan_engine.c`, `scan_profile.c`, `bypass_plan.c`; S5-U26, U59 | See FR7, FR8 |
| 5 | Reverse and reattempt | Reverse before the first turn when too close | `bypass_plan.c`; S5-U64 | Host only |

## 6. Test coverage

### By level

| Level | Count and where | Latest result |
|---|---|---|
| Unit (host) | S1-U04 to U07; S2 52 cases; S3 784 checks; S4 37 cases; S5 48 cases + telemetry view | All pass, with one known defect reported (F-18) (2026-11-25, [log](../testing/host/logs/2026-11-25_host-tests-final2.txt)) |
| Unit (target) | The same plus the target-only cases (for example S2-U01 to U19, S2-U37, U39, U71, U72) in the `TEST=1` image | Builds (2026-11-25); last on-target run 2026-11-17 (incomplete) |
| Hardware | S1-H01 to H03, S2-H01 to H11, S3-H01 to H06, S4-H01 to H11, S5-U46 to U50 | Mixed; see each log |
| Integration | INT-04 to INT-07, letters over MQTT | Pass |
| System | SYS-01 to SYS-11 | See section 4 |
| Acceptance | The assessed demonstration | [demo report](DEMO_REPORT_2026-11-24.md) |

### By technique

| Technique | Examples |
|---|---|
| White-box | Branch coverage of the host-built application code: 78.2% (89.7% of lines) |
| Black-box | SYS-01 to SYS-11; INT tests |
| Boundary | S5-U04, U05 (range limits); S4-U36 (lift at 299 and 300 mg); S4-U41 (6 mm floor); S4-U42 (1 mg steps, no jump over 1 mm); S2-U79, U80 (phase and hold limits ±1 ms) |
| Negative | S1-U04 (malformed commands); S3 invalid patterns; S5-U06, U07; S2-U33 |
| Fault injection (host) | S4-U08, U11 (invalid and stale IMU); S5-U21, U24 (bad echo); S2-U29 to U31 (stall, timeout, encoder); S3-U03 (noise blip, broken frames) |
| State transition | S2-U41 to U52 (controller statechart); S2-U20 to U32 (motion statechart) |
| Regression | `make -C testing/host` before each commit (a crashed test now fails the run, F-19); mission UF2 hash compared after every test-, comment- or build-only change |

### Coverage figures (host, 2026-11-25, [log](../testing/host/logs/2026-11-25_coverage-2.txt))

27 application files are built on the host. Hardware drivers, RTOS task
files and the WiFi stack are not, so they are outside these figures.

| Scope | Lines | Branches |
|---|---|---|
| All 27 files | 4730 / 5272 = **89.7%** | 2011 / 2572 = **78.2%** |
| Without the IMU I2C driver (hardware path) | 93.2% | 80.5% |
| Lowest: `pid.c`, `odometry.c`, `motion_api.c` | 62% to 77% | 52% to 53% |

The low files have their unit tests (S2-U03 to U14) in the on-target
`TEST=1` image only, so the host figures understate their testing.

## 7. Test conditions

Physical conditions recorded for the main system and hardware tests.
"Not recorded" means the value was not written down at the time; we do not
fill it in afterwards.

| Condition | Value | Source |
|---|---|---|
| Line | 18 mm black tape; front sensors 30 mm apart, centre to centre | S3 decisions |
| Line speed | 40 mm/s (`CTRL_LINE_BASE_MM_S`); curves 32 to 38 mm/s | `task_controller.c` |
| Bypass speed | About 100 to 130 mm/s legs, 30 mm/s spins | `task_controller.c`; SYS-07 notes |
| Barcode prints | Large and three-quarter prints; small A print on demo day; narrow runs 120 to 220 ms and wide 420 to 540 ms at 40 mm/s (large print, 2026-11-11) | `s3_barcode.c` comments |
| Barcode sensor height | Lowered on demo day for the small print; value **not recorded** | Team observation |
| Barcode mounting | Demo repeat print wrinkled and not taped (team); other runs **not recorded** | Demo report |
| Hump | 10 mm (S4-H08); 5 mm on the demo course (team ruler) | S4 log; demo report |
| Obstacle | 15 × 13 × 10 cm (2026-11-20 first SYS-07); 15 × 23 cm, 230 to 240 mm from the sensor (SYS-05/07); 23 cm wide (no-route); 15 × 20 cm on the demo | System test log |
| Lighting | "Course lighting as on the day"; level **not recorded** | System test log |
| Supply | Power bank; USB console when it could stay plugged in | System test log |
| IMU | Car kept still about 10 s after power-on (calibration) | SYS-10 notes |
| Firmware | Commit and build line in every result row | All test logs |
| Network | Pixel hotspot; broker and dashboard on the laptop | System test log |

## 8. Gaps and outstanding validation

| Gap | Effect | Next step |
|---|---|---|
| SYS-09 never passed | No proof of a full autonomous course | Full-course runs with the console captured |
| SYS-02 below 4 of 5 | Barcode reliability not shown | 5 passes per print with sensor height and print condition recorded; 10 ms sampling first ([P1](post_demo_proposals.md)) |
| A 20 ms noise blip inside a frame can give a wrong letter (F-18, S3-U03 known defect) | "Never a wrong letter" is shown only on the recorded captures, not for all inputs | Stray-mark length check, prototyped on the host ([P2](post_demo_proposals.md)) |
| FR2 sampling at 20 ms, not 10 ms (F-01) | FR2 not met as written | [P1](post_demo_proposals.md), then re-measure bar durations |
| A car fully off the line reads as centred (F-17) | Line loss is not detected, so FR11's line-loss STOP cannot trigger | Sensor layout change ([P4](post_demo_proposals.md)) |
| SYS-03 12-run set never run | Turn reliability not shown | 3 runs per command, console captured |
| SYS-04 flat lap not run | False humps on a flat lap not excluded | One flat lap |
| SYS-06 width fails | MR7 shape not met | Measured width correction or a larger beam allowance ([P6](post_demo_proposals.md)); geometry checked by S5-U72 |
| NFR3 fault bitmask not implemented | Simultaneous faults not visible | [P8](post_demo_proposals.md); Tribunal 2 Entries 11 and 19 |
| Watchdog off (`S2_WATCHDOG` 0) | A stalled task is not reset | Per-task check-in, then arm it after a car run ([P7](post_demo_proposals.md), Tribunal 2 Entry 4) |
| On-target `TEST=1` run last completed 2026-11-17 | Target-only unit tests not re-run on the v4 code | Run the `TEST=1` image |

## 9. Assessment considerations from the brief

The brief (page 11) lists what teams are assessed on. Our honest position
against each, with where the evidence is:

| Consideration | What we can show | Status | Main gap |
|---|---|---|---|
| Successful completion of mission objectives | Each mission element worked on the car at least once (sections 2 and 4) | Partial | No full-course run (SYS-09); no turn in the assessed runs |
| Navigation accuracy | Straight 500 mm worst +10 mm; 90° turns inside ±5°; U-turn inside ±10° (S2-H10, small samples) | Partial | Few runs per motion; angles judged by eye |
| Obstacle avoidance effectiveness | SYS-07 3/3; demo attempt 3 with no contact | Pass on the formal test | Attempts 1 and 2 of the demo did not complete the bypass |
| Barcode recognition reliability | No wrong letter seen on the car; 4/4 and 2/2 drives on 17 Nov; 1 of 3 attempts on demo day | Partial | Below 4 of 5; 20 ms sampling (F-01); a host-found wrong letter from a noise blip (F-18, fix P2 not applied) |
| Hump measurement capability | Every hump detected; within ±2 mm on every recorded run | Partial | 6 mm minimum reading; no flat lap |
| System robustness | Latched STOP, fault paths and recovery seen on the car; host fault injection; [fault table](tribunal_closure.md) | Partial | Line loss and task stalls not detected (F-17, F-08) |
| Software design quality | Pure logic modules tested on the host; single PWM writer; event flags for STOP; 0 warnings under strict flags; a range comment on every cast; [review](software_quality_review.md), [BARR-C](barr_c_compliance.md) | Strong | 13 functions over 100 lines (task loops and state machines) |
| Resource efficiency | No floating point; 2 KB application RAM; 31-byte frame; [data widths](data_widths.md) | Strong | Power not measured |
| Team integration quality | 14 active interfaces (IF-06 to IF-21) in `TEAM.md` section 5; INT-04 to INT-07 pass; one mission image | Strong | — |
| Demonstration performance | [Demo report](DEMO_REPORT_2026-11-24.md) | Partial | See the report |

Completion time: SYS-01 laps took 2 min 8 s to 2 min 15 s at 40 mm/s; no
full-course time exists.

## 10. Requirement and design quality (Lecture 5)

Lecture 5 asks that each requirement is a "shall" statement with a
tolerance, can be verified by a test, is traceable, and says *what* the
system must do rather than *how*. It also asks for IDs on design boxes and
arrows, and for sequence diagrams and statecharts. This section checks the
final requirements against those rules. The requirement wording and the
acceptance criteria in section 4 are left unchanged; the suggested wording
below is for a future revision only.

### Requirement review

| ID | Verifiable with a tolerance? | Prescribes design? | Issue | Suggested wording |
|---|---|---|---|---|
| FR1 | Yes (one START, no touch, SYS-09) | No | — | — |
| FR2 | Partly: "follow" has no measured limit | Yes: sensor count, adaptive thresholds and "every 10 ms" are design choices | Our only open FR2 item (10 ms) is a design detail, not a mission need | "The car shall stay on the 18 mm line through curves, crossings and humps for a full lap at 40 mm/s, with a line report at least every 30 ms." Move 10 ms sampling to the design (P1) |
| FR3 | Yes (4 of 5, no wrong letter) | No | "Never a wrong letter" cannot be proved by testing alone; F-18 shows an input that breaks it | Keep, and add "on the five course prints" to bound the test |
| FR4 | Partly: no limit on how late the turn may start | Yes: "encoder-based" | Turn accuracy sits in NFR4 | "...shall start the held turn within 100 mm of the junction" (value to be agreed) |
| FR5 | Yes (±2 mm, flat lap) | No | — | — |
| FR6 | Yes (250 mm, missing echo unsafe) | Yes: "≥ 65 ms between triggers" comes from the HC-SR04 datasheet (≥ 60 ms cycle) | A design constraint inside a requirement | Move the trigger gap to the design (section 6 of the architecture) |
| FR7 | Yes (±20 mm or ±5%) | No | — | — |
| FR8 | Partly: "aim for 50 mm" cannot pass or fail | No | — | "...with no contact and at least 30 mm clearance" (value to be agreed) |
| FR9 | No time limit in FR9 itself | Yes: "urgent event flag" | The limit is in NFR1 (20 ms) | "A local or remote STOP shall turn the motors off within 20 ms of being received" |
| FR10 | Yes (rate from NFR2; invalid never shown as a reading) | No | — | — |
| FR11 | Yes (each latch and recovery step is observable) | No | — | — |
| FR12 | No time limit | No | A lift detected after 10 s would still pass | "...shall latch STOP within 200 ms of a lift" (value to be agreed) |
| NFR1 | Yes (20 ms) | Yes: 10 ms period, event flags | Acceptable as a stated design constraint | — |
| NFR2 | Yes (1 s, 5 s, ≥ 95%) | No | — | — |
| NFR3 | Partly: "where appropriate" cannot be checked | Yes: bitmask, packing | Judged by analysis ([data_widths.md](data_widths.md)), not by test | "Each field shall use no more bits than its valid range and resolution need" |
| NFR4 | Yes (±20 mm, ±5°, ±10°) | No | — | — |
| NFR5 | Partly: no time limit to READY | No | — | "...shall reach READY within 5 s of power-on" (value to be agreed) |
| NFR6 | Yes (inspection) | Yes, by intent: a platform constraint from the brief | — | — |

The values marked "to be agreed" are proposals, not measured results.

### Requirement to design element

The boxes are named as in the [software structure](final_architecture.md#2-software-structure)
diagram; the arrows carry the interface IDs from `TEAM.md` section 5.
`motion_q` and `safety_flags` are internal to subsystem 2 and have no IF
number.

| Requirement | Design boxes | Interfaces (arrows) | Diagram |
|---|---|---|---|
| FR1, FR11 | BTN, WIFI, CTRL, MC | IF-08, IF-09, `motion_q` | Controller statechart (architecture section 4) |
| FR2 | IR, S3, CTRL, MC | IF-06, `motion_q` | Mode statechart (section 4) |
| FR3, FR4 | BC, S3, CTRL, MC | IF-21, `motion_q` | Barcode-to-turn sequence (section 5) |
| FR5 | IMU, S4, CTRL, WIFI | IF-14, IF-17, IF-18, IF-10 | Hump-to-telemetry sequence (section 5) |
| FR6, FR7, FR8 | US, SERVO, S5, CTRL, MC | IF-11, IF-12, IF-13, IF-20 | Obstacle-to-bypass sequence (section 5) |
| FR9 | BTN, WIFI, CTRL, MC | IF-08, `safety_flags` | Controller statechart |
| FR10, NFR2 | CTRL, WIFI, DASH | IF-10, IF-18, IF-20 | Hump-to-telemetry sequence |
| FR12 | IMU, S4, CTRL | IF-19 | Controller statechart |
| NFR1 | MC, CTRL | `safety_flags`, `motion_q` | RTOS task table (section 3) |
| NFR3 | WIFI | IF-10 | [data_widths.md](data_widths.md) |

