# Subsystem 2 test log

**Owner:** Dalton (2504003)  ·  **Planned subsystem:** Vehicle Controller and motion control  ·  **Code:** `app_program/subsystem2/`

S2-U01 and S2-U02 test the starter producer, kept until subsystem 3 stops
using it. S2-U03 onwards test the Vehicle Controller and Motion Control:
S2-U* run automatically in a `TEST=1` build, S2-H* are bench checks, and
S2-I* are integration demos with teammates. A clean build is not a passed
test: record a row in Results for every run on the Pico W.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | PID proportional term is kp * error | PID speed control | auto | `subsystem2_test.c`, synthetic gains | `[TEST] S2-U03 PASS` |
| S2-U04 | PID output stays within its limits | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | PID integral adds ki * error per sample | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | PID integral does not wind up while saturated | PID speed control | auto | `subsystem2_test.c`, 50 saturated samples | `[TEST] S2-U06 PASS` |
| S2-U07 | PID derivative ignores setpoint steps | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | PID rejects invalid tuning and inputs | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | PID reset clears history | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | counts convert to millimetres | Odometry | auto | `subsystem2_test.c`, synthetic geometry | `[TEST] S2-U10 PASS` |
| S2-U11 | count delta survives counter wraparound | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U11 PASS` |
| S2-U12 | speed from counts over an interval | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U12 PASS` |
| S2-U13 | turn angle sign and magnitude | Odometry, turns | auto | `subsystem2_test.c` | `[TEST] S2-U13 PASS` |
| S2-U14 | odometry rejects invalid geometry and overflow | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U14 PASS` |
| S2-U15 | motor duty maps to A/B compare values: +d gives A=d, B=0; -d gives A=0, B=d; 0 gives brake (0, 0) | Robo Pico truth table | auto | `subsystem2_test.c`, `drv_motor_map()` | `[TEST] S2-U15 PASS` |
| S2-U16 | motor duty beyond +/-DRV_MOTOR_DUTY_MAX, and a NULL output, are rejected with the output unchanged | drv_motor | auto | `subsystem2_test.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | GPIO IRQ registration rejects pin >= 30, radio pins 23-25 and 29, no edge, level events and a NULL callback | GPIO dispatcher | auto | `subsystem2_test.c`; invalid calls only, no pin is enabled | `[TEST] S2-U17 PASS` |
| S2-U18 | quadrature step: individual A edges decode to +1/-1; a latched rise+fall pair uses current A/B levels to decode +2/-2; no edge gives 0 | drv_encoder (x2 decoding) | auto | `subsystem2_test.c`, `drv_encoder_step()` | `[TEST] S2-U18 PASS` |
| S2-U19 | encoder snapshot rejects NULL and returns E_OK with counts that do not change while the encoders are idle | drv_encoder | auto | `subsystem2_test.c`; encoders not initialised | `[TEST] S2-U19 PASS` |
| S2-U20 | Motion Control start: hardware OK gives Idle + READY; failed init gives Fault(INIT) + FAULT bit, zero duty | Motion statechart: Init | auto | `test_motion.c`, synthetic wheel model | `[TEST] S2-U20 PASS` |
| S2-U21 | Speed state drives both wheels to the targets; a zero target brakes that wheel at once and keeps it braked (duty 0, no reverse drive) while the other wheel remains driven; the braked wheel drives again within 50 ms of a new non-zero target | Motion statechart: Speed | auto | `test_motion.c` | `[TEST] S2-U21 PASS` |
| S2-U22 | MOVE stops at the target distance (295 to 330 mm for 300 mm) and reports DONE only after 100 ms of stillness | Motion statechart: Move, Stopping | auto | `test_motion.c` | `[TEST] S2-U22 PASS` |
| S2-U23 | TURN with a positive angle drives left back and right forward and ends within 5 degrees of 90 with heading positive (left) | Turn sign (decisions S5 section 4) | auto | `test_motion.c` | `[TEST] S2-U23 PASS` |
| S2-U24 | straight correction: the left wheel slips for the first 200 ms (about 40 mm lost) and is 20% weaker after; a 500 mm MOVE still ends with the wheel distances within 5 mm and heading within 2 degrees. The same case with the straight PID off must fail (checked when written) | Straight-line PID | auto | `test_motion.c`, slipping then weakened wheel | `[TEST] S2-U24 PASS` |
| S2-U25 | latched STOP in every state: Init stays Init; Idle confirms; Speed, Move and Turn go to zero duty at once and report STOPPED; Stopping still finishes; Fault stays Fault | Latched STOP precedence | auto | `test_motion.c` | `[TEST] S2-U25 PASS` |
| S2-U26 | MOVE, TURN and SPEED while STOP is latched are rejected (REJ_STOP) and the duty stays zero | Latched STOP | auto | `test_motion.c` | `[TEST] S2-U26 PASS` |
| S2-U27 | a new command replaces the current one while STOP is clear; an ID not newer than the last is rejected (REJ_STALE), also after STOP clears | Command replacement, stale commands | auto | `test_motion.c` | `[TEST] S2-U27 PASS` |
| S2-U28 | stop confirmation: STOPPED and motion_is_stopped() no sooner than 100 ms of stillness; wheels that keep turning give Fault(STOP_TIMEOUT) after 2 s, not before | Stop confirmation (decisions S5 section 6) | auto | `test_motion.c`, forced wheel speed | `[TEST] S2-U28 PASS` |
| S2-U29 | a stalled wheel (full duty, no counts) gives Fault(STALL) after the stall time and not before, with zero duty | Stall fault | auto | `test_motion.c`, stuck wheel | `[TEST] S2-U29 PASS` |
| S2-U30 | a MOVE that progresses too slowly gives Fault(TIMEOUT) | Motion timeout | auto | `test_motion.c`, capped wheel speed | `[TEST] S2-U30 PASS` |
| S2-U31 | repeated invalid snapshots or a burst of decode errors give Fault(ENCODER) | Encoder fault | auto | `test_motion.c` | `[TEST] S2-U31 PASS` |
| S2-U32 | in Fault, motion commands are rejected; RESET is rejected while a wheel turns and accepted once both are still, returning to Idle | Fault only cleared by RESET with wheels stopped | auto | `test_motion.c` | `[TEST] S2-U32 PASS` |
| S2-U33 | invalid parameters are rejected (REJ_PARAM): over-limit speed, zero distance or speed, zero or over 360 degree angle | Command validation | auto | `test_motion.c` | `[TEST] S2-U33 PASS` |
| S2-U34 | odometry: a straight 500 mm MOVE gives travel and x within 25 mm with y and heading near 0; NEW_RUN zeroes the pose and increments run_id | Odometry for S4/S5 | auto | `test_motion.c` | `[TEST] S2-U34 PASS` |
| S2-U35 | motion API commands: forward/backward/left/right build MOVE/TURN with the right sign and speed; STOP/RESET/NEW_RUN have no payload; zero and over-range values are refused (E_PAR) without sending | motion_api | auto | `test_motion.c`, `motion_api_make()` | `[TEST] S2-U35 PASS` |
| S2-U36 | remote_q record: START and RESET build a 12-byte record with request ID, receipt time and zeroed reserved bytes; STOP, INVALID (0), unknown codes and request ID 0 are refused with the output unchanged | IF S1 to S2 (remote_q) | auto | `subsystem2_test.c`, `s2_remote_make()` | `[TEST] S2-U36 PASS` |
| S2-U37 | telemetry snapshot from robot_state: distance copied when valid, clamped to 65535 above range, 0 and invalid when negative or unknown; stopped follows the wheels-still bit; time copied | IF S2 to S1 (telemetry) | auto | `subsystem2_test.c`, `s2_telemetry_from_state()` | `[TEST] S2-U37 PASS` |
| S2-U38 | distance delta for S4: forward 12 mm gives 12 valid; 0 gives 0 valid; reversing, over 65535 mm, a run_id change or an invalid sample give invalid | IF S2 to S4 (distance feed) | auto | `subsystem2_test.c`, `s2_distance_delta()` | `[TEST] S2-U38 PASS` |
| S2-U39 | interface record sizes: remote 12, scan 12, navigation 12, robot state 20 bytes (used to size the queues) | Provisional interfaces | auto | `subsystem2_test.c`, `sizeof` | `[TEST] S2-U39 PASS` |
| S2-U40 | controller_flags bits are distinct single bits inside CTRL_FLG_ALL; remote codes equal S1's COMMAND_PARSER values; result codes equal S1 PROPOSAL ack codes | Provisional interfaces | auto | `subsystem2_test.c` | `[TEST] S2-U40 PASS` |
| S2-U41 | boot: Initialising with STOP latched; Ready once Motion Control is Idle, valid and fault-free; STOP still latched in Ready | Controller: Initialising, Ready | auto | `test_controller.c`, injected motion status and clock | `[TEST] S2-U41 PASS` |
| S2-U42 | Initialising that never sees healthy Motion Control goes to Fault(INIT) after the init limit, not before | Controller: Initialising | auto | `test_controller.c` | `[TEST] S2-U42 PASS` |
| S2-U43 | START in Ready with healthy, stopped motion: drains motion_q, clears STOP, starts a new run, Running, result COMPLETED. START is REJECTED in Initialising, Running, Stopped, Fault and while motion is unhealthy | Controller: START rule | auto | `test_controller.c` | `[TEST] S2-U43 PASS` |
| S2-U44 | demo script: each step is sent only after the previous one reports DONE; after the last step the controller latches STOP and goes to Stopped | Controller: Running demo | auto | `test_controller.c`, 3-step script | `[TEST] S2-U44 PASS` |
| S2-U45 | in Running, REMOTE_STOP, LOCAL_STOP and HAZARD each latch STOP, wake Motion Control, go to Stopped, stop the script, and clear only that bit | Latched STOP, flag rules | auto | `test_controller.c` | `[TEST] S2-U45 PASS` |
| S2-U46 | STOP in Initialising, Ready, Stopped and Fault re-latches STOP and leaves the state unchanged | Latched STOP | auto | `test_controller.c` | `[TEST] S2-U46 PASS` |
| S2-U47 | fault paths: MOTION_FAULT bit (Running and Ready), SCAN_FAULT bit, a REJECTED or FAULT result for the active step, a failed send, and no result within the limit each give Fault with the right code and STOP latched | Controller: Fault | auto | `test_controller.c` | `[TEST] S2-U47 PASS` |
| S2-U48 | RESET in Stopped is REJECTED until the wheels are confirmed stopped, then goes to Ready, drains motion_q, and keeps STOP latched | Controller: Stopped recovery | auto | `test_controller.c` | `[TEST] S2-U48 PASS` |
| S2-U49 | RESET in Fault is REJECTED while the wheels turn; once stopped it goes to Initialising and sends a Motion Control RESET if motion is in Fault, then Ready when motion is healthy | Controller: Fault recovery | auto | `test_controller.c` | `[TEST] S2-U49 PASS` |
| S2-U50 | remote commands older than 2 s are EXPIRED, a repeated request ID is DUPLICATE, unknown commands are REJECTED; robot_state shows the last request ID, command and result | IF S1 to S2 rules | auto | `test_controller.c` | `[TEST] S2-U50 PASS` |
| S2-U51 | STOP while remote_q is full: (a) logic: a STOP bit and two queued STARTs in one cycle end Stopped with STOP latched and both STARTs REJECTED; (b) adapters on a real 2-record test queue: the third send returns false, the STOP adapter still sets the bit | Remote STOP bypasses remote_q | auto | `test_controller.c`, temporary queue and flag | `[TEST] S2-U51 PASS` |
| S2-U52 | nothing but START in Ready clears STOP: RESET, repeated ticks, no-flag cycles and a duplicate START never request a clear (WiFi reconnect sends nothing) | WiFi reconnect never clears STOP | auto | `test_controller.c` | `[TEST] S2-U52 PASS` |
| S2-U53 | robot_state: state, fault, STOP latched, wheels stopped, run distance since START and its validity | IF S2 to S1 (robot_state) | auto | `test_controller.c` | `[TEST] S2-U53 PASS` |
| S2-U54 | watchdog load value: 2 counts per microsecond (erratum RP2040-E1), so 1 ms gives 2000 and 200 ms gives 400000; 8388 ms (the 24-bit limit) is accepted; 0 ms and 8389 ms are refused (0) | drv_watchdog | auto | `subsystem2_test.c`, `drv_watchdog_load_value()` | `[TEST] S2-U54 PASS` |
| S2-U55 | duty ramp: an increase is limited to the step per period; a sign change outputs 0 first, then ramps from 0; a decrease toward 0 and a target of 0 (STOP, Fault) are immediate | Duty ramp | auto | `test_motion.c`, `motion_ramp()` | `[TEST] S2-U55 PASS` |
| S2-U56 | the ramp cannot cause a false stall: with the ramp slower than the stall time, a free wheel never stalls, and a stuck wheel's stall time starts only once the ramped duty reaches the stall level | Duty ramp, stall fault | auto | `test_motion.c`, synthetic wheel model | `[TEST] S2-U56 PASS` |
| S2-U57 | line reports accept only the agreed position values and preserve their capture time | IF S3 to S2 | auto | `subsystem2_test.c` | `[TEST] S2-U57 PASS` |
| S2-U58 | BLACK/BLACK is qualified before crossing, crossing and exit are bounded, the exit remains straight for its minimum time, and normal following resumes after stable WHITE/WHITE or a persistent single-black curve | Junction policy | auto | `test_line_control.c` | `[TEST] S2-U58 PASS` |
| S2-U59 | WHITE/WHITE commands straight, a single-black observation becomes a sustained curve at the configured time, asymmetric curve holds and timer wrap are handled | Two-sensor curve policy | auto | `test_line_control.c` | `[TEST] S2-U59 PASS` |
| S2-U60 | a hazard scan sends one motion STOP and one REQUEST with a nonzero ID; a second hazard while scanning sends nothing | Scan & Plan (IF-11) | auto | `test_scan_control.c` | `[TEST] S2-U60 PASS` |
| S2-U61 | STOP_CONFIRM and AUTHORISE are sent once, only after both wheels are still | Scan & Plan, stop before scan | auto | `test_scan_control.c` | `[TEST] S2-U61 PASS` |
| S2-U62 | a result for this request ends the scan (PLAN, CLEAR, otherwise FAILED); another request's result or one before stillness is ignored | Scan & Plan (IF-13) | auto | `test_scan_control.c` | `[TEST] S2-U62 PASS` |
| S2-U63 | no result within the limit sends CANCEL (TIMEOUT); an abort sends STOP; aborting when idle sends nothing | Scan & Plan | auto | `test_scan_control.c` | `[TEST] S2-U63 PASS` |
| S2-U64 | the bypass requests the plan's segments in order, skips empty ones, then the approach turn and the search; an out-of-range value, unknown kind or too many segments fails before any motion | Bypass (IF-13) | auto | `test_bypass_control.c` | `[TEST] S2-U64 PASS` |
| S2-U65 | only DONE for the bypass's own command ID advances it; REJECTED, FAULT or STOPPED for that ID, or a failed send, fails it; an abort ends it silently | Bypass, motion results | auto | `test_bypass_control.c` | `[TEST] S2-U65 PASS` |
| S2-U66 | a forward hazard is ignored during in-place turns and fails a forward MOVE or the search on the next cycle; a hazard when the align turn ends fails it instead of resuming | Bypass, hazard level | auto | `test_bypass_control.c` | `[TEST] S2-U66 PASS` |
| S2-U67 | the line must be seen for 100 ms; the car then brakes to a confirmed standstill (S2 decision 20), failing if not still within 3 s; the step, measured from where it stopped, brings the axle to where the sensors first saw it (forward or back); the car then spins towards the original direction and stops on the first sensor on black once it has turned the align angle less 30 degrees, failing past the align angle plus 125 degrees (170 degrees for the robot's 45 degree align); once still it ends DONE unless it is more than 90 degrees from the start heading (the line taken the wrong way; a line curving 60 degrees passes), wrapping at 180 (revised 2026-11-12, S2 decision 22); the search fails past the plan's distance or 5 s | Reacquisition (S5 decision 9) | auto | `test_bypass_control.c` | `[TEST] S2-U67 PASS` |
| S2-U68 | with a look angle in the plan, after the segments the bypass looks; an unrelated motion DONE or another look's answer is ignored; readings not beyond clear_mm each give a 50 mm step and a new look; a reading beyond clear_mm but within 100 mm of the nearest side reading steps again (revised 2026-11-12, S2 decision 22); the first reading beyond both gives the extra move, then the approach turn and the search | Passing by measurement (S2 decisions 19, 22) | auto | `test_bypass_control.c` | `[TEST] S2-U68 PASS` |
| S2-U69 | a faulted look, a look that could not be sent and a look unanswered after 3 s fail with FAIL_LOOK; a hazard while looking is ignored but fails a step; a step that would pass pass_max_mm fails with FAIL_DEPTH | Passing by measurement (S2 decision 19) | auto | `test_bypass_control.c` | `[TEST] S2-U69 PASS` |
| S2-U70 | turn to look: each look follows a turn of look_turn_mdeg; the turn back and the approach turn are to the start heading, so a 1.5 degree turn overshoot and 2 and 1 degree step drift are corrected (start heading 180 degrees, at the wrap); a miss before pass_min_mm of stepping steps on; an oversized look turn is an invalid plan | Passing by measurement (S2 decision 22) | auto | `test_bypass_control.c` | `[TEST] S2-U70 PASS` |
| S2-H01 | each motor drives the expected way | Robo Pico truth table, design review section 2 | manual | Wheels off the ground. Build with `S2_HW_TEST` set to 2 in `task_motion.h`. Per motor: 30% forward 2 s, brake 3 s, 30% backward 2 s, coast 3 s. Watch the wheel and the board's MxA/MxB LEDs | Forward: wheel turns forward, MxA LED on. Backward: reverse, MxB LED on |
| S2-H02 | brake and coast differ as the truth table says | Robo Pico truth table | manual | During S2-H01, turn each wheel by hand in the brake and coast steps | Brake: noticeably stiff. Coast: turns freely |
| S2-H03 | encoder count sign and counts per revolution | drv_encoder, calibration | manual | GP28 battery-sense jumper open. `S2_HW_TEST` set to 1 (motors stay off) prints both counts every 100 ms for 60 s. Mark each wheel, turn it exactly one revolution forward by hand, then one back. Repeat 3 times per wheel | Count changes by the same amount each way, opposite signs, error count unchanged. Record counts per rev and which encoders count down going forward |
| S2-H04 | full-speed run, split to find where the CPU froze on 2026-11-02 | drv_motor, drv_encoder, power, PostTri1 interrupt load | manual | Wheels off the ground. `S2_HW_TEST` set to 3 in `task_motion.h` runs only this (mode 2 runs it after S2-H01/H02). H04a: motor 1 alone 100% for 3 s; H04b: motor 2 alone 100% for 3 s; H04c: both 100% for 5 s. Each step brakes, then prints for 2 s; counts print every 100 ms throughout | Every step ends with a "done" line and the wheels brake; error counts stay 0. Record counts/s per wheel (the interrupt rate). If the log stops, the last printed line shows which step froze |
| S2-H05 | watchdog stops the motors when the CPU stops kicking | drv_watchdog, RP2040 watchdog and PSM | manual | Wheels off the ground. `S2_HW_TEST` set to 4 in `task_motion.h`. Motor 1 runs at 30% while the watchdog (`MC_WATCHDOG_MS`) is kicked for 3 s, then kicking stops; the log prints the time every 20 ms | Motor stops within the watchdog time of the "stopped kicking" line (last log line gives the elapsed time); the board reboots and prints `[S2] previous reset: WATCHDOG` |
| S2-H06 | system timer runs at 1 MHz after a cold power-up | kernel `startup_clock()` watchdog TICK, RP2040 datasheet 4.7.2 | manual | Flash `TEST=1 S2_CAL_TEST=1`, switch the robot fully off (battery and USB), power it on, open the serial port, press GP20 once with the wheels off the ground | `[S2] CAL timer:` reports 1000 us per ms of kernel time within 1%, after both a BOOTSEL flash and a cold power-up |
| S2-H07 | distance scale (counts per revolution and wheel circumference together) | odometry, `MC_WHEEL_CIRC_UM` | manual | Floor, `TEST=1 S2_CAL_TEST=1`. Mark the floor at the front axle, press GP20 (forward 1000 mm), measure the axle travel with a tape. Repeat 3 times | Record each measured distance. New circumference = old x measured / 1000 (median of 3); accept when a re-run is within 1% |
| S2-H08 | track width (in-place turn angle) | odometry, `MC_TRACK_MM` | manual | Floor, after S2-H07 is applied (the turn uses the distance scale), `TEST=1 S2_CAL_TEST=2`. Align the robot with a tape line, press GP20 (left 360 degrees), measure the physical angle turned. Repeat 3 times | Record each measured angle. New track = old x odometry angle / measured angle (median of 3; the odometry angle includes any overshoot): turning too far means the track is smaller than configured; accept when a re-run is within 3 degrees |
| S2-H09 | speed PID step response at the move speed and at line-follow speeds | per-wheel speed PID (`MC_PID_*`) | manual | Floor, battery and USB. `TEST=1 S2_CAL_TEST=1 S2_MOTION_LOG=1` (forward 1000 mm at `MC_MOVE_SPEED_MM_S`), then `TEST=1 S3_LINE_TEST=1 S2_MOTION_LOG=1` on the track. Save the `[MC]` lines | Measured speed reaches the target within 300 ms, overshoot under 20%, no sustained oscillation (speed sign or duty alternating period to period); record rise time, overshoot and steady-state ripple per wheel |

| S2-I01 | remote STOP while moving (with Hong Yih) | IF-06, latched STOP | manual | `NET=lwip MQTT=1` build with S1's bridge. START over MQTT, send STOP while the demo script drives. Repeat with `remote_q` filled by two pending commands | Robot stops; STOP still latched until RESET then START; `S2_DEMO_LOG` prints latch-to-motor-off; record receipt-to-flag from `s2_remote_stop_info()` separately |
| S2-I02 | stop confirmation before scan (with Timothy) | IF-10, decisions S5 section 6 | manual | Drive, trigger a STOP, read `motion_stop_confirmed()`; Timothy's engine must only start scanning after STOP_CONFIRM | STOP_CONFIRM sent only after 100 ms still; no scan while moving |
| S2-I03 | hump height from the distance feed (with Khang) | IF-09 | manual | Khang's task calls `motion_get_status()` + `s2_distance_delta()` each 10 ms; drive the demo script over a known hump | S4 reports a valid height; `b_distance_valid` false while reversing; run_id change resets his run maximum |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Calibration (fill in, then update motion_config.h)

| Quantity | Method | Trial 1 | Trial 2 | Trial 3 | Value used | Date / commit |
|---|---|---|---|---|---|---|
| Counts per wheel rev, left (x2) | S2-H03, one hand-turned rev | 1336 | 1336 | 1324 | 1330 (provisional; repeat with 10 revolutions) | 2026-11-04, measured on `de808dc` |
| Counts per wheel rev, right (x2) | S2-H03 | 1307 | 1332 | 1257 | 1330 (as left) | 2026-11-04, measured on `de808dc` |
| Wheel circumference (um) | roll 10 revs on the floor, measure, divide by 10 | | | | | |
| Track width (mm) | wheel contact centre to centre | | | | | |
| Left/right encoder counts down going forward? | S2-H01 (bench) | | | | S3 line profile: left no (recheck); right yes | The 2026-11-05 straight-line log reported right speed -114 mm/s and a decreasing GP0/GP1 count while forward was commanded; this sign is inverted only in the line-test profile |
| Left/right motor needs negative duty to go forward? | S2-H01 (bench) | | | | motor 1 (right wheel) no, motor 2 (left wheel) yes | 2026-11-02, `9f43bc3`; wheels per pin mapping 2026-11-03 |
| Still threshold (mm/s) | robot at rest: largest speed reading over 10 s, plus margin | | | | | |
| Interrupt rate at full speed (edges/s per wheel) | S2-H04 | | | | | |

## PID tuning

| Date | Loop | Kp | Ki | Kd | Setpoint | Rise time | Overshoot | Steady error | Notes |
|---|---|---|---|---|---|---|---|---|---|
| | wheel speed | | | | e.g. 200 mm/s step | | | | |
| | straight-line difference | | | | 0 mm, 500 mm move | | | | |

Final gains: speed Kp ___ Ki ___ Kd ___; straight Kp ___ Ki ___ Kd ___.

## Accuracy (10 runs each, after calibration)

Straight 500 mm (target: within 20 mm):

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Distance (mm) | | | | | | | | | | | | |
| Lateral drift (mm) | | | | | | | | | | | | |

Left 90 deg (target: within 5 deg):

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Angle (deg) | | | | | | | | | | | | |

Right 90 deg (target: within 5 deg):

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Angle (deg) | | | | | | | | | | | | |

U-turn 180 deg (target: within 10 deg):

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Angle (deg) | | | | | | | | | | | | |

STOP latency, latch to motors commanded off (target: within one 20 ms
cycle; `S2_DEMO_LOG` prints it from `s2_stop_timing()`):

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | Mean | Worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Latch to motor off (us) | | | | | | | | | | | | |
| Receipt to flag, remote STOP (ms, S2-I01) | | | | | | | | | | | | |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-02 | S2-U01 to S2-U53 | `806cd45` | Dalton | PASS (53 passed, 0 failed) | log name to add | `TEST=1` on the Pico W |
| 2026-11-02 | S2-H01 | `9f43bc3` | Dalton | PASS | log name to add | Both motors drive both ways. Right side reversed (positive duty drove it backward, its encoder counted up); inverts set in `9f43bc3`. Run with the PWM reset fix and new encoder pins, before the inverts |
| 2026-11-02 | S2-H04 | `9f43bc3` | Dalton | FAIL | log name to add | CPU hung with both motors at 100%; serial output stopped and the motors kept running. Happened with and without working encoders. Under investigation: split diagnostic `8e0f87e`, duty ramp `469c1f0`, watchdog `a2577d4` |

Note, 2026-11-02 demo image (`806cd45`, `S2_DEMO_AUTOSTART`): Ready, Running,
then Fault(motion) after about 1 s, because the motors never moved (PWM block
held in reset, fixed in `9f43bc3`) and stall detection fired. STOP latch to
motor off: 389 us (one sample, from that image).

Note, 2026-11-03: the team pin mapping puts motor 1 (GP8/GP9) on the right
wheel and motor 2 (GP10/GP11) on the left; encoder 1 (GP7/GP28) is left and
encoder 2 (GP0/GP1) right. The 2026-11-02 rows above call motor 2 "right".
The motor result stands (motor 2 needs negative duty); the encoder direction
must be measured again.

Note, 2026-11-04: the bounded S3 line trial commanded equal positive logical
wheel speeds but the car spun suddenly to its physical right. This indicates
the right wheel (motor 1) was running backward while the configured left wheel
ran forward. `MC_RIGHT_MOTOR_INVERT` was changed to 1 for the next wheels-raised
trial. This remains provisional until S2-H01 confirms both physical directions.

Correction, 2026-11-04: subsequent S2-H01 and S2-H03 bench measurements on
`origin/feature/full-system-integration` (`de808dc`) supersede the preceding
line-trial inference. Motor 1/encoder 1 is the right wheel and needs no motor
inversion; motor 2/encoder 2 is the left wheel and needs motor inversion. The
right encoder counts up going forward, the left counts down, and the provisional
x2 count is 1330 per wheel revolution.

Note, 2026-11-04 pin remap: the agreed mapping now routes motor 1 encoder 1
to GP26/GP27 and motor 2 encoder 2 to GP7/GP28. The 1330-count result belongs
to the same physical motor encoders and remains provisional. The configured
right-up/left-down signs must be confirmed again with S2-H01/H03 before an
unattended run; a successful build is not that confirmation.
| 2026-11-10 | S2-H06 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=1`) | Dalton | PASS | `logs/2026-11-10_s2-h06-bootsel.txt`, `logs/2026-11-10_s2-h06-coldboot.txt` | Wheels off the ground, USB power only (unplugging is a full power-off). After BOOTSEL flash: 37,942,811 us in 37,820 ms; after cold power-up: 25,222,882 us in 25,100 ms (before the kernel TICK fix a cold boot gave about 23 us per ms). Both include the same ~123 ms offset; between the runs 12,719,929 us per 12,720 ms = 1000.0 us/ms. Each run: forward 1000 mm DONE, odometry x 1002 mm |
| 2026-11-10 | S2-H07 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=1`, `MC_WHEEL_CIRC_UM` 204000) | Dalton | MEASURED; re-run pending | `logs/2026-11-10_s2-h07.txt` | Floor, battery and USB. Three runs, each DONE with odometry x 1001 mm; tape-measured axle travel 1115, 1115, 1115 mm (recorded as 111.5 cm). New `MC_WHEEL_CIRC_UM` = 204000 x 1115 / 1001 = 227233; acceptance re-run with the new value still to do |
| 2026-11-10 | S2-H07 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=1`, `MC_WHEEL_CIRC_UM` 227233) | Dalton | PASS | `logs/2026-11-10_s2-h07-rerun.txt` | Acceptance re-run, floor, battery and USB. Odometry x 1001, 1002, 1002 mm (all DONE); tape-measured 1007, 1004, 1006 mm (median 1006, +0.5%), within the 1% criterion. Value kept |
| 2026-11-10 | S2-H08 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=2`, `MC_TRACK_MM` 130, `MC_WHEEL_CIRC_UM` 227233) | Dalton | MEASURED; re-run pending | `logs/2026-11-10_s2-h08.txt` | Floor, battery and USB. Three runs, each DONE, x/y 0 mm; odometry 362.7, 360.5, 363.2 degrees (heading wraps). Physical turn estimated by eye at about 380 degrees (single estimate, not per run). New `MC_TRACK_MM` = 130 x 362.7 / 380 = 124; acceptance re-run still to do |
| 2026-11-10 | S2-H08 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=2`, `MC_TRACK_MM` 124, then 122) | Dalton | FAIL (still over-rotating); 120 provisional | `logs/2026-11-10_s2-h08-rerun.txt` | Track 124: six turns, all DONE, odometry 361.8..362.3 degrees; one measured: front-left corner about 1.5 cm past the line (corner reported 4.4 cm from the axle midpoint, which is less than half the 10.5 cm front width, so not usable as a radius). Track 122: odometry 363.5 degrees; front about 2.6 cm too far left (radius not measured). Readings are not consistent enough to give an angle; stepped to 120 (about -6 degrees). Re-measure with front and rear side-edge offsets |
| 2026-11-10 | S2-H08 | working tree after `e72d707` (`TEST=1 S2_CAL_TEST=2`, `MC_TRACK_MM` 120) | Dalton | PASS (by eye) | `logs/2026-11-10_s2-h08-track120.txt` | Floor, battery and USB. Seven turns, all DONE, x/y 0 mm, odometry 361.4..362.9 degrees; physical end 1-2 degrees from the starting line, estimated by eye (within the 3 degree criterion). `MC_TRACK_MM` 120 kept |
| 2026-11-10 | S2-H08 | `f9966c0` (`MC_TRACK_MM` 120) | Dalton | ADJUSTED | `logs/2026-11-10_s2-h08-track120.txt` | Re-observed: the 360 degree turn overshoots slightly to the left (about 2 degrees by eye). Stepped to 119 (about -2.9 degrees, the finest 1 mm step); re-check pending |
| 2026-11-10 | S2-H08 | working tree after `f9966c0` (`TEST=1 S2_CAL_TEST=2`, `MC_TRACK_MM` 119) | Dalton | PASS (by eye) | `logs/2026-11-10_s2-h08-track119.txt` | Floor, battery and USB. Two turns, both DONE, x/y 0 mm, odometry 363.5 and 362.1 degrees; physical end on the starting line by eye. `MC_TRACK_MM` 119 kept |
| 2026-11-10 | S2-H09 (200 mm/s) | working tree after `b6109d6` (`TEST=1 S2_CAL_TEST=1 S2_MOTION_LOG=1`) | Dalton | PASS at 200 mm/s | `logs/2026-11-10_s2-h09-move200.txt` (lines 35-140: second run; lines 1-33 are buffered output from an earlier unrecorded run) | Floor, battery and USB. 0 to 200 mm/s: 42/51, 123/132, 188/196, 205/196 mm/s at 50/100/150/200 ms; the rise is limited by the duty ramp (+312 per period), not the PID. Steady 196..205 mm/s (the 4.3 mm/s speed quantisation), duty 1700..1830 of 6250, no oscillation. Stop: still within about 150 ms, about 8 mm coast; DONE at x 1003 mm; encoder errors 0/0. Gains unchanged |
| 2026-11-10 | S2-H09 (line-follow speeds) | working tree after `b6109d6` (`TEST=1 S3_LINE_TEST=1 S2_MOTION_LOG=1`) | Dalton | PASS at 32..40 mm/s; 5 mm/s at the resolution limit | `logs/2026-11-10_s2-h09-line.txt` | Track, two GP20 sessions, 2713 speed lines, no fault or stall. Settled means within 1.3 mm/s of target at 32..40 mm/s (left 40: 40.1, sd 3.8; right 40: 39.5, sd 3.6; sd equals the 4.3 mm/s speed quantisation), no oscillation. Inside wheel at 5 mm/s: mean 5.4/5.6 but half the samples read 0 and peaks reach 25..29 mm/s (one count per 40 ms window, plus stick-slip below the motor breakaway duty). Peak duty 1120 of the 1250 (20%) line-test cap. Gains unchanged |
| 2026-11-10 | S2-U01 to S2-U59 | `4f780dc` (`TEST=1`) | Dalton | FAIL (58 passed, 1 failed) | `logs/2026-11-10_s2-selftest-u21fail.txt` | First self-test on this branch after the remap and timer fix. S2-U21 failed: a zero SPEED target reset the wheel only when the command arrived, so the same period reverse-drove it (duty -312 from 200 mm/s). Reproduced on the host |
| 2026-11-10 | S2-U01 to S2-U59 | working tree after `4f780dc` (`TEST=1`) | Dalton | PASS (59 passed, 0 failed) | `logs/2026-11-10_s2-selftest.txt` | Fix: in SPEED a zero wheel target holds duty 0 and clears that PID every period. S2-U21 now also checks the brake holds for 5 periods and that the wheel drives again; the revised test fails against the old logic (host). Wheels off the ground |
| 2026-11-10 | S2-H04 | working tree after `4f780dc` (`TEST=1 S2_HW_TEST=3`) | Dalton | PASS | `logs/2026-11-10_s2-h04.txt` | Wheels off the ground, first full-speed run on this branch (cold-boot timer fix, agreed pin mapping; image predates the S2-U21 fix, which the bench does not use). Every step printed done and braked; 0 decode errors in 440 readings (11-04: 2 and 11). H04a motor 1 moved only encoder 2 (GP7/GP28, left); H04b motor 2 only encoder 1 (GP0/GP1, right). No-load 100%: 4787, 5208, and 5165/5154 counts/s together (about 820..890 mm/s at 227233 um / 1330 counts) |
| 2026-11-10 | S2-H05 | working tree after `4f780dc` (`TEST=1 S2_HW_TEST=4`) | Dalton | PASS | `logs/2026-11-10_s2-h05-coldboot.txt` | Wheels off the ground, USB power only; USB unplugged 21:04:08 to 21:04:15 (cold power-up). Every cycle before and after: last line "100 ms since last kick", then reset, within `MC_WATCHDOG_MS` 200 (before the TICK fix a cold boot gave about 8.6 s); motor 1 stopped at reset; cycle about 11 s. The reset-cause line prints before the USB port reconnects, so it is not in the log. The image predates the S2-U21 fix (its self-test prints S2-U21 FAIL); the bench drives the motors directly, so H04/H05 are unaffected |
| 2026-11-10 | — (comparison, line-follow inside-wheel curve speed) | working tree after `4f780dc`, `TEST=1 S3_LINE_TEST=1 S2_MOTION_LOG=1` with `CTRL_LINE_LEFT/RIGHT_CURVE_MM_S` 10 and 0 (scratch builds; source kept at 5) | Dalton | 5 kept | `logs/2026-11-10_line-inner10.txt`, `logs/2026-11-10_line-inner0.txt`; 5 is `logs/2026-11-10_s2-h09-line.txt` | 10 mm/s: no fault, but the wider arc could not hold the loop (repeated sustained RIGHT 35/10), stopped by GP20. 0 (brake): junction and curves worked, then Fault(STALL) on the right wheel: duty 1250 (the 20% line-test cap), 0 mm/s, counts fixed for 500 ms while pivoting on the braked left wheel. 5 mm/s is the only value that completes the lap; S2 decision 8 |
| 2026-11-10 | S2-U01 to S2-U59 | working tree after `700bfe2` (`TEST=1`, subsystem 5 integrated) | Dalton | PASS (59 passed, 0 failed) | `../subsystem5/logs/2026-11-10_s5-integration-selftest.txt` | `robot_state.mode` and `scan_q` added (S2 decision 9); S2-U39 sizes unchanged |
| 2026-11-11 | S2-H03 | working tree after `85c1a92` (`TEST=1 S2_HW_TEST=1`, left encoder moved to Grove 4, S2 decision 13) | Dalton | PASS | `logs/2026-11-11_s2-h03-grove4.txt` | Motors off, one hand turn forward then back per wheel (single turn, not the 3 repeats). Encoder 2 (left, GP16/GP17) 0 to 1323 and back to 66; encoder 1 (right, GP0/GP1) 0 to -1280 and back to 4; 0 decode errors on both. Left counts up and right down going forward, as before the swap, so the GP16 = A, GP17 = B order is right. 1323 per hand turn is consistent with 1330 counts/rev. Same image: S2 67/67 (incl. S2-U60..U67), S3 2/2, S4 2/2, S5 39/39 on target |
