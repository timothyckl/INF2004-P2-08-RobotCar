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
| S2-U18 | quadrature step: A rise with B low = +1, A rise with B high = -1, A fall with B high = +1, A fall with B low = -1; both edges or no edge = 0 | drv_encoder (x2 decoding) | auto | `subsystem2_test.c`, `drv_encoder_step()` | `[TEST] S2-U18 PASS` |
| S2-U19 | encoder snapshot rejects NULL and returns E_OK with counts that do not change while the encoders are idle | drv_encoder | auto | `subsystem2_test.c`; encoders not initialised | `[TEST] S2-U19 PASS` |
| S2-U20 | Motion Control start: hardware OK gives Idle + READY; failed init gives Fault(INIT) + FAULT bit, zero duty | Motion statechart: Init | auto | `test_motion.c`, synthetic wheel model | `[TEST] S2-U20 PASS` |
| S2-U21 | Speed state drives both wheels to the targets (sign, and within 20 mm/s after 1 s) | Motion statechart: Speed | auto | `test_motion.c` | `[TEST] S2-U21 PASS` |
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
| S2-H01 | each motor drives the expected way | Robo Pico truth table, design review section 2 | manual | Wheels off the ground. Build with `S2_HW_TEST` set to 2 in `task_motion.h`. Per motor: 30% forward 2 s, brake 3 s, 30% backward 2 s, coast 3 s. Watch the wheel and the board's MxA/MxB LEDs | Forward: wheel turns forward, MxA LED on. Backward: reverse, MxB LED on |
| S2-H02 | brake and coast differ as the truth table says | Robo Pico truth table | manual | During S2-H01, turn each wheel by hand in the brake and coast steps | Brake: noticeably stiff. Coast: turns freely |
| S2-H03 | encoder count sign and counts per revolution | drv_encoder, calibration | manual | GP28 battery-sense jumper open. `S2_HW_TEST` set to 1 (motors stay off) prints both counts every 100 ms for 60 s. Mark each wheel, turn it exactly one revolution forward by hand, then one back. Repeat 3 times per wheel | Count changes by the same amount each way, opposite signs, error count unchanged. Record counts per rev and which encoders count down going forward |
| S2-H04 | encoder edges are not missed at full speed | drv_encoder, GPIO dispatcher, PostTri1 interrupt load | manual | Wheels off the ground, `S2_HW_TEST` set to 2: after S2-H01, both motors 100% forward for 10 s | Error counts stay 0. Record counts/s per wheel: that is the interrupt rate per wheel (one interrupt per A edge) |

| S2-I01 | remote STOP while moving (with Hong Yih) | IF-06, latched STOP | manual | `NET=lwip MQTT=1` build with S1's bridge. START over MQTT, send STOP while the demo script drives. Repeat with `remote_q` filled by two pending commands | Robot stops; STOP still latched until RESET then START; `S2_DEMO_LOG` prints latch-to-motor-off; record receipt-to-flag from `s2_remote_stop_info()` separately |
| S2-I02 | stop confirmation before scan (with Timothy) | IF-10, decisions S5 section 6 | manual | Drive, trigger a STOP, read `motion_stop_confirmed()`; Timothy's engine must only start scanning after STOP_CONFIRM | STOP_CONFIRM sent only after 100 ms still; no scan while moving |
| S2-I03 | hump height from the distance feed (with Khang) | IF-09 | manual | Khang's task calls `motion_get_status()` + `s2_distance_delta()` each 10 ms; drive the demo script over a known hump | S4 reports a valid height; `b_distance_valid` false while reversing; run_id change resets his run maximum |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Calibration (fill in, then update motion_config.h)

| Quantity | Method | Trial 1 | Trial 2 | Trial 3 | Value used | Date / commit |
|---|---|---|---|---|---|---|
| Counts per wheel rev, left (x2) | S2-H03, one hand-turned rev | | | | | |
| Counts per wheel rev, right (x2) | S2-H03 | | | | | |
| Wheel circumference (um) | roll 10 revs on the floor, measure, divide by 10 | | | | | |
| Track width (mm) | wheel contact centre to centre | | | | | |
| Left/right encoder counts down going forward? | S2-H03 | | | | | |
| Left/right motor needs negative duty to go forward? | S2-H01 | | | | | |
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
