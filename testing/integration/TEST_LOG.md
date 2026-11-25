# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |

| INT-03 | front line reports drive only through Vehicle/Motion Control | IF-06 | manual | Wheels raised; run `_test_s3line`, present the four sensor combinations | WHITE/WHITE drives straight, each single-black pattern corrects in the expected direction, persistent BLACK/BLACK enters bounded crossing, and GP20 stops both wheels |
| INT-04 | MQTT STOP latches the controller through the urgent flag | IF-08 | manual | `NET=lwip MQTT=1`; publish `STOP` while RUNNING | next telemetry frame STOPPED, STOP latched, wheels 0 (S1-H01) |
| INT-05 | MQTT START/RESET reach the controller through `remote_q`, aged on the controller clock | IF-09 | manual | publish `START` in Ready, `START` in Stopped, `RESET` in Stopped/Fault | START runs only from Ready; RESET returns to Ready; nothing expires on arrival (S1-H01) |
| INT-06 | telemetry snapshot from S2, S4 and S5 encoded as version 2 | IF-10, IF-18, IF-20 | manual | subscribe and decode with `testing/subsystem1/decode_telemetry_v2.py` during a lap | frames 1 s apart; state, line, speeds, hump and obstacle fields match the run (S1-H02) |
| INT-07 | telemetry version 3: barcode letter, command and stop cause | IF-10, IF-21, IF-19 | manual | `NET=lwip MQTT=1 TEST=1 S3_LINE_TEST=1 S3_NAV_F=1`; dashboard and `decode_telemetry_v2.py` during a lap with barcode F and a junction; end by picking the car up; repeat ending with dashboard STOP and with GP20 | 29-byte version 3 frames; `bar=F/LEFT` and a "barcode F decoded" event within 1 s of the console `DECODED F`; NAVIGATE (LEFT) at the junction; stop cause pick-up, remote and button respectively |

Host-automated integration cases (post-demo, 2026-11-25). They run the
real Vehicle Controller (`task_controller.c`, through `controller_cycle()`)
with the other S2 modules, the real S3 filter and submit calls, the real
S1 bridge calls and the real Motion Control logic, on a working kernel
model (`testing/host/stubs/kernel_model.c`), a simulated drivetrain and a
geometric track (`testing/host/s2_integration_main.c`). Run with
`make -C testing/host int`; each check prints `[HOST] INT-xx PASS|FAIL`.
They are synthetic: ideal wheels and sensors and no timing jitter, so
they show that the interfaces and state machines fit together, not that
the car does.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-08 | S1 → S2: MQTT STOP through the bridge latches STOP; no duty while latched; START alone does not clear it; RESET then START runs | FR10, NFR1; IF-08, IF-09 | host auto | `controller_raise_remote_stop()` after 2 s of following; 3 s with reports still arriving; START; RESET, START | STOPPED with cause remote in one period; duty 0 and wheels still; still STOPPED after START; RUNNING and following after RESET, START |
| INT-09 | S3 → S2: line reports drive the wheels; missing reports latch STOP; `line_q` capacity | FR2, FR11; IF-06 | host auto | centred track; stop the S3 reports; fill `line_q` with the controller paused | both wheels 40 ± 2 mm/s; a 400 ms gap tolerated, STOP with cause line after 500 ms; 6 reports accepted, the rest refused |
| INT-10 | S3 → S2 → Motion: each barcode command at a + junction 1.5 m on | FR4; IF-21, IF-06 | host auto | command decoded 0.2 m in; RIGHT, LEFT, U-TURN, STRAIGHT in turn | RIGHT and LEFT: NAVIGATE, then following at ∓90 ± 20°; U-TURN: following back (beyond ±160°); STRAIGHT: no NAVIGATE, past the junction at 0 ± 5° |
| INT-11 | S3 ↔ S5 through S2: a held barcode command survives an obstacle scan (the demo-day case, S2 decision 36) | FR4, FR6; IF-21, IF-11, IF-13 | host auto | RIGHT decoded 0.6 m in; hazard at 1.0 m, S5 answers CLEAR after 5 s; right branch at 3.0 m | scan authorised once, only with the wheels still; SCAN for the whole scan; then NAVIGATE at the junction, more than 60 s after the decode, and following at −90 ± 20° |
| INT-12 | S4 → S2: a pick-up latches STOP and drops a held command | FR11; IF-19, IF-21 | host auto | RIGHT held; `CTRL_FLG_PICKUP`; RESET, START; drive past the junction | STOPPED with cause pick-up and the command dropped; after restart the junction is crossed with no turn |
| INT-13 | S3 → S2 → S1: barcode records reach the published state | FR3, FR10; IF-21, IF-10 | host auto | unmapped Q; an invalid record; RIGHT then LEFT; a letter during the turn | letter, command and count published; invalid record ignored; LEFT replaces RIGHT; a letter during a manoeuvre is reported and dropped |
| INT-14 | S5 → S2: a scan with no route | FR6, FR8; IF-11, IF-13 | host auto | hazard at 0.8 m, S5 answers NO_ROUTE | the scan starts only with the wheels still; STOP with cause scan; the car stays still |
| INT-15 | all tasks → Motion Control: a stalled controller stops the motors without the controller (S2 decision 37) | NFR1, FR11; Tri2 Entry 4 | host auto | 10 s healthy run; then the controller task blocked for 600 ms while following; then released; RESET, START | no stall on the healthy run; motors 0 between 500 and 510 ms with `MOTION_FAULT_TASK`; FAULT with STOP latched (cause fault, class motion); RESET and START run again |

Note, 2026-11-03: subsystem 4 replaced its starter monitor, so
`subsystem4_test.c` no longer contains the INT-02 check and no task receives
IF-03 (`TEAM.md` section 5). INT-02 cannot run until IF-03 is retired or
given a new receiver.

Note, 2026-11-20: the starter producer, consumer and IF-01 to IF-04 were
removed (S2 decision 29). INT-01 and INT-02 are retired; their IDs stay
unused.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-17 | INT-04 | `4f84aa5` line image, power bank | Dalton | PASS | `../subsystem1/logs/2026-11-17_s1-h02-telemetry.txt` | STOPPED in the next frame, 0.33 s after the command (S1-H01 row of 2026-11-17) |
| 2026-11-17 | INT-05 | `42e1797`, `4f84aa5` line images | Dalton | PASS | as above | Failed on `48bbd94` (every command expired: clock mismatch, S1 decision 3); after the fix START from Ready ran, START in Stopped was rejected, RESET from Stopped and from Fault gave Ready |
| 2026-11-17 | INT-06 | `4f84aa5` line image, power bank | Dalton | PASS | as above | 197 of 200 periods sent, 3 skipped for a busy state lock; line, speeds, heading, 10 mm hump and an obstacle scan (no route, nearest 253 mm) all reported (S1-H02 row of 2026-11-17) |
| 2026-11-20 | INT-07 | working tree after `1a2df7f` (telemetry v3, uncommitted), `NET=lwip MQTT=1 TEST=1 S3_LINE_TEST=1 S3_NAV_F=1`, power as used, broker 10.158.213.224 | Dalton | PASS (partial: barcode and remote, scan and fault causes); pick-up and button causes not exercised | `../subsystem1/logs/2026-11-20_int-07-dashboard.jsonl` | 501 frames, all version 3, none bad. Barcode events: F (LEFT) at 14:05:03 then NAVIGATE (LEFT) 14:05:07-14:05:11; A (LEFT) 14:07:39 then NAVIGATE (LEFT) 14:07:47-14:07:51; F (LEFT) again 14:09:52; A (LEFT) 14:06:44 with the run then scanning. Unmapped letters reported as command none: Q at 14:03:54 and 14:05:26, Z at 14:10:20 (not confirmed against the course; possible misreads, previously invisible). Stop causes seen: remote (6 runs ended by dashboard STOP), scan (no route at 14:08:11, right after START), fault (S5 scan fault at 14:09:24; RESET returned Ready). No run in this log ended with a pick-up or GP20, so those causes still need a run; the tester reported the session as working. Console not logged, so the 1 s barcode-to-event timing was not checked |
| 2026-11-20 | INT-07 (pick-up endings; follow-up to the row above) | as above | Dalton | PASS (barcode, remote, scan, fault and pick-up causes); GP20 button cause not exercised | `../subsystem1/logs/2026-11-20_int-07-dashboard.jsonl` (copied again after 14:17, events from 14:16:48) | Two runs ended by lifting the car: START 14:16:48, `STOP latched: pick-up` 14:16:51; START 14:17:00, `STOP latched: pick-up` 14:17:06; RESET between them returned Ready. Tester confirmed Q, Z, A and F were all on the course, so the Q and Z reports in the row above were real barcodes with no mapped command, not misreads. A GP20 stop is still needed to show the button cause |
| 2026-11-20 | INT-07 (button ending) | `02c79e6` (same firmware as the INT-07 rows above) | Dalton | PASS (button cause); INT-07 now complete | `../system/logs/2026-11-20_system-dashboard.jsonl` | Run started 14:55:29, stopped with GP20 at 14:55:39: `STOP latched: button`. Every cause INT-07 asks for (pick-up, remote, button) has now been seen on the car, with barcode letter and command |
| 2026-11-25 | INT-08, INT-09, INT-10, INT-12, INT-13, INT-14 (host) | post-demo, after the F-01 commit (`controller_cycle()` extracted) | host (Apple clang 17) | PASS (20 checks, 0 failed) | `../host/logs/2026-11-25_verify-int-harness.txt` | Synthetic: ideal drivetrain and sensors. Not evidence that the car works |
| 2026-11-25 | INT-08 to INT-14 (host), incl. new INT-11 | post-demo change after the harness commit (S2 decision 36) | host (Apple clang 17) | PASS (23 checks, 0 failed) | `../host/logs/2026-11-25_verify-f03.txt`, `../host/logs/2026-11-25_int-mutations.txt` | Synthetic. INT-11 fails with the 60 s limit put back (mutation log) |
| 2026-11-26 | INT-08 to INT-15 (host), incl. new INT-15 | post-demo change after the F-04 commit (S2 decision 37) | host (Apple clang 17) | PASS (27 checks, 0 failed) | `../host/logs/2026-11-26_verify-f08.txt`, `../host/logs/2026-11-26_health-mutations.txt` | Synthetic. INT-09 now also shows the controller's line time-out acts before the S3 loop's longer liveness limit |
