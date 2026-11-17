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

Note, 2026-11-03: subsystem 4 replaced its starter monitor, so
`subsystem4_test.c` no longer contains the INT-02 check and no task receives
IF-03 (`TEAM.md` section 5). INT-02 cannot run until IF-03 is retired or
given a new receiver.

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
