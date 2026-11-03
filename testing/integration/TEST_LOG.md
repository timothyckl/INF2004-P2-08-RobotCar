# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |

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

## Integration additions (2026-11-03)

INT-01 and INT-02 are retired with the starter interfaces IF-01 to IF-05.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-03 | Line Sensor publishes IF-06 every 10 ms | IF-06 | auto | `subsystem3_test.c` (TEST=1) | ≥80 samples per second, latest ≤50 ms old |
| INT-04 | line following end to end | IF-06, D1 | manual | robot on a straight then curved line | follows; STOP stops within 20 ms of the flag |
| INT-05 | barcode turn at the junction | IF-06, IF-07 | manual | each command A..D | correct manoeuvre once |
| INT-06 | obstacle scan, bypass and reacquire | IF-11..IF-14 | manual | known obstacle on the line | bypasses and resumes; faults safely when blocked |
| INT-07 | MQTT commands and telemetry | IF-08..IF-10 | manual | broker on the LAN | as S1-H01..H03 |
| INT-08 | hump distance into S4 | IF-15 | manual | ramp of known height | height valid, compared with measurement |

No on-target or hardware integration run has been recorded yet.
