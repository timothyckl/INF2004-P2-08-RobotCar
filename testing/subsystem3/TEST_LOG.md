# Subsystem 3 — test log

**Owner:** Edmund (2501578)  ·  **Planned subsystem:** IR line following and barcode decoding  ·  **Code:** `app_program/subsystem3/`

The starter integration case remains while the new cases exercise deterministic
line-position, PD-control, junction, and recovery logic. Hardware calibration and
track performance still require recorded on-robot tests.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | centred classification | line-position estimation | auto | Supply both-white sample | Equal normalized motor targets |
| S3-U03 | PD steering direction and derivative decay | PD controller | auto | Hold right sensor over black for three samples | Left target exceeds right; correction settles to P term |
| S3-U04 | output saturation | motor-target safety | auto | Apply maximum gains near maximum base target | Both targets remain within configured limits |
| S3-U05 | junction confirmation | junction detection | auto | Supply both-black history and incremental encoder travel | Junction event occurs only after both thresholds |
| S3-U06 | recovery and reacquisition | recovery state machine | auto | Start right recovery, observe an edge, then centred samples | Line-lost and line-reacquired events occur in order |
| S3-U07 | invalid argument rejection | defensive API | auto | Initialize with null storage and an out-of-range correction | Initialization fails safely |

Host versions of S3-U02 through S3-U06 can be run without hardware:

```sh
cd testing/subsystem3/unit_tests
make test
```

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
