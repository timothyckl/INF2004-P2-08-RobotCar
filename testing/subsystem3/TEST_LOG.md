# Subsystem 3 test log

Owner: Edmund (2501578)
Scope: front IR line sensing and barcode decoding

## Defined test cases

| ID | Test | Method | Expected result |
|---|---|---|---|
| S3-H01 | Front-left DOUT on GP6 over white then black | Guided calibration | At least 45/50 readings agree on each surface and the digital level changes |
| S3-H02 | Front-right DOUT on GP16 over white then black | Guided calibration | At least 45/50 readings agree on each surface and the digital level changes |
| S3-H03 | Combined digital polarity calibration | Automatic after S3-H01/H02 | Both sensor observations are accepted by `s3_ir_calibrate` |
| S3-H04 | Button-controlled two-sensor line-follow bench trial | Wheels raised; `TEST=1 S3_LINE_TEST=1 CRASH_REPORT=0` | GP20 starts/stops, WHITE/WHITE commands straight, each single BLACK produces the corresponding correction, and BLACK/BLACK produces STOP |
| S3-U01 | Legacy statistics remain self-consistent | On-target automatic check | Received count is nonzero; min/max remain ordered and in 0..100 |
| INT-01 | Samples flow from subsystem 2 to subsystem 3 | On-target automatic check | At least five new samples arrive in two seconds |

The H cases require a person to position the named sensor when prompted. The
calibration build produces PASS/FAIL results, then continuously reports WHITE
or BLACK. In this dedicated build, the other application subsystems are not
started.

## Physical-test procedure

First use `TEST=1 S3_HW_TEST=1 CRASH_REPORT=0` to adjust and verify raw digital
transitions. Then build `TEST=1 S3_CAL_TEST=1 CRASH_REPORT=0`, flash the
`_test_s3cal.uf2`, and save the serial output. Follow the four prompts while
holding the named sensor at its normal height. Do not move the motors.

Run S3-H04 only after those cases pass. Raise the wheels, use the `_test_s3line`
image, confirm the saved LOW=WHITE/HIGH=BLACK message, place the car over the
line, and press the GP20 programmable button. Press GP20 again to verify STOP,
then restart only after both wheels have stopped. Capture the entire run with
`capture_serial.ps1` and save a video showing both wheels. This is a supervised
integration observation, not validation of unattended floor tracking.

## Results

Add one row for every physical run. Never replace earlier evidence.

| Date | Test ID | Commit | Tester | Result | Log/evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-H01..H03 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD-s3-cal.txt` | Record surfaces, lighting, height, counts, and raw levels |
