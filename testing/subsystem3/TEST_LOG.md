# Subsystem 3 test log

Owner: Edmund (2501578)
Scope: front IR line sensing and barcode decoding

## Defined test cases

| ID | Test | Method | Expected result |
|---|---|---|---|
| S3-H01 | Front-left DOUT on GP6 over white then black | Guided calibration | At least 45/50 readings agree on each surface and the digital level changes |
| S3-H02 | Front-right DOUT on GP16 over white then black | Guided calibration | At least 45/50 readings agree on each surface and the digital level changes |
| S3-H03 | Combined digital polarity calibration | Automatic after S3-H01/H02 | Both sensor observations are accepted by `s3_ir_calibrate` |
| S3-H04 | Button-controlled two-sensor line-follow bench trial | Wheels raised; `TEST=1 S3_LINE_TEST=1 CRASH_REPORT=0` | GP20 starts/stops, WHITE/WHITE commands straight at 45 mm/s, BLACK/WHITE corrects left, WHITE/BLACK corrects right, and persistent BLACK/BLACK enters the bounded crossing and exit policy |
| S3-H05 | Line-report queue remains usable across STOP/restart | Leave the line-test image stopped for at least 2 s, then press GP20; repeat STOP, RESET and START | START accepts a fresh report without `invalid report` or `report delivery failed`; line following starts normally on every repetition |
| S3-H06 | GP20 recovery after an observed Motion Control fault | After an incidental supervised fault, remove the obstruction and wait until both wheels stop; press GP20 once | Serial reports `FAULT RESET requested`, Motion Control returns healthy, START follows automatically, and the original one-shot fault diagnostic identifies the indicated stalled wheel without bypassing the stall detector |
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

For the 2026-11-05 installed-wiring profile, verify before power-up that the
left motor uses motor 1 (GP8/GP9), the right motor uses motor 2 (GP10/GP11),
the left encoder uses GP7/GP28 and the right encoder uses GP0/GP1. Use only the
USB console and keep `CRASH_REPORT=0`; this wiring is not the full-system map.

## Results

Add one row for every physical run. Never replace earlier evidence.

| Date | Test ID | Commit | Tester | Result | Log/evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-H01..H03 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD-s3-cal.txt` | Record surfaces, lighting, height, counts, and raw levels |
| 2026-11-05 | S3-H04 steering mapping | working tree | Edmund | FIXED IN CODE; hardware retest pending | Sensor classification was correct; vehicle repeatedly turned left | Left correction now slows/reverses the left wheel and drives the right wheel; right correction is the mirror image |
| 2026-11-05 | S3-H04 curve trial | working tree | Edmund | PARTIAL | Robot followed and adjusted on the straight, then turned but left the line halfway through the curve | Strengthen sustained curve to 40/0 and hold the remembered direction for at most 100 ms through WHITE/WHITE; physical retest pending |
| 2026-11-05 | S3-H04 loop-junction trial | working tree | Edmund | PARTIAL | Curve tracking became consistent, but the robot left the forward route after returning to the T junction and continued on WHITE/WHITE | Preserve 40/40 through BLACK/BLACK; after a loop curve, use a bounded previous-direction probe to reacquire the forward line or STOP |
| 2026-11-05 | S3-H04 post-junction regression | working tree | Edmund | FAIL | Loop and junction exit worked, but ordinary curves no longer turned sufficiently | Restore the previously effective immediate 40/0 single-black correction while retaining junction-exit state |
| 2026-11-05 | S3-H04 incoming-branch regression | working tree | Edmund | FIXED IN CODE; hardware retest pending | Normal curves and the loop worked, but the directional junction probe turned back onto the incoming branch | Replace the directional probe with a bounded 250 ms 40/40 straight-exit window, then resume normal feedback |
| 2026-11-05 | S3-H04 junction qualification trial | working tree | Edmund | FAIL; revised code pending hardware retest | The 250 ms exit still returned to the incoming branch, while ordinary curves no longer turned | Qualify BLACK/BLACK for 100 ms while retaining the prior curve command; after a confirmed junction clears, hold 40/40 for 600 ms |
| 2026-11-05 | S3-H04 speed revision | working tree | Edmund | FIXED IN CODE; hardware retest pending | Requested line-follow speed increase | Raise the forward/outside-wheel target from 40 mm/s to 45 mm/s without changing steering or junction timing |
| 2026-11-05 | S3-H04 directional curve trial | working tree | Edmund | FIXED IN CODE; hardware retest pending | Logs repeatedly showed sustained LEFT followed by WHITE/WHITE and straight travel; the RIGHT loop and junction exit completed | Keep working RIGHT behavior; strengthen only sustained LEFT to -10/45 mm/s and extend its WHITE/WHITE hold from 100 ms to 300 ms |
| 2026-11-05 | S3-H04 intermittent loop-exit trial | working tree | Edmund | FIXED IN CODE; hardware retest pending | Both normal curve directions worked, but either loop could intermittently steer onto another branch after the junction | Keep straight after the 600 ms minimum until WHITE/WHITE is stable for 100 ms; STOP if the exit cannot centre by 1500 ms |
| 2026-11-05 | S3-H04 stale/left-exit trial | working tree | Edmund | FIXED IN CODE; hardware retest pending | One run stopped on a stale-report watchdog; another left-loop exit saw RIGHT black then accepted off-track WHITE/WHITE | Allow 500 ms between supervised-test reports; after the exit minimum, correct away from a side-black branch before accepting stable WHITE/WHITE |
| 2026-11-05 | S3-H04 on-line sensor geometry | working tree | Edmund | FIXED IN CODE; hardware retest pending | Both front sensors now reliably report BLACK/BLACK while centred on the normal line | Make BLACK/BLACK straight and WHITE/WHITE lost; search the recent direction for at most 800 ms, with no junction state in this trial |
| 2026-11-05 | S3-H04 centred-line hunting | working tree | Edmund | FIXED IN CODE; hardware retest pending | With both front sensors positioned over the line, the car alternated LEFT/RIGHT while seeking BLACK/BLACK | Soften ordinary corrections from 0/45 to 30/45 mm/s while retaining the established sustained-curve targets |
| 2026-11-05 | S3-H04 straight-line overshoot retest | working tree | Edmund | FIXED IN CODE; hardware retest pending | The 30/45 mm/s correction still shifted too far left and right on a straight | Use 38/45 mm/s ordinary corrections, require 100 ms before reversing steering, and delay strong-curve entry to 250 ms |
| 2026-11-05 | S3-H04 separated-sensor restoration | working tree | Edmund | FIXED IN CODE; hardware retest pending | Front sensors were moved back to either side of the line so the centred pattern is WHITE/WHITE | Restore the pre-side-by-side mapping, 45 mm/s steering profile, curve holds, BLACK/BLACK junction qualification and bounded straight exit |
| 2026-11-05 | S3-H04 separated-sensor oscillation | working tree | Edmund | FIXED IN CODE; hardware retest pending | Serial repeatedly alternated sustained LEFT/RIGHT around WHITE/WHITE and the car moved abruptly | Use 35/45 gentle corrections, require 300 ms for a strong curve, use 15/45 strong curves, and reduce both WHITE/WHITE curve holds to 50 ms |
| 2026-11-05 | S3-H04 damped normal-curve retest | working tree | Edmund | FIXED IN CODE; hardware retest pending | Straight oscillation improved, but the 300 ms, 15/45 sustained stage could not follow an ordinary curve and left the course | Keep gentle corrections and the 50 ms hold; enter sustained steering after 200 ms with a stopped inside wheel |
| 2026-11-05 | S3-H04 intermittent curve acquisition | working tree | Edmund | FIXED IN CODE; hardware retest pending | Normal curve steering was intermittent and sometimes remained straight long enough to leave the line | Sample every 10 ms, retain three-sample stability, and refresh line reports every 30 ms |
| 2026-11-05 | S3-H04 left-loop loss and automatic stops | working tree | Edmund | FIXED IN CODE; hardware retest pending | LEFT loop could leave the line; several runs stopped at the junction/exit bound, and two separate runs reported RIGHT stall/no feedback on GP0/GP1 | Resume normal tracking after a side remains stable for 150 ms following the 600 ms exit minimum, extend bounded crossing/exit windows, hold sustained LEFT through WHITE/WHITE for 150 ms, and retain the independent stall safety check |
| 2026-11-05 | S3-H04 normal-right-curve authority | working tree | Edmund | FIXED IN CODE; hardware retest pending | Logs repeatedly reached sustained RIGHT, but the vehicle continued straight through the normal right curve | Use a 45/-10 mm/s sustained RIGHT target, retain it for 150 ms through WHITE/WHITE, and print the applied strong-curve target in the serial log |
| 2026-11-05 | S3-H04 right-wheel stall during strong LEFT | working tree | Edmund | FIXED IN CODE; hardware retest pending | Motion Control measured 0 mm/s on the right wheel at PWM 1250 during a 0/45 strong LEFT target and latched RIGHT stall/no feedback | Replace pivot-style strong turns with rolling 10/40 and 40/10 arcs, retain the stall safety detector, inspect GP0/GP1 if it recurs, and print unchanged sensor state every 500 ms |
| 2026-11-05 | S3-H04 rolling-arc left-loop and dual-stall retest | working tree | Edmund | FIXED IN CODE; hardware retest pending | First run left the LEFT loop using 10/40 arcs and ended by GP20 STOP; second run latched both-wheel stall at PWM 1250/1250 and 0/7 mm/s while sensor heartbeats continued | Reduce straight/correction targets to 40/40 and 32/40, tighten curves to rolling 5/35 arcs, extend LEFT hold to 250 ms, and retain the stall detector |
