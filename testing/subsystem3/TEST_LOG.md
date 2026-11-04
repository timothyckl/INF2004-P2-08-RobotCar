# Subsystem 3 — test log

**Owner:** Edmund (2501578)  ·  **Planned subsystem:** IR line following and barcode decoding  ·  **Code:** `app_program/subsystem3/`

The cases below test the current consumer starter example, not the planned subsystem.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Integration additions (2026-11-03, feature/full-system-integration)

Test IDs never change. Host runs (`make -C testing/host`) use synthetic input
on a computer; they are not on-target or hardware results.

Retired with the starter consumer: S3-U01, INT-01.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U02 | IF-06 snapshot consistent on target | IF-06 | auto | `subsystem3_test.c` | steering only when valid, within ±1000 |
| S3-U03 | line tracking: table, filter, lost, fault, barcode bit ignored | FR2 | auto (host) | `testing/subsystem3/test_track_scan.c` | 38 checks pass (with S3-U04) |
| S3-U04 | barcode capture: A..D, reverse, speed, contrast, single bar, overflow | FR3 | auto (host) | `test_track_scan.c` | as above |
| S3-H01 | IR polarity per sensor on white and black | S3_IR_BLACK_LEVELS | manual | print raw GPIO, white then black, each sensor | 1 = black confirmed or config changed |
| S3-H02 | sensor spacing against line width; steering truth table | straddle table | manual | ruler; slide line under the sensors | table matches geometry |
| S3-H03 | following straight and curved track | FR2 | manual | robot on track, logs | stays on line |
| S3-H04 | printed barcodes decode at speed | FR3 | manual | each code ×10 | correct symbol each pass |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-03 | base logic (656 checks), S3-U03, S3-U04 | `ebb59a5` | host (gcc) | PASS | `../integration/logs/2026-11-03_host-suites.txt` | host only |
| 2026-11-04 | S3-U02 | `1573b62` | Dalton | PASS | `../integration/logs/2026-11-04_target-selftest.txt` | `TEST=1` on the Pico W |
| 2026-11-04 | S3-H01 | `1573b62` | Dalton | FAIL | observed on Robo Pico Grove status LEDs | All three high on black, low on white (polarity 0x07 holds). Left and barcode DO/AO were swapped at the module: DO arrived on GP17/GP27, firmware reads GP16/GP26. On these cables white = Grove first pin, yellow = second. Rewired to the pin map (DO on white: GP16, GP6, GP26; AO on yellow: GP17, GP27; front-right AO unconnected); barcode DO on GP26 confirmed by pot test. Re-test pending: pot test on GP16 and GP6, four-row crosstalk table (GP26 was seen lit with front-right on black), Grove 5 GP26 wire insulated |
| 2026-11-04 | S3-H02 | `1573b62` | Dalton | INCOMPLETE | measurement notes | Sensor spacing 21 mm (centre-to-centre not confirmed), line width 18 mm: both front sensors on white when centred, the straddle case `s3_track.c` assumes; about 1.5 mm margin per side. Risk: LOST after 1.5 s centred on a long straight, check in S3-H03. Line slide-through table not yet recorded |
| 2026-11-04 | S3-U02 | `1cd5330` | Dalton | PASS | `../subsystem4/logs/2026-11-04_s4-h01-4.txt` | `TEST=1 S4_HW_TEST=1` on the Pico W |
