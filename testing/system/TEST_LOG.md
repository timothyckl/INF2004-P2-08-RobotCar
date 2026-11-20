# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | follows the line track autonomously | Brief MR1; line following | manual | Common set-up (below). Start with GP20; let the car run 3 laps of the line track with no barcode, hump or obstacle on it; end with a lift. | Each lap completed with no wheel leaving the track for more than a correction and no STOP before the lift; telemetry `FOLLOW` throughout, stop cause pick-up at the end |
| SYS-02 | detects and decodes each barcode on the course | Brief MR2; S3 barcode, IF-21 | manual | Common set-up. Drive each course barcode 5 times (separate passes). Note the printed letter. | Dashboard `barcode X decoded` with the printed letter on at least 4 of 5 passes per barcode, and never a different letter |
| SYS-03 | carries out each barcode command at the next junction | Brief MR3; S2 decision 26 | manual | Common set-up. For each of LEFT, RIGHT, STRAIGHT and U-TURN (letters as announced, else A-D), 3 runs: barcode, then junction. | Car takes the commanded branch and resumes following on it, all 12 runs; telemetry `NAVIGATE (<command>)` then `FOLLOW`; no stop cause junction |
| SYS-04 | detects humps and reports the highest peak | Brief MR4, MR5; IF-18 | manual | Common set-up. Measure each course hump with a ruler. 3 runs over the humps; also 1 lap with no hump. | Every hump crossed is detected (console `s4: hump N mm under the ...`; the dashboard shows only the run maximum); telemetry highest hump within 2 mm of the tallest hump crossed; no hump reported on the flat lap |
| SYS-05 | detects an obstacle on the line and stops before it | Brief MR6; IF-12, IF-13 | manual | Common set-up. Box on a straight (S2 decision 22) on the line; 3 runs. | Car stops and enters `SCAN` with no contact; forward range in telemetry within 20 mm of the ruler distance at the stop |
| SYS-06 | profiles obstacle shape and position | Brief MR7; IF-20 | manual | Same runs as SYS-05; measure the box width and its offset from the line. | Scan result plan with a side; nearest point within 20 mm of the ruler; width within the greater of 20 mm or 5% and centre offset recorded (S5-U49 criterion; known to fail, see S2 decision 22: record the error) |
| SYS-07 | bypasses the obstacle without contact and reacquires the line | Brief MR8, MR9 | manual | Same runs as SYS-05. | All 3 runs: no contact; telemetry `BYPASS` then `REACQUIRE` then `FOLLOW` on the original line beyond the box; no stop cause bypass |
| SYS-08 | reports telemetry over WiFi for a whole run | Brief MR10; IF-10, S1 decisions 1, 4 and 5 | manual | During SYS-09: dashboard and `decode_telemetry_v2.py` logging, `stats` topic recorded. | Version 3 frames about 1 s apart; at least 95% of periods sent (INT-06 sent 197 of 200); state, mode, barcode, hump and obstacle fields match the run; heartbeat every 5 s |
| SYS-09 | completes the full course in one autonomous run | Brief MR1-MR10; overview (no remote control) | manual | Course with line, barcodes, humps and an obstacle. Start with GP20; send no dashboard command until the end; end with a lift at the finish. Time the run. 3 runs. | Every element handled in each run with no STOP before the lift; record completion time and any element that failed |
| SYS-10 | picking the car up stops it | Brief Buddy 4 (collision detection); IF-19 | manual | Common set-up. Lift the car by hand during following, 5 times, once slowly and level. | Every lift: wheels stop, stop cause pick-up; no pick-up stop during SYS-01 laps or over humps (S4-H11) |
| SYS-11 | boots to Ready from a cold power-bank start | Robustness; S5 decision 12 | manual | Power off for at least 20 s, then on from the power bank; 5 times; dashboard open. | READY with no fault on all 5 boots (power-bank boots came up in FAULT on 2026-11-17 before S5 decision 12) |

**Common set-up.** One committed build of the line image
(`make NET=lwip MQTT=1 TEST=1 S3_LINE_TEST=1 S3_NAV_F=<n> -j8`), the
same for the whole set, and its commit in every result row. Power bank,
USB console captured to `logs/` when it can stay plugged in. Broker and
dashboard on the laptop (`testing/subsystem1/dashboard/README.md`); copy
each dashboard session log into `logs/`. Course lighting as on the day.

**Criteria agreed 2026-11-20** (Dalton, integration branch): SYS-02 at
least 4 of 5 passes per barcode and no wrong letter; SYS-03 all 12 runs;
SYS-04 within 2 mm, which also serves as the S4-H08 acceptance figure
for these runs; SYS-08 at least 95% of periods sent.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
