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
| SYS-08 | reports telemetry over WiFi for a whole run | Brief MR10; IF-10, S1 decisions 1, 4 and 5 | manual | During SYS-09: dashboard and `decode_telemetry_v2.py` logging, `stats` topic recorded. | Version 3 frames (version 4 from 2026-11-24, S1 decision 7; version 5 from 2026-11-26, S1 decision 8) about 1 s apart; at least 95% of periods sent (INT-06 sent 197 of 200); state, mode, barcode, hump and obstacle fields match the run; heartbeat every 5 s |
| SYS-09 | completes the full course in one autonomous run | Brief MR1-MR10; overview (no remote control) | manual | Course with line, barcodes, humps and an obstacle. Start with GP20; send no dashboard command until the end; end with a lift at the finish. Time the run. 3 runs. | Every element handled in each run with no STOP before the lift; record completion time and any element that failed |
| SYS-10 | picking the car up stops it | Brief Buddy 4 (collision detection); IF-19 | manual | Common set-up. Lift the car by hand during following, 5 times, once slowly and level. | Every lift: wheels stop, stop cause pick-up; no pick-up stop during SYS-01 laps or over humps (S4-H11) |
| SYS-11 | boots to Ready from a cold power-bank start | Robustness; S5 decision 12 | manual | Power off for at least 20 s, then on from the power bank; 5 times; dashboard open. | READY with no fault on all 5 boots (power-bank boots came up in FAULT on 2026-11-17 before S5 decision 12) |

**Common set-up.** One committed build of the mission image
(`make NET=lwip MQTT=1 S3_NAV_F=<n> -j8`, S2 decision 28), the same for
the whole set, and its commit in every result row. Rows before
2026-11-20 16:00 used the equivalent `TEST=1 S3_LINE_TEST=1` image. Power bank,
USB console captured to `logs/` when it can stay plugged in. Broker and
dashboard on the laptop (`testing/subsystem1/dashboard/README.md`); copy
each dashboard session log into `logs/`. Course lighting as on the day.

**Criteria agreed 2026-11-20** (Dalton, integration branch): SYS-02 at
least 4 of 5 passes per barcode and no wrong letter; SYS-03 all 12 runs;
SYS-04 within 2 mm, which also serves as the S4-H08 acceptance figure
for these runs; SYS-08 at least 95% of periods sent.

**Demo image, 2026-11-24** (Dalton, 2026-11-23; the assessed runs used a later image, see the 24 November section below): commit `4a53c23`, tag
`demo-2026-11-24`, built with `make NET=lwip MQTT=1 S3_NAV_F=0 -j8`
(single core, S2 decision 33) and `MQTT_BROKER_IP` 10.157.130.224 on
Pixel_1232. Kept as `build_make/DEMO_2026-11-24_4a53c23.uf2`, SHA-256
`a0adfa6dfa783f19d3bf1b0f04c8991070f8158450f09bc894e95da000c54336`. The
file is not in Git: it contains the WiFi credentials from
`config/wifi_secrets.h`. If the laptop's address on the hotspot changes,
rebuild from the tag with the new address; the checksum then changes.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | SYS-11 | `02c79e6` (image flashed 14:00, same firmware) | Dalton | FAIL (4 of 5 boots) | `logs/2026-11-20_system-dashboard.jsonl` (from 14:37) | Power bank only, off at least 20 s each time. Boots 1, 2, 3 and 5 reached the broker (first frames 14:37:00, 14:38:10, 14:39:29, 14:41:41): READY, fault none, STOP latched with cause start-up, steady for 18 to 30 frames; the heartbeat restarted at `hb 0` each time, so each was a real power cycle. No boot came up in FAULT (the 2026-11-17 failure). Boot 4, between 14:39:46 and 14:41:41, never connected: no frame or heartbeat within the 30 s the tester waited, although the code retries a WiFi join every 5 s (`cyw43_utk.c`) and MQTT every 3 s (`mqtt_utk.c`). No console on the power bank, so the stage it stopped at (join, DHCP or MQTT) and whether the car itself reached READY are unknown. Next: repeat, and on a boot that does not connect wait at least 90 s before powering off, to tell a slow join from a stuck one |
| 2026-11-20 | SYS-11 (repeat) | `02c79e6` (same image as the row above) | Dalton | PASS (5 of 5 boots) | `logs/2026-11-20_system-dashboard.jsonl` (boots from 14:44) | Power bank only. Heartbeat restarted at `hb 0` at 14:44:36, 14:46:39, 14:48:08, 14:49:17 and 14:50:18, each after 37 to 44 s of silence (off time plus connect time), so all five were cold starts. Every boot: READY, fault none, STOP latched with cause start-up, steady for 24 to 86 frames. Every boot connected, so the 90 s wait was not needed. Over both runs today 9 of 10 cold boots reached the broker. The boot 4 failure of the first run is not explained and may recur; it is not closed by this pass |
| 2026-11-20 | SYS-01 | `02c79e6` (same image as SYS-11) | Dalton | PASS (tester-observed; telemetry covers lap 1 and about 30 s of lap 2) | `logs/2026-11-20_system-dashboard.jsonl` (from 14:55:44) | Power bank, started with GP20 at 14:55:44 after two deliberate short starts (14:54:04, ended by a lift at 14:55:00; 14:55:29, ended by GP20 at 14:55:39). Laps by stopwatch: 2 min 15 s, 2 min 8 s, 2 min 11 s; tester saw no departure from the track and no stop; lifted after lap 3 and the car stopped. Telemetry: FOLLOW only and no stop from 14:55:44, 4459 mm by 14:58:00, then no frame or heartbeat after 14:58:30 while the car was driving (42 mm/s, 5311 mm), and none by 15:05. The car kept following without WiFi, as required, so laps 2-3 and the final pick-up stop are not in the log. The link loss is a SYS-08 finding (see the note below) |
| 2026-11-20 | — (observation: WiFi link lost while driving) | `02c79e6` | Dalton | OPEN | `logs/2026-11-20_system-dashboard.jsonl` | During SYS-01 the car stopped publishing at 14:58:30 and did not reconnect in the remaining ~4 min of driving, although the firmware rejoins WiFi every 5 s and retries MQTT every 3 s. The stats topic just before showed 275 of 280 periods sent, 3 refused (err -1), 2 without a snapshot. With SYS-11 boot 4 (no connection within 30 s) this is the second link failure today on the Pixel hotspot. Cause not captured: no console on the power bank. Not a SYS-01 failure (the car must keep following without WiFi); it threatens SYS-08 and SYS-09 |
| 2026-11-20 | — (WiFi link lost while driving: cause) | `02c79e6` | Dalton | CLOSED (test set-up) | as the observation above | The phone running the hotspot was carried out of the room by accident during SYS-01, so the car was out of range from 14:58:30; not a car or firmware fault. Closes the link-loss observation above. SYS-11 boot 4 (no connection within 30 s) has no such explanation and stays unexplained. Keep the hotspot phone in the room, screen on, for SYS-08 and SYS-09 |
| 2026-11-20 | SYS-10 | `02c79e6` (same image as SYS-11) | Dalton | FAIL (0 of 5 lifts detected as pick-up; the car still stopped each time) | `logs/2026-11-20_system-dashboard.jsonl` (boot at 15:31:40; lifts 15:32:04 to 15:34:57) | About 1.2 m of tape, power bank, started with GP20. Each lift: the wheels spun 1 to 2 s, then STOP latched with cause line, not pick-up. Telemetry shows why: in the air both front sensors read black (`BOTH`), the controller took it as a crossing and drove on until its line check stopped the run. The IMU lift detection never fired on this boot. On the previous boot (tester asked to ignore it) the same image detected three lifts as pick-up within about 1 s (15:30:28, 15:30:42, 15:31:04), as did INT-07 (14:16:51, 14:17:06). So detection depends on the boot. Subsystem 4 state is console-only, so the cause is not captured; candidates: boot calibration not completed (car handled at power-on) or an IMU read fault (I2C cable to the GY-511 disturbed). Next: repeat with the USB console captured (`s4: ... calibrat...`, `s4: car lifted`, `s4: IMU read failed`) |
| 2026-11-20 | SYS-10 (repeat) | `02c79e6` (same image) | Dalton | PASS (all lifts detected as pick-up) | `logs/2026-11-20_system-dashboard.jsonl` (boot at 15:38:36, lifts 15:39:50 to 15:41:41) | Same tape and procedure, new boot. Six lifts in the log, every one `STOP latched: pick-up` (15:39:50, 15:41:02, 15:41:19, 15:41:27, 15:41:32, 15:41:41); tester reports the 5 counted lifts, the last slow and level, all pick-up. Also in the session: one run stopped by a scan (15:39:39, an object at the end of the tape) and one by GP20 (15:41:09); neither is a lift. The console was not captured, so the cause of the failed boot in the row above is still unknown; it stays open. Until it is found, keep the car still for about 10 s after power-on (IMU calibration, S4 decision 9) before a run |
| 2026-11-20 | SYS-07 (first attempt) | `a55d032` (same image as SYS-10) | Dalton | FAIL (3 of 3 bypasses ended by a false pick-up) | `logs/2026-11-20_system-dashboard.jsonl` (copied again; runs 15:49:40 to 15:51:26) | Box 15 x 13 x 10 cm on a taped straight. Each run stopped in SCAN, planned a left bypass, then latched STOP with cause pick-up with no one touching the car: 15:49:52 (BYPASS), 15:50:34 (REACQUIRE), 15:51:26 (BYPASS). The bypass drives at about 130 mm/s with spins; the same runs reported false humps of 7 and 16 mm. Fixed by S4 decision 16 (no IMU detection in BYPASS or REACQUIRE). Contact and stop distances not recorded, so SYS-05 and SYS-06 have no result from these runs |
| 2026-11-20 | — (scan, box 23 cm wide facing the car) | `29bceff` (`NET=lwip MQTT=1 S3_NAV_F=0`) | Dalton | FAIL (no route on the first scan of every approach) | `logs/2026-11-20_system-dashboard-sys07.jsonl` (20:17:41 to 20:22:56) | Box 23 cm across the line, 15 cm deep, centred on a straight taped line. Every first scan of an approach ended `no-route` and latched STOP cause scan (5 of 5: 20:17:51, 20:20:54, 20:21:59, 20:22:48, and 20:21:05 a second scan); after GP20, a rescan from about the same place planned a left bypass 3 of 4 times, and all three bypasses rejoined the line. Nearest point 236..256 mm. At 250 mm a 23 cm box puts its edges at about +/-25 degrees, close to the limit of the +/-45 degree scan (S2 decision 22: a box wider than about 25 to 30 cm shows no edge). Not a SYS-05/06/07 result: the box is wider than the scan was designed for. One run (20:21:53) ended STOP cause line after reacquiring: the left line sensor read 206..268 over black, fixed by adjustment (S3 test log, `92abaf1`) |
| 2026-11-20 | SYS-05 | `29bceff` (`NET=lwip MQTT=1 S3_NAV_F=0`) | Dalton | PASS (3 of 3; run 1 at the 20 mm limit) | `logs/2026-11-20_system-dashboard-sys07.jsonl` (20:37:59 to 20:40:30) | Box turned: 15 cm across the line, 23 cm deep, centred on the line. Every run stopped in `SCAN` with no contact. Ruler, ultrasonic sensor front to box: 230, 240, 230 mm. Scan nearest point 250, 242, 247 mm (+20, +2, +17); forward range on entering SCAN 250, 256 (last frame before), 248 mm. Image built after S4 decision 16 and the S3-H01 sensor adjustment |
| 2026-11-20 | SYS-06 | `29bceff` | Dalton | FAIL (width and centre offset not reported) | as SYS-05 | Same runs. Every scan planned a bypass with side L and a nearest point within 20 mm of the ruler (see SYS-05). The shape-valid flag (flags1 W) never set, so width and centre offset were 0 in every frame: the box width (150 mm) and its offset (0, centred) cannot be compared. Known limit, S2 decision 22 |
| 2026-11-20 | SYS-07 | `29bceff` | Dalton | PASS (3 of 3) | as SYS-05 | Same runs. Each went `SCAN` → `BYPASS` (24..25 s) → `REACQUIRE` (7..8 s) → `FOLLOW` on the original line beyond the box, with no contact (tester). No pick-up or hump during any bypass or reacquisition (hump maximum 0 throughout), so S4 decision 16 fixes the first-attempt FAIL. Runs ended by GP20 (20:38:53, 20:40:30) and by a lift (20:39:45, `STOP latched: pick-up`; tester confirms the car was lifted) |
| 2026-11-20 | SYS-10 (after S4 decision 16) | `29bceff` (`NET=lwip MQTT=1 S3_NAV_F=0`) | Dalton | PASS (5 of 5 lifts detected as pick-up) | `logs/2026-11-20_system-dashboard-sys07.jsonl` (21:48:50 to 21:49:21) | Power bank, straight taped line, image flashed again after the left line sensor's AO wiring was fixed (S3 test log). Five lifts while following, each `STOP latched: pick-up`: 21:49:01, 21:49:06, 21:49:11, 21:49:16, 21:49:21 (the last slow and level, tester). No pick-up stop and no hump (hump maximum 0) while following. One scan at 21:48:55 (something briefly in front of the sensor) ended clear and following resumed. With the SYS-07 runs on the same image, decision 16 stops false pick-ups in the bypass without losing real lifts |
| 2026-11-23 | — (S3 line loop stops after MQTT connects, dual core) | `5416b39` + S1 decision 6, and `5416b39` alone (`SMP=1 NET=lwip MQTT=1 S3_NAV_F=0`) | Dalton | FAIL (4 of 6 boots) | `logs/2026-11-23_console.txt`, `logs/2026-11-23_system-dashboard-1.jsonl`, `-2.jsonl` | On the boots at 16:14, 16:24, 16:27 and 16:45 the `s3 line:` printout stopped within about 3 s of `mqtt: connected`, and every START ended `[S2] line: no report after START; STOP` (dashboard: STOP latched: line, within 1 s). The 16:45 boot ran the committed code without S1 decision 6, so that change is not the cause. No `ADC read failed` or `report delivery failed`. WiFi, MQTT and the controller kept running. On the 16:49 (diagnostic marker) and 16:52 boots the loop kept running. Addressed by S2 decision 33 |
| 2026-11-23 | — (S4 IMU task stops after calibration, dual core) | `5416b39` + S1 decision 6 (`SMP=1`) | Dalton | FAIL | `logs/2026-11-23_console.txt` (17:08:53 boot) | USB and power bank. `stationary calibration complete` at 17:08:54, then no `s4:` line: no `s4: new run` at START, and a lift ended STOP latched: line, not pick-up. No `IMU read failed`. Addressed by S2 decision 33 |
| 2026-11-23 | INT (barcode letters over MQTT, S1 decision 6) | `5416b39` + S1 decision 6 (`SMP=1`) | Dalton | PASS (1 junction) | `logs/2026-11-23_system-dashboard-2.jsonl` (17:01 to 17:03) | Dashboard table sent LEFT F, RIGHT B, STRAIGHT C, UTURN D. After the car restarted at 17:02:17 the dashboard re-sent it (`car restarted`, `barcode letters re-sent after restart`). Telemetry: `barcode F decoded: LEFT`, then `NAVIGATE (LEFT)` and back to `FOLLOW` 4 s later. Earlier, with the boot map, F decoded as `none`, as expected |
| 2026-11-23 | SYS-10 (single core, first check) | `5416b39` + S1 decision 6, `SMP=0`, with the temporary S3 stage marker | Dalton | PASS (1 lift; SYS-10 needs 5) | `logs/2026-11-23_console.txt` (17:12:35 boot), `logs/2026-11-23_system-dashboard-2.jsonl` | `s4: new run 1` at START (17:13:15), `s4: car lifted; STOP requested` (17:13:19), dashboard STOP latched: pick-up. Next run: `s4: new run 2`, over 30 s of following, `s4: hump 11 mm under the right wheel` (hump not measured). One `s4: terrain fault 3; sensor restart` at boot, then calibrated. SYS-10 and the other SYS cases must be rerun on the committed single-core image (S2 decision 33) |
| 2026-11-24 | — (practice before the assessed runs) | `4a53c23` rebuilt for the hotspot broker (version 3 frames) | Team | OBSERVATION | `logs/2026-11-24_demo-dashboard-1.jsonl` (13:14 to 14:06) | Barcodes A (LEFT), Q, Z and F decoded; F at 13:42:33 (line 2512) followed by `NAVIGATE (UTURN)` and back to `FOLLOW` (lines 2526, 2540). Left and right bypasses back to `FOLLOW`; one `STOP latched: bypass`, motion faults at 13:18:12, 13:41:22 and 13:59:26. Hump 10 mm twice (lines 758, 1139; hump not measured). Shape-valid widths 315, 327, 219, 220 mm for the 15 x 20 cm box: not accurate |
| 2026-11-24 | SYS-04 (assessed demo, 3 attempts) | `a41c19c` (tag `demo-2026-11-24-flashed`; `NET=lwip MQTT=1 S3_NAV_F=0`, UF2 `8cf2c419…`) | Team | PARTIAL (height within 2 mm on 3 of 3; flat lap not run) | `logs/2026-11-24_demo-dashboard-2.jsonl` lines 1191, 2554 (frame 2556), 3138 (frame 3140) | Hump 5 mm (team ruler). Readings 6, 6, 7 mm: errors +1, +1, +2 mm. The design cannot report below 6 mm (`S4_RISE_UM` 6000 um, rounding to whole mm); see the demo report D-03 |
| 2026-11-24 | SYS-02 (assessed demo) | `a41c19c` (same image) | Team | FAIL (1 of 3 attempts decoded) | as above, lines 2800, 3100, 3303, 3390, 3538 | Course barcode O. Not decoded in attempts 1 and 2; decoded as RIGHT in attempt 3 (14:51:46, 14:54:57) and on three later short runs. No wrong letter. Letter table had O under STRAIGHT until 14:48:34 (lines 1026, 2487); O read as STRAIGHT twice between attempts (lines 1601, 2390) |
| 2026-11-24 | SYS-03 (assessed demo) | `a41c19c` (same image) | Team | FAIL (no turn started) | as above | RIGHT decoded and held, but no `NAVIGATE` in the whole session. Junction and hold messages are console-only and the console was not captured, so the cause is not known. Two repeat runs ended by a pick-up 3 to 5 s after the decode |
| 2026-11-24 | SYS-05, SYS-07 (assessed demo) | `a41c19c` (same image) | Team | PARTIAL (attempt 3 passed; 1 and 2 did not) | as above, lines 1279, 2615, 2926, 3010, 3240 | Attempt 1: left bypass, `STOP latched: bypass` in REACQUIRE. Attempt 2: `FAULT: scan`. Attempt 3: three left bypasses back to `FOLLOW` with no contact (team), car lifted once and restarted. Avoidance marks received (team) |
| 2026-11-24 | SYS-06 (assessed demo) | `a41c19c` (same image) | Team | FAIL | as above | Shape never valid; raw width (edge missing) 166 to 332 mm for the 15 x 20 cm box |
| 2026-11-24 | SYS-08 (assessed demo) | `a41c19c` (same image) | Team | OBSERVATION (frames throughout; rate not counted) | as above | Version 4 frames and heartbeats from 14:19:50 to 15:03:50; dashboard events match the run |
| 2026-11-24 | SYS-09 (assessed demo) | `a41c19c` (same image) | Team | FAIL | as above | Full course not completed in one run: barcode turn not executed |
| 2026-11-25 | SYS-08 (analysis of the 2026-11-24 assessed session log) | `a41c19c` (demo image) | Dalton | PASS (98.6% of periods received) | `logs/2026-11-24_demo-dashboard-2.jsonl` (stats line at 15:03:50) | No new run. From the last car restart (heartbeat back to `hb 0`, 14:32:25) to 15:03:50 the car counted 1890 periods, 1867 sent, 23 refused (err -1); the dashboard received 1863 frames (98.6%, criterion 95%). Heartbeat `hb 377` at 15:03:50 matches 5 s spacing. The dashboard records each heartbeat twice (748 lines from 14:32:26 to 15:03:50 for about 375 heartbeats); a logging duplicate, not a car fault |
| 2026-11-25 | — (single-core check, S2 decision 33, from the 2026-11-24 logs) | `4a53c23` rebuild (13:14 to 14:06) and `a41c19c` (14:17 to 15:03), both `SMP=0` | Dalton | OBSERVATION (no line-loop stall seen) | `logs/2026-11-24_demo-dashboard-1.jsonl`, `-2.jsonl` | No new run. About 100 min of single-core running with many STARTs: no `STOP latched: line` (the symptom of the 2026-11-23 dual-core stall), 15 pick-up stops (the S4 task kept running). Four motion faults (13:18:12, 13:41:22, 13:59:26, 14:43:33) and one scan fault (14:49:54); their detail is console-only. Not a formal re-run of SYS-01 to SYS-11 on one core |

## 24 November 2026 — assessed demonstration

Full account: [demo report](../../docs/DEMO_REPORT_2026-11-24.md).

**Image.** The assessed runs used the telemetry version 4 image built at
14:17 (S1 decision 7), not the `4a53c23` image recorded above as the demo
image: the assessed session log carries version 4 frames. Its source is
`a41c19c`, tag `demo-2026-11-24-flashed`; a clean rebuild (`make -B`,
2026-11-25) gives the same UF2 SHA-256,
`8cf2c419ae181224b0c3c7ec78f7bf651e6c17350860a7ce3198b198f416bf3f`. The
practice session (13:14 to 14:06) sent version 3 frames, consistent with
`4a53c23` rebuilt for the hotspot's broker address.

**Logs.** Both dashboard logs were copied here unchanged on the evening of
24 November. SHA-256 at that time:

| File | SHA-256 |
|---|---|
| `logs/2026-11-24_demo-dashboard-1.jsonl` | `2d8456242367b7f011a81f1df6d5eb6ce1ab495fd6c094fef64c803c0bf2cf5c` |
| `logs/2026-11-24_demo-dashboard-2.jsonl` | `4e45008633ee0749180ffca8f17bb0391597f1152dd417d43d95cb0a3f5b3441` |

**Attempts** (laptop time): 1 from 14:34:01, 2 from 14:48:36, 3 from
14:51:32 to 14:56:30. The USB console was not captured during the
assessment.

**Marks** (team record): hump 0.5 for attempt 1 and 0.5 for attempt 2;
attempt 3 (7 mm) was not given the 0.5 mark after a verbal miscommunication;
the request for reconsideration is in the demo report, D-03. Obstacle
avoidance marks received in attempt 3.

**Video.** Backup evidence recorded a few hours before the demo:
<https://www.youtube.com/watch?v=B1mWqMT2HIk> (see `videos/README.md`). It
shows earlier working behaviour, not the assessed attempts.

The [host review](../host/HOST_REVIEW_2026-11-24.md) passed all five suites
on the demonstrated source. Host tests do not replace these physical results.

## Post-demo firmware, 2026-11-26: car tests still to run

The changes made after the demo (review `docs/software_quality_review.md`
section 9; commits `2fddf92` to `134de47` and the final consolidation) are
verified on the host only ([log](../host/logs/2026-11-26_verify-final.txt):
every suite passes, 11 builds with 0 warnings, mission UF2 SHA-256
`279108eb318dbe078a658558ffd4ba12057102683a58e84ccb094def98f8d245`). None
has run on the car. Before the post-demo image is used, run the full set
above on it with its commit in every row. These parts need particular attention:

| Change | Car test | What to record |
|---|---|---|
| F-01: S3 loop on a 10 ms cyclic handler (S3 d10) | SYS-01, SYS-02 | The `s3 loop:` line every 10 s: period 10 ms, worst pass, overruns, late wakes |
| F-18: 31-run barcode repair (S3 d9) | SYS-02 | No wrong letter; captures that the demo image rejected now decode |
| F-03: held command with no time limit (S2 d36) | SYS-03, and one run with an obstacle stop before the junction | The command is still held after the stop (telemetry `held`) and runs at the junction |
| F-04: hump detection on the uncalibrated height, FOLLOW only (S4 d17) | SYS-04 with ruler heights | Each hump detected; then fit the scale and offset with `testing/subsystem4/fit_hump_calibration.py` (blocked until these measurements exist) |
| F-08: liveness guard (S2 d37) | SYS-09 with no `[S2] health:` line; then one `S2_WATCHDOG=1` run | No false trip on healthy tasks; the watchdog stays off by default until this passes |
| F-06: telemetry version 5 (S1 d8) | SYS-08 | Fault detail, held command and junction count match the run |
| F-24: task stack high-water marks | Any full run | The `stack` lines at the run summary; at least 25% of each stack never used |

