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
| S3-U03 | line tracking: table, filter, centred both-white, generic loss, fault, barcode bit ignored | FR2 | auto (host) | `testing/subsystem3/test_track_scan.c` | 40 checks pass (with S3-U04) |
| S3-U04 | barcode capture: standard Code 39 A--Z lookup, every letter forwards/reversed, speed, bounded terminal-sentinel repair, ambiguous payload rejection, contrast, single bar, overflow | FR3 | auto (host) | `test_track_scan.c` | host suite passes |
| S3-H01 | IR polarity per sensor on white and black | S3_IR_BLACK_LEVELS | manual | print raw GPIO, white then black, each sensor | 1 = black confirmed or config changed |
| S3-H02 | sensor spacing against line width; steering truth table | straddle table | manual | ruler; slide line under the sensors | table matches geometry |
| S3-H03 | following straight and curved track | FR2 | manual | robot on track, logs | stays on line |
| S3-H04 | printed barcodes decode at speed | FR3 | manual | each code ×10 | correct symbol each pass |
| S3-H05 | side-barcode ADC2 classification after Grove 4/7 swap | decision 5, S3 ADC calibration | manual | final mounted height; log at least 50 samples each on white, printed black and faded black; then cross each code at controlled speed | white and both black surfaces separate with margin; no contact with paper; threshold remains between observed classes; no missed narrow run at 10 ms sampling |
| S3-H06 | supplied Z diagnostic sample | decision 6, FR3 capture | manual | run the controlled 40 mm/s diagnostic over Z at least 10 times in each direction | every complete frame has 29 runs and decodes Z; no false navigation command |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-03 | base logic (656 checks), S3-U03, S3-U04 | `ebb59a5` | host (gcc) | PASS | `../integration/logs/2026-11-03_host-suites.txt` | host only |
| 2026-11-04 | S3-U02 | `1573b62` | Dalton | PASS | `../integration/logs/2026-11-04_target-selftest.txt` | `TEST=1` on the Pico W |
| 2026-11-04 | S3-H01 | `1573b62` | Dalton | FAIL | observed on Robo Pico Grove status LEDs | All three high on black, low on white (polarity 0x07 holds). Left and barcode DO/AO were swapped at the module: DO arrived on GP17/GP27, firmware reads GP16/GP26. On these cables white = Grove first pin, yellow = second. Rewired to the pin map (DO on white: GP16, GP6, GP26; AO on yellow: GP17, GP27; front-right AO unconnected); barcode DO on GP26 confirmed by pot test. Re-test pending: pot test on GP16 and GP6, four-row crosstalk table (GP26 was seen lit with front-right on black), Grove 5 GP26 wire insulated |
| 2026-11-04 | S3-H02 | `1573b62` | Dalton | INCOMPLETE | measurement notes | Sensor spacing 21 mm (centre-to-centre not confirmed), line width 18 mm: both front sensors on white when centred, the straddle case `s3_track.c` assumes; about 1.5 mm margin per side. Risk: LOST after 1.5 s centred on a long straight, check in S3-H03. Line slide-through table not yet recorded |
| 2026-11-04 | S3-U02 | `1cd5330` | Dalton | PASS | `../subsystem4/logs/2026-11-04_s4-h01-4.txt` | `TEST=1 S4_HW_TEST=1` on the Pico W |
| 2026-11-04 | S3-H01 | `eca059e` | Dalton | FAIL | observed on Robo Pico Grove status LEDs | Robot held in the air: GP6 (front-right DO), GP26 (barcode DO), GP17 and GP27 (AO) lit; GP16 (front-left DO) not lit. On the table (white): none lit. The front-left module's AO shows dark but its DO stays low: its comparator threshold differs from the other two. Effect: in the air the line logic sees black under the right sensor only (`S3_LINE_RIGHT`), a valid line, so START was accepted with the robot lifted (`../subsystem1/TEST_LOG.md`, S1-H01 2026-11-04). Next: adjust the front-left pot, re-test in the air, on white and on the line. Open design point for S3/S2: with all three matched, lifting reads both black (junction), which is valid for up to 2 s, and the line fault latches until the line task restarts |
| 2026-11-04 | S3-H01 | `eca059e` | Dalton | PASS | observed on Robo Pico Grove status LEDs | After adjusting the front-left and barcode pots. Each sensor on black tape: its DO LED lit (GP16, GP6, GP26); on white: off. A first check found the barcode DO (GP26) not switching on tape while its AO (GP27) showed dark; its pot was adjusted. Robot in the air: both front DOs lit (both black), as expected with no reflection. Polarity 0x07 (high = black) holds for all three. Crosstalk table from the earlier row not recorded |
| 2026-11-04 | S3-H03 (straight) | `48e8c91`, `NET=lwip MQTT=1`, power bank on the Pico W USB | Dalton | FAIL (setup) | `logs/2026-11-04_s3-h03-1.txt`, `logs/2026-11-04_s3-h03-commands-1.txt` | Straight tape line, robot on the floor. Hotspot "Edmund" joined but gave no IP until it was toggled off and on (console: "joined AP, waiting for IP" for 55 s). A lowercase `start` (17:27:15) is not a command. START at 17:39:13 (27 s after the power-bank boot), 17:40:58 and 17:41:32 (after a power cycle, only GP6 lit) all refused: telemetry stayed stopped with no run distance. No console on the power bank, so the refused check is not known |
| 2026-11-04 | S3-H03 (straight) | `48e8c91` + `S2_DEMO_LOG` 1 (local, not committed), on USB to the Mac | Dalton | FAIL | `logs/2026-11-04_s3-h03-console-1.txt`, `logs/2026-11-04_s3-h03-commands-1.txt` | Before START: Ready, fault 0, line 0x11 (GP6 on tape), age 10 ms. Run 1 (START 17:43:31): followed with line valid (0x11, then 0x01 centred) to 251 mm, wheels stopped while Running, then scan fault (fault 3, S5 `CTRL_FLG_SCAN_FAULT`) and Fault state; line LOST (0x02) while standing centred. Stop-then-scan matches the forward monitor treating no echo as invalid (`docs/integration.md` section 5); what was ahead of the sensor was not recorded, and the HC-SR04 is still on 3V3. Runs 2 to 5 (each after RESET, 17:45:09 to 17:46:23): Running, then Stopped with STOP latched and fault 0 at 320, 282, 290 and 296 mm; no STOP in the command log. Cause not identified: a STOP flag (remote or on-board) or a hazard outside line following both latch STOP; mode was not logged. Stays on the line over about 0.3 m only; the 1.5 s LOST risk (S3-H02) was seen while standing, not while moving |
| 2026-11-10 | S3-U02 | `76db3d8` | Dalton | PASS | `../integration/logs/2026-11-10_target-selftest.txt` | `TEST=1` on the Pico W; image built from `893f4dc` (code identical at `76db3d8`); first on-target run after the analogue line-sensor remap and the S5 changes of 2026-11-08; console dropped 0 bytes. Checks IF-06 consistency only, not the analogue sensor values (S3-H01 re-test still needed) |
| 2026-11-12 | S3-H06 smaller Z, normal direction, three passes | working tree | Edmund | PARTIAL; decoder update complete, hardware retest pending | Three pasted COM4 logs: one complete 29-run frame and two 27-run observer frames | Complete frame: sentinel `NWNNWNWNN`, gap `N`, payload `NWWNWNNNN` (`0x0D0`), gap `N`, sentinel `NWNNWNWNN`. Added `0x0D0` as a second Z entry; retained `0x1C0` for the working larger print. Incomplete frames remain rejected. Host tests: 790 checks, 0 failures. Diagnostic firmware compiled and linked; hardware evidence is still required. |
| 2026-11-12 | S3-H06 smaller Z, opposite direction, three passes | working tree | Edmund | PARTIAL PASS, 3/3 reverse passes | Three pasted COM4 logs | All frames had 29 runs and produced `DECIPHERED=Z`; successful-scan counters advanced through 4, 5 and 6. Ten passes per direction remain required for S3-H06 completion. |
| 2026-11-12 | S3-U04 bounded partial-frame fallback | working tree | host (gcc) | PASS | Exact 25-run failed capture replayed forwards and reversed | Scanner suite: 47 checks, 0 failures. Base suite: 790 checks, 0 failures. The observed 25-run Z decodes in both orientations; a 23-run capture is rejected. Firmware compiled and linked, then converted to the `_s3bardrive.uf2` image. |
| 2026-11-12 | S3-H06 smallest Z, normal direction, three passes | working tree | Edmund | FAIL; threshold recalibrated, retest pending | Three pasted COM4 logs | Each pass split into a 7-run fragment and a 3-run fragment; no frame approached the 24-run fallback floor. At about 40 mm/s, detected runs were 120..160 ms and 520 ms. ADC values still classified white reached 491..523 while ordinary moving white remained at or below 352 and the threshold was 554..584. Reduced only the barcode black/release margins to 250/180, giving thresholds 404..434 and release 334..364. |
| 2026-11-12 | S3-H06 smallest Z after threshold recalibration | working tree | Edmund | INTERMITTENT; diagnostic speed revision pending retest | Three pasted COM4 logs | One pass captured all 29 runs and decoded Z; another decoded while the observer recorded 21 runs; a failed pass recorded only 7- and 3-run fragments. Successful thin-black runs were only 20 ms at 40 mm/s, while the failed frame retained the 120..140 ms thick bars and merged the missing thin bars into a 540 ms white run. Added a diagnostic-only 25 mm/s line-follow profile and scaled diagnostic run/quiet limits to 1400/1200 ms; normal mission tuning remains 40 mm/s and 800/700 ms. |
| 2026-11-12 | S3-H06 smallest Z at 25 mm/s | working tree | Edmund | DEFERRED | One pasted COM4 log | Slowing the diagnostic did not solve the smallest print: only 11/29 runs were captured, with missed thin bars merged into 840 ms and 1200 ms white gaps. This points to the sensor/print optical geometry rather than decoder strictness. Reverted the diagnostic to the known-good 40 mm/s and 800/700 ms framing; retain both working Z variants and capture the remaining supplied letters before revisiting this extreme print. |
| 2026-11-12 | S3-H06 restore proven Z profile | working tree | Edmund | CODE RESTORED; hardware regression pending | Prior biggest and three-quarter Z evidence | Restored the complete known-good profile: 40 mm/s diagnostic speed, 800/700 ms framing and barcode margins 400/300 (black threshold 554..584, release 454..484). Retained Z patterns `0x1C0` and `0x0D0`, reverse decoding and the bounded partial-frame fallback. Smallest-print sensitivity and slow-speed experiments are not retained. |
| 2026-11-12 | S3-U04 standard A--Z table | working tree | host (gcc) | PASS | Base suite: 794 checks; scanner suite: 93 checks; 0 failures | Replaced the example-only dictionary with the standard Code 39 A--Z lookup. `0x1C0` is now correctly W; `0x0D0` is Z. Each letter is exercised forwards and backwards. |
