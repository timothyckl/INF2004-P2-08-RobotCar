# Subsystem 1 — test log

**Owner:** Hong Yih (2501110)  ·  **Planned subsystem:** WiFi/MQTT communication  ·  **Code:** `app_program/subsystem1/`

S1-U01–S1-U03 tested the LED starter example, which was retired when
subsystem 1 was merged into this branch (S1 decision 2); they are kept so
the IDs are never reused. S1-U04–S1-U07 test the hardware-independent
communication logic. S1-H01–S1-H03 need the robot, WiFi and a broker.

Earlier runs of S1-U04–U07 and S1-H01/H02 (telemetry version 1, 2026-10-27
to 2026-11-10) are recorded with their logs on
`feature/full-system-integration`. They are not results for this branch.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | *retired:* cyclic handler is running | LED starter | — | — | — |
| S1-U02 | *retired:* LED task sleeps between ticks | LED starter | — | — | — |
| S1-U03 | *retired:* LED on GP19 toggles every 250 ms | LED starter | — | — | — |
| S1-U04 | Bounded command parsing | Command syntax | auto / host | `comms_test_commands()`; valid tokens, malformed input, nulls, explicit lengths and oversized input | All cases pass; unsupported bytes rejected |
| S1-U05 | Binary telemetry encoding, version 2 (revised 2026-11-17, S1 decision 1) | Telemetry v2 | auto / host | `comms_test_telemetry()`; 27-byte golden frame with every field set, every undersized buffer, NULL arguments, every validity flag clear, single flags on their bits, codes above 14 sent as 15, signed and unsigned extremes | Exact bytes; guarded fields zero when their flag is clear; no overwrite on failure |
| S1-U06 | Heartbeat and retries | Isolated scheduling logic | auto / host | `comms_test_timers()`; initialisation bounds, pending attempts, retry deadlines, reconnect, clock wrap and delayed polling | Correct actions without busy retries or catch-up bursts |
| S1-U07 | MQTT service bridge | Service-task integration boundary | auto / host | Fragmentation, exact topic, malformed lengths, STOP routing, regular routing, failed delivery, telemetry provider and counters | STOP never uses regular path; invalid input rejected; failures reported |
| S1-H01 | MQTT START/STOP/RESET reach the controller | IF-08, IF-09 | manual | `NET=lwip MQTT=1`; publish to the command topic | STOP latches; START only from Ready |
| S1-H02 | telemetry published every second (revised 2026-11-17 for version 2) | IF-10 | manual | subscribe to the telemetry topic; decode with the v2 layout in `telemetry_encoder.h` | 27-byte version 2 frames, 1 s apart; fields match the serial log |
| S1-H03 | WiFi/broker loss does not move the robot | safety | manual | drop WiFi while stopped and while running | no START; STOP still works |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-17 | S1-U04..U07 | working tree after `2c40262` (telemetry v2) | host (cc) | PASS (4/4) | `make -C testing/host all` | host only; `-Wconversion -Wsign-conversion -Werror`. Not yet run on the Pico W |
| 2026-11-17 | S1-H02 (still) | `4e4b3a5`, `NET=lwip MQTT=1`; local `wifi_secrets.h`: hotspot Pixel_1232, broker 10.126.176.224 (Mac, mosquitto) | Dalton | PASS (stationary) | `logs/2026-11-17_s1-h02-telemetry.txt`, `logs/2026-11-17_s1-h01-console.txt` | Joined on the first attempt (ip 10.126.176.101), MQTT connected and subscribed. 42 frames, all 27 bytes, version 2, none malformed; robot-time spacing 1000 ms in all 41 gaps. Flags stopped, link up, STOP latched, motion valid, forward valid; READY, no fault, distance invalid (no run), speeds 0, forward 377 mm. Wheels not turned: the version 1 frame loss while driving (2026-11-04, `feature/full-system-integration`) is not yet re-checked |
| 2026-11-17 | S1-H01 (STOP only) | as above | Dalton | PARTIAL | broker log (mosquitto `-v`, not saved) | `STOP` published at QoS 1 reached the car and was acknowledged (PUBACK). STOP was already latched in Ready, so no state change could be seen; START and RESET not sent (START may move the car) |
| 2026-11-17 | S1-H01 (RESET in Fault) | `48bbd94` line image (`NET=lwip MQTT=1 TEST=1 S3_LINE_TEST=1`), Pixel_1232, battery (no USB console) | Dalton | FAIL | `logs/2026-11-17_s1-h02-telemetry.txt` (FAULT frames from t=3355777216) | After a restart the controller was in Fault (scan fault, cause not captured: no console). `RESET` was acknowledged by MQTT but the state stayed FAULT. Cause: `received_ms` stamped with lwIP `sys_now()` (operating time) while the controller ages commands on `tk_get_tim`, so every START/RESET expired. Fixed in `42e1797` (S1 decision 3) |
| 2026-11-17 | S1-H01 (START, STOP while running) | `42e1797` line image, Pixel_1232, USB console | Dalton | PASS (START); STOP delivered late | `logs/2026-11-17_s1-h01-fix-console.txt`, `logs/2026-11-17_s1-h02-telemetry.txt` | Remote `START` in Ready: RUNNING/FOLLOW within one frame, line following on the track. Remote `STOP` after about 140 s: the broker sent it at once but the car's PUBACK came 11 s later, inside a 30 s telemetry blackout with `mqtt: publish failed, err=-3` on the console; the car then reported STOPPED, STOP latched, wheels 0. Line following itself continued normally throughout (console). Remote STOP is not a reliable emergency stop over this link; GP20 and pick-up stop remain the local stops |
| 2026-11-17 | S1-H02 (driving) | as above | Dalton | FAIL (frame loss) | as above | 140 s run, 4777 mm, curves both ways and the 10 mm hump. 76 frames received of about 141 sent at 1 s: 52 gaps of about 1 s, 24 longer (2 to 4 s), one of 30 s. Only 4 publish failures were printed, so most frames were dropped silently (unchecked `mqtt_publish()` return or a skipped snapshot). Stationary frames before and after: every gap 1 s. Same symptom as version 1 on 2026-11-04. Content correct when received: RUNNING/FOLLOW, line LEFT/RIGHT/CENTRE with line-fresh set, wheel speeds 38 to 47 mm/s, heading, distance rising, hump 10 mm valid from the hump on, STOPPED after STOP |
