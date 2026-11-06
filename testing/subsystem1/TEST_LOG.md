# Subsystem 1 — test log

**Owner:** Hong Yih (2501110)  ·  **Planned subsystem:** WiFi/MQTT communication  ·  **Code:** `app_program/subsystem1/`

S1-U01–S1-U03 test the LED starter example. S1-U04–S1-U07 test the
hardware-independent communication logic using provisional message formats.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-U04 | Bounded command parsing | Command syntax, provisional contract | auto / host | `comms_test_commands()`; valid tokens, malformed input, nulls, explicit lengths and oversized input | All cases pass; unsupported bytes rejected |
| S1-U05 | Binary telemetry encoding | Provisional codec v1 | auto / host | `comms_test_telemetry()`; golden bytes, each flag, integer limits, valid zero, invalid distance, nulls and every undersized buffer | Exact bytes; no overwrite on failure |
| S1-U06 | Heartbeat and retries | Isolated scheduling logic | auto / host | `comms_test_timers()`; initialisation bounds, pending attempts, retry deadlines, reconnect, clock wrap and delayed polling | Correct actions without busy retries or catch-up bursts |
| S1-U07 | MQTT service bridge | Planned service-task integration boundary | auto / host | Fragmentation, exact topic, malformed lengths, STOP routing, regular routing, failed delivery, telemetry provider and counters | STOP never uses regular path; invalid input rejected; failures reported |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

### Communication logic run — 2026-10-27

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S1-U04–S1-U06 | Working changes based on `f5a0213` | Codex | PASS, host only | [Host output](logs/2026-10-27_logic-host.txt) | MSVC C11 mode compiling C99-compatible code, `/W4 /WX /analyze`; three groups passed |

The four new C modules also compile for Cortex-M0+ with ARM GCC 10.3.1,
`-std=c99 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow
-Wstrict-prototypes -pedantic`, with tests enabled. The disabled-test module
also compiles with `APP_TEST=0`. This is compilation evidence, not execution
on the Pico W. No complete firmware build, RTOS/concurrency, live MQTT,
STOP timing or hardware validation was performed in this run.

Reproduce the host run with `run_host.ps1` as documented in
`app_program/subsystem1/README.md`. Record on-target results separately.

### Service bridge run — 2026-10-29

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-29 | S1-U04–S1-U07 | Working changes based on `75a488f` | Codex | PASS, host only | [Host output](logs/2026-10-29_service-bridge-host.txt) | MSVC C11 mode, `/W4 /WX /analyze`; four groups passed |

The new bridge and the existing logic modules also pass ARM GCC 10.3.1
syntax checking for Cortex-M0+ with strict C99 warnings. No full firmware,
Pico W, broker, RTOS concurrency or physical STOP timing was exercised.

## Integration additions (2026-11-03, feature/full-system-integration)

Test IDs never change. Host runs (`make -C testing/host`) use synthetic input
on a computer; they are not on-target or hardware results.

Retired with the starter LED / IF-05 compatibility task: S1-U01, S1-U02,
S1-U03.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-H01 | MQTT START/STOP/RESET reach the controller | IF-08, IF-09 | manual | `NET=lwip MQTT=1`; publish to the command topic | STOP latches; START only from Ready |
| S1-H02 | telemetry published every second | IF-10 | manual | subscribe to the telemetry topic | 8-byte frames, 1 s apart |
| S1-H03 | WiFi/broker loss does not move the robot | safety | manual | drop WiFi while stopped and while running | no START; STOP still works |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-03 | S1-U04..U07 | `ebb59a5` | host (gcc) | PASS | `../integration/logs/2026-11-03_host-suites.txt` | host only |
| 2026-11-04 | S1-U04..U07 | `1573b62` | Dalton | PASS (4/4) | `../integration/logs/2026-11-04_target-selftest.txt` | `TEST=1` on the Pico W |
| 2026-11-04 | S1-U04..U07 | `1cd5330` | Dalton | PASS (4/4) | `../subsystem4/logs/2026-11-04_s4-h01-4.txt` | `TEST=1 S4_HW_TEST=1` on the Pico W; console dropped 0 bytes |
| 2026-11-04 | S1-H01 | `aa9979f` (image `NET=lwip MQTT=1` built 14:19, commit inferred from build time) | Dalton | FAIL | `logs/2026-11-04_s1-mqtt-attempt1-wifi-fail.txt` | WiFi joined the Pixel_1232 hotspot (ip 10.214.80.101), then every MQTT connect returned status 256 and retried. On a later boot in the same capture the join failed once with "SSID not found" before succeeding. No command reached the controller |
| 2026-11-04 | S1-H01 | — | Dalton | INVALID | `logs/2026-11-04_s1-mqtt-attempt2-foreign-firmware.txt` | Board was running a different image (S3/S2 bounded line-follow test mode), not this branch's MQTT build. Not a result for S1-H01 |
| 2026-11-04 | S1-H01 | `aa9979f` (as above) | Dalton | FAIL | `logs/2026-11-04_s1-mqtt.txt` | WiFi link up, ip 10.214.80.101; broker configured at 10.214.80.224:1883. Every connect attempt returned status 256 (lwIP `MQTT_CONNECT_DISCONNECTED`: the TCP connection to the broker failed or closed, not a broker refusal). Controller stayed Ready with STOP latched throughout. Check the broker listens on the LAN, not only localhost, and the host firewall allows 1883 |
| 2026-11-04 | S1-H02 | — | Dalton | NOT RUN | `logs/2026-11-04_s1-telemetry-attempt1.txt`, `logs/2026-11-04_s1-telemetry.txt` | Both subscriber captures are empty: no broker connection, so no telemetry was published |

Note, 2026-11-04: the earlier status 256 failures had two causes on the
broker laptop, not in the firmware: it was not on the robot's network, and
no broker was running (mosquitto 2.x without a config listens on localhost
only). The bench broker now runs with `listener 1883 0.0.0.0` and
`allow_anonymous true`, on the private hotspot only. The Pico W radio is
2.4 GHz only (Pico W datasheet), so the hotspot must offer 2.4 GHz.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-04 | S1-H01 | `eca059e`, `NET=lwip MQTT=1` | Dalton | FAIL | `logs/2026-11-04_s1-mqtt-2.txt` | Pixel_1232 hotspot had moved to 5 GHz (laptop on channel 44): every join "SSID not found". Not a firmware fault. Changed to an iPhone hotspot on 2.4 GHz (channel 6) |
| 2026-11-04 | S1-H01 | `eca059e`, `NET=lwip MQTT=1`; local `wifi_secrets.h`: iPhone hotspot, broker 172.20.10.3 | Dalton | PARTIAL | `logs/2026-11-04_s1-mqtt-3.txt`, `logs/2026-11-04_s1-telemetry-3.txt` | Joined (ip 172.20.10.2), connected and subscribed. START, STOP, RESET, START sent by `mosquitto_pub`; telemetry shows each took effect: START ran, STOP stopped the wheels and held them still for 11 s, RESET then START began a new run. Not shown: START refused outside Ready (no START sent while Stopped), and the controller states, because the console capture was stopped early (ends at "hb 11", before the commands). **Finding:** START was accepted with the robot held in the air; S2-U60 expects START refused without a fresh valid line. Cause not yet found. Robot still driving (wheels in the air) when the capture ended |
| 2026-11-04 | S1-H02 | as above | Dalton | FAIL | `logs/2026-11-04_s1-telemetry-3.txt` | 45 frames, all 8 bytes, version 1, flags consistent with the commands (still/moving, distance valid after START). While stopped: robot time steps exactly 1000 ms, no frame lost, but broker arrival bunched (3 frames at 15:49:51). While the wheels turned: frames lost, robot-time gaps of 8, 2, 2, 5, 2, 2 and 2 s (about 16 frames missing). Link-up flag (0x04) never set |
| 2026-11-04 | S1-H01 | `f20ffba`, `NET=lwip MQTT=1` | Dalton | FAIL (setup) | `logs/2026-11-04_s1-mqtt-4.txt`, `logs/2026-11-04_s1-telemetry-4.txt` | Image without controller state output. No START accepted (telemetry: no run in 240 frames); commands not logged. Superseded by runs 5 and 6 |
| 2026-11-04 | S1-H01 | `f20ffba` + `S2_DEMO_LOG` 1 (local, not committed) | Dalton | FAIL (setup) | `logs/2026-11-04_s1-mqtt-5.txt`, `logs/2026-11-04_s1-telemetry-5.txt`, `logs/2026-11-04_s1-commands-5.txt` | Commands arrive (STOP at 16:19:24 logged as latch to motor off 705 us), but every START refused in Ready: run 6 showed the line fault was latched (see below) |
| 2026-11-04 | — (diagnostic) | `f20ffba` + line fields in the `S2_DEMO_LOG` line | Dalton | — | `logs/2026-11-04_s1-mqtt-6.txt` | Line flags with the robot on a prop: one front sensor on tape 0x11 (valid, seen); both on tape 0x15 (junction) then 0x1c after 2 s (fault); fault then stays set on white, one side or both (0x08, 0x0a, 0x18). The S3 line fault is cleared only when the line task starts (`s3_track.c`), so after 2 s of both black START is refused until reboot. Also: line valid (0x01) at boot before any black is seen. Raised with subsystem 3 |
| 2026-11-04 | S1-H01 | `f20ffba` + line logging; local `wifi_secrets.h`: hotspot "Edmund", broker 172.20.10.4 | Dalton | PASS | `logs/2026-11-04_s1-telemetry-7.txt`, `logs/2026-11-04_s1-commands-7.txt`; tester observation | Robot on a prop, wheels free, tape under one front sensor. Telemetry against the command times: START 17:07:49 began a run (moving); STOP 17:08:03 stopped the wheels, distance held; START 17:08:16 refused (Stopped); RESET 17:08:27; START 17:08:38 with the robot lifted refused. Tester confirms the wheels ran and stopped as listed. The console capture (`logs/2026-11-04_s1-mqtt-7.txt`) stopped before the commands and shows a boot of another image (S3/S2 line-follow test mode); the tester reports flashing `MQTT_s2log_edmund.uf2` afterwards. Controller states were not captured on the console |
| 2026-11-04 | S1-H02 | as above | Dalton | FAIL | `logs/2026-11-04_s1-telemetry-7.txt` | Frames 8 bytes, 1000 ms apart while still. While the wheels turned, robot-time gaps of 2, 2, 3, 2, 3 s: frames lost, as in the earlier run |
