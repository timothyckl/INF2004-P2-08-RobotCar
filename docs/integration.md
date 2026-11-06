# Integration record

Branch `feature/full-system-integration`, 2026-11-03. This file describes the
integrated firmware, what is still provisional, and what has and has not been
verified. Agreed decisions are in `docs/decisions.md`; pins in `AGENTS.md`;
owners, tasks and interfaces in `TEAM.md`.

**Status words used below.** *Compiled*: builds without warnings.
*Host-tested*: deterministic cases executed on a computer (`make -C
testing/host`). *On-target-tested*: executed on the Pico W (`make TEST=1`).
*Validated*: shown on the assembled robot. **Nothing in this branch has been
on-target-tested or validated yet.** A passing build or host test is not
evidence that the robot works.

## 1. Runtime architecture

| Task | Subsystem | Priority, core | Period | Role |
|---|---|---|---|---|
| `controller` | S2 | 14, core 1 | flag wait, at most 10 ms | Mission states, STOP latch, authorisation |
| `motion` | S2 | 15, core 1 | 10 ms cyclic | Only writer of motor PWM; encoders, PID, watchdog kick |
| `line_sensor` | S3 | 16, core 0 | 10 ms cyclic | IR line tracking, barcode capture, IF-06 |
| `s4_imu_task` | S4 | 18, any | 10 ms cyclic | LSM303DLHC on I2C1, terrain, highest hump |
| `obstacle_scan` | S5 | 20, any | 10 ms cyclic | HC-SR04, servo, forward monitor, scan, plan |
| `cyw43_utk_task` | kernel (S1 hooks) | 4 | event driven | WiFi, MQTT, S1 service bridge |

Start-up: every `subsystemN_init()` creates objects (STOP is latched at
creation), then every `subsystemN_start()` starts tasks. Subsystem 2 creates
the shared GPIO dispatcher in its init, so S3 (GP16) and S5 (GP5) can register
in their start. The robot never moves at boot: START must arrive over MQTT
(or from the optional, default-off `S2_DEMO_AUTOSTART`).

## 2. Data flows (all connected and invoked at runtime)

| Path | Mechanism | Code |
|---|---|---|
| IR sensors → S3 → controller → Motion → motors | ADC0/GP26 and ADC1/GP27 every 10 ms → adaptive thresholds → `s3_track` → IF-06 snapshot → FOLLOW policy renews `motion_drive()` → `motion_logic` → `drv_motor` | `task_line.c`, `controller_logic.c` (`follow`), `task_motion.c` |
| Barcode → decoder → nav command → manoeuvre | GP16 edges (dispatcher) → ring → `s3_scan` → `s3_bar_decode` → `s3_nav_stage`/`take` at a junction → IF-06 `nav_cmd`/`nav_id` → NAVIGATE turn or move | `task_line.c`, `controller_logic.c` (`navigate`) |
| Ultrasonic → hazard → scan → plan → bypass → reacquire | forward monitor every 65 ms → `CTRL_FLG_HAZARD` → SCAN (`motion_stop`, `scan_q`) → `scan_engine` sweep → `bypass_plan` → BYPASS segments → REACQUIRE | `scan_task.c`, `scan_engine.c`, `bypass_plan.c`, `controller_logic.c` |
| IMU → terrain → status | I2C1 every 10 ms → `s4_terrain_process` → `subsystem4_get_status()` | `task_imu.c`, `terrain_logic.c` |
| Encoders → odometry → Motion and S4 | dispatcher ISR → `drv_encoder` snapshot → `motion_logic` pose → `motion_get_status()` → `s4_odometry` | `task_motion.c`, `s4_odometry.c` |
| MQTT command → bridge → controller | `mqtt_utk.c` callbacks → `mqtt_service.c` → `service_bridge` → STOP: `controller_flags`; START/RESET: `remote_q` | S1, `s2_bridge.c`, `task_controller.c` |
| Robot state → telemetry | controller publishes robot_state → `robot_read_telemetry()` → 8-byte codec every 1 s | `s2_s1_telemetry.c`, `mqtt_utk.c` |

Not connected: terrain status and hump height reach the console only. The
agreed telemetry codec has no terrain, line, barcode or obstacle fields, and
choosing a wire format is S1's open decision (PostTri1 R4). The controller
does not act on terrain results; the design makes them report-only.

## 3. Safety mechanisms

- STOP latched at boot. Only START in Ready clears it, after `motion_q` is
  drained. REMOTE_STOP and LOCAL_STOP latch from every mode (S2-U45, U46, U66).
- Motion Control rejects every motion command while STOP is latched. The
  hardware watchdog (200 ms) is kicked only by Motion Control.
- Speed lease: a drive that is not renewed within 100 ms brakes by itself
  (S2-U57, U58).
- Faults latch STOP: motion fault, scan fault, delivery failure, line fault
  (S3 sensor fault, IF-06 stale 500 ms, failed reacquisition).
- Bounded waits only: every cross-task read uses `TMO_POL` or at most a
  10 ms lock. Interrupts only timestamp edges into single-producer rings
  with DMB barriers.
- UART0 and I2C0 cannot take the encoder pins (`docs/decisions.md`,
  integration decision 4).
- A WiFi or broker loss produces no controller input, so it cannot start the
  robot or clear STOP.

## 4. Provisional values (owner review and measurement needed)

| Value | Where | Basis |
|---|---|---|
| Follow 150 mm/s, search 100 mm/s, junction 60 mm | `controller_config.h` | UNMEASURED |
| IF-06 max age 50 ms, line fault 500 ms | `controller_config.h` | UNMEASURED |
| Found 100 ms, search 5 s / plan distance, heading 15° | `controller_config.h` | AGREED (S5 decision 9) |
| Speed lease 100 ms | `motion_config.h` | UNMEASURED |
| IR polarity 1 = black (all three) | `s3_config.h` | documented, not verified (S3-H01) |
| Straddle geometry table, filter 2 samples, fault 2 s, steer 500 ‰ | `s3_track.c` | UNMEASURED; both-white is centred and cannot also prove loss |
| Code 39 `*`, A..D patterns; run max 400 ms, quiet 250 ms | `s3_config.h` | assumed symbology; confirm with course barcodes |
| Navigation command age 5 s | `s3_config.h` | UNMEASURED |
| Servo 1500 µs + 10 µs/°, ±60° | `drv_servo.c` | Tower Pro SG90 sheet; calibrate (S5-U47) |
| Servo settling 300 ms, hazard 250 mm | `scan_task.c` | UNMEASURED |
| Plan half width 80, margin 30, depth 200 mm | `scan_task.c` | UNMEASURED; clearance 50 and search 500 AGREED |
| Odometry alignment 20 ms | `s4_odometry.h` | two motion periods |
| MQTT topics, QoS, 1 s telemetry | `mqtt_service.h` | S1 PROPOSAL, unapproved |

## 5. Known deviations and limitations

- **Straddle geometry.** Both front sensors read white when the black line is
  centred between them. Firmware therefore treats both-white as STRAIGHT and
  cannot distinguish a centred line from a completely missing line using
  these two front readings alone. Measure sensor spacing and line width and
  validate the behaviour on every course curve before relying on it.
- **No echo is invalid, not clear** (S5 decision 3, S5-U23): with nothing
  within about 4 m ahead, the forward monitor requests a stop, which starts a
  scan. Unless the course always has a reflector in range, the team should
  decide whether an echo that started but did not end within 30 ms means
  "clear beyond range".
- **Monitoring restart.** The 150 ms freshness counts from a monitoring
  restart until the first new reading, so returning from a scan is not a
  false hazard.
- **Coarse scan only.** The 5° fine refinement (S5-U27), request expiry
  (S5-U30, U31) and packed result formats (S5-U39) are not implemented.
- **Bypass geometry.** A front scan cannot see obstacle depth, so a depth is
  assumed. The swept footprint is the sideways step's rectangle.
- **Priorities.** Obstacle Scan (20) runs below Line Sensor (16), the
  reverse of the design order. ECHO timing comes from the interrupt.
- **HC-SR04 at 3V3.** Outside the datasheet's 5 V; unvalidated.
- **Analogue line thresholds** came from bench observations in the supplied
  ZIP and still require validation at the final sensor height and course.
- **Telemetry** carries only the provisional 8-byte codec (section 2).
- **Hump height** is invalid until a distance is valid, and is an
  unvalidated estimate (S4 decision 4); there is no gyroscope.

## 6. Requirement traceability

Requirements from `docs/Project Description.pdf` and the design review FRs.

| Requirement | Subsystem → files | Runtime path | Tests (executed) | Status |
|---|---|---|---|---|
| Follow a line autonomously (FR1, FR2) | S3 `s3_track`, `task_line`; S2 `follow`, `motion_drive` | §2 row 1 | S3 track 38 checks; S2-U57..U61 (host) | Host-tested; hardware pending |
| Detect and decode barcodes (FR3) | S3 `s3_scan`, `s3_bar` | §2 row 2 | S3 track/scan; S3 base 656 checks (host) | Host-tested; symbology unconfirmed |
| Execute left/right/straight/U-turn | S2 `navigate`, Motion turn/move | §2 row 2 | S2-U62, U22, U23 (host) | Host-tested; hardware pending |
| Encoder-based motion (FR4) | S2 Motion | encoder ISR → PID | S2-U20..U35, U55, U56 (host) | Host-tested; S2 bench rows in TEST_LOG |
| Detect humps; highest peak (FR5) | S4 `terrain_logic`, `s4_odometry` | §2 rows 4, 5 | S4 odometry 12 (host); S4-U03..U22 on target only | Partly host-tested; height unvalidated |
| Detect obstacles (FR6) | S5 `forward_monitor`, `scan_task` | §2 row 3 | S5-U25 (host) | Host-tested; 3V3 ranging unvalidated |
| Profile obstacles | S5 `scan_engine` | sweep, median | S5-U19..U24, U26, U28 (host) | Host-tested, coarse only |
| Bypass and reacquire line (FR7) | S5 `bypass_plan`; S2 BYPASS/REACQUIRE | §2 row 3 | S5-U36, U37, U53, U54; S2-U63..U65 (host) | Host-tested; hardware pending |
| Telemetry over WiFi (FR8) | S1 bridge, `mqtt_utk` | §2 rows 6, 7 | S1-U04..U07 (host); MQTT build | Compiled; broker test pending |
| Safety: STOP, faults, timeouts | S2 controller, Motion | §3 | S2-U41..U53, U57, U58, U66 (host) | Host-tested; timing pending |

## 7. Phase 2 findings

| Finding | Classification |
|---|---|
| UART0 guard inert | Implemented and verified (each guard forced to fire) |
| I2C0 placeholder on GP0/GP1 | Implemented and verified (compile-time check) |
| S1 bridge not created | Implemented; needs hardware (broker) validation |
| S3/S5 without task or hardware | Implemented; needs hardware validation |
| Provisional S2 interfaces | Implemented as IF-06..IF-16; owner review pending |
| Starter producer/consumer/LED/IF-03 | Implemented (retired) and verified by build |
| HC-SR04 at 3V3 | Deferred by decision D3: wiring kept, unvalidated |
| Documentation | Implemented (this file, TEAM.md, decisions.md, README) |
| Motion API lacked continuous drive | Implemented and host-tested (lease) |
| Analogue IR logic from supplied ZIP | Adapted to IF-06 and the integrated controller; physical validation pending |
| Fine scan, request expiry, packed formats | Deferred: not required for a first robot run; listed in §5 |

## 8. Hardware tests still required

In order, wheels off the ground first:

1. IR: analogue values on white/black per front sensor, barcode polarity,
   sensor spacing against line width, and steering truth table at speed
   (S3-H01..H03); both front DOUT wires and barcode AOUT disconnected.
2. Encoders and motors: counts per revolution, direction, PID tuning
   (S2-H01..H05 in `testing/subsystem2/TEST_LOG.md`).
3. Line following on straight and curved track; speed lease and STOP latency
   under load.
4. Barcode: printed symbology, decode at speed, turn at the junction (S3-H04).
5. HC-SR04 at 3V3: known distances ×10 (S5-U46, U48); servo calibration
   (S5-U47).
6. Scan, plan and bypass with known obstacles; reacquisition (S5-U49, U50).
7. IMU hump height against measured humps (S4 decision 5).
8. MQTT: START/STOP/RESET, stale and duplicate commands, telemetry,
   WiFi/broker loss without movement.
9. Whole-course runs, recording commit, wiring and logs (`testing/README.md`).
