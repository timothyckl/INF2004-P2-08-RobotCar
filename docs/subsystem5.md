# Subsystem 5: Obstacle Scanning and Bypass Planning

Subsystem 5 watches the way ahead with the HC-SR04, scans an obstacle with
the servo-mounted sensor once the car has stopped, and plans a route
around it. It never drives the motors: the Vehicle Controller (subsystem 2)
asks for each scan, confirms the stop, and drives the plan through Motion
Control. The interfaces are IF-11 to IF-14 and IF-20 in `TEAM.md`
section 5. Most integration decisions are recorded under subsystem 2
(decisions 9 to 22) and subsystem 5 (decisions 1 to 13) in
`docs/decisions.md`. Tests are in `testing/subsystem5/TEST_LOG.md`.

## Hardware and configuration

| Item | Setting | Source |
|---|---|---|
| HC-SR04 TRIG | GP4, 12 us pulse (datasheet minimum 10 us) | `AGENTS.md`; HC-SR04 datasheet |
| HC-SR04 ECHO | GP5, timed through the shared GPIO dispatcher | as above |
| HC-SR04 supply | 3V3, so ECHO needs no level shifter; 200 and 500 mm ranged within 20 mm on 2026-11-17 | S5 decision 13 |
| SG90 servo | GP12, PWM slice 6 channel A, 50 Hz; 1400 us + 10 us per degree, positive = left, limited to ±45 degrees by the chassis | `drv_servo.h`; S2 decision 22 |
| Servo supply | servo header V+ | `AGENTS.md` |

S5 decision 2 (GP2/GP3/GP15, 5 V with a level shifter) predates the team
pin mapping and is superseded by `AGENTS.md`.

## Software structure

| Module | Responsibility |
|---|---|
| `drv_ultrasonic.c` | Trigger pulse and echo timing; never retriggers while ECHO is high |
| `drv_servo.c` | Servo PWM and angle limits |
| `range_measurement.c` | Echo width to mm at 340 m/s; 20 to 4000 mm accepted |
| `forward_monitor.c` | SAFE / UNSAFE / INVALID / STALE verdict from forward readings |
| `scan_engine.c` | Coarse scan sequencing, safety-gated on the controller's authorisation |
| `scan_profile.c` | Obstacle edges, nearest point, width and centre from a 5 degree profile |
| `bypass_plan.c` | Left or right route as turn and move segments |
| `scan_look.c` | One stationary side look while passing the obstacle |
| `scan_lifecycle.c`, `scan_task.c` | The Obstacle Scan task (priority 20) and its interface to subsystem 2 |
| `scan_telemetry.c` | Decoded obstacle status for telemetry (IF-20) |
| `obstacle_test.c`, `scan_test_adapter.c`, `test_interface.c` | On-target test runner (only in `TEST=1` images) |

The pure modules also run on the host (`make -C testing/host`).
`tools/check_subsystem5_barr_c.py` runs on every build and checks the
subsystem 5 files against the team's BARR-C subset.

## Behaviour

1. **Forward monitor.** One forward reading every 65 ms while the car
   follows the line, bypasses or reacquires. A valid reading under 250 mm
   (`g_task_hazard_mm`) raises `CTRL_FLG_HAZARD`. Data older than 150 ms is
   stale. No trigger is sent for the first 1 s after power-on, while the
   HC-SR04 starts up (S5 decision 12).
2. **Scan.** After the controller confirms the wheels are still, the servo
   steps from -45 to +45 degrees in 15 degree steps and takes the median of
   three readings at each angle. A forward median of at least 500 mm means
   the way is clear and following resumes (S2 decision 12).
3. **Profile.** The scan is refined in 5 degree steps around the edges
   (`SCAN_ENGINE_FINE_MAX` angles at most) into a 5 degree profile.
   For each side, the edge is the last beam that still hits the obstacle
   before one passes it, widened by a 10 degree beam allowance (S2
   decision 18). A side with no passing beam within 45 degrees is
   unobserved and cannot be used.
4. **Plan.** The route keeps the whole body, from the measured extents, at
   least 50 mm from the obstacle. It turns 45 degrees, never 90 (S2
   decision 17), and prefers the left on a tie (S2 decision 16). No safe
   route gives `NO_ROUTE`, and the controller stops.
5. **Pass and rejoin.** The controller turns towards the obstacle and asks
   for side looks until a look reads past the far end. It then turns back,
   searches for the original line, stops before it and spins onto it (S2
   decisions 19 to 22).

## Telemetry

`scan_telemetry_get()` gives subsystem 1 the forward range, the scan result
(plan, clear, no route, fault, cancelled), the bypass side and the observed
nearest point, width and centre (S1 decision 1).

## Known limitations

- The hazard distance (250 mm) and the servo settling time (300 ms) are
  UNMEASURED (S5-U38, S5-U47). The car runs at 40 mm/s while following;
  faster following would need the hazard distance re-derived.
- Ranging uses 340 m/s. S5-U48 read about 3% short at 1000 mm (a constant
  bias) and up to 26 mm out at 250 mm (2026-11-10).
- The observed width is not accurate enough for the S5-U49 criterion (the
  greater of 20 mm or 5%); SYS-06 records the error rather than passing
  (S2 decision 22).
- The plan assumes an obstacle up to 300 mm deep; a deeper one ends the
  bypass with a stop.
- Lifting the car during a scan or a bypass is not detected (S4 decisions
  15 and 16).
- The full bypass has not yet run on the build with S4 decision 16 (SYS-07
  is to be repeated).
