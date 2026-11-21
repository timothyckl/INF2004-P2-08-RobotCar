# Subsystem 5: Obstacle Scanning and Bypass Planning

Subsystem 5 watches the way ahead with the HC-SR04, scans an obstacle with
the servo-mounted sensor once the car has stopped, and plans a route
around it. It never drives the motors: the Vehicle Controller (subsystem 2)
asks for each scan, confirms the stop, and drives the plan through Motion
Control. The interfaces are IF-11 to IF-14 and IF-20 in
[TEAM.md, section 5](../../TEAM.md#5-interfaces-between-subsystems). The
[agreed decisions](../../docs/decisions.md) include subsystem 2's integration
decisions 9–22 and subsystem 5's hardware and ranging decisions.
[Test cases and results](../../testing/subsystem5/TEST_LOG.md) are recorded
separately from the code.

## Hardware and configuration

| Item | Setting | Source |
|---|---|---|
| HC-SR04 TRIG | GP4, 12 us pulse (datasheet minimum 10 us) | `AGENTS.md`; HC-SR04 datasheet |
| HC-SR04 ECHO | GP5, timed through the shared GPIO dispatcher | as above |
| HC-SR04 supply | 3V3, so ECHO needs no level shifter; 200 and 500 mm ranged within 20 mm on 2026-11-17 | S5 decision 13 |
| SG90 servo | GP12, PWM slice 6 channel A, 50 Hz; 1400 us + 10 us per degree, positive = left, limited to ±45 degrees by the chassis | `drv_servo.h`; S2 decision 22 |
| Servo supply | servo header V+ | `AGENTS.md` |

S5 decision 2 (GP2/GP3/GP15, 5 V with a level shifter) predates the team
pin mapping and is superseded by [AGENTS.md](../../AGENTS.md). The HC-SR04
datasheet specifies 5 V; the team's 3V3 installation has only the limited
on-car ranging evidence described below.

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
   three readings at each angle. `CLEAR` requires a forward bin and every
   observed bin from -15 to +15 degrees to reach at least 500 mm; the
   controller then resumes following (S2 decisions 12 and 22).
3. **Profile.** The scan is refined in 5 degree steps around the edges
   (`SCAN_ENGINE_FINE_MAX` angles at most) into a 5 degree profile.
   For each side, the edge is the last beam that still hits the obstacle
   before one passes it, widened by a 10 degree beam allowance (S2
   decision 18). A side with no passing beam within 45 degrees is
   unobserved and cannot be used.
4. **Plan.** The planner uses modelled vehicle extents to target at least
   50 mm of whole-body clearance; physical clearance over the course is
   not yet validated. It turns 45 degrees, never 90 (S2 decision 17), and
   selects the left route whenever that route is valid (S2 decision 16).
   A route may start by reversing (up to 150 mm) to make room for the
   turn. No safe route gives `NO_ROUTE`, and the controller stops. There
   is no automatic reattempt: a scan without a route, or a failed bypass,
   latches STOP (stop cause scan or bypass). A new run needs GP20, which
   resets and restarts once the wheels are still, or a remote `RESET`
   followed by `START`.
5. **Pass and rejoin.** The controller turns towards the obstacle and asks
   for side looks until a look reads past the far end. It then turns back,
   searches for the original line, stops before it and spins onto it (S2
   decisions 19 to 22).

## Profile and avoidance analysis

The scan records three readings per angle and uses their median. Positive
angles point left. `scan_profile_make()` distinguishes a hit from a ray that
passes the obstacle using the forward distance plus a provisional 200 mm
depth window. It refines the first hit-to-miss transition on each side in
5-degree steps, applies a 10-degree beam allowance, and reports lateral
extents from the sensor axis. If both edges are observed, reported width is
`left extent + right extent` and centre offset is
`(left extent - right extent) / 2`. A missing or inconsistent edge stays
unknown; the planner cannot treat unobserved space as a gap.

The firmware does not report a measured left or right clearance. For
each side it has the obstacle's edge (its lateral extent) and whether a
beam passed it; a side with an observed edge is then checked against the
whole vehicle. The clearance it reports with a plan
(`estimated_min_clearance_mm`) is the configured 50 mm target, not a
measurement, and the free space beyond each edge is not ranged.

### Worked on-car scan and route

In [scan 1 on 2026-11-12](../../testing/subsystem5/logs/2026-11-12_s5-recentred.txt),
the forward reading was 250 mm. On the right, -35 degrees read 295 mm and
-40 degrees read 510 mm; on the left, +35 degrees read 290 mm and +40
degrees read 559 mm. The latter readings exceeded the 450 mm hit window.
After fine-ray and beam processing the reported extents were 147 mm right
and 145 mm left. Thus the firmware reported a 292 mm span and a -1 mm
centre offset (left positive). These are *observed ray extents*, not a
ruler measurement of the object's physical width.

The same capture reports a left route with a 353 mm axle offset. Its
simple side-clearance lower bound is 145 mm observed left edge + 63 mm
of vehicle on the right of the axle + 50 mm target clearance + 30 mm
provisional error margin = 288 mm. The later 35-degree turn to look brings
the front corner closer: `ceil(63 cos 35° + 132 sin 35°) = 128 mm` from
the axle. Adding the edge and the 80 mm clearance-plus-margin target gives
`145 + 128 + 80 = 353 mm`, the recorded offset. The planner also checks
the body swept through turns. At 45 degrees, 353 mm sideways requires about
`353 × sqrt(2) = 499 mm` of diagonal travel; the recorded command was
500 mm. The route began with a 20 mm reverse, a 45-degree left turn,
that diagonal, then a 45-degree turn back to straight.

The controller then stopped for each side look, stepped forward in 50 mm
increments until the look passed the obstacle, searched diagonally for the
line, stopped before stepping onto it, and spun until a front sensor found
black. The [same serial capture](../../testing/subsystem5/logs/2026-11-12_s5-recentred.txt)
ends with `back on the line, resuming line following`. This shows one
completed bypass and reacquisition; the closest physical distance between
the car and box was not measured.

### What the profile does not establish

An earlier [geometry trial](../../testing/subsystem5/logs/2026-11-11_s5-bypass45-2.txt)
used a ruler-measured 144 mm wide box centred 240 mm ahead. With the older
15-degree beam allowance, [scan 4](../../testing/subsystem5/logs/2026-11-11_s5-beam15.txt)
reported 125 mm left and 168 mm right, or 293 mm total: 149 mm wider
than the box. That is evidence of ultrasonic beam and pose uncertainty,
not a width calibration for the present 10-degree allowance. The
[S5-U49 width criterion](../../testing/subsystem5/TEST_LOG.md#pico-w-hardware-cases)
and [SYS-06 profile criterion](../../testing/system/TEST_LOG.md#test-cases)
are not yet met. The 50 mm clearance is a planning target; no on-car
closest-approach measurement validates it. The [SYS-07 system attempt](../../testing/system/TEST_LOG.md#results)
stopped on false pick-up detections in all three bypasses. S4 decision 16
changed that detection gate, but the revised full bypass has not been
repeated.

## Telemetry

`scan_telemetry_get()` gives subsystem 2 the forward range, scan result
(plan, clear, no route, fault, cancelled), bypass side, and observed nearest
point, width and centre. Subsystem 2 maps that status into subsystem 1's
telemetry snapshot (S1 decision 1).

## Build and test

From the repository root, build the normal mission image or the on-target
logic-test image:

```sh
make -C build_make NET=lwip MQTT=1 -j8
make -C build_make TEST=1 -j8
make -C testing/host s5
```

The host command runs synthetic logic cases. The `TEST=1` command builds a
UF2; the logic cases run only after flashing it to the Pico W. Sensor,
servo and full-bypass checks need separate on-car runs. Follow the
[test procedure and results](../../testing/subsystem5/TEST_LOG.md) when
interpreting a build or a serial capture.

### Recorded evidence

| Claim | Evidence | Limit |
|---|---|---|
| On-target synthetic scan logic passed 39 cases after integration | [2026-11-10 self-test](../../testing/subsystem5/logs/2026-11-10_s5-integration-selftest.txt) | Synthetic inputs do not validate servo travel or obstacle clearance. |
| Placed obstacles raised a forward hazard and a clear scan resumed following | [hazard/late-echo run](../../testing/subsystem5/logs/2026-11-10_s5-stage1-late-clear.txt), [scan-clear run](../../testing/subsystem5/logs/2026-11-10_s5-stage2-clear.txt) | These are specific on-car positions, not a coverage study. |
| The current turn-to-look arrangement returned to the line in straight-on trials | [recentred servo run](../../testing/subsystem5/logs/2026-11-12_s5-recentred.txt), [straight trial](../../testing/subsystem5/logs/2026-11-12_s5-straight-final.txt) | Some later attempts found no route or were stopped by hand. |
| 3V3 ranging worked at two measured distances | [200/500 mm readings](../../testing/subsystem1/logs/2026-11-17_robustness-telemetry.txt), [S5 result record](../../testing/subsystem5/TEST_LOG.md#results) | Seven frames per distance; this does not establish the HC-SR04's full operating range at 3V3. |
| Bypass and pick-up interaction exposed a system fault | [SYS-07 result](../../testing/system/TEST_LOG.md#results), [dashboard capture](../../testing/system/logs/2026-11-20_system-dashboard.jsonl) | All three 2026-11-20 bypasses stopped on a false pick-up. S4 decision 16 changed the detection gate; a repeat is pending. |

The [full subsystem 5 log](../../testing/subsystem5/TEST_LOG.md#results)
also records failed range-accuracy trials and incomplete bypass attempts.

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
