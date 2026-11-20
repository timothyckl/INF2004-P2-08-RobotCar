# Subsystem 2: Vehicle Controller and Motion Control

Subsystem 2 drives the two DC motors from encoder feedback and owns the
vehicle's mission state and its latched STOP. Motion Control is the only code
that writes motor PWM. The Vehicle Controller is the only code that sends
motion commands or clears STOP. Cross-subsystem interfaces are in
[TEAM.md, section 5](../../TEAM.md#5-interfaces-between-subsystems); agreed
changes are in [the subsystem 2 decisions](../../docs/decisions.md#subsystem-2--vehicle-and-motion-control-dalton).
Test procedures and the full result history are in the
[subsystem 2 test log](../../testing/subsystem2/TEST_LOG.md).

## Hardware and configuration

| Item | Setting | Source |
|---|---|---|
| Motor 1 (left wheel) | GP8 (M1A, forward) / GP9 (M1B, backward), PWM slice 4 | Robo Pico datasheet; full lap 2026-11-10 (decision 6) |
| Motor 2 (right wheel) | GP10 (M2A, backward) / GP11 (M2B, forward), PWM slice 5 | as above |
| Motor PWM | 20 kHz: 125 MHz clk_sys, divider 1, TOP 6249; duty -6250..+6250 | Robo Pico datasheet table 1 row 10 |
| Driver truth table | A high B low forward; A low B high backward; both low brake; both high coast | Robo Pico datasheet section 4 |
| Encoder 1 (right wheel, motor 2) | GP0 (A) / GP1 (B); counts down going forward; `CRASH_REPORT=0` | full lap 2026-11-10 (decision 6) |
| Encoder 2 (left wheel, motor 1) | GP16 (A) / GP17 (B), Grove port 4; counts up going forward | S2-H03 2026-11-11 (decision 13) |
| Encoder power | 3V3 and GND | as above |
| Motor plug | PH2.0-6P: 01 encoder GND, 02 encoder A, 03 encoder B, 04 encoder VCC, 05 motor +, 06 motor - | as above |
| Motor wires | M1A GP8 yellow, M1B GP9 black; M2A GP10 yellow, M2B GP11 black | as above |
| Encoder type | TT motor two-channel Hall (confirmed on the robot; no datasheet) | owner |
| Watchdog | RP2040 watchdog, 200 ms, kicked by Motion Control; armed only with `S2_WATCHDOG=1` (decision 28) | Pico SDK register headers; S2-H05 on the bench |
| GPIO interrupt | IO_IRQ_BANK0, IRQ 13, NVIC priority 2, on core 0 | SDK `hardware/regs/intctrl.h` |
| Start/stop button | GP20, active low, debounced 30 ms | Robo Pico datasheet; mission image |

The authoritative pin mapping is `AGENTS.md`, section *Agreed pin mapping*.
Wheel pairing, viewed from behind: the left wheel is motor 1 with encoder 2,
and the right wheel is motor 2 with encoder 1 (decision 6). `motion_config.h`
follows this mapping in every build.

### Bench findings

- **PWM block in reset (2026-11-02).** The kernel releases `RESETS_RESET_PWM`
  only when it uses PWM as its physical timer (`USE_PTMR`, 0 in
  `config/config.h`), so the motors did not move at any duty.
  `drv_motor_init()` clears that reset itself and never asserts it, because
  the servo (subsystem 5) shares the block.
- **Full-speed freeze (2026-11-02).** With both motors at 100% the CPU hung
  and the motors kept running. The duty ramp and the watchdog were the
  response. On this branch S2-H04 (full speed, both motors) passed on
  2026-11-10 with no hang; the cause of the first hang was not identified.
- **Left encoder dropout (2026-11-16).** An intermittent Grove 4 connection
  froze encoder 2 while the wheel turned, which gave the left stall faults
  of that day. After the cable was fixed S2-H03 showed no dropout.

## Software structure

| Module | Responsibility |
|---|---|
| `drv_motor.c` | PWM set-up, signed duty, brake, coast; one function maps sign to A/B |
| `drv_encoder.c` | x2 quadrature counting in the dispatcher interrupt; lock-free snapshot |
| `drv_watchdog.c` | RP2040 hardware watchdog: start, kick, last-reset reason |
| `drv_gpio_irq.c` | Shared GPIO edge-interrupt dispatcher for the whole team |
| `pid.c`, `odometry.c` | Integer PID; counts to mm, speed and turn angle |
| `motion_logic.c` | Pure Motion Control statechart |
| `task_motion.c` | 10 ms Motion Control task; status under a mutex |
| `motion_api.c` | Public motion API: builds and queues commands |
| `controller_logic.c` | Pure Vehicle Controller top-level statechart and STOP causes |
| `line_control.c` | Pure line following: report to wheel speeds, junction crossing |
| `scan_control.c` | Pure Scan & Plan handshake with subsystem 5 |
| `bypass_control.c` | Pure execution of subsystem 5's bypass plan and line reacquisition |
| `nav_control.c` | Pure barcode manoeuvre at the next junction |
| `task_controller.c` | Controller task: runs the modules above; GP20 button |
| `robot_state.c` | robot_state under state_mutex; `robot_read_telemetry()` for subsystem 1 |
| `s2_bridge.c` | Adapters for subsystem 1's service bridge |
| `motion_config.h` | Every tuning value, marked UNMEASURED or AGREED |
| `subsystem2_test.c`, `test_*.c` | On-target and host unit cases |

The pure modules make no kernel calls, so they also run on the host
(`make -C testing/host`).

### Tasks

| Task | Priority | Core | Period |
|---|---|---|---|
| controller | 14 | 1 | waits on `controller_flags`, at most 10 ms |
| motion | 15 | 1 | 10 ms cyclic handler, plus an immediate wake-up on STOP |

The kernel tick is 10 ms (`CNF_TIMER_PERIOD`). Of the 10 ms and 20 ms
task periods considered here, 10 ms halves the worst-case STOP latency and gives
subsystem 4 one distance update per IMU sample. Both tasks share core 1, so a
STOP latched by the controller reaches Motion Control as soon as the
controller blocks.

### Motion Control

States: Init, Idle, Speed, Move, Turn, Stopping, Fault (design review
section 6.2).

- Each wheel has a speed PID (mm/s in, duty out). Speed is measured over the
  last 4 periods.
- Move adds a second PID on the left/right distance difference. The speed
  loops keep speeds equal but cannot win back distance already lost; this loop
  can. Test S2-U24 passes with it and fails without it.
- Latched STOP and stop commands always win: zero duty in the same step.
  Commands are rejected while STOP is set. A new command replaces the current
  one only while STOP is clear. Command IDs that are not newer are rejected.
- Stop confirmation: both wheels still for 100 ms, Fault after 2 s
  (`motion_stop_confirmed()`).
- Stall, encoder fault and Move/Turn timeout go to Fault (zero duty,
  `CTRL_FLG_MOTION_FAULT`). Only RESET with both wheels still leaves Fault.
- If `safety_flags` cannot be read, STOP is treated as set.
- Duty ramp: each wheel's |duty| grows by at most `MC_DUTY_STEP_MAX` per period
  (full scale in 200 ms), so both motors never jump from 0 to 100% at once.
  Decreases, STOP and Fault are immediate; a sign change passes through 0. It
  is applied before the stall check, so stall timing starts only once the
  ramped duty reaches the stall level.
- Duty cap: the mission image limits duty to 20% of full scale
  (`MC_CONTROL_DUTY_MAX`), as in every car run so far.
- Hardware watchdog (only with `S2_WATCHDOG=1`): armed when the task starts
  and kicked only by Motion Control, every period. If the CPU hangs or the
  task is starved for `MC_WATCHDOG_MS` (200 ms), the chip resets; the reset
  puts the PWM block back in reset and the pads back to their pull-downs, so
  the motors stop. The load value is doubled for erratum RP2040-E1, and PSM
  `WDSEL` is set (its reset value 0 would reset nothing). Motion Control
  does not arm it in `TEST=1` images; `S2_HW_TEST=4` starts it explicitly
  for S2-H05. After such a reset the boot prints
  `[S2] previous reset: WATCHDOG`.

### Vehicle Controller

Top-level states: Initialising, Ready, Running, Stopped, Fault.

- STOP is latched at boot and in every state except Running. Only START in
  Ready clears it, after draining `motion_q`. START comes from GP20 or from
  subsystem 1 (`remote_q`).
- START needs Ready, Motion Control healthy, and the wheels confirmed still.
- REMOTE_STOP, LOCAL_STOP (GP20 and on-board failures), LINE_STOP, HAZARD and
  PICKUP latch STOP and go to Stopped. MOTION_FAULT and SCAN_FAULT latch STOP
  and go to Fault. The first STOP cause is kept for telemetry unless a
  fault replaces it (decision 27).
- RESET: Stopped to Ready, or Fault to Initialising, only with the wheels
  confirmed still.
- Remote START/RESET older than 2 s are EXPIRED; a repeated request ID is a
  DUPLICATE. Nothing else (including WiFi reconnection) clears STOP.

Running has substates, published as `robot_state.mode` (`S2_MODE_*`):

| Mode | What the controller does | Decisions |
|---|---|---|
| FOLLOW | Steers from subsystem 3's line reports (`line_q`); crosses junctions straight; softens strong curves while a barcode is read | 8, 25 |
| NAVIGATE | At the junction after a mapped barcode, turns left, right, straight or U-turn, then resumes following | 26 |
| SCAN | Stops before the obstacle, confirms the stop to subsystem 5, waits for its plan | 9, 11, 12, 15 |
| BYPASS | Drives subsystem 5's plan segments around the obstacle | 14, 16 to 19 |
| REACQUIRE | Searches for the original line, stops before it, spins onto it, then FOLLOW | 20 to 22 |

The mission image starts these steps; the `S2_CAL_TEST` images run a fixed
manoeuvre script instead, and the self-test image runs neither.

## Motion API (`motion_api.h`)

```c
ER motion_move_forward (uint16_t distance_mm);   /* 1..32767 */
ER motion_move_backward (uint16_t distance_mm);
ER motion_turn_left (uint32_t angle_mdeg);       /* 1..360000, in place */
ER motion_turn_right (uint32_t angle_mdeg);
ER motion_stop (void);
ER motion_reset (void);
ER motion_new_run (void);
ER motion_get_status (motion_status_t * p_status);
bool motion_stop_confirmed (void);
```

The command functions only queue a command (non-blocking): `E_OK`, `E_PAR`,
`E_OBJ` (not created) or `E_TMOUT` (queue full). Call them from the Vehicle
Controller task only. Results appear in `motion_get_status()`.

## Build options

| Option | File | Default | Effect |
|---|---|---|---|
| `S2_WATCHDOG` | `task_motion.c` | 0 | 1 arms the hardware watchdog in a `TEST=0` image (decision 28) |
| `S2_CAL_TEST` | makefile | 0 | One manoeuvre per GP20 press: 1 forward 1000 mm, 2 left 360 degrees, 3 forward 450 mm (S2-H06..H08, S4-H08); 4 forward 500 mm, 5 left 90, 6 right 90, 7 left 180 degrees (S2-H10). Prints odometry and each STOP's latency (S2-H11); needs `TEST=1` |
| `S2_HW_TEST` | makefile | 0 | 1: encoder print, motors off; 2: also spins motors; 3: S2-H04 full-speed check; 4: S2-H05 watchdog check. Tasks not started; needs `TEST=1` |
| `S2_MOTION_LOG` | makefile | 0 | Prints the speed loop every 50 ms, for PID tuning |
| `S2_DEMO_AUTOSTART` | `task_controller.h` | 0 | Local START 5 s after Ready. The robot moves by itself |
| `S2_DEMO_LOG` | `task_controller.h` | 0 | Console status and STOP latency every 500 ms |

```sh
make -C build_make NET=lwip MQTT=1 -j8   # mission firmware
make -C build_make TEST=1 -j8            # self-tests S2-U03..S2-U78
make -C testing/host                     # host cases
```

Compilation does not run the on-target tests: flash the UF2 and capture the
USB serial output, then record results in `testing/subsystem2/TEST_LOG.md`.

## Calibration

| Value | Setting | Evidence |
|---|---|---|
| Counts per revolution | 1330 (x2 counts) | S2-H03, 2026-11-04 and 2026-11-11 |
| Wheel circumference (effective) | 227233 um | S2-H07 PASS, 2026-11-10: 1000 mm commanded, 1004 to 1007 mm measured (within 1%) |
| Track width (effective) | 119 mm | S2-H08, 2026-11-10: a 360 degree turn ends square by eye |
| Speed and straight-line PID gains | Set once, kept | S2-H09 step response passed at 200 mm/s and at 32 to 40 mm/s (PID tuning report below) |
| Still threshold, stall, timeouts, tolerances, ramp | UNMEASURED | `motion_config.h` |

## PID tuning report

**Loops.** Each wheel has a speed PID (`pid.c`): measured speed in mm/s
in, signed duty out, every 10 ms. Speed is taken over the last four
periods, so one encoder count changes it by 4.3 mm/s (227 233 um / 1330
counts per 40 ms). Above it, Move runs a straight-line PID on the
left-minus-right distance; its output (mm/s), limited to the Move speed,
is added to the left wheel's target and taken from the right's. Both use integer gains scaled by 1024,
clamp their output, stop integrating while saturated, and take the
derivative of the measurement, not of the error (S2-U03 to S2-U09).

| Loop | Kp | Ki | Kd | Output limit |
|---|---|---|---|---|
| Wheel speed | 8.0 duty per mm/s | 1.0 | 0 | ±6250 duty; ±1250 (20%) in the mission |
| Straight line | 2.0 mm/s per mm | 0.1 | 0 | ±400 mm/s, then the Move speed |

**Method.** The gains in `motion_config.h` were set before any car run
and then checked rather than swept: S2-H09 logged target, measured
speed and duty every 50 ms (`S2_MOTION_LOG=1`) for a 200 mm/s Move and
for line following at 32 to 40 mm/s. Its criteria were the target reached
within 300 ms, overshoot under 20% and no sustained oscillation. Both
passed, so the gains were kept; no other gain set was tried.

| Case (S2-H09, 2026-11-10) | Rise | Overshoot | Steady state |
|---|---|---|---|
| 0 to 200 mm/s Move ([log](../../testing/subsystem2/logs/2026-11-10_s2-h09-move200.txt)) | 188/196 mm/s at 150 ms, at target by 200 ms | 205 mm/s peak, within one 4.3 mm/s count | 196 to 205 mm/s, duty 1700 to 1830; no oscillation |
| Line following at 32 to 40 mm/s ([log](../../testing/subsystem2/logs/2026-11-10_s2-h09-line.txt)) | — (targets change every report) | — | 40 mm/s target: mean 40.1 (left) and 39.5 (right), sd 3.8 and 3.6 mm/s, equal to the speed quantisation; peak duty 1120 of the 1250 cap |

The rise is limited by the duty ramp (312 duty per period, full scale in
200 ms), not by the PID. The weak point is the 5 mm/s inside-wheel target
of a sustained curve: half its samples read 0 and peaks reach 25 to 29
mm/s, because 5 mm/s is about one count per 40 ms window and the motor
sticks below its breakaway duty. A 0 mm/s inside wheel (braked) stalled
the right wheel against the 20% cap, and 10 mm/s could not hold the loop,
so 5 mm/s was kept (S2 decision 8). The straight-line PID is justified by
S2-U24: with a wheel slipping for 200 ms and 20% weaker after, a 500 mm
Move ends within 5 mm and 2 degrees only with it on.

Not measured: a step response under the mission's 20% cap at Move speed
(at 200 mm/s the wheels needed 27 to 29% duty, so mission moves run
slower, about 130 mm/s in the bypass), and the straight-line loop's
response on the floor.

## Motion accuracy evaluation

Targets from Tribunal 1 (A4): straight within 2 cm per 50 cm, 90 degree
turns within 5 degrees, U-turns within 10 degrees.

| Measure | Result | Status |
|---|---|---|
| Distance, 1000 mm Move (S2-H07, 3 runs) | Tape 1007, 1004, 1006 mm against odometry 1001 to 1002 mm: +0.5% | Meets the target scaled to 1000 mm (±40 mm); 3 runs, not 10 |
| 360 degree turn (S2-H08, track 119 mm, 2 runs) | Odometry 362.1 and 363.5 degrees; physical end on the start line by eye | Used to set the track; no protractor reading |
| Turns in the mission | Barcode left, right, straight and U-turn at junctions, and the bypass's 45 degree turns, completed on the car ([S3 test log](../../testing/subsystem3/TEST_LOG.md#results), [S5 README](../subsystem5/README.md)) | These end on the line or on odometry with no measured angle |
| Straight 500 mm, 10 runs | — | **To run:** S2-H10, `S2_CAL_TEST=4` |
| Left and right 90 degrees, 10 runs each | — | **To run:** S2-H10, `S2_CAL_TEST=5`, `6` |
| U-turn 180 degrees, 10 runs | — | **To run:** S2-H10, `S2_CAL_TEST=7` |
| STOP latency, latch to motor off, 10 runs | — | **To run:** S2-H11, GP20 during `S2_CAL_TEST=1`; target 20 ms |

Odometry error sources: wheel slip, which turns show directly because
there is no gyroscope; the track width set by eye; and Move not slowing
near the target, so the stop distance depends on braking (about 8 mm of
coast after 200 mm/s in S2-H09). The calibration images use the full
duty range; the mission caps duty at 20%, so its moves and turns run
slower than these tests.

## Known limitations

- The hardware watchdog is not armed in the mission (decision 28). S2-H05
  showed it resetting the board on the bench; a car run with
  `S2_WATCHDOG=1` is still needed.
- `S2_DEMO_LOG` and the mission's own console lines print from the
  controller, above Motion Control on the same core. With the watchdog
  armed, a print that blocks for over 200 ms would reset the board.
- The PID gains were checked at two speed ranges only (S2-H09), and the
  mission's 20% duty cap keeps moves below the 200 mm/s API speed; line
  following runs at 40 mm/s (`CTRL_LINE_BASE_MM_S`).
- Move does not slow down near the target, so overshoot depends on braking.
- Turns use encoder odometry only (the IMU has no gyroscope), so wheel slip
  shows up directly as angle error. The track width was set by eye.
- The encoder decoder uses the current A/B levels when both A edge bits were
  latched before the shared interrupt runs. It recovers the direction and
  counts the pair as two steps. More than one coalesced pair may undercount,
  so interrupt load still requires physical validation.
- S2-U37, S2-U39 and S2-U40 (revised for telemetry version 3) have passed on
  the host only, not yet on the Pico W.

## Test evidence

The [test log](../../testing/subsystem2/TEST_LOG.md#results) records the
procedure, build, result and limitations for each run. Selected serial
captures make the main claims above traceable:

| Claim | Evidence | Limit |
|---|---|---|
| Left encoder on GP16/GP17 counts up; right on GP0/GP1 counts down | [S2-H03 Grove 4 check](../../testing/subsystem2/logs/2026-11-11_s2-h03-grove4.txt), [cable-fix repeat](../../testing/subsystem2/logs/2026-11-16_s2-h03-cable-fixed.txt) | The second capture is one 60 s hand-turn session; a subsequent stall-free drive was still required. |
| Full-speed bench run and watchdog reset | [S2-H04](../../testing/subsystem2/logs/2026-11-10_s2-h04.txt), [S2-H05](../../testing/subsystem2/logs/2026-11-10_s2-h05-coldboot.txt) | Wheels were raised; the watchdog remains off in the mission image. |
| Distance and turn calibration | [S2-H07 distance repeat](../../testing/subsystem2/logs/2026-11-10_s2-h07-rerun.txt), [S2-H08 track-width repeat](../../testing/subsystem2/logs/2026-11-10_s2-h08-track119.txt) | Distance was tape-measured; the turn end was judged by eye. |
| PID behaviour at the move and line-follow speeds | [S2-H09 move](../../testing/subsystem2/logs/2026-11-10_s2-h09-move200.txt), [S2-H09 line follow](../../testing/subsystem2/logs/2026-11-10_s2-h09-line.txt) | The 5 mm/s inside-wheel target is near the encoder's speed resolution. |
| On-target logic after the pin remap | [S2-H03 console, including 67/67 S2 tests](../../testing/subsystem2/logs/2026-11-11_s2-h03-grove4.txt) | The later telemetry-version-3 cases have a [host-only result](../../testing/subsystem2/TEST_LOG.md#results). |

The [system test log](../../testing/system/TEST_LOG.md#results) records the
later combined mission runs. Those runs do not validate the unmeasured PID
gains, watchdog operation during a drive, or the revised telemetry cases on
the Pico W.
