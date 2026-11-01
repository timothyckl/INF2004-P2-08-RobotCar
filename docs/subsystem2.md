# Subsystem 2: Vehicle Controller and Motion Control

Subsystem 2 drives the two DC motors from encoder feedback and owns the
vehicle's mission state and its latched STOP. Motion Control is the only code
that writes motor PWM. The Vehicle Controller is the only code that sends
motion commands or clears STOP. This document describes the Week 10 partial
integration build. Every interface is PROVISIONAL until the team agrees it in
`TEAM.md`, and every tuning value is UNMEASURED until calibrated.

## Hardware and configuration

| Item | Setting | Source |
|---|---|---|
| Motor 1 | GP8 (M1A) / GP9 (M1B), PWM slice 4 | design review section 2, Robo Pico datasheet |
| Motor 2 | GP10 (M2A) / GP11 (M2B), PWM slice 5 | as above |
| Motor PWM | 20 kHz: 125 MHz clk_sys, divider 1, TOP 6249; duty -6250..+6250 | Robo Pico datasheet table 1 row 10 |
| Driver truth table | A high B low forward; A low B high backward; both low brake; both high coast | Robo Pico datasheet section 4 |
| Encoder 1 | GP26 (A) / GP27 (B) | design review section 2 |
| Encoder 2 | GP7 (A) / GP28 (B); keep the GP28 battery-sense jumper open | as above |
| Encoder type | TT motor two-channel Hall (confirmed on the robot; no datasheet) | owner |
| GPIO interrupt | IO_IRQ_BANK0, IRQ 13, NVIC priority 2, on core 0 | SDK `hardware/regs/intctrl.h` |

The sample ADC driver turns off digital input on GP26 to GP28 at start-up
(`device/adc/sysdepend/rp2040/adc_rp2040.c`). `DEVCNF_USE_ADC` must be set to
0 in `config/config_device.h`; this is a shared file and the change waits for
agreement with Khang, whose branch edits the next line.

## Software structure

| Module | Responsibility |
|---|---|
| `drv_motor.c` | PWM set-up, signed duty, brake, coast; one function maps sign to A/B |
| `drv_encoder.c` | x2 quadrature counting in the dispatcher interrupt; lock-free snapshot |
| `drv_gpio_irq.c` | Shared GPIO edge-interrupt dispatcher for the whole team |
| `pid.c`, `odometry.c` | Integer PID; counts to mm, speed and turn angle |
| `motion_logic.c` | Pure Motion Control statechart |
| `task_motion.c` | 10 ms Motion Control task; status under a mutex |
| `motion_api.c` | Public motion API: builds and queues commands |
| `controller_logic.c` | Pure minimal Vehicle Controller statechart |
| `task_controller.c` | Controller task; creates the controller objects |
| `robot_state.c` | robot_state under state_mutex |
| `s2_bridge.c`, `s2_s1_telemetry.c` | Adapters for subsystem 1's service bridge |
| `motion_config.h` | Every tuning value, marked UNMEASURED or AGREED |
| `subsystem2_test.c`, `test_motion.c`, `test_controller.c` | On-target tests |

### Tasks

| Task | Priority | Core | Period |
|---|---|---|---|
| controller | 14 | 1 | waits on `controller_flags`, watchdog 10 ms |
| motion | 15 | 1 | 10 ms cyclic handler, plus an immediate wake-up on STOP |
| producer_task (starter) | 15 | 1 | 300 ms, kept until subsystem 3 stops using it |

The kernel tick is 10 ms (`CNF_TIMER_PERIOD`), so 10 ms and 20 ms are the
only exact periods. 10 ms halves the worst-case STOP latency and gives
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

### Vehicle Controller (Week 10 subset)

States: Initialising, Ready, Running, Stopped, Fault.

- STOP is latched at boot and in every state except Running. Only START in
  Ready clears it, after draining `motion_q`.
- START needs Ready, Motion Control healthy, and the wheels confirmed still.
- REMOTE_STOP, LOCAL_STOP and HAZARD latch STOP and go to Stopped. MOTION_FAULT
  and SCAN_FAULT latch STOP and go to Fault.
- RESET: Stopped to Ready, or Fault to Initialising, only with the wheels
  confirmed still.
- Remote START/RESET older than 2 s are EXPIRED; a repeated request ID is a
  DUPLICATE. Nothing else (including WiFi reconnection) clears STOP.
- Running holds the demo script (forward 500 mm, left 90 deg, forward 500 mm)
  when `S2_DEMO_SCRIPT` is 1. Line following, barcode manoeuvre, scan and plan,
  bypass and line reacquisition are marked TODO in `run_step()`.

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

## Interfaces for teammates (PROVISIONAL)

| With | Interface |
|---|---|
| S1 Hong Yih | `controller_raise_remote_stop()` sets `CTRL_FLG_REMOTE_STOP` (bit 3); `controller_send_remote_command()` sends START/RESET to `remote_q` (2 x 12-byte records, his `remote_command_t` layout); `robot_read_telemetry()` fills `telemetry_encoder_snapshot_t`. All in `s2_s1_adapter.h`, which has no kernel include because S1's `<stddef.h>` `size_t` conflicts with the kernel's |
| S3 Edmund | S3 calls `s3_nav_take()` with stopped = `SAFETY_FLG_STOP`, authorised = `S2_RS_NAV_AUTHORISED`, junction = his own detection; result as `s2_nav_request_t`. Not yet consumed |
| S4 Khang | `motion_get_status()` every 10 ms and `s2_distance_delta()` for `distance_delta_mm`; call `s4_terrain_start_new_run()` when `run_id` changes |
| S5 Timothy | `s2_scan_cmd_t` on `scan_q` (header only); STOP_CONFIRM once `motion_stop_confirmed()` |
| Everyone with an edge interrupt | `drv_gpio_irq_register()`; rules in `drv_gpio_irq.h` |

## Build options

| Option | File | Default | Effect |
|---|---|---|---|
| `S2_HW_TEST` | `task_motion.h` | 0 | 1: bench encoder print; 2: also spins motors. Tasks not started. Never commit non-zero |
| `S2_DEMO_SCRIPT` | `task_controller.h` | 1 | Running performs the demo script |
| `S2_DEMO_AUTOSTART` | `task_controller.h` | 0 | Local START 5 s after Ready. The robot moves by itself |
| `S2_DEMO_LOG` | `task_controller.h` | 0 | Console status and STOP latency every 500 ms |

```sh
make -C build_make            # firmware
make -C build_make TEST=1     # firmware with S2-U01..S2-U53
```

Compilation does not run the tests: flash the UF2 and capture the USB serial
output, then record results in `testing/subsystem2/TEST_LOG.md`.

## Calibration needed

All in `motion_config.h`, all UNMEASURED: counts per revolution (S2-H03),
wheel circumference and track width (ruler), motor and encoder polarity
(S2-H01, S2-H03), speed and straight-line PID gains, API speeds, the still
threshold, stall duty and time, timeout factor, tolerances, and the
controller's init (3 s) and result (20 s) limits. The tables in
`TEST_LOG.md` record each value and the accuracy runs.

## Removing the starter

The starter producer (IF-01, IF-02) is still built and started. Removing it
breaks:

- `app_program/subsystem3/task_consumer.c` (Edmund's branch and main): it
  includes `subsystem2.h` for `SAMPLE_MSG`, waits on `mbx_samples`, and
  returns blocks with `tk_rel_mpf(mpf_samples, ...)` and
  `tk_sig_sem(sem_slots, 1)`. It would no longer link.
- `app_program/subsystem4/task_monitor.c` on main and on the S1 and S5
  branches reads `sem_slots` / `mpf_samples` with `tk_ref_sem` /
  `tk_ref_mpf`. Khang's branch has already removed this.
- `subsystem2_test.c` S2-U01/S2-U02 (mine; delete with the producer).
- `app_main.c` comments describing IF-01/IF-02 (shared file).

Edmund needs to delete `task_consumer.c` and its `mbx_samples` mailbox
creation and start-up from `subsystem3.c`/`subsystem3.h`, or replace them with
his real line-sensor task, in the same merge that removes the producer from
subsystem 2. TEAM.md section 5 then drops IF-01 and IF-02.

## Known limitations

- Nothing has run on the Pico W yet; every result row is open.
- All tuning is UNMEASURED; distances and angles are only as good as the
  calibration.
- Move does not slow down near the target, so overshoot depends on braking.
- Turns use encoder odometry only (the IMU has no gyroscope), so wheel slip
  shows up directly as angle error.
- The encoder decoder cannot tell direction if both A edges arrive before the
  interrupt runs; it counts an error instead (fault after 2 in one period).
- Telemetry `link_up` is left false for S1 to fill, and S1's callback does
  not pass a request ID yet, so the adapter numbers commands itself.
- HAZARD is treated as STOP until Scan and Plan exists.
