# Team and subsystems

Group **P2-8**. Ownership below comes from `docs/AAI08_Design_Review.pdf`
and `docs/AAI08_PostTri1.pdf`. The latter records subsequent design decisions
and outstanding validation.

The starter examples have been retired; every entry below describes the
integrated robot firmware (branch `feature/full-system-integration`). Keep
this file current so five people can work in one firmware image without
resource conflicts. Software integration is not hardware validation:
`docs/integration.md` lists what still has to be shown on the robot.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Owner | Planned subsystem | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Hong Yih | WiFi/MQTT, telemetry, command validation/forwarding, heartbeat and reconnection | 2501110 | Not provided | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Dalton | Vehicle Controller; motors, encoders, PID speed/distance control and turns | 2504003 | Not provided | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Edmund | IR calibration, line following, barcode decoding and navigation commands | 2501578 | Not provided | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Khang | IMU calibration, motion/hump detection, highest peak and event reporting | 2503012 | Not provided | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Timothy | Servo/ultrasonic scanning, obstacle profiling, bypass planning and line reacquisition | 2501530 | Not provided | `app_program/subsystem5/` | `testing/subsystem5/` |

The Vehicle Controller authorises movement and recovery; only Motion Control
writes motor PWM. Sensor subsystems report results or plans rather than driving
motors directly. Networking remains subject to the service-task rule in section 3.

Post-Tribunal decisions retain latched STOP and stop-before-scan. Remote STOP
bypasses the normal command queue via `controller_flags`; START/RESET use
`remote_q`. These are planned interfaces, not existing application objects.
Compact telemetry summaries keep raw readings local; field ranges, widths,
byte order and safe shared-data access still need agreement and testing.

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

The authoritative GPIO mapping is `AGENTS.md`, section *Agreed pin mapping*;
the claims below must match it. The proposed robot wiring is in
`docs/AAI08_Design_Review.pdf` section 2. The rows below follow the team pin
mapping of 2026-11-03, which records the assembled robot (`docs/decisions.md`,
subsystem 2, decision 1): the wheel encoders use GP7/GP28 (right) and GP0/GP1
(left; subsystem 2, decision 4), the IMU uses I2C1 on GP2/GP3, the HC-SR04 GP4/GP5, the servo GP12,
and the line and barcode sensors use GP26/GP27 (front analogue) and GP16
(barcode digital). Because GP0/GP1 carry an encoder, building with
`CRASH_REPORT=1` (UART0 on GP0/GP1) is prohibited. The kernel ADC driver owns
ADC0/GP26 and ADC1/GP27 for the two front line sensors. ADC2/GP28 stays disabled
because GP28 carries the right encoder. Reconcile any other wiring with the
actual board and datasheets before configuring hardware.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S2 | ENC2_A, left encoder (GPIO interrupt); `CRASH_REPORT=1` prohibited |
| GP1 | S2 | ENC2_B, left encoder; `CRASH_REPORT=1` prohibited |
| GP2 | S4 | IMU SDA1 (I2C1) |
| GP3 | S4 | IMU SCL1 (I2C1) |
| GP4 | S5 | HC-SR04 TRIG |
| GP5 | S5 | HC-SR04 ECHO; input level must not exceed Pico limits |
| GP6 | | front-right DOUT disconnected in the active analogue mapping |
| GP7 | S2 | ENC1_A, right encoder (GPIO interrupt) |
| GP8 | S2 | M1A motor 1 PWM (slice 4A), right wheel forward |
| GP9 | S2 | M1B motor 1 PWM (slice 4B), right wheel backward |
| GP10 | S2 | M2A motor 2 PWM (slice 5A), left wheel backward |
| GP11 | S2 | M2B motor 2 PWM (slice 5B), left wheel forward |
| GP12 | S5 | SG90 servo PWM (slice 6A) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S3 | side barcode sensor DOUT; demo liveness LED must not use this pin |
| GP17 | | unused; old front-left analogue cable disconnected; not ADC-capable |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | front-left line sensor AOUT, ADC0 |
| GP27 | S3 | front-right line sensor AOUT, ADC1 |
| GP28 | S2 | ENC1_B, right encoder; battery-sense jumper open |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: Subsystem 4 contains the IMU sensor code and owns I2C1.
Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | not initialised; placeholder pins GP20/GP21 are never configured (`i2c_cnf_sysdep.h` refuses claimed pins) |
| I2C1 | S4 | `s4_imu_task` | GP2 (SDA1), GP3 (SCL1) |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | Line Sensor task | ADC0/GP26 front-left AOUT; ADC1/GP27 front-right AOUT; ADC2/GP28 disabled for S2 encoder; ADC3/GP29 disabled for radio |
| PWM slice 4 | S2 | Motion Control task (`motion`) | GP8 / GP9 |
| PWM slice 5 | S2 | Motion Control task (`motion`) | GP10 / GP11 |
| PWM slice 6 channel A | S5 | Obstacle Scan task (`obstacle_scan`) | GP12 |
| GPIO edge interrupt (IO_IRQ_BANK0) | S2 dispatcher (`drv_gpio_irq`) | callbacks per pin: S2 encoders (GP7, GP0), S3 barcode (GP16), S5 ECHO (GP5) | one callback per pin |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  and calls subsystem 1's hooks (`app_program/subsystem1/mqtt_service.c`):
  commands from the command topic go to the controller through subsystem
  2's non-blocking adapters, and a telemetry snapshot is published every
  second (IF-08 to IF-10 below). This shared-library change was made at the
  integration owner's direction (`docs/decisions.md`); further changes
  still need **team and instructor agreement**. It needs the
  `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

The design orders application work as Vehicle Controller, Motion Control,
Obstacle Scan, Line Sensor, IMU/Terrain, then Wireless Comms. The bands keep
the starter's order, so Obstacle Scan (20) runs below Line Sensor (16): a
known deviation, acceptable because ECHO edges are timestamped in the
interrupt, not by the task (`docs/integration.md`). Wireless Comms runs in
the kernel's WiFi task (4), above all application tasks; its hooks never
wait.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | none: runs in the kernel WiFi task `cyw43_utk_task` |
| 2 | 14–15 | `controller` – 14 – core 1; `motion` – 15 – core 1 (10 ms cyclic) |
| 3 | 16–17 | `line_sensor` – 16 – core 0 (10 ms cyclic) |
| 4 | 18–19 | `s4_imu_task` – 18 – any (10 ms cyclic) |
| 5 | 20–21 | `obstacle_scan` – 20 – any (10 ms cyclic) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 to IF-05 | | starter examples | | retired 2026-11-03 with the starter tasks | — |
| IF-06 | S3 → S2 | snapshot under S3's mutex, read with `TMO_POL` | `s3_line_get()` | `s3_line_status_t` (`subsystem3.h`): steering, line, flags, event counters, `nav_cmd` + `nav_id` | integration owner; S3/S2 review pending |
| IF-07 | S2 → S3 | robot_state flag + latched STOP | `s2_get_robot_state()`, `safety_flags` | `S2_RS_NAV_AUTHORISED` (FOLLOW only); STOP discards staged barcode commands | integration owner; review pending |
| IF-08 | S1 → S2 | event flag (owned by S2) | `controller_flags` | `CTRL_FLG_REMOTE_STOP` via `controller_raise_remote_stop()` | S2 proposal; S1 review pending |
| IF-09 | S1 → S2 | message buffer (owned by S2) | `remote_q` | `s2_remote_cmd_t` START/RESET via `controller_send_remote_command()` | S2 proposal; S1 review pending |
| IF-10 | S2 → S1 | snapshot under state_mutex (`TMO_POL`) | `robot_read_telemetry()` | provisional 8-byte codec (`telemetry_encoder.h`) | S1 proposal; review pending |
| IF-11 | S2 → S5 | message buffer (owned by S2) | `scan_q` | `s2_scan_cmd_t`: REQUEST, AUTHORISE, STOP_CONFIRM, CANCEL, STOP | S2 proposal; S5 review pending |
| IF-12 | S5 → S2 | event flag (owned by S2) | `controller_flags` | `CTRL_FLG_HAZARD`, `CTRL_FLG_SCAN_FAULT` | S2 proposal; S5 review pending |
| IF-13 | S5 → S2 | snapshot under S5's mutex (`TMO_POL`) | `scan_task_get()` | `scan_task_status_t`: forward verdict, result, `bypass_plan_t` | integration owner; review pending |
| IF-14 | S2 → S5 | robot_state field | `s2_get_robot_state()` | `mode` (`S2_MODE_*`): S5 raises hazards only in FOLLOW, BYPASS, REACQUIRE | integration owner; review pending |
| IF-15 | S2 → S4 | status under motion mutex | `motion_get_status()` + `s2_distance_delta()` | forward distance per IMU sample; `run_id` change = new run | S2 proposal; S4 review pending |
| IF-16 | S2 → S3, S5 | shared GPIO interrupt dispatcher | `drv_gpio_irq_register()` | one callback per pin, timestamped edges | PostTri1 (single dispatcher) |

The controller logic copies the IF-06 and IF-13 constants it uses; a
mismatch fails the build (`task_controller.c`).

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are counted from the integrated code for the
normal build; `TEST=1` adds one subsystem 2 event flag and message buffer.
Kernel driver objects (I2C, ADC, serial) are not included.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | | 2 | 1 | 1 | 1 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | 2 | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | 2 | 1 | 1 | 1 |
| Message buffer | 12 | | 3 | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | 1 | 1 | 1 | 1 |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
