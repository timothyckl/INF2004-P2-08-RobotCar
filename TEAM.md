# Team and subsystems

Group **P2-8**. Ownership below comes from `docs/AAI08_Design_Review.pdf`
and `docs/AAI08_PostTri1.pdf`. The latter records subsequent design decisions
and outstanding validation.

The firmware still runs the starter examples. Pin, task, interface and object
entries marked *example* describe that code, not the planned robot subsystems.
Update them as each implementation replaces the example. Keep this file current
so five people can work in one firmware image without resource conflicts.

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

The proposed robot wiring is in `docs/AAI08_Design_Review.pdf` section 2. The
authoritative GPIO mapping is `AGENTS.md`, section *Agreed pin mapping*; the
claims below must match it. Left and right are viewed from behind the robot,
ultrasonic sensor facing away. Motor 1 (GP8/GP9) and the GP16/GP17 encoder
drive the left wheel; motor 2 (GP10/GP11) and the GP0/GP1 encoder drive the
right wheel. The front line sensors' analogue outputs use ADC0/GP26 (left) and
ADC1/GP27 (right), and the barcode sensor's analogue output ADC2/GP28. The
IMU uses I2C1 on GP2/GP3, the HC-SR04 GP4/GP5 and the servo GP12. Because
GP0/GP1 carry an encoder, building with `CRASH_REPORT=1` (UART0 on GP0/GP1) is
prohibited.
Reconcile any other wiring with the actual board and datasheets before
configuring hardware.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S2 | right encoder A (GPIO interrupt); `CRASH_REPORT=1` prohibited |
| GP1 | S2 | right encoder B; `CRASH_REPORT=1` prohibited |
| GP2 | S4 | IMU SDA1 (I2C1) |
| GP3 | S4 | IMU SCL1 (I2C1) |
| GP4 | S5 | HC-SR04 TRIG |
| GP5 | S5 | HC-SR04 ECHO |
| GP6 | | |
| GP7 | | |
| GP8 | S2 | M1A motor 1 PWM (slice 4A), left wheel forward |
| GP9 | S2 | M1B motor 1 PWM (slice 4B), left wheel backward |
| GP10 | S2 | M2A motor 2 PWM (slice 5A), right wheel backward |
| GP11 | S2 | M2B motor 2 PWM (slice 5B), right wheel forward |
| GP12 | S5 | SG90 servo PWM (slice 6A) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S2 | left encoder A (GPIO interrupt, Grove 4); demo liveness LED must not use this pin |
| GP17 | S2 | left encoder B (Grove 4) |
| GP18 | | |
| GP19 | *example:* S1 | external LED (LED + 330 Ω to GND) |
| GP20 | S2 | Robo Pico programmable button; active-low START/STOP in line test |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | front-left line sensor AO (ADC0); DOUT disconnected |
| GP27 | S3 | front-right line sensor AO (ADC1); DOUT disconnected |
| GP28 | S3 | side barcode sensor AO (ADC2, Grove 7); DOUT removed; battery-sense jumper open |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
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
| I2C0 | | | |
| I2C1 | S4 | | GP2 (SDA1), GP3 (SCL1) |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | | ADC0/GP26, ADC1/GP27 (front sensors) and ADC2/GP28 (barcode sensor); ADC3/GP29 (radio) disabled |
| PWM slices (list) | S2: slices 4, 5; S5: slice 6 | Motion Control; Obstacle Scan | GP8-GP11; GP12 |
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
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

The table below is the starter's allocation, not the robot's final priorities.
The design orders application work as Vehicle Controller, Motion Control,
Obstacle Scan, Line Sensor, IMU/Terrain, then Wireless Comms. Numeric priorities
and affinities remain to be reconciled with this port's kernel service tasks.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | *example:* led_task – 12 – any |
| 2 | 14–15 | *example:* producer_task – 14 – core 1 |
| 3 | 16–17 | *example:* consumer_task – 16 – core 0 |
| 4 | 18–19 | *example:* monitor_task – 18 – any |
| 5 | 20–21 | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* |
| IF-06 | S3 → S2 | message buffer (owned by S2) | `line_q` | `s2_line_report_t`: capture time plus CENTER/LEFT/RIGHT/BOTH_BLACK/STOP; non-blocking, interpreted only by the Vehicle Controller | Edmund + Dalton, 2026-11-04 |
| IF-11 | S2 → S5 | message buffer (owned by S2) | `scan_q` | `s2_scan_cmd_t`: REQUEST, AUTHORISE, STOP_CONFIRM, CANCEL, STOP | S2 proposal; S5 review pending |
| IF-12 | S5 → S2 | event flag (owned by S2) | `controller_flags` | `CTRL_FLG_HAZARD`, `CTRL_FLG_SCAN_FAULT` | S2 proposal; S5 review pending |
| IF-13 | S5 → S2 | snapshot under S5's mutex (`TMO_POL`) | `scan_task_get()` | `scan_task_status_t`: forward verdict, result, `bypass_plan_t` | integration owner; review pending |
| IF-14 | S2 → S5 | robot_state field | `s2_get_robot_state()` | `mode` (`S2_MODE_*`): S5 raises hazards only in FOLLOW, BYPASS, REACQUIRE | integration owner; review pending |
| IF-16 | S2 → S3, S5 | shared GPIO interrupt dispatcher | `drv_gpio_irq_register()` | one callback per pin, timestamped edges | PostTri1 (single dispatcher) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | |
| Semaphore | 16 | | 1 | | | |
| Event flag | 16 | | | | 1 | |
| Mailbox | 12 | | | 1 | | |
| Mutex | 12 | | | 1 | | |
| Message buffer | 12 | | 3 | | 1 | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | | | | |
| Alarm handler | 12 | | | | 1 | |

(The kernel's USB and WiFi service tasks also use a few of these.)
