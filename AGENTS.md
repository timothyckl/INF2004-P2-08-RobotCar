# Project

Robot car firmware running on the micro T-Kernel 3.0 port for the RP2040
(Raspberry Pi Pico W). See `README.md` for setup and build commands.

## Project context

- `docs/Project Description.pdf` defines the mission; `docs/AAI08_Design_Review.pdf`
  records the initial architecture and ownership; `docs/AAI08_PostTri1.pdf`
  records later decisions and open validation. Use the later decisions where
  they revise the initial design. `TEAM.md` maps members to subsystem folders.
- `docs/decisions.md` records agreed resolutions to tribunal gaps and implementation issues (currently subsystem 5). All members must record new agreed decisions there, labelled by subsystem—not unresolved proposals.
- The starter examples have been retired; `docs/integration.md` records the
  integrated architecture and what remains unvalidated. Do not present
  proposed timing, data widths, calibration values or test criteria as
  validated results.
- The Vehicle Controller authorises movement/recovery; only Motion Control
  writes motor PWM. Preserve latched STOP and stop-before-scan. Network APIs
  remain confined to the built-in WiFi service task (see `TEAM.md`).
- Minimise application-data bit widths based on valid ranges and precision;
  validate overflow and concurrent access before finalising packed formats.

## Required practices

- Before operating, wiring, configuring, testing, or writing/changing code for any
  hardware, read its relevant datasheet sections. Verify pin assignments,
  electrical limits, timing, interfaces, and registers as applicable; do not guess.
- Use the hardware-to-document mapping below. If the required device or detail
  is not covered, obtain the correct datasheet before proceeding.
- Follow **BARR-C:2018** in `docs/barr_c_coding_standard_2018.pdf` for all new or
  modified C code. Read the applicable rules before coding and check changes
  against them before finishing.
- Keep application code and drivers in `app_program/subsystemN/`; `device/`
  contains kernel drivers. Track shared hardware assignments in `TEAM.md`.

## Software engineering principles

- **Defect prevention:** Prefer practices and tools that prevent or detect defects
  before execution, including compiler warnings and static analysis.
- **Quality attributes:** Prioritise reliability, readability, maintainability,
  efficiency, and portability over programmer convenience.
- **Encapsulation and low coupling:** Expose only necessary interfaces and keep
  implementation details private to their modules.
- **Modularity and cohesion:** Give each source module one clear responsibility or
  entity to control.
- **Quality assurance:** Use version control, defect tracking, architecture and
  design reviews, peer reviews, and automated analysis.
- **Embedded robustness:** Account explicitly for concurrency, interrupts, shared
  state, integer ranges, overflow, and implementation-defined behaviour.
- **Governance:** Enforce agreed standards and document the reason and approval
  for every deviation.
- **YAGNI:** Do not add functionality or abstractions until they are actually
  needed.
- **DRY:** Keep each piece of knowledge in one authoritative place rather than
  duplicating it.
- **KISS:** Choose the simplest design that correctly meets the requirements.

## Hardware references

| Hardware | Local datasheet / manual |
|---|---|
| Raspberry Pi Pico W board (RP2040-based) | [pico_w_datasheet.pdf](docs/datasheets/pico_w_datasheet.pdf) |
| Cytron Robo Pico carrier board | [Robo-Pico-datasheet.pdf](docs/datasheets/Robo-Pico-datasheet.pdf) |
| HC-SR04 ultrasonic ranging module | [HCSR04-datasheet.pdf](docs/datasheets/HCSR04-datasheet.pdf) |
| ST188 infrared reflective sensor module | [ST188-datasheet.pdf](docs/datasheets/ST188-datasheet.pdf) |
| TCRT5000 / TCRT5000L reflective optical sensor | [TCRT5000-datahsheet.pdf](docs/datasheets/TCRT5000-datahsheet.pdf) |
| LM393 dual comparator | [LM393-datasheet.pdf](docs/datasheets/LM393-datasheet.pdf) |
| LSM303DLHC accelerometer / magnetometer | [LSM303DLHC-datasheet.pdf](docs/datasheets/LSM303DLHC-datasheet.pdf) |

## Agreed pin mapping

This section is the authoritative GPIO reference. `TEAM.md` section 2 records
per-pin ownership claims and must match it; change both together.

| Device | Signal | GPIO | Interface | Owner / constraint |
|---|---|---|---|---|
| GY-511 / LSM303DLHC IMU | SDA / SCL | GP2 / GP3 | I2C1 | IMU/Terrain; power from 3V3 |
| HC-SR04 | TRIG | GP4 | GPIO output | Obstacle Scan generates the trigger pulse |
| HC-SR04 | ECHO | GP5 | GPIO input | Obstacle Scan times the pulse; currently powered from 3V3, see below |
| Front-left line sensor | DO / AO | GP16 / GP17 | GPIO input / unused | Line Sensor: left line guide; AO connected, not used |
| Front-right line sensor | DO | GP6 | GPIO input | Line Sensor: right line guide; AO not connected |
| Side barcode sensor | DO / AO | GP26 / GP27 | GPIO input / unused | Line Sensor: barcode only, never steering; AO connected, not used |
| Right wheel encoder (encoder 1) | ENC1_A / ENC1_B | GP7 / GP28 | GPIO interrupts | Motion Control; keep the GP28 battery-sense jumper open |
| Left wheel encoder (encoder 2) | ENC2_A / ENC2_B | GP0 / GP1 | GPIO interrupts | Motion Control; requires `CRASH_REPORT=0` |
| Motor 1 (right wheel) | M1A_PWM / M1B_PWM | GP8 (forward) / GP9 (backward) | PWM | Motion Control only |
| Motor 2 (left wheel) | M2A_PWM / M2B_PWM | GP10 (backward) / GP11 (forward) | PWM | Motion Control only |
| SG90 servo | SERVO_PWM | GP12 | PWM | Obstacle Scan; power from the servo header V+ |
| Pico W radio | Internal WiFi signals | GP23 / GP24 / GP25 / GP29 | Internal SPI/PIO | Reserved; do not use for other functions |

Configuration constraints:

- Build with `CRASH_REPORT=0`: UART0 and its mirror would take GP0/GP1, which
  carry the left encoder. The makefile refuses `CRASH_REPORT=1` and
  `CONSOLE=uart`, and `kernel/sysdepend/pico_rp2040/hw_setting.c` and
  `config/config_device.h` fail to compile if UART0 would claim the pins.
- Disable or reassign the demo liveness LED from GP16.
- Keep RP2040 ADC pin initialisation disabled on GP26 to GP28: GP26 is the
  barcode sensor's digital output and GP28 carries the right encoder. GP17 and
  GP27 carry analogue outputs that the current logic does not use.
- GP13, GP14, GP15 and GP18 to GP22 are unused.
- The HC-SR04 datasheet specifies a DC 5 V supply, but the module is currently
  powered from 3V3 (team pin mapping, 2026-11-03). That wiring is outside the
  datasheet and is not validated. Obstacle Scan must confirm ranging at 3V3 or
  move to 5 V; at 5 V, ECHO must be level-shifted to 3.3 V because Pico GPIO is
  not 5 V tolerant.
- Keep all signal grounds common.
