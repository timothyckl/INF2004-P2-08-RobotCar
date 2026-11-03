# Project

Robot car firmware running on the micro T-Kernel 3.0 port for the RP2040
(Raspberry Pi Pico W). See `README.md` for setup and build commands.

## Project context

- `docs/Project Description.pdf` defines the mission; `docs/AAI08_Design_Review.pdf`
  records the initial architecture and ownership; `docs/AAI08_PostTri1.pdf`
  records later decisions and open validation. Use the later decisions where
  they revise the initial design. `TEAM.md` maps members to subsystem folders.
- `docs/decisions.md` records agreed resolutions to tribunal gaps and implementation issues (currently subsystem 5). All members must record new agreed decisions there, labelled by subsystem—not unresolved proposals.
- Distinguish planned robot behaviour from the starter examples still in the
  firmware. Do not present proposed timing, data widths or test criteria as
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

| Device | Signal | GPIO | Interface | Owner / constraint |
|---|---|---|---|---|
| GY-511 / LSM303DLHC IMU | SDA / SCL | GP2 / GP3 | I2C1 | IMU/Terrain; power from 3V3 |
| HC-SR04 | TRIG | GP4 | GPIO output | Obstacle Scan generates the trigger pulse |
| HC-SR04 | ECHO | GP5 | GPIO input | Obstacle Scan times the pulse; Pico inputs are not 5 V tolerant |
| IR front-left / front-right / barcode | DO | GP16 / GP6 / GP26 | GPIO inputs | Line Sensor; GP26 is barcode-only |
| IR front-left / barcode | AO | GP17 / GP27 | Analog connections | Connected but unused by current digital logic |
| Left wheel encoder | ENC_A / ENC_B | GP7 / GP28 | GPIO interrupts | Motion Control; keep the GP28 battery-sense jumper open |
| Right wheel encoder | ENC_A / ENC_B | GP0 / GP1 | GPIO interrupts | Motion Control; UART0 and crash-report mirror are prohibited |
| Right motor | forward / backward | GP8 / GP9 | PWM | Motion Control only |
| Left motor | backward / forward | GP10 / GP11 | PWM | Motion Control only |
| SG90 servo | SERVO_PWM | GP12 | PWM | Obstacle Scan; power from the servo header V+ |
| Pico W radio | Internal WiFi signals | GP23 / GP24 / GP25 / GP29 | Internal SPI/PIO | Reserved; do not use for other functions |

Configuration constraints:

- Disable or relocate UART0 and its mirror from GP0/GP1; those pins belong to
  the right encoder.
- Disable or reassign the demo liveness LED from GP16.
- Keep sample ADC pin initialization disabled on GP26–GP28. GP26 is a digital
  barcode input, GP27 carries a currently unused analog sensor output, and
  GP28 belongs to the left encoder.
- The HC-SR04 datasheet specifies a 5 V supply. The current 3.3 V test wiring
  recorded in `updatedmapping.md` is outside that specification and is not
  accepted as validated operation. Use a safe 5 V arrangement with ECHO level
  shifting before final hardware validation, and keep all grounds common.
