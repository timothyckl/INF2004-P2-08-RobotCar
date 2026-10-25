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
| GY-511 / LSM303DLHC IMU | SDA / SCL | GP0 / GP1 | I2C0 | IMU/Terrain; power from 3V3 |
| HC-SR04 | TRIG | GP2 | GPIO output | Obstacle Scan generates the trigger pulse |
| HC-SR04 | ECHO | GP3 | GPIO input | Obstacle Scan times the pulse; level-shift 5 V to 3.3 V |
| IR line sensors 1–3 | IR1_OUT / IR2_OUT / IR3_OUT | GP4 / GP16 / GP6 | GPIO inputs | Line Sensor |
| Motor 1 encoder | ENC1_A / ENC1_B | GP26 / GP27 | GPIO interrupts | Motion Control |
| Motor 2 encoder | ENC2_A / ENC2_B | GP7 / GP28 | GPIO interrupts | Motion Control; keep the GP28 battery-sense jumper open |
| Motor 1 | M1A_PWM / M1B_PWM | GP8 / GP9 | PWM | Motion Control only |
| Motor 2 | M2A_PWM / M2B_PWM | GP10 / GP11 | PWM | Motion Control only |
| SG90 servo | SERVO_PWM | GP15 | PWM | Obstacle Scan; power from V+ |
| Pico W radio | Internal WiFi signals | GP23 / GP24 / GP25 / GP29 | Internal SPI/PIO | Reserved; do not use for other functions |

Configuration constraints:

- Disable or relocate UART0 and its mirror from GP0/GP1.
- Disable or reassign the demo liveness LED from GP16.
- Do not enable the sample ADC driver on encoder pins GP26–GP28.
- GP5 and GP17 are unused. Connect IR sensor 3 to GP6 only; its Grove GP26
  contact must remain disconnected because GP26 belongs to encoder 1.
- Power the HC-SR04 from 5 V and keep all signal grounds common.
