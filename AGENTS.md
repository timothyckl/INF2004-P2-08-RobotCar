# Project

Robot car firmware running on the micro T-Kernel 3.0 port for the RP2040
(Raspberry Pi Pico W). See `README.md` for setup and build commands.

## Project context

- `docs/Project Description.pdf` defines the mission; `docs/AAI08_Design_Review.pdf`
  records the initial architecture and ownership; `docs/AAI08_PostTri1.pdf`
  records later decisions and open validation. Use the later decisions where
  they revise the initial design. `TEAM.md` maps members to subsystem folders.
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
