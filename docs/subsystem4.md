# Subsystem 4: IMU terrain sensing

Subsystem 4 is a standalone LSM303DLHC accelerometer service for the Pico W.
It performs startup identity/configuration checks, stationary calibration,
terrain state processing, candidate impact detection, bounded fault recovery,
and mutex-protected status publication. It does not command the motors or
authorise movement.

## Hardware and configuration

| Item | Setting |
|---|---|
| Module | GY-511 with ST LSM303DLHC |
| Supply | Pico W 3V3 and common ground |
| Bus | I2C1 at 400 kbit/s through the kernel I2C service (`iicb`) |
| Pins | SDA GP2, SCL GP3 |
| Accelerometer address | `0x19` |
| Magnetometer address | `0x1E` |
| Accelerometer mode | 100 Hz, XYZ, +/-2 g, high resolution, BDU |
| Console constraint | S4 no longer uses UART0 pins; project wiring still assigns GP0/GP1 to the right encoder |

Before power-up, verify that the assembled module provides external SDA/SCL
pull-ups to 3V3; the LSM303DLHC I2C interface requires pull-ups and the RP2040
internal pulls are not a substitute for checking the module wiring.

The driver checks magnetometer identity bytes `0x48 0x34 0x33`, writes
`CTRL_REG1_A=0x57` and `CTRL_REG4_A=0x88`, and verifies both values by
read-back. Six acceleration bytes are read in one auto-increment transaction.
In high-resolution +/-2 g mode, the signed left-justified 12-bit values convert
at 1 mg/LSB.

These choices follow `docs/datasheets/LSM303DLHC-datasheet.pdf`, the RP2040
GPIO function assignments in `docs/datasheets/pico_w_datasheet.pdf`, and the
Robo Pico Grove 2 mapping in `docs/datasheets/Robo-Pico-datasheet.pdf`.

## Software structure

| Module | Responsibility |
|---|---|
| `drv_lsm303dlhc.c` | I2C identity, configuration, status, burst read, decoding |
| `terrain_logic.c` | Hardware-independent calibration, filtering, state and faults |
| `task_imu.c` | data-ready polling each kernel tick, retry/recovery, event reporting |
| `subsystem4.c` | Lifecycle and mutex-protected public snapshot |
| `subsystem4_test.c` | On-target synthetic and driver-decode tests |

The runtime state sequence is `CALIBRATING -> MONITORING -> ON_HUMP ->
MONITORING`. A fault can be entered from active states. Recovery
always returns through fresh calibration.

## Calibration and provisional thresholds

At startup, keep the vehicle stationary and level. `task_imu.c` turns each
sample into the vehicle's axes (x forward, y left, z up) before processing;
the board is mounted with X left, Y backwards and Z up (decision 7).
The task collects 50 valid samples, about 1.5 s at the measured rate. It rejects
calibration if any axis spans more than 80 mg. The mean X/Y offsets and the Z
one-g offset are then removed from later samples.

Runtime estimates currently compiled into `task_imu.c` are:

| Parameter | Provisional value |
|---|---:|
| Ascent/descent threshold | 120 mg |
| Level threshold | 50 mg |
| Impact threshold | 1200 mg summed absolute dynamic axes |
| State confirmation | 4 consecutive samples |
| Maximum valid sample gap | 50 ms |
| Consecutive invalid limit | 3 samples |

None of these values is physically validated. Replace them only after recording
the stationary distribution, representative flat driving, turns, vibration,
humps, safe impulses, temperature, mounting, and supply conditions.

## Height and maximum retention

Height comes from tilt and the vehicle's contact geometry (decision 8). The car
stands on its left wheel, right wheel (track 119 mm) and front ball caster
(88 mm ahead of the axle). Roll and pitch, each a moving average of the last
16 samples (about 470 ms, decision 10), fix the plane through the
three contacts; with the lowest on the floor, the hump height is the highest
contact's rise. A hump starts at 6 mm (decision 11) and ends below 2 mm, each held for three
samples, and reports its peak and the contact that crossed it. A hump not ended
within 12 s is dropped (decision 10). These are provisional values, not validated results.

Each sample the task reads Motion Control's snapshot (`motion_get_status()`).
A changed `run_id` means START began a new run, which clears the run maximum.
While the mean wheel speed is changing, forward acceleration would read as
pitch, so pitch is ignored and only the roll estimate is used. Local sensor
recovery keeps the run maximum.

A hump that the whole car climbs onto reads level on top and is not measured.

## Builds and tests

From the repository root:

```sh
make -C build_make TEST=1 -j8
make -C build_make TEST=1 S4_HW_TEST=1 -j8
```

Add `S4_RAW_LOG=1` to print every sample in the vehicle's axes with the
filtered tilts, pitch gate and state (S4-H02, S4-H03, S4-H07, S4-H08);
`testing/subsystem4/s4_raw_stats.py` summarises such a log.

The first image includes deterministic terrain and raw-register decoding tests.
The second also waits for live identity/configuration and stable calibration.
Flash the corresponding UF2 and capture the USB serial output. Compilation on
a computer does not execute these tests. Full procedures and the append-only
evidence table are in `testing/subsystem4/TEST_LOG.md`.

## Faults and recovery

- Wrong identity or control-register read-back rejects startup.
- I2C transfer errors, repeated invalid samples, stale
  valid samples, or unstable stationary calibration produce a fault.
- The task closes the device, waits one second, reopens, revalidates, and
  recalibrates. It never requests vehicle movement during recovery.
- An accelerometer overrun is not a fault: the kernel tick is 10 ms, so
  reads are 10 to 20 ms apart and the newest of the 100 Hz samples is read
  (decision 6).
- A missing data-ready indication is polled again on the next period and
  is not immediately treated as a sensor failure.

## Known limitations and integration work

- No physical hardware was available during this implementation, so wiring,
  I2C timing, actual task period, axis orientation, offsets, noise, thresholds,
  detection rates, height error, and impact behaviour remain unverified.
- The LSM303DLHC has an accelerometer and magnetometer but no gyroscope. The
  magnetometer is used only for identity checking; heading and turn rate are
  not implemented.
- Linear acceleration and vibration contaminate gravity-derived tilt. The IIR
  output and threshold state machine reduce noise but do not solve that
  observability limit.
- The 32-bit microsecond timestamp wraps by design; elapsed calculations are
  wrap-safe only for intervals shorter than one full wrap period.
- Runtime thresholds are fixed provisional constants. Calibration evidence and
  agreed acceptance criteria are required before final tuning.
- Integration still needs a controller-owned run/reset contract, an
  encoder-distance interface from Motion Control, and agreed event delivery.
- The legacy S3 message buffer and event flag are still created so the starter
  code builds unchanged. Subsystem 4 does not consume them; remove them only in
  a coordinated integration change.
