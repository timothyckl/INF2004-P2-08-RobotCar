# Subsystem 4 - test log

**Owner:** Khang (2503012). **Scope:** LSM303DLHC IMU calibration, motion and
terrain classification, hump detection, candidate height estimation, highest
valid peak retention, impact detection, and sensor fault handling.

The implementation remains independent of the other application subsystems.
Synthetic odometry is injected into logic tests; real encoder integration is
outside the scope of these standalone cases.

## Status and execution

All automated cases execute on the Raspberry Pi Pico W in a `TEST=1` firmware
image. A computer cross-compiles and records the serial output; it does not
execute the target test cases.

- `BLOCKED` means the case is executable but its production binding is absent.
- `PLANNED` means hardware, calibration, or an agreed acceptance threshold is
  still required.
- A successful build is not a passing on-board test.
- Synthetic input proves deterministic logic only. It does not validate sensor
  wiring, physical timing, mounting, vibration rejection, or measurement
  accuracy.

Build with:

```sh
make -C build_make TEST=1 -j8
```

For the connected-IMU startup case only, build with:

```sh
make -C build_make TEST=1 S4_HW_TEST=1 -j8
```

Do not use the hardware-test result without recording the exact board, sensor,
wiring, mounting, commit, and captured serial log.

Flash the resulting UF2, capture the USB serial console at 115200 baud, and
record the commit and saved evidence in the results table.

## Retired starter IDs

| ID | Previous starter purpose | Disposition |
|---|---|---|
| S4-U01 | Starter message-buffer size | Retired; not evidence of IMU behavior. Do not reuse. |
| S4-U02 | Starter LED pause/resume observation | Retired; not executed as an IMU test. Do not reuse. |

The starter integration case `INT-02` remains documented in the shared log but
is no longer executed by Subsystem 4 after the starter monitor is replaced.

## Initial on-target logic cases

| ID | Initial state / input | Expected result |
|---|---|---|
| S4-U03 | Stable stationary samples through the configured calibration count. | Calibration completes once and reports ready. |
| S4-U04 | Calibration samples span more than the configured limit. | Calibration fault; no ready or terrain result. |
| S4-U05 | One above-threshold forward sample between level samples. | No confirmed hump. |
| S4-U06 | Confirmed positive tilt, confirmed negative tilt, then confirmed level samples. | Exactly one complete ascent/descent hump. |
| S4-U07 | Confirmed ascent that returns directly to level. | No completed hump result. |
| S4-U08 | Invalid samples reach the configured consecutive limit. | Sensor fault and invalid current result. |
| S4-U09 | Invalid sample count below the configured limit followed by valid data. | No sensor fault; invalid streak clears. |
| S4-U10 | Valid timestamps cross `UINT32_MAX`. | Elapsed-time handling remains correct across wraparound. |
| S4-U11 | Gap between valid samples exceeds the configured maximum. | Stale-data fault. |
| S4-U12 | Two valid humps with different injected heights. | Higher valid height becomes the run maximum. |
| S4-U13 | Local recovery after a valid maximum. | Fresh calibration is required; run maximum is retained. |
| S4-U14 | Explicit new-run reset after a valid maximum. | Stored maximum and its validity are cleared. |
| S4-U15 | Hump sequence without valid injected distance. | Hump may be detected, but height remains invalid. |
| S4-U16 | Acceleration magnitude exceeds the configured impact threshold. | One impact event is reported without changing motor state. |
| S4-U17 | Null context, configuration, sample, or result access. | Calls reject or ignore invalid pointers without a crash. |
| S4-U18 | Known raw bytes for +100 mg, -200 mg, and +1000 mg. | Axis order, 1 mg/LSB scaling, validity, and timestamp decode correctly. |
| S4-U19 | Raw values at the signed 12-bit limits. | Values decode as -2048 mg and +2047 mg without implementation-defined shifting. |
| S4-U20 | Null raw-data or destination pointer passed to the decode helper. | Decode is rejected without dereferencing the pointer. |

Boundary cases use small synthetic thresholds to remain deterministic. These
values are not physical calibration evidence.

## Planned driver and hardware cases

| ID | Procedure | Expected result / evidence |
|---|---|---|
| S4-H01 | Verify I2C addresses, magnetometer identity registers, and configuration read-back. | Correct identity/configuration or a bounded startup fault; serial log. |
| S4-H02 | Place the stationary module in six known orientations. | Document axis signs, mounting mapping, offsets, and repeatability. |
| S4-H03 | Record stationary samples for an agreed interval. | Measured bias, range, noise, and selected calibration-span limit. |
| S4-H04 | Hold the module at independently measured angles. | Measured tilt error; no accuracy claim until a criterion is agreed. |
| S4-H05 | Run at the configured rate and record timestamps/status. | Measured task period, execution time, overruns, and missed samples. |
| S4-H06 | Disconnect or suppress the sensor, then restore it. | Bounded fault entry and recovery only after fresh validation. |
| S4-H07 | Record flat movement, turns, and representative vibration. | False detections measured before thresholds are finalised. |
| S4-H08 | Traverse known humps repeatedly with reference observations. | Detection and candidate-height error recorded; method remains experimental until accepted. |
| S4-H09 | Apply a safe controlled acceleration impulse. | Impact behavior measured without collision or damage. |
| S4-H10 | Compare magnetic readings with representative current wiring inactive and active. | Interference and any usable heading limitations documented. |

## Acceptance criteria still requiring agreement

The design documents do not establish numerical limits for tilt error, hump
height error, missed detections, false detections, or magnetometer-derived turn
rate. These must be agreed before physical PASS results are claimed. The
LSM303DLHC contains no gyroscope; turn-rate and encoder-assisted height remain
experimental measurements.

## Results

Results are append-only. Do not record synthetic tests as physical validation.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
