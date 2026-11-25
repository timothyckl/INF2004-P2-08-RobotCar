# Subsystem 4: IMU terrain sensing

Subsystem 4 reads the LSM303DLHC accelerometer on the Pico W. It checks the
sensor at start-up, calibrates while stationary, estimates hump height,
detects candidate impacts and lifts, and publishes a mutex-protected status
snapshot. The task reads subsystem 2's vehicle state and motion snapshot to
gate detection and reset the run maximum. A detected lift raises
`CTRL_FLG_PICKUP`; the Vehicle Controller then latches STOP. Hump maximums
feed telemetry, while candidate impacts are reported on the console. This
subsystem does not command the motors or authorise movement.

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

These choices follow the [LSM303DLHC datasheet](../../docs/datasheets/LSM303DLHC-datasheet.pdf),
the [Pico W datasheet](../../docs/datasheets/pico_w_datasheet.pdf), and the
[Robo Pico datasheet](../../docs/datasheets/Robo-Pico-datasheet.pdf).

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

These thresholds remain provisional. The recorded flat-driving, 10 mm hump
and lift trials cover only the tested set-up; temperature, other hump heights
and safe impacts have not been characterised.

## Height and maximum retention

Height comes from tilt and the vehicle's contact geometry (decision 8). The car
stands on its left wheel, right wheel (track 119 mm) and front ball caster
(88 mm ahead of the axle). Roll and pitch, each a moving average of the last
16 samples (about 340–470 ms in recorded mission runs, depending on the
actual sample interval; decision 10), fix the plane through the
three contacts; with the lowest on the floor, the hump height is the highest
contact's rise. A hump starts when that uncalibrated height reaches 6 mm
(decision 11) and ends below 2 mm, each held for three samples. Its peak is
then calibrated: times 0.851 (the measured 10 mm hump, decision 12), less an
offset (0 until a two-point calibration, decision 17), rounded to whole mm and
never below 0; the reading and the contact that crossed it are reported.
Calibration applies only to the reading, never to detection, so changing it
cannot move the detection threshold (it did until 2026-11-25: the 0.851 scale
had raised the real start to 7.05 mm). A hump not ended within 12 s is dropped
(decision 10). The 10 mm right-wheel hump was measured on the car; these
settings are not validated across other heights or surfaces.
Humps, impacts and lifts are detected only while the robot is Running in
FOLLOW (read with `s2_get_robot_state()`), and not for 32 samples after it
starts to; a hump in progress when that stops is dropped (decisions 12, 16
and 17). The obstacle bypass (BYPASS, REACQUIRE) is excluded because its fast
turns read as lifts and humps; a junction turn (NAVIGATE) because its stop,
step and spin read up to 6.6 mm (decision 17). A hump crossed during a
junction turn is therefore not measured, and a lift then is not detected.
All these settings are in `s4_config.h`, the one place they are defined. The `S2_CAL_TEST` images do not enter a
driving mode, so they report no humps.

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

`testing/subsystem4/replay_raw.c` feeds such a log back through
`terrain_logic.c` with any scale, offset and rise, and prints each hump's
uncalibrated peak beside the readings the car logged (S4-U44 runs it on the
six logged drives on every host test run).

**Two-point calibration (decision 17), to be done on the car.**

1. Make reference humps of 5, 10 and 15 mm (ruler), each as wide as the
   course hump, and build `S4_RAW_LOG=1 NET=lwip MQTT=1`.
2. Cross each one under the right wheel at least five times while following
   the line, capturing the console to a log per height. Record one lap of
   the same floor with no hump.
3. Run
   `python3 testing/subsystem4/fit_hump_calibration.py <replay> 5:<log5> 10:<log10> 15:<log15> --flat <flatlog>`
   (`<replay>` built as in `testing/host/Makefile`, target `s4`). It fits
   peak = gain x true + offset by least squares, prints
   `S4_HEIGHT_SCALE_PERMILLE` and `S4_HEIGHT_OFFSET_UM`, the worst reading
   error with them, and the floor's largest bump against the 6 mm rise.
4. Put the two constants in `s4_config.h`, record the logs and the fit in
   the test log, and re-run SYS-04.

With today's data (one height) the tool gives a gain-only scale of 866 from
six unrounded crossings, against 851 from four rounded ones; the difference
(0.17 mm at 10 mm) is inside the crossing-to-crossing spread (10.8 to
12.5 mm), so 851 is kept.

The first image includes deterministic terrain and raw-register decoding tests.
The second also waits for live identity/configuration and stable calibration.
Flash the corresponding UF2 and capture the USB serial output. Compilation on
a computer does not execute these tests. Full procedures and the append-only
evidence table are in the [subsystem 4 test log](../../testing/subsystem4/TEST_LOG.md).

## Faults and recovery

- Wrong identity or control-register read-back rejects startup.
- I2C transfer errors, repeated invalid samples or stale valid samples
  cause the task to close the device, wait one second, reopen, revalidate
  and recalibrate.
- Unstable stationary calibration restarts calibration in place; the sensor
  is not reopened. Neither recovery path requests vehicle movement.
- An accelerometer overrun is not a fault: the kernel tick is 10 ms and
  `tk_dly_tsk(10)` lasts two or three ticks, so reads are 20 or 30 ms apart
  (review F-26) and the newest of the 100 Hz samples is read (decision 6).
- A missing data-ready indication is polled again on the next period and
  is not immediately treated as a sensor failure.

## Known limitations and integration work

- Verified on the robot ([test log](../../testing/subsystem4/TEST_LOG.md#results)): wiring, axis
  orientation, still and driving noise, and the 10 mm right-wheel hump
  (S4-H08, 9 to 11 mm). Impact behaviour, temperature and other hump heights
  are not verified.
- The LSM303DLHC has an accelerometer and magnetometer but no gyroscope. The
  magnetometer is used only for identity checking; heading and turn rate are
  not implemented.
- Linear acceleration and vibration contaminate gravity-derived tilt. The
  moving average, pitch gate and driving gate (decisions 10 and 12) reduce
  them; at speeds well above line following (about 180 mm/s) vibration
  still reads as humps.
- The 32-bit microsecond timestamp wraps by design; elapsed calculations are
  wrap-safe only for intervals shorter than one full wrap period.
- Runtime thresholds are fixed provisional constants. SYS-04 has an agreed
  2 mm height criterion for course humps; wider terrain and impact criteria
  still require agreement and measurement.
- Run reset (START, IF-17), driving state (IF-14) and the telemetry read of
  the run maximum (IF-18, `subsystem4_get_status()`, never waits) are in
  place. Impacts use the same driving gate as humps (decision 14).
- The starter message buffer and event flag (IF-03) were removed on
  2026-11-20 (S2 decision 29).

## Terrain analysis report

The [S4 test log](../../testing/subsystem4/TEST_LOG.md#results) is the record
of methods, firmware revisions and results. The trials below compare the
height estimate while still, driving on flat ground and crossing a
ruler-measured hump. They use different firmware revisions; only the later
S4-H08 runs include the current 0.851 height scale and driving gate. The
[system test log](../../testing/system/TEST_LOG.md#test-cases) defines SYS-04:
the highest hump in telemetry must be within 2 mm of the tallest course hump,
with none reported on a flat lap. That complete system case has not yet run.

| Condition | Recorded result | Interpretation |
|---|---|---|
| Stationary after calibration, 60 s | Mean estimated height 0.18 mm, 99th percentile 0.48 mm, maximum 0.85 mm; no event ([S4-H03](../../testing/subsystem4/logs/2026-11-13_s4-h03-2.txt)) | Still noise alone was below the 2 mm level threshold in this set-up. |
| Five 450 mm flat passes at about 180 mm/s, before the longer tilt average | One false hump per pass, 18–24 mm; moving height 99th percentile 20.9 mm ([S4-H07](../../testing/subsystem4/logs/2026-11-13_s4-h03-2.txt)) | Motion and vibration dominated the still noise; the original filter was unsuitable at that speed. |
| Line following at about 40 mm/s with the 16-sample tilt average | About 82 s of driving: one 4 mm event on a smooth floor undulation; no vibration hump reported ([S4-H07](../../testing/subsystem4/logs/2026-11-13_s4-h07-line-avg.txt)) | The longer average reduced vibration effects in this trial, but did not prove zero false positives. |
| Ruler-measured 10 mm hump under the right wheel, after scaling and driving gate | Four crossings reported 11, 10, 10 and 9 mm, once per crossing; mean 10 mm ([S4-H08](../../testing/subsystem4/logs/2026-11-13_s4-h08-gated.txt)). A later three-pass repeat reported 10, 10 and 9 mm, with one additional false 6 mm hump and candidate impact ([repeat](../../testing/subsystem4/logs/2026-11-16_s4-raw-d14.txt)). | The known course-height case met the later SYS-04 ±2 mm height figure in these passes. It does not establish accuracy for other heights, contact points or surfaces. |
| Course laps over the 10 mm hump with telemetry | Two laps on 2026-11-17: the hump reported once per lap as 10 mm and valid in telemetry, with no pick-up stop afterwards; lap 1's console also shows no impact ([S4 test log](../../testing/subsystem4/TEST_LOG.md#results), [console](../../testing/subsystem1/logs/2026-11-17_s1-h01-fix-console.txt)) | The unconfirmed post-hump stops of 2026-11-16 did not recur in these laps. |
| Lift while following | Ten runs ended with a pick-up STOP in the S4-H11 session, although some post-hump stops were not confirmed as deliberate lifts ([S4-H11](../../testing/subsystem4/logs/2026-11-16_s4-h11-pickup-raw.txt)). SYS-10 then failed 0/5 on one boot and passed 5/5 on the next ([system results](../../testing/system/TEST_LOG.md#results)). | Lift detection is boot-dependent in the system record; the failed boot's cause was not captured. |

The measured hump was 195 mm across the path and 50 mm along it, with only
the right wheel crossing it ([initial S4-H08 trial](../../testing/subsystem4/logs/2026-11-13_s4-h08-10mm.txt)). The 0.851 scale was set from that one
height, so the result is calibration to the course case rather than an
independent accuracy study. The run maximum reached subsystem 2's summary
as 9 mm in the [IF-18 drive](../../testing/subsystem4/logs/2026-11-13_s4-if18.txt);
that capture does not establish delivery of every MQTT frame.

Candidate impact detection has no controlled physical validation. A false
candidate impact accompanied the extra 6 mm hump in the later repeat.
During three 2026-11-20 bypasses, rapid turns produced false 7 and 16 mm
humps and three false pick-up stops ([SYS-07](../../testing/system/TEST_LOG.md#results),
[dashboard capture](../../testing/system/logs/2026-11-20_system-dashboard.jsonl)).
[S4 decision 16](../../docs/decisions.md#16-no-detection-during-the-obstacle-bypass-2026-11-20)
now disables detection in BYPASS and REACQUIRE; that revision has not yet
been re-tested on the car. The evidence supports the measured 10 mm hump
case, while general terrain classification, impact behaviour and reliable
pick-up detection remain unverified.

The brief also lists motion classification (stationary, accelerating,
turning, climbing and descending a hump, sudden impact) and turn-rate
monitoring. Subsystem 4 classifies less: its states are calibrating,
monitoring, on a hump and fault, and its events are ready, hump complete,
impact, lift and fault (`terrain_logic.h`). Climbing and descending are one
on-hump state, and stationary, accelerating and turning are not reported.
Turn rate is not measured, because the LSM303DLHC has no gyroscope;
subsystem 2's encoder odometry supplies the heading instead.
