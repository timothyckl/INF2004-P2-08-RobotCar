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

The logic and decode cases (S4-U03 to S4-U34, less the retired IDs) also run on a computer under
strict C99 warnings with `make -C testing/host s4`; S4-H01 then reports
BLOCKED. A host pass checks the logic against synthetic input only.

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
| S4-U05 | One sample of a 10 mm one-wheel tilt between level samples. | No hump. |
| S4-U06 | A 10 mm one-wheel tilt held, then level. | One completed hump with a valid height. |
| S4-U07 | Retired 2026-11-12 with the ascent/descent method (decision 8). | Do not reuse. |
| S4-U08 | Invalid samples reach the configured consecutive limit. | Sensor fault and invalid current result. |
| S4-U09 | Invalid sample count below the configured limit followed by valid data. | No sensor fault; invalid streak clears. |
| S4-U10 | Valid timestamps cross `UINT32_MAX`. | Elapsed-time handling remains correct across wraparound. |
| S4-U11 | Gap between valid samples exceeds the configured maximum. | Stale-data fault. |
| S4-U12 | Humps of 5 mm then 10 mm. | 10 mm becomes the run maximum. |
| S4-U13 | Local recovery after a valid maximum. | Fresh calibration is required; run maximum is retained. |
| S4-U14 | Explicit new-run reset after a valid maximum. | Stored maximum and its validity are cleared. |
| S4-U15 | Retired 2026-11-12: height no longer uses wheel distance (decision 8). | Do not reuse. |
| S4-U16 | Dynamic acceleration exceeds the configured impact threshold. | One impact event is reported per crossing. |
| S4-U17 | Null context, configuration, sample, or result access. | Calls reject or ignore invalid pointers without a crash. |
| S4-U18 | Known raw bytes for +100 mg, -200 mg, and +1000 mg. | Axis order, 1 mg/LSB scaling, validity, and timestamp decode correctly. |
| S4-U19 | Raw values at the signed 12-bit limits. | Values decode as -2048 mg and +2047 mg without implementation-defined shifting. |
| S4-U20 | Null raw-data or destination pointer passed to the decode helper. | Decode is rejected without dereferencing the pointer. |
| S4-U21 | A one-wheel tilt held longer than `max_hump_ms`, then level. | No hump and no maximum; monitoring resumes once level. |
| S4-U22 | Retired 2026-11-12: height no longer sums distance (decision 8). | Do not reuse. |
| S4-U23 | 10 mm under the left wheel: roll +84 mg, pitch -57 mg. | 10 mm, left wheel. |
| S4-U24 | 10 mm under the right wheel: roll -84 mg, pitch -57 mg. | 10 mm, right wheel. |
| S4-U25 | 10 mm under both wheels: pitch -114 mg. | 10 mm, both wheels. |
| S4-U26 | 10 mm under the caster: pitch +114 mg. | 10 mm, caster. |
| S4-U27 | Pitch of 114 mg with pitch marked unreadable (accelerating). | No hump. |
| S4-U28 | 4-sample average; roll alternating +240/-240 mg each sample for 40 samples. | No hump starts. |
| S4-U29 | 4-sample average; 10 mm under the left wheel held 8 samples, then level. | 10 mm, left wheel. |
| S4-U30 | Average length 0, then `S4_AVERAGE_MAX_SAMPLES` + 1. | Both configurations rejected (fault). |
| S4-U31 | Hump detection disabled during a held 10 mm tilt; then enabled until a hump starts, and disabled before level. | No hump either time; the started hump is dropped unreported. |
| S4-U32 | Height scale 500 per mille; 10 mm under the left wheel. | 5 mm. |
| S4-U33 | Height scale 0, then `S4_HEIGHT_SCALE_MAX_PERMILLE` + 1. | Both configurations rejected (fault). |
| S4-U34 | Impact-sized sample while detection is disabled; enabled while still above the threshold; then level and a new crossing. | No impact until the new crossing, which is reported. |
| S4-U35 | Calibrated at 1 g. Two samples at 100 mg (a drop); one at +200 mg, then rest; two at +199 mg; then two at +200 mg, with a two-sample setting. | No lift until the second +200 mg sample. |
| S4-U36 | Averaged tilt of 299 mg forward and left, then 300 mg lateral (average of one sample). | No lift at 299 mg; lift at 300 mg. |
| S4-U37 | 400 mg tilt while disabled; enabled; held, levelled and tilted again; disabled and re-enabled with a new tilt. | No lift while disabled, one on enabling, none again until re-armed, then one. |
| S4-U38 | Lift up threshold 0, tilt threshold 0, samples 0, unaveraged tilt threshold 0, unaveraged samples 0. | Each configuration rejected (fault). |
| S4-U39 | 16-sample average filled level; a two-sample jolt (-900 mg forward, 260 mg left); 249 mg held three samples; then 250 mg lateral held three samples (unaveraged rule 250 mg, 3 samples). | No lift until the third 250 mg sample. |

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
| S4-H11 | Line following (`S3_LINE_TEST=1 S4_RAW_LOG=1`): lift the car by hand during a run, at least five times, including once slowly and level; also full laps over the hump without lifting. | Every lift latches STOP (`[S2] S4 PICKUP`) and the wheels stop; record the delay from the lift. No pickup during laps or over the hump. |

## Hump-measurement procedures (S4-H03, S4-H07, S4-H08)

Run in this order: the still noise (S4-H03) bounds what the moving and hump
results can show. All three use one image and one analysis script:

```sh
make -C build_make TEST=1 S2_CAL_TEST=3 S4_RAW_LOG=1 -j8
python3 testing/subsystem4/s4_raw_stats.py testing/subsystem4/logs/<log>.txt
```

The image runs the self-tests, then S4 calibrates (keep the car still) and
each GP20 press drives 450 mm forward. `S4_RAW_LOG=1` prints one `s4 raw`
line per IMU sample. The script splits samples into still and moving at the
`GP20: START requested` and `CAL hump pass ... finished` lines, recomputes the
firmware's height estimate (`s4_hump_height_um`) from the filtered tilts, and
reports it against the 2 mm level and 4 mm rise thresholds. Copy its output
into the result notes. The pass limits below are proposed, not agreed.

**Setup for all three.** Flat, hard floor; battery and USB; the GP28
battery-sense jumper open. Record the floor surface and the IMU board's
height above the floor (ruler, for reference; the tilt method does not use
it).

**S4-H03, still.** After calibration completes, leave the car untouched on
the floor for 60 s without pressing GP20. Then lift the car so the wheels
are clear, press GP20 and let the 450 mm pass run in the air, to separate
motor vibration from tilt; set the car down only after it finishes.
Proposed pass: no hump or impact event; still height p99 below 2 mm (the
level threshold), so noise alone can neither start nor hold a hump.

**S4-H07, driving straight on a flat floor.** On the floor, press GP20 for
at least five 450 mm passes, waiting for each to finish. Proposed pass: no
hump or impact event while moving; the pitch gate shows `ignored` around
the start and the stop. Any event names the gate settings (15 mm/s change,
8-sample hold) or the thresholds to revisit. Repeat later with the
`_s3line` image on the track (curves, scan and bypass spins).

**S4-H08, known humps.** Measure the hump with a ruler (height at its
highest point, width, profile) and photograph it. Start each pass with the
chosen contact about 100 mm before the hump; the 450 mm pass then crosses a
hump up to 195 mm long and settles (`task_controller.c`, mode 3). Run at least five passes per contact (left wheel, right wheel, and
both wheels if the hump is wide enough), pressing GP20 once per pass. Record
for each pass: measured height, reported height and contact, run maximum,
and any missed or extra event. Proposed pass: every pass detected once with
the right contact; height error to be agreed from the spread seen here.

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
| 2026-11-12 | S4-H01 | `227c7a8` + uncommitted stage 1 | Dalton | FAIL | `logs/2026-11-12_s4-h01.txt` | Identity and read-back passed on I2C1 GP2/GP3; every read then faulted (cause not yet printed), S4 restarted each second. |
| 2026-11-12 | S4-H01 | `227c7a8` + uncommitted stage 1 | Dalton | FAIL | `logs/2026-11-12_s4-h01-2.txt` | Each failure was an overrun (`-43`); the 10 ms kernel tick cannot read every 100 Hz sample (S4 decision 6). S4-U03..U20 18/18 on the Pico. |
| 2026-11-12 | S4-H01 | `227c7a8` + uncommitted stage 1 | Dalton | PASS | `logs/2026-11-12_s4-h01-3.txt` | Overrun no longer a fault. Car stationary: calibration complete, 200 samples in 5720 ms (about 29 ms per sample during the self-tests). S4 19/19. Mounting orientation not yet recorded (S4-H02). |
| 2026-11-12 | S4-H05, S4-H07 (partial) | `227c7a8` + uncommitted stage 1 | Dalton | OBSERVED | `logs/2026-11-12_s4-stage1-lap.txt` | `_s3line` image, no self-tests: calibration 200 samples in 5890 ms (about 29 ms per sample, under the 50 ms gap limit). From GP20 START to STOP (line following, one scan and left bypass with five turn-to-look cycles, line spin, one CLEAR scan): no S4 event and no fault. Two humps before START and two impacts after STOP occurred while both front IR sensors read about 4000 (car lifted and carried). One lap only; thresholds remain provisional. |
| 2026-11-12 | S4-H02 (axis check) | `227c7a8` + uncommitted stage 1 | Dalton | INCONCLUSIVE | `logs/2026-11-12_s4-h02-tilt.txt` | Hand tilts with the axis-mapped `_s3line` image: one hump and one impact, but the IR readings cannot separate pitch from roll. Likely cause, from the raw run: nose-up left S4 in ASCENDING (no time limit) and setting the car down completed the hump. |
| 2026-11-12 | S4-H02 (axis signs) | `227c7a8` + uncommitted stage 1 | Dalton | PASS | `logs/2026-11-12_s4-h02-raw.txt` | `S4_RAW_LOG=1`. Level: sensor x -20, y +27, z +900 mg. Nose up (about 11 deg): sensor y about -190, vehicle fwd +190. Left side up: sensor x about +145, vehicle left +145. Confirms X left, Y backwards, Z up (S4 decision 7). Z is about 100 mg below 1 g, beyond the typical +/-60 mg zero-g offset (datasheet table 3, typical only); calibration removes it. Six-orientation repeatability and angle accuracy not yet done (S4-H03, S4-H04). |
| 2026-11-13 | S4-H03 (still) | `d6a90ca` (`TEST=1 S2_CAL_TEST=3 S4_RAW_LOG=1`) | Dalton | INCOMPLETE | `logs/2026-11-13_s4-h03.txt` | Cancelled before the motor pass. Calibration 50 samples in 1020 ms (about 20 ms per sample). Two events from handling, not noise: 11 mm (both wheels; up fell to 758 mg, left spiked 120 mg for 0.3 s) and 5 mm (caster). Clean window: height p99 0.89 mm, single-sample spikes to 3.8 mm, no event. |
| 2026-11-13 | S4-H03 (still) | `d6a90ca` (`TEST=1 S2_CAL_TEST=3 S4_RAW_LOG=1`) | Dalton | MEASURED | `logs/2026-11-13_s4-h03-2.txt` (first boot) | Car on the floor, untouched 60 s after calibration (50 samples in 1360 ms): height mean 0.18 mm, p99 0.48 mm, max 0.85 mm; tilt within +/-9 mg; no event; about 28 ms per sample. Still noise is well below the 2 mm level threshold. |
| 2026-11-13 | S4-H03 (motor vibration) | as above | Dalton | FAIL (proposed criterion) | `logs/2026-11-13_s4-h03-2.txt` (runs 1 and 2) | Run 1 held in the hand is invalid (car tilted about 19 degrees nose up; the hump timed out, no event). Run 2 on a block, wheels free, hands off: tilt swung -52..+36 mg forward and -14..+69 mg left; two false humps, 6 and 8 mm, left wheel. At rest on the block the offset was about 3 mm (block not level). |
| 2026-11-13 | S4-H07 (flat floor, straight) | as above | Dalton | FAIL (proposed criterion) | `logs/2026-11-13_s4-h03-2.txt` (after the second boot) | Recalibrated on the start spot (one stale fault and sensor restart just after boot, recovered). Five 450 mm passes, each gave one false hump: 23 (right wheel), 18 (caster), 23 (both wheels), 21 (both wheels), 24 mm (caster). While moving: accel fwd -796..+555 mg, up 403..1403 mg; filtered tilt fwd sd 73..99 mg, left sd 32..51 mg; height p99 20.9 mm, 67 % of samples above the 4 mm rise threshold. The pitch gate covered start and stop only. Vibration, not tilt, dominates: the current filter and thresholds cannot reject driving on this floor, so the 2026-11-12 hump heights include vibration. Other events in the log came from carrying the car between passes. |
| 2026-11-13 | S4-H07 (trial: 25 Hz output rate) | `d6a90ca` + `CTRL_REG1_A` 0x37, gap limit 120 ms (reverted) | Dalton | FAIL; trial reverted | `logs/2026-11-13_s4-h07-25hz.txt` | Still on the table 60 s: height p99 0.65 mm, max 1.0 mm, samples 39 ms apart (every sample read). Then on the floor, three 450 mm passes: false humps of 35 (caster), 33 (both wheels), 42 (right wheel) and 27 mm (caster); accel fwd -426..+383 mg, left -408..+164 mg, alternating every 2 to 3 samples (about 8 to 12 Hz). The ODR/9 bandwidth (datasheet table 8, about 2.8 Hz) did not remove it. Offline averaging of the cruise samples: 600 ms leaves 10 to 13 mm at 25 Hz against 5 to 8 mm at 100 Hz, so 25 Hz was reverted. At 25 Hz sensor X (vehicle left) repeated its previous value in 83 of 207 moving samples (13 of 1644 at 100 Hz); cause not yet known. Events between passes came from carrying the car (table to floor, back to the start spot). |
| 2026-11-13 | S4-H07 (line following on straight tape, about 40 mm/s) | `d6a90ca` + 100 Hz, `TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1` | Dalton | MEASURED; one stall | `logs/2026-11-13_s4-h07-line.txt` (first and second boots) | Floor, straight tape. Run 1 (8 s, ended by an obstacle scan at the tape end) and the first part of run 2: height p99 4.3 mm, max 5.4 mm, up sd 8 mg against 107 mg in the 180 mm/s passes; no false hump. Pressing GP20 jolted the car (a 14 mm hump at START), and the hazard stop's braking rolled it (left tilt +254 mg). During run 2 the IMU task stopped printing with no fault and did not recover until reset; the obstacle task (priority 20) still ran, so it was blocked, not starved. Cause not found. |
| 2026-11-13 | S4-H07 (line following, stall watcher build) | as above + `s4_watch_task` diagnostic (`S4_RAW_LOG` only) | Dalton | MEASURED | `logs/2026-11-13_s4-h07-line.txt` (third boot) | Four runs of 43 to 46 s (about 177 s of driving), IMU did not stall (no watcher report). Height p99 3.9 to 5.1 mm, up sd 9 to 10 mg. Two false humps while driving: 6 mm (both wheels, run 1) and 7 mm (both wheels, run 4); other events came from GP20 presses, the stop, or carrying the car. Offline, an extra mean of the filtered tilt over 400 ms keeps the cruise maximum at 2.4 to 3.0 mm in all four runs (150 ms: 3.3 to 5.7 mm; 250 ms: 2.7 to 4.1 mm). |
| 2026-11-13 | S4-H07 (line following, 16-sample tilt average, S4 decision 10) | `6625ebf` + decision 10, `TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1` | Dalton | MEASURED | `logs/2026-11-13_s4-h07-line-avg.txt` | About 21 ms per sample in this image, so the window is about 340 ms. First boot (calibrated off the tape, resting offset about 2.2 mm): runs of 10.9 and 9.1 s, no hump while driving. Second boot, recalibrated on the tape (resting tilt 0/0 mg): runs of 31.0 and 31.5 s; tilt sd 9 to 10 mg forward and 6 mg left (15 and 9 mg with the old filter); height p99 3.4 to 3.6 mm. One hump of 4 mm (right wheel) about 21 s into the first long run: a smooth tilt of about -20 mg forward and -10 to -25 mg left held for about 0.6 s, so a floor undulation, not vibration. In about 82 s of driving, no vibration hump (two in 177 s before). Events after STOP came from handling. No IMU stall. |
| 2026-11-13 | S4-H08 (10 mm hump, right wheel) | `159de9b` (`TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`, rise 4 mm) | Dalton | MEASURED | `logs/2026-11-13_s4-h08-10mm.txt` | Hump 10 mm high (ruler), 195 mm across the path, 50 mm along it; line following at about 40 mm/s, calibrated on the start spot. Right wheel only (course constraint, S4 decision 11). Clean crossings (runs 2, 4, 5, 9): 11, 12, 12, 12 mm, right wheel, each detected once, no other event while driving; mean about 11.8 mm (+1 to +2 mm). Run 6 stopped on top: steady 11.2 mm (roll -94 mg). Not valid: run 1 (hump on the left; the wheel did not climb it), run 3 (the hump reached under the caster: 8 mm, caster), runs 6 to 8 (the line follower read BOTH BLACK at the hump, confirmed a junction and stopped at its recovery bound). The last is a line-sensing issue for subsystem 3: with the hump about 3 cm right of the tape centre, crossing it stopped line following in three of eight runs. Events between runs came from carrying the car. |
| 2026-11-13 | S4-H08 (correction to the row above) | — | Dalton | CORRECTION | — | The line-follow stops in runs 6 to 8 were not a subsystem 3 issue: the line sensors had been moved and not adjusted back. Only one hump height (10 mm) is used on the course. |
| 2026-11-13 | S4-H08 (10 mm hump, right wheel, S4 decision 12) | `03237ef` (`TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`; scale 851, rise 6 mm, humps only while driving) | Dalton | PASS | `logs/2026-11-13_s4-h08-gated.txt` | Same hump and setup as the earlier S4-H08 row; calibrated on the start spot. Four runs: 11, 10, 10, 9 mm, right wheel, exactly one hump per run (mean 10.0 mm, all within +/-1 mm; run 1 raw peak 12.5 mm, scaled 10.6). No hump while idle, at GP20 START or STOP, or while the car was placed and carried between runs (raw log `humps off`); handling still raises impact events, which are not gated. |
| 2026-11-13 | S4 stage 3, IF-18 (highest hump in telemetry) | `630e98c` + S4 decision 13 and the stall watcher (`TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`) | Dalton | PASS | `logs/2026-11-13_s4-if18.txt` | Line following over the 10 mm hump (right wheel): `s4: hump 9 mm under the right wheel`, then at GP20 STOP `[S2] run summary: distance 593 mm, highest hump 9 mm`, read through `s2_read_telemetry()` (IF-18). The first image's two runs were stopped by `line: stale report` / `no report after START`: the S3 line loop stopped printing mid-run and never resumed, while the IMU and the lower-priority obstacle task kept running (blocked, not starved; the same symptom as the IMU stall). The watcher now covers the S3 line loop too. |
| 2026-11-16 | S4 decision 14 (impacts only while driving) | `f25c7e8` (`TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`) | Dalton | PASS | `logs/2026-11-16_s4-raw-d14.txt` | Calibrated on the start spot (50 samples in 1270 ms). Idle 45 s, then picked up, carried, set down and tapped without GP20 (15:31:30..15:31:50: tilt up to 478 mg, up 610..1207 mg): no hump, no impact, every raw line `detect off`. Two flat line-follow runs (658 and 680 mm, curves and a junction each): no hump, no impact, run summary `highest hump 0 mm (none)`. The right front IR sensor had been refitted earlier the same day at a new spacing (S3 log, 2026-11-16). |
| 2026-11-16 | S4-H08 (10 mm hump, right wheel, repeat) | `f25c7e8` (same image) | Dalton | PASS (height); one false hump | `logs/2026-11-16_s4-raw-d14.txt` | Three runs: 10, 10, 9 mm, right wheel; each run summary matches. Run 3 also reported `hump 6 mm under the both wheels` and an impact at 15:34:26 with no physical hump (tester): a single sideways spike of -379 mg (up 735 mg) and about -60 mg forward tilt for 0.2 s, about 2 s after a junction as a sustained LEFT began. Cause not confirmed (caster snapping round is one candidate); the run maximum stayed at the real 10 mm. The 41 mm hump of the 2026-11-16 S3 line run did not recur. Runs 4 and 5 lost the line on the hump (next row). |
| 2026-11-16 | Line following over the hump (S3/S2 observation) | `f25c7e8` (same image) | Dalton | FAIL (2 of 3 hump runs) | `logs/2026-11-16_s4-raw-d14.txt` | Runs 4 and 5 met the hump within 1 s of a junction exit, while the car was turning left. From the hump to STOP both front sensors read WHITE/WHITE (right 177..193, a normal white level), which the controller treats as centred, so the car drove straight off the track. Tester: the right IR sensor did not detect the black line on the hump. Candidates: the raised front-right sensor reads tape as white, or today's new sensor spacing; the four clean runs of 2026-11-13 with the old spacing stayed on the line. Run 3 crossed the hump and kept following. Not an IMU fault: the heights were correct. Cause: the left sensor read the tilted white as BLACK (S3 log); fixed by `ac46f35` (S3 decision 8), 5/5 hump runs then stayed on the line |
| 2026-11-16 | S4-H11 (pick-up stop, first trial) | `6fbb53d` without the unaveraged-tilt rule (built from the working tree before it was added; `TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`) | Dalton | FAIL (delay, light lift) | `logs/2026-11-16_s4-h11-pickup.txt` | Three runs, lifted while line following. Run 1 (tipped nose-up, +93 mg up): averaged tilt fired about 530 ms after the lift began. Run 2 (sharp lift, +206 mg): upward rule, about 200 ms. Run 3, a light lift (about 100 mg tilt each axis, at most +184 mg up): not detected; the line sensors read 3700+ off the floor and the junction recovery bound stopped the car about 2 s later. Tester: deliberate lifts stop with about half a second of delay, a light lift does not. Fixed for the delay by the unaveraged rule in `6fbb53d`; the light lift reads like a hump and remains undetected |
| 2026-11-16 | S4-H11 (pick-up stop with the unaveraged-tilt rule) | `6fbb53d` (`TEST=1 S3_LINE_TEST=1 S4_RAW_LOG=1`) | Dalton | PASS (lifts); hump laps provisional | `logs/2026-11-16_s4-h11-pickup-raw.txt` | Ten runs, each ended by `[S2] S4 PICKUP: car lifted, STOP latched`. Upward rule in runs 1, 2, 5 to 10 (+275 to +514 mg), 0 to 20 ms after the lift began; unaveraged tilt rule in run 3 (about 40 ms) and run 4 (about 240 ms, tilt building slowly). Averaged rule never needed. Four hump crossings (10 to 12 mm, right wheel; runs 4, 8, 9, 10) gave no pickup; the following stops came 0.5 to 5 s after the hump ended with the car level. Not confirmed by the tester that those stops were lifts. Runs 5 to 7 lasted 1 to 2 s, so the 0.7 s settle gate was short of the lift. No light lift recorded in this session. Bypass manoeuvres not covered |
