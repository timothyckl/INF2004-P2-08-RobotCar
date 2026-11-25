# Post-demo engineering proposals

These are the corrections still outstanding. Since 2026-11-25 post-demo
corrections are applied on `main` one at a time (S2 decision 35); each one
applied is removed from this list and described where it now lives (the
code, its tests, its decision record and section 9 of the
[software review](software_quality_review.md)). Git history keeps the
earlier versions of this document. The demonstrated image (`a41c19c`, UF2
SHA-256 `8cf2c419…`) contains none of them.

Findings are numbered as in the [software review](software_quality_review.md).
"Host evidence" means synthetic tests on the computer; nothing here has
been run on the car.

| ID | Requirement | Problem | Proposed change | Car test needed |
|---|---|---|---|---|
| P1 (F-01) | FR2, FR3 | S3 loop runs every 20 ms, not 10 ms | Cyclic handler wakes the loop every 10 ms | SYS-01, SYS-02 |
| P3 (F-02, F-03) | FR4 | One-sided branches are not junctions; no record of why no turn happened | Junction event in telemetry first; then a bench test of the course junction | SYS-03 |
| P4 (F-17) | FR11 | A car fully off the line reads as centred | Hardware: a centre sensor; software alone cannot see it | SYS-01 |
| P5 (F-04) | FR5 | No hump reads below 6 mm; small humps read high | Two-point calibration, then review the 6 mm rise | SYS-04 with a flat lap |
| P6 (F-05) | FR7 | Width overstated (SYS-06) | Set the beam allowance from a measured beam, and report width only when both edges are clear | SYS-06 |
| P7 (F-08) | NFR1, Tri2 Entry 4 | Watchdog off | Arm it, fed only when every critical task has checked in | One run with it armed |
| P8 (F-06) | FR10, Tri2 Entries 11, 19 | One fault class in telemetry | Version 5 frame: fault detail and a junction bit | Dashboard and SYS-08 |
| P9 (SYS-09) | FR1, MR1–MR10 | No full course completed in one run | A staged run plan that removes the known blockers first | SYS-09 |

### Cost and what stops each one going in now

"Complexity" counts the code that would change. Costs are worked out from
the code.

| ID | Complexity | Timing, memory, CPU | Host evidence today | What blocks it now |
|---|---|---|---|---|
| P1 | Small: one cyclic handler and one semaphore in `s3_follow.c` | Loop runs twice as often; its worst-case time is not measured, so its CPU share is unknown. No new buffers | S3-U02, U03 already sample at 10 ms | Changes line following; needs SYS-01 and SYS-02 on the car |
| P3 | Step 1 is P8; step 3 is unknown until the bench test | — | S2-U58, U74 to U80 | Junction shape unknown; needs a bench drive with the console captured |
| P4 | Hardware: one sensor, one ADC input. No free ADC input is left (ADC3/GP29 is the radio), so it needs a digital sensor on a spare GPIO or a multiplexer | One more read per S3 pass | None | Needs new hardware and wiring |
| P5 | Small in code (two constants and one formula); most of the work is measurement | None | S4-U41, U42 | Needs reference humps and a flat lap on the car |
| P6 | Medium: allowance value, a validity rule, optional 2.5° fine steps | 2.5° steps add servo moves and echoes: a longer scan | S5-U72 | Needs beam measurements with the real box |
| P7 | Medium: a check-in word shared by four tasks | One word under the existing mutex; a few instructions per cycle | None | A wrong timeout resets the car mid-run; needs one armed run first (S2 d28) |
| P8 | Medium: encoder, decoder, dashboard and golden frame | +1 byte per frame (8 bit/s) | S1-U05 pattern | Changes the frame format; dashboard and SYS-08 must be re-run |
| P9 | Process, no code | — | — | Needs the course and track time |

## P1 — 10 ms sampling for the line and barcode loop (F-01)

**Where.** `app_program/subsystem3/s3_follow.c`, `s3_follow_run()`;
`S3_LINE_SAMPLE_MS` (10); kernel tick `CNF_TIMER_PERIOD` (10 ms,
`config/config.h`).

**Current behaviour.** The loop ends each pass with
`tk_dly_tsk(S3_LINE_SAMPLE_MS)`. µT-Kernel rounds a delay up to whole
ticks and starts counting at the next tick, so a 10 ms delay on a 10 ms
tick lasts between 10 and 20 ms, plus the loop's own run time. All 953 bar
durations logged on the car (629 on 2026-11-17, 324 on 2026-11-24) are
within 1 ms of a multiple of 20 ms, so the loop actually runs every 20 ms.
The IMU task, which uses the same pattern, was measured at about 21 ms per
sample (S4 decision 12 note).

**What sampling the barcodes need.** Bar timings are measured to the
sampling period, so each element should span at least about three samples
for one sample of error to stay near a third of its length:

| Quantity | Formula | 20 ms (now) | 10 ms (proposed) |
|---|---|---|---|
| Travel per sample at 40 mm/s | v × T | 0.8 mm | 0.4 mm |
| Shortest reliable narrow element (3 samples) | 3 × v × T | 2.4 mm | 1.2 mm |
| Large print narrow element (runs 120–220 ms, 2026-11-11) | | 4.8–8.8 mm: 6–11 samples, fine | 12–22 samples |
| Demo-day console: runs of one or two samples | | 30 of 324 runs (19–39 ms) | would be two to four samples |

The optics set a second limit. The TCRT5000 datasheet gives a peak
operating distance of 2.5 mm and a useful range of 0.2–15 mm; further from
the paper the spot widens and a narrow space between two bars reads grey.
This matches the team's observation that the small print decoded only
after the sensor was lowered. Faster sampling cannot fix a blurred spot,
so P1 must go with a recorded sensor height.

**Proposed design.** Create a 10 ms cyclic handler (as Motion Control does
in `task_motion.c`) that signals a semaphore, and replace the
`tk_dly_tsk()` with a wait on it. The period becomes exactly one tick. The
filter keeps 3 samples, so it then spans 30 ms instead of 60 ms; the
report period (`S3_LINE_REPORT_MS` 30) is met instead of becoming 40.

| Alternative | Why not chosen |
|---|---|
| `tk_dly_tsk(5)` | Still rounds to one or two ticks; the period would jitter between 10 and 20 ms |
| 1 ms kernel tick | Ten times the tick interrupts for every task; changes all timing in the system |
| Edge interrupts on the barcode sensor | Sensor is analogue (ADC2) with an adaptive threshold; would need a comparator output and a new design (Tribunal 1 R2 considered and deferred this) |

**Risks.** The S3 loop runs at priority 1, above everything else. Twice as
many passes doubles its CPU share; if a pass (ADC reads, filter, decoder,
console print) ever takes more than 10 ms, the semaphore count builds up.
The loop's worst-case time has not been measured. The steering filter
responds twice as fast, so line following must be re-checked.

**Acceptance.** On the car: logged bar durations become multiples of
10 ms; SYS-01 three laps; SYS-02 five passes per print with the sensor
height recorded, at least 4 of 5 and no wrong letter. Regression: all S3
host checks (S3-U02, S3-U03 already model 10 ms sampling).

## P3 — Junction detection and the missing turn (F-02, F-03)

**Where.** `app_program/subsystem2/line_control.c` (junction confirmation),
`nav_control.c`, `task_controller.c` `nav_receive()`, `nav_update()`.

**What the demo logs show.** RIGHT was decoded five times in the assessed
session and the car never entered NAVIGATE. In the 1 Hz frames after each
decode the line position never read BOTH, while BOTH frames appear at other
times. The practice session completed a U-turn with the same logic. The
held command lasts 60 s from the decode and the policy is not stepped
during a scan or bypass (S2-U80); the hold, the junction confirmation and
any expiry are printed only on the USB console, which was not captured.
**The cause of the missing turn is therefore not established**, and we do
not claim that the command expired.

**Design limitation.** A junction is confirmed only when both front
sensors read black for 100 ms. The sensors are 30 mm apart over 18 mm tape.
A cross or T covers both; a branch leaving on one side while the main line
continues covers only one, which the line follower treats as a curve. The
shape of the demo junction was not recorded.

**Proposed changes, in order.**

1. Add a "junction confirmed" bit and the held command to telemetry (P8),
   so the next missed turn shows why.
2. Bench test: drive the car slowly over a copy of the course junction with
   the console captured; record what the two sensors report.
3. Only if a one-sided branch is confirmed: treat "one sensor black for
   longer than a curve normally lasts, while a command is held" as a
   junction. Changing this without step 2 could make curves look like
   junctions, so it is not proposed blind.

**Acceptance.** SYS-03: 3 runs of each command, console captured.
Regression: S2-U58, U74 to U77, U79, U80.

## P4 — Detecting a car that has left the line (F-17)

**Where.** `line_control.c`, `task_controller.c` `line_step()`.

**Limitation.** The two sensors straddle the tape, so both white is the
normal centred reading; a car that has left the line completely also reads
both white. The current checks catch a stale or invalid report, a crossing
over 1.5 s and an exit over 2.5 s, but not this case.

**Can software detect it?** Not reliably with these sensors. A rule such as
"no sensor has seen black for X mm" would also fire on a long straight
where the car stays centred, which is the normal case; the length of the
longest straight on the course was not recorded. Curve holding (a sensor
seeing black turns the car until the other side is seen) is why SYS-01
passed, but it is mitigation, not detection.

**Proposed change.** Hardware: a third, centre sensor over the tape (the
brief lists three IR sensors for buddy 3). Centre black and both sides
white then means centred; all three white means lost, and the controller
can latch STOP with cause line. Until then the limitation stays in the
fault table.

| Alternative | Why not chosen |
|---|---|
| Time-out when neither sensor has seen black for a set distance | Fires on a long straight while the car is correctly centred, the normal case |
| Use the barcode sensor as a third line sensor | It sits at the side for barcodes and never steers (pin mapping); moving it breaks barcode reading |
| Wiggle the car to look for the tape | Changes line following on every straight and adds no certainty on curves |

**Acceptance.** On the car: lift-and-place the car 50 mm beside the tape
while following; STOP with cause line within 200 ms (value to be agreed),
five times out of five, and no false line-loss stop on three SYS-01 laps.

## P5 — Hump measurement below 6 mm (F-04)

**Where.** `app_program/subsystem4/task_imu.c` (`S4_RISE_UM` 6000,
`S4_HEIGHT_SCALE_PERMILLE` 851); `terrain_logic.c` `s4_hump_update()`,
`s4_hump_complete()`.

**Why 6 mm is the minimum.** A hump starts only when the scaled height
reaches 6 mm and the result is rounded to whole millimetres (S4-U41). The
6 mm rise was chosen because the floor reads up to about 4 mm while driving
(S4 decision 11). An ideal 5 mm step scales to about 4.3 mm and is not a
hump at all; the demo hump read 6, 6 and 7 mm, so the car's raw estimate
over it was 7 to 8 mm.

**Can 5 mm be measured reliably?** Not with the current margins. Noise at
rest is about 10 mg, about 1 mm of height, and driving adds floor
undulations of up to about 4 mm. A 5 mm hump is only about 1 mm above the
largest floor reading, so lowering the rise to, say, 4 mm would trade
missed small humps for false humps on the floor.

**Calibration error.** The scale 0.851 comes from one 10 mm hump (11.75 mm
raw). A 5 mm hump reading 7 to 8 mm raw suggests an offset of a few
millimetres rather than a fixed percentage. One point cannot separate the
two.

**Proposed procedure.**

1. Drive over reference humps of 5, 10 and 15 mm (measured with a ruler),
   five times each; log the raw peak (`S4_RAW_LOG`).
2. Fit raw = gain × true + offset by least squares; use both in
   `s4_hump_complete()` (integer µm arithmetic, as now).
3. Run a flat lap with the same build and record the largest floor peak;
   set the rise above it with a margin.

**Acceptance.** SYS-04 within ±2 mm on all three heights and no hump on the
flat lap. Regression: S4-U41 (update its constants with the new rise),
S4-U42 (monotonic reading), S4-U05, U06, U12 to U14.

## P6 — Obstacle width (F-05, SYS-06)

**Where.** `app_program/subsystem5/src/scan_profile.c`
`scan_profile_make()`, `scan_profile_side()`; beam allowance 10 degrees in
`scan_task.c`.

**What limits the width.** S5-U72 and its
[log](../testing/subsystem5/logs/2026-11-25_s5-width-geometry.txt) separate
the causes using synthetic scans of a box face 240 mm away:

| Case | 150 mm box | Note |
|---|---|---|
| Ideal thin rays, no allowance | 128 mm | Edges known only to the 5 degree ray spacing: about 23 mm per side at 240 mm |
| 21 degree beam, raw span | 282 mm | Matches the 166–332 mm raw widths logged on demo day |
| 21 degree beam, mission allowance 10 degrees | 246 mm | The SYS-06 overstatement |
| 21 degree beam, allowance 20 degrees | 168 mm | Within the 20 mm criterion |
| Ideal rays, allowance 20 degrees | no shape | An allowance larger than the edge angle removes the box |

So the edge logic and geometry are sound; the error comes from the beam
and from an allowance that does not match it. S2 decision 18 measured an
effective half-beam of about 21 degrees, wider than the datasheet's 15
degree measuring angle, but it varied with the car's heading at the stop
(the box vanished between 20 and 35 degrees on different stops). That is
why 15 degrees was rejected then: two scans put an edge inside the box.

**Proposed change.** Do not subtract a fixed amount from the width.
Instead: (1) measure the effective beam on the car against the 15 × 20 cm
box at 200, 250 and 300 mm, five stops each, heading recorded; (2) set the
allowance to the smallest measured half-beam, so an edge is never placed
inside the box (safe for planning); (3) report a width as valid only when
both edges are found and the two edge rays are within the 5 degree fine
spacing. Even then the 5 degree spacing limits the width to about ±23 mm
at 240 mm, just outside the 20 mm criterion; meeting it would need 2.5
degree fine steps near each edge (servo resolution is 10 µs per degree,
so this is possible) at the cost of a longer scan.

**Acceptance.** SYS-06 with the box both ways round. Regression: S5-U59 to
U63, U67, U72; SYS-07 to show bypass planning is unchanged.

## P7 — Watchdog (F-08)

**Where.** `app_program/subsystem2/task_motion.c` (`S2_WATCHDOG` 0),
`drv_watchdog.c`.

**Why it is off.** S2 decision 28 left it off until one car run with it
armed shows no unwanted reset; S2-H05 checked it only on the bench (the
motor stopped within the 200 ms timeout when kicking stopped, 2026-11-10).
It must never be armed in a `TEST=1` image, because the unit tests can
starve the motion task on purpose. As written, only Motion Control would
kick it, so a stalled controller or line loop would not be noticed while
Motion Control kept running (Tribunal 2 Entry 4). The 2026-11-23 dual-core
stalls showed exactly that kind of failure: one task stopped, the others
ran on.

**Proposed design.** Each safety-critical task (controller, motion, S3
loop, IMU) sets its own bit in a check-in word each cycle; Motion Control
kicks the watchdog only when every bit is set, then clears them. Timeout
200 ms: about 20 controller periods and 10 S3 periods at the current
20 ms, long enough for normal jitter, short enough to stop the motors
quickly. After a watchdog reset the car boots into INITIALISING with STOP
latched (as now) and `drv_watchdog_caused_reset()` is reported, so it never
drives off on its own.

**Risks.** A task that legitimately blocks longer than 200 ms (an I2C
retry, a long console print) would reset the car; worst-case blocking times
are not measured. Hence the car run before arming.

**Acceptance.** One full SYS-01 run armed with no reset; then a forced stall
of each task (a test build) resets the board and the motors stop.

## P8 — Fault detail and junction events in telemetry (F-06)

**Where.** `app_program/subsystem1/telemetry_encoder.c`, `robot_state.c`,
`testing/subsystem1/dashboard/`.

**Problem.** The frame carries one 4-bit fault class. On demo day four
motion faults latched STOP correctly, but whether each was a stall, an
encoder fault or a timeout is unknown. The junction and hold messages
(P3) are console-only.

**Proposed format (version 5, 32 bytes).** Append one byte: bits 0–2 the
motion fault detail (`motion_status_t.fault`, 0..7), bit 3 junction
confirmed since the last frame, bits 4–6 the held barcode command
(`S2_NAV_*`, 0..4), bit 7 reserved. One byte per second, 8 bit/s more. The
dashboard and `decode_telemetry_v2.py` already read older versions by
length and version byte, so versions 2 to 4 still decode. The junction bit
is set by the controller and cleared when the snapshot is taken, under the
existing state mutex, so no event is lost between frames.

**Not proposed.** A full independent fault bitmask (Tribunal 2 Entries 11
and 19) adds more bits than the evidence needs today; Tribunal 2 asked for
analysis, not implementation.

**Acceptance.** S1-U05 golden frame for version 5; dashboard tests; SYS-08
shows the detail on a forced stall.

## P9 — A full course in one run (SYS-09)

**Where it stands.** Every mission element has worked on the car on its
own (traceability section 2), but no run has finished the whole course.
On demo day attempt 3 decoded the barcode, bypassed the obstacle three
times and measured the hump, but did not turn at the junction (section 4
of the [demo report](DEMO_REPORT_2026-11-24.md)).

**Why it has not been met.** The run depends on every element working in
one go, so the blockers add up: the junction turn (P3), barcode reads
below 4 of 5 (P1; F-18 is fixed) and line loss going unnoticed (P4). There is no
separate root cause to fix.

**Proposed plan.** Apply and car-test P8, then P1 and the F-18 fix, then do the P3
bench test, one at a time, each with SYS-01 re-run to show line following
is unchanged. Then three full-course runs with the console captured and
the time recorded.

**Acceptance.** SYS-09 as written: the whole course after one START, no
touching, three runs.
