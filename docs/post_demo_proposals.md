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
| P3 (F-02) | FR4 | One-sided branches are not junctions (the hold time limit, part of F-03, is fixed) | Junction event in telemetry first; then a bench test of the course junction | SYS-03 |
| P4 (F-17) | FR11 | A car fully off the line reads as centred | Hardware: a centre sensor on GP6 (digital, 3V3); software alone cannot see it (logs: over 1.1 m of both-white while following correctly) | SYS-01 |
| P5 (F-04) | FR5 | Small humps read high (the threshold part is fixed, S4 d17) | Two-point calibration on the car with the prepared tool | SYS-04 with a flat lap |
| P6 (F-05) | FR7 | Width overstated (SYS-06) | Set the beam allowance from a measured beam, and report width only when both edges are clear | SYS-06 |
| P7 (F-08) | NFR1, Tri2 Entry 4 | Watchdog off by default (the task check-ins and liveness guard are done, S2 d37) | Arm `S2_WATCHDOG=1` by default after one car run | One run with it armed |
| P9 (SYS-09) | FR1, MR1–MR10 | No full course completed in one run | A staged run plan that removes the known blockers first | SYS-09 |

### Cost and what stops each one going in now

"Complexity" counts the code that would change. Costs are worked out from
the code.

| ID | Complexity | Timing, memory, CPU | Host evidence today | What blocks it now |
|---|---|---|---|---|
| P3 | Step 1 done (telemetry v5, S1 d8); step 3 is unknown until the bench test | — | S2-U58, U74 to U80 | Junction shape unknown; needs a bench drive with the console captured |
| P4 | Hardware: one sensor, one ADC input. No free ADC input is left (ADC3/GP29 is the radio), so it needs a digital sensor on a spare GPIO or a multiplexer | One more read per S3 pass | None | Needs new hardware and wiring |
| P5 | Code done (offset in `s4_config.h`, tool `fit_hump_calibration.py`); the work left is measurement | None | S4-U41 to U44 | Needs reference humps of 5, 10 and 15 mm and a flat lap on the car |
| P6 | Medium: allowance value, a validity rule, optional 2.5° fine steps | 2.5° steps add servo moves and echoes: a longer scan | S5-U72 | Needs beam measurements with the real box |
| P7 | Code done (`task_health.c`, `S2_WATCHDOG` make option); the default is one line in the makefile | 20 bytes of RAM; a few loads per 10 ms | S2-U81, U82, INT-15 | A wrong limit would stop or reset the car mid-run; needs one armed car run first (S2 d28) |
| P9 | Process, no code | — | — | Needs the course and track time |

## P3 — Junction detection and the missing turn (F-02, F-03)

**Where.** `app_program/subsystem2/line_control.c` (junction confirmation),
`nav_control.c`, `task_controller.c` `nav_receive()`, `nav_update()`.

**What is already done.** The demo-day telemetry shows the held RIGHT
expired 2 s into the obstacle stop in both assessed runs (58 s from the
decode), so no junction after the obstacle could be turned at. That limit
is removed (S2 decision 36) and INT-11 tests the case. Whether the course
junction came before or after the obstacle is not in the logs, so a
one-sided junction (below) may also have played a part.

**Model evidence.** In the host integration model (INT-11), a junction
whose branch leaves to the right while the line carries on is first seen
as "sustained RIGHT": the follower steers into the branch as on a curve
before both sensors go black. That is the behaviour described below; it
was not measured on the car.

**Design limitation.** A junction is confirmed only when both front
sensors read black for 100 ms. The sensors are 30 mm apart over 18 mm tape.
A cross or T covers both; a branch leaving on one side while the main line
continues covers only one, which the line follower treats as a curve. The
shape of the demo junction was not recorded.

**Proposed changes, in order.**

1. Done: telemetry version 5 carries the held command and a junction
   count (S1 decision 8), so the next missed turn shows why.
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

**Can software detect it? Re-assessed 2026-11-26 with the logs: no, not
safely.** Over every dashboard log (767 stretches of correct following
with both sensors white), the longest is about 1.1 m, the median 40 mm
([analysis](../testing/system/logs/2026-11-26_line-loss-analysis.txt)):
on the start straight the car stays centred for over a metre. A
"no black for X mm" rule would need X well above that, about 2 m, to
avoid stopping on a straight whose length on the course is not recorded,
and a car that had really left the line would be off the mat by then. The
heading cannot help either: on a straight and after missing a curve the
car holds a steady heading alike. So no software rule is added; curve
holding (`CTRL_LINE_*_HOLD_MS`) remains the mitigation that makes SYS-01
pass.

**Proposed change (hardware, needs approval).** A third, centre sensor
over the tape: centre black with both sides white is centred, all three
white is lost, and the controller can latch STOP with cause line. Checked
against the datasheets and the pin map:

- No ADC input is free (ADC0-2 are the line and barcode sensors, ADC3/GP29
  is the radio), so it must be a digital output: a TCRT5000 module's LM393
  comparator DOUT. The LM393 output is open-collector, pulled up by the
  module to its supply; powered from 3V3 (Robo Pico Grove 3V3, 300 mA in
  total for the Grove ports), DOUT stays at or below 3.3 V, within the
  RP2040's IOVDD + 0.5 V pin limit.
- A free pin: GP6 or GP7 on Grove 5 (unused in the pin map); GP18, GP21
  and GP22 are taken on the board by the NeoPixel, the second button and
  the buzzer.
- Mounting between the two front sensors (30 mm apart over 18 mm tape),
  its threshold set on the module's trimmer; common ground.
- Software then: one GPIO read per S3 pass and a lost-line rule in
  `line_control.c`, with tests; about a day's work once the sensor exists.

| Alternative | Why not chosen |
|---|---|
| Time-out when neither sensor has seen black for a set distance | Correct following shows over 1.1 m of both-white on the start straight; a safe limit would let a lost car leave the mat |
| Use the barcode sensor as a third line sensor | It sits at the side for barcodes and never steers (pin mapping); moving it breaks barcode reading |
| Wiggle the car to look for the tape | Changes line following on every straight and adds no certainty on curves |

**Acceptance.** On the car: lift-and-place the car 50 mm beside the tape
while following; STOP with cause line within 200 ms (value to be agreed),
five times out of five, and no false line-loss stop on three SYS-01 laps.

## P5 — Hump calibration (F-04)

**Done (S4 decision 17).** Detection compares the uncalibrated height, so
the start is back at the intended 6 mm (the 0.851 scale had raised it to
7.05 mm), and runs only while following. The settings are in
`s4_config.h`. A calibration offset is in the code (0 for now), applied only
to the reading. `testing/subsystem4/replay_raw.c` replays logged samples
through the terrain logic, and `fit_hump_calibration.py` fits the gain and
offset; the procedure is in `app_program/subsystem4/README.md`.

**Left: the measurement.** Only one reference height (10 mm, six
crossings, 10.8 to 12.5 mm uncalibrated) has raw logs; the demo's 5 mm
hump has rounded readings only. One height cannot separate a gain from an
offset, so no offset is fitted. Cross reference humps of 5, 10 and 15 mm
at least five times each with `S4_RAW_LOG=1`, record a flat lap, run the
tool, review the constants, and set them.

**Acceptance.** SYS-04 within ±2 mm on all three heights, every crossing
detected, and no hump on the flat lap. Regression: S4-U41 to U44.

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

**Re-assessed 2026-11-26: blocked on car measurements.** A synthetic
sweep through the real `scan_profile.c` over the beam range seen on the car
shows no allowance meets ±20 mm for both box faces, and that the demo's
missing edges come from the box's half-angle plus the beam reaching the
servo's 45 degrees ([log](../testing/subsystem5/logs/2026-11-26_s5-width-feasibility.txt)).
Changing the allowance for the width alone would also move the bypass
offset (the same extents), so nothing changes without the measurement in
step (1). What the car test must record: stop distance, heading at the
stop, box orientation, every ray of five scans per distance.

**Acceptance.** SYS-06 with the box both ways round. Regression: S5-U59 to
U63, U67, U72; SYS-07 to show bypass planning is unchanged.

## P7 — Watchdog (F-08)

**Done (S2 decision 37).** The controller, S3 loop, scan and IMU tasks
check in each pass; Motion Control stops the motors and faults when one
stalls, in every mission image; the hardware watchdog, a make option
(`S2_WATCHDOG=1`, refused with `TEST=1`), is fed only while all are
healthy. Tests S2-U81, U82 and INT-15.

**Left.** Build `make NET=lwip MQTT=1 S2_WATCHDOG=1`, run SYS-01 (three
laps) with the console captured and confirm no `health:` line and no
reset; then force a stall of each task in a test build and confirm the
motors stop and the board resets. Only then make 1 the default.

## P9 — A full course in one run (SYS-09)

**Where it stands.** Every mission element has worked on the car on its
own (traceability section 2), but no run has finished the whole course.
On demo day attempt 3 decoded the barcode, bypassed the obstacle three
times and measured the hump, but did not turn at the junction (section 4
of the [demo report](DEMO_REPORT_2026-11-24.md)).

**Why it has not been met.** The run depends on every element working in
one go, so the blockers add up: the junction turn (P3: the time limit is fixed; one-sided junctions remain), barcode reads
below 4 of 5 (F-01 and F-18 are fixed, car test pending) and line loss going unnoticed (P4). There is no
separate root cause to fix.

**Proposed plan.** Car-test telemetry version 5, then the 10 ms sampling and F-18 fixes, then do the P3
bench test, one at a time, each with SYS-01 re-run to show line following
is unchanged. Then three full-course runs with the console captured and
the time recorded.

**Acceptance.** SYS-09 as written: the whole course after one START, no
touching, three runs.
