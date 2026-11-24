# Software quality and logic review

A review of the application code in `app_program/` (49 `.c` files and
their headers, about 25,000 lines including tests), carried out after the
demo on 2026-11-24 and 25. It covers correctness, safety, robustness and
maintainability for each subsystem and for the integrated system. Kernel,
SDK, lwIP and CYW43 sources are third-party and are reviewed only where the
application uses them.

The review does not change the demonstrated firmware. Every finding that
would change behaviour is a **proposal**; post-demo changes that were
applied are marked **Applied** and change only tests, tools or the build,
which we checked by rebuilding the mission image and comparing its
checksum with the demo image (`8cf2c419…`).

## 1. Method

| Step | Tool or method | Result |
|---|---|---|
| Strict compile of every application file with the target compiler and flags, plus 18 extra warnings (`-Wconversion`, `-Wsign-conversion`, `-Wshadow`, `-Wnull-dereference`, `-Wduplicated-cond`, …) | `testing/host/target_static_check.py` | **0 warnings** in application code; 117 in kernel headers (third-party) |
| GCC static analyser (`-fanalyzer`) on every application file | same script | 1 finding, a false positive (F-12) |
| Host unit tests | `make -C testing/host` | All pass: S1 4/4, S2 51/51, S3 782/782 checks, S4 36/36, S5 47/47 |
| Line and branch coverage | `testing/host/coverage.sh` | 89.7% of lines, 78.0% of branches (27 host-built files) |
| Manual review | Reading the code against the requirements, the decisions and the demo logs | Findings below |
| Evidence from the car | Demo-day dashboard logs, console logs | Used to confirm or rule out findings |

Raw output: [static analysis](../testing/host/logs/2026-11-25_static-analysis.txt),
[tests and coverage](../testing/host/logs/2026-11-25_host-tests-and-coverage.txt).

**Severity.** *High*: can stop a mission requirement being met.
*Medium*: weakens reliability, diagnosis or accuracy. *Low*:
maintainability or documentation.

## 2. Summary of findings

| ID | Subsystem | Finding | Severity | Status |
|---|---|---|---|---|
| F-01 | S3 | The barcode and line loop samples every **20 ms**, not 10 ms | High | Proposal |
| F-02 | S2 | A junction is recognised only when **both** front sensors are black, so a one-sided branch reads as a curve | High | Proposal (needs a car test) |
| F-03 | S2 | No turn started after RIGHT was decoded on demo day; the cause cannot be shown from telemetry | High | Open |
| F-04 | S4 | No hump can read below 6 mm; small humps read high | Medium | Explained; proposal |
| F-05 | S5 | Obstacle width is overstated by the ultrasonic beam (SYS-06) | Medium | Explained; proposal |
| F-06 | S1/S2 | Telemetry carries one fault class; the detail behind the four demo motion faults was lost | Medium | Proposal |
| F-07 | Build | Subsystems 1 to 4 were built with no warning flags | Medium | **Applied** (S2 d34) |
| F-08 | S2 | Watchdog off in the mission image | Medium | Proposal (Tribunal 2 Entry 4) |
| F-09 | S3 | Barcode white limits measured at the old sensor height; baseline pinned at its upper limit on demo day | Low | Proposal |
| F-10 | S2 | A held barcode command keeps ageing during a scan and bypass | Low | Documented trade-off |
| F-11 | Tools | Dashboard received every heartbeat twice | Low | **Applied** (`47a7147`) |
| F-12 | S3 | `-fanalyzer` reports an out-of-bounds write in `s3_nav.c` | — | False positive |
| F-13 | S3 | The letter table can change while the S3 loop reads it | Low | Safe by design; documented |
| F-14 | Tests | Six tests ran but were not defined in any test log | Low | **Applied** (`ad2b48c`) |
| F-15 | S4 tests | Mission hump settings repeated in a test | Low | Proposal |
| F-16 | Process | Barcode letter table entered wrongly on demo day | Low | Proposal |
| F-17 | S2/S3 | A car that runs fully off the line reads as centred and is not stopped | High | Proposal (needs a track measurement) |

## 3. Subsystem 1 — WiFi, MQTT and telemetry

**Design.** All network calls (CYW43, lwIP, MQTT) are confined to the
kernel's WiFi service task and `subsystem1/`: a search of the other
subsystems finds none. `service_bridge.c` moves data between that task and
the application through narrow functions only: STOP is raised on the
controller's urgent event flag (`controller_raise_remote_stop()`), START and
RESET go through `remote_q`, and telemetry is read as a snapshot
(`robot_read_telemetry()`) under a mutex with `TMO_POL`, so a slow network
can never block a control task. `command_parser.c` parses commands with a
fixed buffer and rejects anything malformed (S1-U04, 100% line coverage).
`telemetry_encoder.c` packs a fixed 31-byte binary frame (version 4) with
validity flags, so the dashboard shows "—" rather than a stale or invalid
value. WiFi rejoins every 5 s and MQTT reconnects every 3 s; a reconnection
never restarts motion (S2-U52, S1-H03).

**Evidence.** On demo day the dashboard received 98.6% of the frames due
(1863 of 1890) and the car's own counters showed 23 refused sends (`err
-1`, lwIP buffer full). The loss is reported, not hidden.

**F-06 (Medium) — one fault class only.** The frame carries a 4-bit fault
class (`S2_FAULT_MOTION`, `S2_FAULT_SCAN`, …). The detail (stall, encoder,
timeout) is printed on the console only. On demo day four motion faults
(13:18:12, 13:41:22, 13:59:26, 14:43:33) latched STOP correctly, but with
no console captured their cause is unknown. *Proposal:* add the motion
fault detail (3 bits) or the independent fault bitmask from Tribunal 2
Entries 11 and 19 in a version 5 frame (one extra byte).

**F-11 (Low, applied) — duplicate heartbeats.** The dashboard subscribed
to `<prefix>/#` and to `<prefix>/heartbeat`, so the broker delivered each
heartbeat twice. Fixed in the dashboard (`subscriptions()`, 3 unit tests);
firmware unaffected.

## 4. Subsystem 2 — Vehicle Controller and Motion Control

**Design.** The controller statechart (`controller_logic.c`) is pure logic
with no RTOS calls, so it is tested on the host (S2-U41 to U52, 93% line
coverage). It boots in INITIALISING with STOP latched, accepts START only
in READY with Motion Control healthy and both wheels still, and handles
urgent event flags (remote STOP, button, hazard, pick-up, line stop)
before any queued command. Motion Control runs from a 10 ms cyclic handler
(exact period, unlike a sleep loop) and is the only code that writes motor
PWM in the mission image (`task_motion.c`, `drv_motor_set()`). The per-wheel
PID (`pid.c`) clamps its output and stops integrating while saturated
(anti-windup, S2-U06). Stall, encoder and timeout faults latch STOP and
need RESET with the wheels still (S2-U29 to U32).

The encoder interrupt (`drv_encoder.c`) updates its counters inside a
sequence-counter window with `dmb` barriers, so a task never reads a
half-updated count. All four shared mutexes use `TA_INHERIT`, so a
low-priority holder is raised and priority inversion is bounded.

**F-02 (High, proposal) — junction geometry.** `line_control.c` enters
JUNCTION_CONFIRM only on `S2_LINE_BOTH_BLACK` and confirms after 100 ms
(`CTRL_LINE_JUNCTION_MS`). The two front sensors are 30 mm apart on 18 mm
tape, so each sits about 6 mm outside the tape. At a cross or a T, the
crossing line covers both sensors. At a branch that leaves on one side
while the main line carries on, only that side's sensor goes black, which
the line follower treats as a curve; no junction is confirmed and a held
command is never acted on. This fits the demo-day log (F-03) but is not
proven: the course junction's shape was not recorded. *Proposal:* test the
course junction shape on the bench with the console captured; if it is a
one-sided branch, confirm a junction on "one sensor black for longer than
a curve entry while the other stays on the line edge", or use a third
sensor.

**F-03 (High, open) — no turn on demo day.** In the assessed session,
RIGHT was decoded five times but `NAVIGATE` never appeared. In the 1 Hz
frames after each decode, the line position never read BOTH, whereas BOTH
did appear at other times (9 frames in the practice log). The frames are
1 s apart, so a short junction crossing could fall between them; this
suggests, but does not prove, that no junction was confirmed while the
command was held. The junction and hold messages are console-only. In the
practice session, the same logic completed a U-turn (13:42:33 to 13:42:50).
*Proposal:* add a "junction confirmed" event to telemetry (one flag bit),
and repeat SYS-03 with the console captured.

**F-17 (High, proposal) — line loss not detected.** The two front sensors
straddle the 18 mm tape, so both white is the normal "centred" reading
(S2 d21 makes the same point for the bypass align). If the car leaves the
line completely, both sensors also read white, and `line_control.c` keeps
commanding straight. The checks that do stop the car cover a stale or
invalid report (500 ms), a crossing longer than 1.5 s and a junction exit
longer than 2.5 s, but not a car that has simply run off. FR11 asks for
persistent line loss to latch STOP, so this is a gap against our own
requirement. In practice curve holding (`CTRL_LINE_*_HOLD_MS`) keeps the
car turning after a sensor sees black, which is why SYS-01 passed.
*Proposal:* track the distance since either sensor last saw black and
latch STOP when it exceeds the longest straight stretch with no edge
contact (to be measured on the track).

**F-08 (Medium, proposal) — watchdog off.** `S2_WATCHDOG` defaults to 0, so
the hardware watchdog (`drv_watchdog.c`, tested in S2-H05) is not armed in
the mission image. A stalled Motion Control task would leave the last duty
on the motors until another check stops them. The dual-core stalls of
2026-11-23 (S2 d33) show tasks can stop. *Proposal:* arm it after one car
run with it enabled (Tribunal 2 Entry 4).

**F-10 (Low, trade-off) — command age during a bypass.**
`nav_control_step()` runs only while following, but the 60 s hold limit
counts from the decode. A long scan and bypass can use most of it.
Junctions passed during BYPASS or REACQUIRE are not used, which is correct:
the car is off the line. We keep this behaviour and record it here.

**Coverage gaps.** `pid.c`, `odometry.c` and `motion_api.c` have 52% to 53%
branch coverage on the host because their unit tests (S2-U03 to U14) run
only in the `TEST=1` image. `nav_control.c` rose to 95% of lines with
S2-U79.

## 5. Subsystem 3 — Line sensors and barcodes

**Design.** Line sensing (`s3_ir.c`) filters each front sensor over 3
samples and classifies black and white against an adaptive white baseline
with separate black and white margins (hysteresis), so a slow change in
lighting moves the baseline instead of the decision. The barcode sensor
(ADC2/GP28) never steers. Its decoder works on bar *durations*
(`s3_scan.c`): a capture starts at a white-to-black edge and ends after
700 ms of white. Narrow and wide are split at the midpoint between the
narrowest and widest run of that capture, so a constant speed change needs
no retuning. `s3_bar_decode()` checks both sentinels and both
inter-character gaps, accepts the payload only when one letter is closest
with at most one element wrong, and otherwise rejects. Reverse reads,
captures of 24 to 28 runs (one damaged sentinel rebuilt) and 31 runs (one
stray mark) are handled. A rejected capture never gives a letter, so a
failure shows as "no letter", which is the safe outcome.

**Host evidence (S3-U02, post-demo).** Over all 26 letters: one misread
element is always corrected or rejected, never a wrong letter; with the
speed changing steadily across one code, every letter decodes while the
last run is 0.4 to 2.5 times the first (2.5:1 print), and no wrong letter
appears at 0.3 or 3.5 times. Speed variation of the size seen on the car
is therefore not, on its own, a reason for a missed read.

**F-01 (High, proposal) — 20 ms sampling.** `s3_follow_run()` loops with
`tk_dly_tsk(10)`, but the kernel tick (`CNF_TIMER_PERIOD`) is 10 ms, so the
delay takes two ticks. Every one of the 953 bar durations logged on the car
(629 on 2026-11-17, 324 on demo day) lies within 1 ms of a multiple of
20 ms. Bar timings therefore have a resolution of 20 ms, which is 0.8 mm at
40 mm/s. For the large print (narrow runs 120 to 220 ms) that is a 9% to
17% error, which the decoder tolerates. For the small print, the team
measured narrow spaces of 20 to 80 ms, that is one to four samples, so one
sample more or less changes a narrow space by 25% to 100%. FR2's "every
10 ms" is also not met in practice. A line report still goes out as soon
as the position changes; only the repeat of an unchanged position comes
every 40 ms instead of 30 ms, well inside the 100 ms freshness and 500 ms
watchdog limits. *Proposal:* drive the loop from a cyclic handler (as
Motion Control does) or wake it on a 10 ms semaphore; then re-run SYS-02
with both prints. Not applied: it changes the timing of the demonstrated
line follower.

**F-09 (Low) — barcode white limits.** `S3_BARCODE_MINIMUM_WHITE` 154 and
`S3_BARCODE_MAXIMUM_WHITE` 184 were measured at the sensor's old height.
After the sensor was lowered on demo day, 23 of the 45 rejected captures
logged a threshold of 584, so the baseline sat at its upper limit of 184.
The black level (about 2,560) is far above 584, so this did not cause
misreads, but the limits no longer describe the sensor. *Proposal:*
re-measure white at the new height (S3-H03-style stationary test).

**F-12 (false positive).** `-fanalyzer` reports a write past the end of
`g_s3_nav_map` at `s3_nav.c:45`. The index is `letter - 'A'` and the
function returns before the write unless `letter` is between `'A'` and
`'Z'`, so the index is 0 to 25. No change.

**F-13 (Low, safe) — letter table updates.** `s3_nav_set_letter()` runs in
the WiFi task; `s3_nav_command()` runs in the S3 loop, which has a higher
priority and can pre-empt it. Each entry is one `volatile` byte, so a read
is never torn. A read between "clear the old letter" and "set the new one"
can return NONE for one letter, which is dropped, not acted on wrongly.

**F-16 (Low, process).** On demo day the table sent at 14:32:51 put O under
STRAIGHT; it was corrected at 14:48:34. The dashboard sends what is typed.
*Proposal:* show the car's current table (echoed in telemetry or a status
message) beside the input, so a wrong entry is visible before START.

## 6. Subsystem 4 — IMU and terrain

**Design.** `terrain_logic.c` is pure logic (98% line coverage). It
calibrates while still (spread must stay within `calibration_span_mg`),
faults on consecutive invalid samples (S4-U08) and on a gap longer than
`max_sample_gap_ms` (stale data, S4-U11), handles timer wrap (S4-U10), and
recovers locally while keeping the run's maximum (S4-U13). Height comes
from tilt and contact geometry (S4 d8): one wheel raised h tilts the car by
h / track in roll and h / (2 × wheelbase) in pitch, so the larger of the
per-contact estimates gives the height. Pitch is ignored while the wheel
speed changes (acceleration reads as pitch, S4-U27), and hump, impact and
lift detection run only while the car drives (S4 d12, d14, d16).

**F-04 (Medium, explained) — 6 mm floor.** A hump starts only when the
scaled height reaches `S4_RISE_UM` (6000 µm, chosen because floor bumps
read up to about 4 mm, S4 d11) and is rounded to whole millimetres, so the
lowest possible reading is 6 mm (S4-U41). An ideal 5 mm step scales to
about 4.3 mm and would not be a hump at all; the demo hump read 6, 6 and
7 mm, so the car's raw estimate over that hump was about 7 to 8 mm. The 10
mm hump read 11.75 mm raw on average (S4-H08), which set the 0.851 scale.
Both points fit a raw reading that is high by a few millimetres rather than
by a fixed percentage, so a single scale factor cannot correct small humps.
*Proposal:* two-point calibration (5 mm and 10 mm) giving a scale and an
offset, then lower the rise threshold only after a flat-lap test shows no
false humps.

**F-15 (Low) — repeated constants.** `S4_RISE_UM` and
`S4_HEIGHT_SCALE_PERMILLE` are private to `task_imu.c`, so S4-U41 repeats
6000 and 851 with a comment. *Proposal:* move them to `subsystem4.h` so the
test uses the same constants (DRY).

## 7. Subsystem 5 — Ultrasonic scan and bypass

**Design.** The echo interrupt (`drv_ultrasonic.c`) stores edge times in a
single-producer ring buffer with a `dmb` barrier and flags an overrun
rather than overwriting. `range_measurement_from_echo()` converts pulse
width to distance with integer arithmetic; the valid range is 117 µs to
23,530 µs (20 mm to 4 m, S5-U04, U05), and an echo still high after 30 ms
means nothing in range, not a clear reading of a near object (S5-U20,
S2 d10). The forward monitor treats
INVALID and STALE readings as unsafe, never as a clear path (FR6, S5-U25).
Triggers are at least 65 ms apart (S5-U19). The scan uses 15° coarse steps
to ±45°, then 5° readings round each edge; each angle is the median of
three (S5-U23). Planning prefers the left side and checks the swept body
of the car against the profile (S2 d16, d17).

**F-05 (Medium, explained) — width overstated (SYS-06).** A ray "hits"
anything inside its beam. S2 d18 measured an effective half-beam of about
21°, wider than the datasheet's 15° measuring angle. For the 150 mm box at
about 240 mm, the corners are at about 17°, but rays out to about
17° + 21° = 38° still return the box, so the raw width is about
2 × 265 mm × sin 38° ≈ 326 mm; the assessed session logged 166 to 332 mm.
The 10° beam allowance pulls each edge in, but the shape is valid only when
both edges pass the contiguity checks, which did not happen on 2026-11-20.
Planning uses the clearance profile, not the width, so bypasses still
worked. *Proposal:* subtract the measured half-beam per edge
(lateral = r · sin(θ_miss − 21°)), check against S5-U49 with 150 mm and
200 mm boxes, and only then report the width as valid.

## 8. Integration

| Area | Review result |
|---|---|
| Ownership | Only Motion Control writes PWM; only the Vehicle Controller authorises movement, scans and recovery; network calls stay in the WiFi task (AGENTS.md rules hold) |
| Inter-task messages | Message buffers for ordinary work (`remote_q`, `line_q`, `scan_q`, `nav_q`, `motion_q`); event flags for urgent STOP; every receive in a control loop uses `TMO_POL` or a bounded wait |
| Shared state | Snapshots under `TA_INHERIT` mutexes read with `TMO_POL`; a busy provider clears its validity flags for that frame instead of blocking (`robot_state.c`) |
| Interrupts | Encoder and echo ISRs do a few loads and stores; data goes to tasks through sequence counters or a ring buffer |
| Stop-before-scan | The scan is authorised only after a STOP confirmed by Motion Control (S5-U09 to U12) |
| Latched STOP | Every stop source latches STOP; only RESET then START clears it (S2-U45 to U52) |
| Task priorities | S3 loop in the initial task at priority 1, USB 3, WiFi 4, controller 14, motion 15, IMU 18, scan 20. The S3 loop's high priority keeps its sampling regular; its sample times come from the hardware timer, so console delays do not distort bar durations |
| Single core | Since S2 d33 the mission runs on one core; about 100 min of demo-day logs show no line-loop stall (system test log) |

**Hardcoded values.** We counted 33 constants marked UNMEASURED or
PROVISIONAL in the code (for example the 60 s hold, the 12 s manoeuvre
limit, the barcode white limits). Each says so where it is defined, and the
measured ones cite the test that set them (for example
`S4_HEIGHT_SCALE_PERMILLE` cites S4-H08). The broker address is compiled in
from `config/wifi_secrets.h`, so a new network needs a rebuild; this is
recorded in the system test log. The course is not hardcoded: letters can
be set over MQTT, and the scan and bypass work from measurements.

## 9. Applied post-demo changes

| Commit | Change | Behaviour change on the car | Verification |
|---|---|---|---|
| `70c74db` | Dashboard shows measured hump and width; fixed "expected" values removed | Display only | Dashboard tests |
| `ad2b48c` | Tests S2-U79, S3-U02, S4-U41; definitions for S2-U73 to U77, S4-U40; coverage and static-analysis scripts | None (tests only) | Host suites pass; mission UF2 unchanged |
| `47a7147` | Dashboard subscribes to each topic once | None (dashboard) | 23 dashboard tests pass |
| `53f7e02` | Strict warnings for all application files (S2 d34) | None | Clean builds, 0 warnings; mission UF2 unchanged |

## 10. Proposed changes not applied

These change the car's behaviour, so they need team approval and car tests
before use. They are listed in priority order.

1. F-01: 10 ms barcode and line sampling from a cyclic handler; re-run SYS-01 and SYS-02.
2. F-02 and F-03: junction event in telemetry; bench test of the course junction; then SYS-03.
3. F-06: fault detail or bitmask in telemetry (version 5 frame).
4. F-08: arm the watchdog after one car run.
5. F-04: two-point hump calibration; then SYS-04 with a flat lap.
6. F-05: beam-corrected width; then SYS-06.
7. F-09, F-15, F-16: re-measure barcode white; move S4 constants to a header; show the letter table on the dashboard.
