# Software quality and logic review

A review of the application code in `app_program/` (49 `.c` files and
their headers, about 25,000 lines including tests), carried out after the
demo on 2026-11-24 and 25. It covers correctness, safety, robustness and
maintainability for each subsystem and for the integrated system. Kernel,
SDK, lwIP and CYW43 sources are third-party and are reviewed only where the
application uses them.

The review was first written against the demonstrated firmware. Changes
made since are marked **Applied**. Until 2026-11-25 they touched only
tests, tools or the build, and the rebuilt mission image stayed
byte-identical to the demo image (`8cf2c419…`). From 2026-11-25 firmware
corrections are also applied on `main` (S2 decision 35); each says whether
it has been tested on the car. A finding still marked **Proposal** is not
in the firmware.

## 1. Method

| Step | Tool or method | Result |
|---|---|---|
| Strict compile of every application file with the target compiler and flags, plus 18 extra warnings (`-Wconversion`, `-Wsign-conversion`, `-Wshadow`, `-Wnull-dereference`, `-Wduplicated-cond`, …) | `testing/host/target_static_check.py` | **0 warnings** in application code; 117 in kernel headers (third-party) |
| GCC static analyser (`-fanalyzer`) on every application file | same script | 1 finding, a false positive (F-12) |
| Host unit tests | `make -C testing/host` | All pass: S1 4/4, S2 56/56, S3 787 checks (no known defect), S4 39/39 plus S4-U44 (replay of 6 car logs), S5 48/48, controller integration 54/54 (INT-08 to INT-27), dashboard 27/27 (2026-11-26, [log](../testing/host/logs/2026-11-26_verify-final-refinement.txt)) |
| Line and branch coverage | `testing/host/coverage.sh` | 84.5% of lines, 73.9% of branches over 33 host-built files (2026-11-26, [log](../testing/host/logs/2026-11-26_coverage.txt)). The 6 newly measured files include `task_controller.c` (61%) and `s2_bridge.c` (53%), now built by the integration suite. The same 27 files as on 2026-11-25: 90.7% of lines, 78.9% of branches (was 89.7% and 78.0%) |
| Manual review | Reading the code against the requirements, the decisions and the demo logs | Findings below |
| Evidence from the car | Demo-day dashboard logs, console logs | Used to confirm or rule out findings |

Raw output: [static analysis](../testing/host/logs/2026-11-25_static-analysis.txt),
[tests and coverage](../testing/host/logs/2026-11-25_host-tests-and-coverage.txt);
after the post-demo tests: [final verification](../testing/host/logs/2026-11-25_final-verification-2.txt)
(host tests, both clean builds, static check, mission checksum) and
[coverage](../testing/host/logs/2026-11-25_coverage-2.txt).

**Severity.** *High*: can stop a mission requirement being met.
*Medium*: weakens reliability, diagnosis or accuracy. *Low*:
maintainability or documentation.

## 2. Summary of findings

| ID | Subsystem | Finding | Severity | Status |
|---|---|---|---|---|
| F-01 | S3 | The barcode and line loop samples every **20 ms**, not 10 ms | High | **Applied** (S3 d10): 10 ms cyclic handler; car test pending |
| F-02 | S2 | A junction is recognised only when **both** front sensors are black, so a one-sided branch reads as a curve | High | Proposal (needs a car test) |
| F-03 | S2 | No turn started after RIGHT was decoded on demo day | High | One cause found and **applied** (S2 d36): the 60 s hold expired at the obstacle; car test pending |
| F-04 | S4 | No hump can read below 6 mm; small humps read high | Medium | **Partly applied** (S4 d17): detection threshold fixed; calibration needs the car |
| F-05 | S5 | Obstacle width is overstated by the ultrasonic beam (SYS-06) | Medium | Explained; proposal |
| F-06 | S1/S2 | Telemetry carries one fault class; the detail behind the four demo motion faults was lost | Medium | **Applied** (S1 d8): version 5 frame; car test pending |
| F-07 | Build | Subsystems 1 to 4 were built with no warning flags | Medium | **Applied** (S2 d34) |
| F-08 | S2 | Watchdog off in the mission image | Medium | **Partly applied** (S2 d37): liveness guard on; watchdog armed only with `S2_WATCHDOG=1` until a car run |
| F-09 | S3 | Barcode white limits measured at the old sensor height; baseline pinned at its upper limit on demo day | Low | Proposal |
| F-10 | S2 | A held barcode command keeps ageing during a scan and bypass | Low | **Applied** with F-03: no time limit (S2 d36) |
| F-11 | Tools | Dashboard received every heartbeat twice | Low | **Applied** (`47a7147`) |
| F-12 | S3 | `-fanalyzer` reports an out-of-bounds write in `s3_nav.c` | — | False positive |
| F-13 | S3 | The letter table can change while the S3 loop reads it | Low | Safe by design; documented |
| F-14 | Tests | Six tests ran but were not defined in any test log | Low | **Applied** (`ad2b48c`) |
| F-15 | S4 tests | Mission hump settings repeated in a test | Low | **Applied** (S4 d17): `s4_config.h` |
| F-16 | Process | Barcode letter table entered wrongly on demo day | Low | Proposal |
| F-17 | S2/S3 | A car that runs fully off the line reads as centred and is not stopped | High | Blocked: needs a centre sensor (P4, hardware approval); no safe software rule |
| F-18 | S3 | A 20 ms noise blip inside a barcode can give a wrong letter (25 of 234 cases) | High | **Applied** (S3 d9): blip repaired; car test pending |
| F-19 | Tests | S4 and S5 host recipes piped through `tee`, so a crashed test passed | Medium | **Applied** (`a618bef`) |
| F-20 | S3 | `s3_nav_stage()` and `s3_nav_take()` held a command a second time, but nothing in the mission called them | Low | **Applied**: removed (mission UF2 unchanged) |
| F-21 | S2 | `motion_init()` does not range-check the move and turn tolerances before they are cast to `int32_t` | Low | **Applied**: refused above the largest goal (S2-U84) |
| F-22 | S4 | The `S4_RAW_LOG` stall print passed a `UW` to `%lx` | Low | **Applied** (`268e044`; debug builds only) |
| F-23 | Tools | The BARR-C checker missed indented local declarations, so it under-counted short names (7.1e) | Low | **Applied**: fixed, and the names renamed (`268e044`) |
| F-24 | S2 | Static stack estimate: the controller task's margin is about 400 bytes once `tm_printf` is counted; no run-time high-water mark | Medium | **Applied** (instrument): painted stacks, high-water printed at run end; car run pending |
| F-25 | S3 | A capture missing runs at one end that also holds a noise blip can give a wrong letter (39 of 702 cases in S3-U04; 3,624 in a wider sweep) | Medium | **Applied** (S3 d11 for 27 and 29 runs, d12 for 25 runs): 0 wrong in S3-U04, U07 and U08; host-verified, car test pending |
| F-27 | S2 | The liveness guard can read a check-in made after its clock read (a higher-priority task preempting Motion Control) as 2^32 ms old and latch a false task fault | Medium | **Applied** (S2 d38): such an age counts as fresh; S2-U81 extended; host-verified |
| F-26 | S4 | The IMU task polls with `tk_dly_tsk(10)` (really 20 ms), so samples arrive 20 or 30 ms apart | Low | Kept: the averaging windows were tuned on it |

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
`telemetry_encoder.c` packs a fixed 32-byte binary frame (version 5 since
2026-11-26; version 4, 31 bytes, in the demo) with
validity flags, so the dashboard shows "—" rather than a stale or invalid
value. WiFi rejoins every 5 s and MQTT reconnects every 3 s; a reconnection
never restarts motion (S2-U52, S1-H03).

**Evidence.** On demo day the dashboard received 98.6% of the frames due
(1863 of 1890) and the car's own counters showed 23 refused sends (`err
-1`, lwIP buffer full). The loss is reported, not hidden.

**F-06 (Medium, applied) — one fault class only.** The version 4 frame
carried a 4-bit fault class (`S2_FAULT_MOTION`, `S2_FAULT_SCAN`, …); the
detail (stall, encoder, timeout) was printed on the console only, so the
cause of the four demo-day motion faults (13:18:12, 13:41:22, 13:59:26,
14:43:33) is unknown. *Applied* (S1 decision 8): version 5 (32 bytes)
appends one byte with the 3-bit motion fault detail, the command held for
the next junction and a 2-bit junction count, so a missed turn now shows
whether a command was held and whether a junction was confirmed (F-03).
The dashboard and decoder read versions 2 to 5. S1-U05 (golden frame),
INT-16 (from the real controller state) and the dashboard tests check
it; SYS-08 on the car is pending.

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
the line follower treats as a curve; no junction is confirmed. In the
host integration model (INT-11) the follower then steers into the branch
as on a curve. Since S2 decision 36 a held command is not dropped by time,
so a missed branch means the command is used at the next confirmed
junction instead. The course junction's shape was not recorded. *Proposal:* test the
course junction shape on the bench with the console captured; if it is a
one-sided branch, confirm a junction on "one sensor black for longer than
a curve entry while the other stays on the line edge", or use a third
sensor.

**F-03 (High, cause found and applied) — no turn on demo day.** In the
assessed session RIGHT was decoded five times but `NAVIGATE` never
appeared. Re-reading the telemetry after the demo (S2 decision 36): in both
full runs of attempt 3 the car reached the obstacle 58 s after the decode
(614 to 2,581 mm, then 592 to 2,589 mm), and the 60 s hold expired 2 s into
the scan. No junction after the obstacle could therefore be turned at.
*Applied:* the time limit is removed; a held command waits for the next
junction and is dropped only by a newer barcode or a STOP. INT-11 runs this
case through the real controller (and fails with the limit put back). The
junction's position relative to the obstacle was not recorded, so F-02 may
also have applied. Car test pending (SYS-03 with an obstacle before the
junction).

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
This is a limit of the sensor layout, not only of the code: a rule such
as "no sensor has seen black for X mm" would also fire on a long straight
where the car stays centred, which is the normal case. *Re-assessed
2026-11-26:* the dashboard logs show correct following with both sensors
white for up to about 1.1 m (767 stretches; [analysis](../testing/system/logs/2026-11-26_line-loss-analysis.txt)),
so a distance rule safe on straights would let a lost car leave the mat;
none is added. *Proposal (P4):* a third, centre sensor over the tape (a
digital module on GP6, checked against the LM393 and RP2040 datasheets),
so that all three white means lost. It needs hardware approval; until
then the limitation stays in the fault table.

**F-08 (Medium, partly applied) — watchdog off.** `S2_WATCHDOG` defaulted
to 0 and only Motion Control fed the watchdog, so a stalled controller (the
motors would hold its last speeds), S3 loop, scan or IMU task went
unnoticed (Tribunal 2 Entry 4). *Applied* (S2 decision 37): those four
tasks check in each pass; Motion Control, before writing the motors each
period, faults (`MOTION_FAULT_TASK`, duties 0) when one has not checked in
within its limit, in every mission image. The hardware watchdog, now a
make option, is fed only while all are healthy. INT-15 blocks the real
controller while driving: motors off at 500 to 510 ms. *Still pending:*
one car run with `S2_WATCHDOG=1` before it is armed by default, and the
limits are not measured on the car.

**F-21 (Low, applied) — tolerances not range-checked.** `control()` in
`motion_logic.c` casts the `uint32_t` tolerances `move_tol_mm` and
`turn_tol_mdeg` to `int32_t`, and `motion_init()` did not check them. It
now refuses a move tolerance above the largest distance a command carries
(32,767 mm) and a turn tolerance above `MOTION_TURN_MAX_MDEG`: anything
larger would end every move at once, and both limits are far below
`INT32_MAX`, so the casts are safe by construction (S2-U84).

**F-24 (Medium, applied as an instrument) — stack margin not measured.**
A static estimate ([log](../testing/host/logs/2026-11-26_stack-usage.txt),
now reproducible with `tools/stack_estimate.py`) puts the deepest
application chains at 1,928 bytes in the S3 loop, 1,000 in the controller,
800 in Motion Control, 632 in the IMU task and 392 in the scan task, with
the kernel, SDK and `tm_printf` frames not counted. The controller's
2,176-byte stack may have only a few hundred bytes spare once `tm_printf`
(about 1 KB) is added. *Applied:* `app_start_task()` now gives the four
(five with `S4_RAW_LOG`) application tasks stacks from a static pool of
the same size the kernel allocated, painted with a known word before they
start; `app_stack_report()` prints how much of each was never used, at
the end of every run (`run_summary()`, on a pick-up). The scan is host-
tested (S2-U83). *Pending:* a car run to read the marks; raise
`TASK_STACK_SIZE` if any margin is under 25%. The S3 loop runs in the
initial task, whose 4 KB stack the kernel sets up, so it is not in the
report; its static estimate is under half of it.

**F-10 (Low, applied) — command age during a bypass.** The 60 s hold
counted from the decode, so a scan and bypass used it up while the car
could not reach a junction. Removed with F-03 (S2 decision 36). Junctions
passed during BYPASS or REACQUIRE are still not used: line following
confirms junctions only while following.

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

**Host evidence (S3-U02, S3-U03, post-demo).** Over all 26 letters: one
misread element (a bar or space read as the wrong width) is always
corrected or rejected, never a wrong letter;
with the speed changing steadily across one code, every letter decodes
while the last run is 0.4 to 2.5 times the first (2.5:1 print), and no
wrong letter appears at 0.3 or 3.5 times. Speed variation of the size seen
on the car is therefore not, on its own, a reason for a missed read. A
letter in place of the start and stop characters is rejected, except P,
which is the mirror image of `*` (P-x-P read backwards is exactly a
reversed `*x*`, so no two-way decoder can tell them apart).

**F-18 (High, applied) — wrong letter from a noise blip.** S3-U03 put a
20 ms blip (one sample at the real 20 ms sampling) inside each run of each
letter. Inside the payload this made a 31-run capture, which the demo
decoder treated as a code with a stray mark beside it; one of the two
29-run windows it tried was then misaligned and, in 25 of 234 cases,
matched a different letter exactly. FR3 requires that this never happens,
though no wrong letter was seen on the car; 25 of 234 is a count of
synthetic cases, not a real-world error rate. The earlier prototype (accept
a window only when the run it leaves out is under half the narrowest run
kept) removed the wrong letters but rejected almost every blipped capture
(4,420 of 4,524 in the extended S3-U03), because it never repaired the
blip. *Applied* (S3 decision 9): the shortest run of a 31-run capture is
the extra mark; at either end it is a stray mark and is dropped, inside the
code it is a blip and the element it split is rebuilt. Over 4,524 cases
(10 and 20 ms blips at three places in every run) there is no wrong letter
and no rejection, and the 19 logged car captures replay unchanged
(S3-U05). Car test pending ([log](../testing/subsystem3/logs/2026-11-25_s3-f18-fix.txt)).

**F-25 (Medium, applied) — damaged end plus a blip.**
Found while fixing F-18. A capture that has lost runs at one end is read by
the 24 to 29-run fallback, which copies the intact sentinel over the
damaged end. A blip in the kept 20 runs shifts the payload by a bar and a
space, and every one of the 39 wrong letters in S3-U04 (of 702) came from
that shifted payload matching another letter exactly; a wider sweep gave
3,624. Repairing the blip by its length was tried first and rejected (63
of 468 blurred-element frames became wrong letters). *Applied* (S3 decision
11, 2026-11-26): the runs seen at the damaged end must agree with the
copied sentinel in all but one element, at the frame's boundary or at each
end's own. In a 27- or 29-run capture a shift always breaks that, while the
recorded damaged ends (a pause on the closing bar) pass. Wrong letters at
27 and 29 runs fell from 2,416 to 0 with no other fault class changed.
A 25-run capture shows only five runs of the damaged end, too few to tell a
shift from real damage (the recorded 25-run Z disagrees in two), so 1,208
synthetic wrong letters remained there. *Applied* (S3 decision 12): in a
24- to 26-run capture every kept run must last at least 50 ms; the blip
that shifted them is one of them, recorded noise lasted 19 to 40 ms and the
shortest recorded element 60 ms. The capture is rejected, not repaired.
Wrong letters at 25 runs fell from 1,208 to 0 (S3-U07) and S3-U08 finds
none in 86,112 cases with blips of 10 to 40 ms; the recorded Z decodes with
one sample of error on every run, which limits relative to the capture's
own narrow width did not achieve (1,360 and 2,385 of 5,000). Cost: 25-run
captures with an element under 50 ms are rejected; a noise pulse of 50 ms
or more is indistinguishable from an element. Host evidence only
([log](../testing/subsystem3/logs/2026-11-26_s3-f25-25run.txt)).

**F-01 (High, applied) — 20 ms sampling.** `s3_follow_run()` looped with
`tk_dly_tsk(10)`. µT-Kernel adds one tick to every delay, so on the 10 ms
tick the delay took two ticks. Every one of the 953 bar durations logged on
the car (629 on 2026-11-17, 324 on demo day) lies within 1 ms of a multiple
of 20 ms. Bar timings therefore had a resolution of 20 ms, 0.8 mm at
40 mm/s: a 9% to 17% error for the large print's narrow runs (120 to
220 ms), but for the 30 demo-day runs of one or two samples, one sample
more or less changed the run by 50% to 100%. FR2's "every 10 ms" was not
met. *Applied* (S3 decision 10): a 10 ms cyclic handler wakes the loop, as
in Motion Control, whose logs show that pattern holding 10 ms on this
board. Wake-ups queued during a slow pass are counted as overruns and
dropped. The steering filter (60 ms), the white-baseline step (160 ms) and
the `line_q` stall margin keep the values that ran on the car; S3-U06
checks the baseline step is time-based. Car test pending (SYS-01, SYS-02,
and the loop's own `s3 loop:` timing line).

**F-09 (Low) — barcode white limits.** `S3_BARCODE_MINIMUM_WHITE` 154 and
`S3_BARCODE_MAXIMUM_WHITE` 184 were measured at the sensor's old height.
After the sensor was lowered on demo day, 23 of the 45 rejected captures
logged a threshold of 584, so the baseline sat at its upper limit of 184.
The black level (about 2,560) is far above 584, so this did not cause
misreads, but the limits no longer describe the sensor. *Proposal:*
re-measure white at the new height (S3-H03-style stationary test).

**F-12 (false positive).** `-fanalyzer` reports a write past the end of
`g_s3_nav_map` at `s3_nav.c:49`. The index is `letter - 'A'` and the
function returns before the write unless `letter` is between `'A'` and
`'Z'`, so the index is 0 to 25. No change.

**F-13 (Low, safe) — letter table updates.** `s3_nav_set_letter()` runs in
the WiFi task; `s3_nav_command()` runs in the S3 loop, which has a higher
priority and can pre-empt it. Each entry is one `volatile` byte, so a read
is never torn. A read between "clear the old letter" and "set the new one"
can return NONE for one letter, which is dropped, not acted on wrongly.

**F-20 (Low, applied) — unused second command hold.** `s3_nav.c` had its
own "stage a command, hand it over at an authorised junction" logic from
before integration. The mission never called it: the Vehicle Controller
holds the command in `nav_control.c` (S2 decisions 26 and 27). Two copies
of the same rule (DRY) invite a fix to the wrong one, and the README
described the unused copy. Both functions and their type were removed;
`--gc-sections` had already left them out of the image, so the mission UF2
is byte-identical. Their 19 host checks went with them; the one property
not already covered by S2 (a STOP drops a command still waiting for a
junction) was added to S2-U77 and fails if `nav_control_abort()` stops
clearing the command
([log](../testing/host/logs/2026-11-25_s3-nav-cleanup-and-f18-recheck.txt)).

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

**F-04 (Medium, partly applied) — 6 mm floor and small humps.** A hump
starts when its height reaches `S4_RISE_UM` (6 mm, chosen in S4 decision 11
because floor bumps read up to about 4 mm). Decision 12 then scaled heights
by 0.851 *before* that comparison, so in the demo firmware a hump really
started at 7.05 mm of measured height, and the lowest possible reading was
6 mm (S4-U41). The demo hump read 6, 6 and 7 mm, so its measured peaks were
7.05 to 8.8 mm: two of three cleared the start by under 0.6 mm. *Applied*
(S4 decision 17), with evidence from replaying every logged IMU sample
through `terrain_logic.c` (`testing/subsystem4/replay_raw.c`, which
reproduces every hump the car logged): detection compares the uncalibrated
height, so the start is back at 6 mm, and runs only while following,
because the junction stop and spin read up to 6.6 and 4.7 mm against 4.0 mm
while following. The readings of the six logged drives are unchanged and
no new hump appears (S4-U44). *Not yet done:* small humps still read high.
The data fit a raw reading that is high by a few millimetres more than by a
fixed percentage, so an offset is needed; the mechanism
(`height_offset_um`, applied only to the reading) and a fitting tool
(`fit_hump_calibration.py`) are ready, but the offset stays 0 until
reference humps of several heights are crossed on the car.

**F-15 (Low, applied) — repeated constants.** The S4 settings were private
to `task_imu.c`, so S4-U41 repeated 6000 and 851. They now live in
`s4_config.h` (`s4_config_mission()`), used by the task, the tests and the
replay, and the tests derive their expectations from them.

**F-27 (Medium, applied) — false stall from a preempted guard.**
`s2_health_guard()` is given `now_ms()` and then reads the check-in
records. The S3 loop (priority 1) and the controller (14) run above Motion
Control (15), so either can preempt it between those reads and check in
with a later time; `now - last` then wraps to about 2^32 ms and the task
reads as stalled, latching `MOTION_FAULT_TASK` until RESET. The window is
short on the single-core mission image (a tick must land in it), wide on
a dual-core build. *Applied* (S2 decision 38): an age above 2^31 - 1 ms is
a check-in after `now_ms` and counts as fresh; S2-U81 covers it and fails
on the old evaluator. Found in this review; never seen on the car.

**F-26 (Low, kept) — IMU sample timing.** The IMU task polls with
`tk_dly_tsk(10)`, which takes 20 ms (as in F-01), against a 50 Hz sensor,
so samples arrive 20 or 30 ms apart (9,237 and 3,496 gaps in the S4-H08
gated log). Unlike the S3 loop, nothing is lost: each sample carries its
own time, and the 16-sample average, the 32-sample settle and the
3-sample confirmation were tuned on this timing; the replay evidence of
decision 17 assumes it too. A cyclic 10 ms poll would make the spacing
regular but would change those windows, so it is not applied without a
car run.

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
worked.

S5-U72 (post-demo) separates the causes with synthetic scans of a box face
240 mm away ([log](../testing/subsystem5/logs/2026-11-25_s5-width-geometry.txt)).
With ideal thin rays and no allowance the 150 mm box reads 128 mm: the
edge logic is sound, and the only error is the 5° ray spacing (about
23 mm per side at 240 mm, so even a perfect sensor misses the ±20 mm
criterion by a little). With a 21° beam the raw span is 282 mm, matching
the car's raw widths; the mission's 10° allowance gives 246 mm, and a 20°
allowance gives 168 mm (+18 mm). So the overstatement comes from the beam
and an allowance that does not match it, not from the geometry. A fixed
subtraction would be wrong, because the effective beam varied between
stops (S2 d18). See proposal P6 (section 10): measure the beam
on the car, set the allowance to the smallest measured half-beam, and
consider 2.5° fine steps near each edge.

*Re-assessed 2026-11-26* ([log](../testing/subsystem5/logs/2026-11-26_s5-width-feasibility.txt)).
Sweeping the effective half-beam over the range the car showed (the box
vanished between 20 and 35 degrees, S2 d18) for both faces of the
15 x 20 cm box at 200 to 300 mm: no single allowance keeps the width
within 20 mm; each is accurate only near one beam (allowance about
beam - 3 degrees), and one larger than the beam puts an edge inside the
box, which planning must never do (the same extents set the bypass
offset). The same sweep explains the demo's raw-only widths: when the
box's half-angle plus the beam nears the servo's 45 degrees, the scan
finds no miss on one side, so no shape (a 200 mm face at 250 mm, 21
degree beam, 5 degree heading error: raw 337 mm; demo raw 166 to
332 mm). **No firmware change is made**: a better width needs a beam
measured on the car (and, for confirmation, a stop further away or a
wider servo), each a physical trade-off to test first. Status: blocked
on car measurements.

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
PROVISIONAL in the code (for example the 12 s manoeuvre
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
| `a618bef` | Host runner keeps a test's own exit status (F-19) | None (test infrastructure) | Stand-in pass, fail and crash programs ([log](../testing/host/logs/2026-11-25_runner-status-check.txt)) |
| `b4871e3` | Tests S2-U80, S3-U03, S4-U42, S5-U72 | None (tests only) | Host suites pass; mission UF2 unchanged |
| `032437c` | Range comments for every narrowing cast (BARR-C 1.6a) | None (comments only) | Mission UF2 byte-identical |
| `cc72bd2` | Removed the unused S3 command staging (F-20); S2-U77 checks that STOP drops a held command; F-18 fix kept as a patch | None (unused code; `--gc-sections` had already dropped it) | Mission UF2 byte-identical; [log](../testing/host/logs/2026-11-25_s3-nav-cleanup-and-f18-recheck.txt) |
| `268e044` | A range comment on every cast; BARR-C naming rules 7.1e, 7.1k, 7.1m; 8.1 in test files; F-22, F-23 | None (comments and names) | Mission UF2 byte-identical; every build option 0 warnings; [log](../testing/host/logs/2026-11-25_barr-c-changes-verification.txt) |
| `2fddf92` | F-18: 31-run captures repaired, not searched (S3 d9); S3-U03 extended, S3-U04, S3-U05; `verify_all.sh` | **Yes**: which barcode captures decode. Car test pending (SYS-02) | S3 774 checks, 0 failures, 1 known defect (F-25); all suites, builds 0 warnings ([log](../testing/host/logs/2026-11-25_verify-f18.txt)) |
| `4bec493` | F-01: S3 loop woken by a 10 ms cyclic handler; overruns counted and dropped; filter, baseline step and `line_q` margin kept in time (S3 d10); S3-U06 | **Yes**: S3 samples every 10 ms instead of 20 ms. Car test pending (SYS-01, SYS-02) | S3 779 checks, 0 failures; all suites, builds 0 warnings ([log](../testing/host/logs/2026-11-25_verify-f01.txt)) |
| `f63a20a` | Host integration suite for the Vehicle Controller (INT-08 to INT-14): the real controller with S1, S3, S4 and S5 interfaces on a kernel model; `controller_cycle()` extracted | None (structure only; +216 bytes for the call) | 20 integration checks pass; all suites, builds 0 warnings ([log](../testing/host/logs/2026-11-25_verify-int-harness.txt)) |
| `b5d380e` | F-03, F-10: a held barcode command waits for the next junction with no time limit (S2 d36); INT-11 | **Yes**: a command is no longer dropped after 60 s. Car test pending (SYS-03) | S2 52/52; 23 integration checks; INT-11 fails with the limit put back ([mutations](../testing/host/logs/2026-11-25_int-mutations.txt), [log](../testing/host/logs/2026-11-25_verify-f03.txt)) |
| `26842cc` | F-04 (part), F-15: hump detection on the uncalibrated height, FOLLOW only; calibration offset and fitting tool; settings in `s4_config.h`; S4 replay tool (S4 d17) | **Yes**: detection starts at 6 mm of measured height (was 7.05 mm) and not during junction turns. Car test pending (calibration, SYS-04) | S4 38/38 and S4-U44 (six logged drives give the car's own readings, no new hump); all suites, builds 0 warnings ([log](../testing/host/logs/2026-11-25_verify-f04.txt), [analysis](../testing/subsystem4/logs/2026-11-25_s4-replay-analysis.txt)) |
| `7c040f9` | F-08 (part): task check-ins and a liveness guard on Motion Control; watchdog fed only while healthy; `S2_WATCHDOG` make option (S2 d37) | **Yes**: a task stalled past its limit stops the motors (Fault). Watchdog still off by default. Car test pending | S2 54/54, integration 27/27, two mutations caught; every build incl. `S2_WATCHDOG=1` 0 warnings ([log](../testing/host/logs/2026-11-26_verify-f08.txt)) |
| `24aaf7a` | F-05 re-assessed: width accuracy and confirmation blocked on car data (no firmware change) | None | Synthetic sweep ([log](../testing/subsystem5/logs/2026-11-26_s5-width-feasibility.txt)) |
| `df78bdc` | F-06: telemetry version 5 (fault detail, held command, junction count) end to end: firmware, decoder, dashboard (S1 d8) | **Yes**: 32-byte frame. Car test pending (SYS-08) | S1 4/4, dashboard 27/27, integration 30/30 incl. INT-16 ([log](../testing/host/logs/2026-11-26_verify-f06.txt)) |
| `42573f6` | F-17 re-assessed from the logs; centre-sensor wiring checked against the datasheets (no firmware change) | None | [analysis](../testing/system/logs/2026-11-26_line-loss-analysis.txt) |
| `134de47` | F-21: tolerance limits in `motion_init()`; F-24: painted task stacks and a high-water report at run end; `tools/stack_estimate.py` | **Yes**: task stacks come from a static pool (same size); a run summary also prints stack marks | S2 56/56 (S2-U83, U84); all suites and builds 0 warnings ([log](../testing/host/logs/2026-11-26_verify-f24.txt)). The strict compile found 2 warnings there, from F-08 and F-24: a `(UB *)` cast that discarded `const` on a `%s` argument. Both casts were removed in the final consolidation, which also made `verify_all.sh` fail on any strict warning in application code: 0 warnings, mission UF2 unchanged ([log](../testing/host/logs/2026-11-26_verify-final.txt)) |
| `09990f9` | F-25 at 25 runs: a kept run under 50 ms rejects a 24 to 26-run capture (S3 d12); S3-U07 now a check, S3-U08 | **Yes**: which damaged captures decode (rejected, not misread). Car test pending | S3 787 checks, 0 failures; +48 bytes; 4 mutations caught ([log](../testing/subsystem3/logs/2026-11-26_s3-f25-25run.txt)) |
| `9c5d7f5` | F-27: a check-in after the guard's clock read is fresh, not a stall (S2 d38); S2-U81 extended | **Yes**: no false task fault from a preempted guard | S2 56/56, integration unchanged; +4 bytes ([log](../testing/host/logs/2026-11-26_verify-f27.txt)) |
| `1d31f8c` | INT-25 to INT-27: STOP and a task fault during a junction turn; 60 s centred straight | None (tests only) | Integration 54/54; 3 mutations caught ([log](../testing/host/logs/2026-11-26_int-turn-stop-mutations.txt)) |

## 10. Proposed changes not applied

These changes are not in the firmware. The demonstrated image (`a41c19c`,
UF2 SHA-256 `8cf2c419…`) contains none of the post-demo changes either;
S2 decision 35 records that post-demo changes are applied on `main` one at
a time, and each one applied moves to section 9. Each proposal below needs
a measurement or a hardware change that software alone cannot supply.

Other open items: re-measure the barcode white limits at the current
sensor height (F-09); show the letter table on the dashboard before a run
(F-16); read the stack high-water marks (`stack ...` lines) after a car run
(F-24).

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

### P3 — Junction detection and the missing turn (F-02, F-03)

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

### P4 — Detecting a car that has left the line (F-17)

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

### P5 — Hump calibration (F-04)

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

### P6 — Obstacle width (F-05, SYS-06)

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

### P7 — Watchdog (F-08)

**Done (S2 decision 37).** The controller, S3 loop, scan and IMU tasks
check in each pass; Motion Control stops the motors and faults when one
stalls, in every mission image; the hardware watchdog, a make option
(`S2_WATCHDOG=1`, refused with `TEST=1`), is fed only while all are
healthy. Tests S2-U81, U82 and INT-15.

**Left.** Build `make NET=lwip MQTT=1 S2_WATCHDOG=1`, run SYS-01 (three
laps) with the console captured and confirm no `health:` line and no
reset; then force a stall of each task in a test build and confirm the
motors stop and the board resets. Only then make 1 the default.

### P9 — A full course in one run (SYS-09)

**Where it stands.** Every mission element has worked on the car on its
own (traceability section 2), but no run has finished the whole course.
On demo day attempt 3 decoded the barcode, bypassed the obstacle three
times and measured the hump, but did not turn at the junction (section 4
of the [demo report](DEMO_REPORT_2026-11-24.md)).

**Why it has not been met.** The run depends on every element working in
one go, so the blockers add up: the junction turn (P3: the time limit is fixed; one-sided junctions remain), barcode reads
below 4 of 5 (F-01 and F-18 are fixed, car test pending) and line loss going unnoticed (P4). There is no
separate root cause to fix.

**Proposed plan.** Car-test telemetry version 5, then the 10 ms sampling, F-18 and F-25 fixes, then do the P3
bench test, one at a time, each with SYS-01 re-run to show line following
is unchanged. Then three full-course runs with the console captured and
the time recorded.

**Acceptance.** SYS-09 as written: the whole course after one START, no
touching, three runs.
