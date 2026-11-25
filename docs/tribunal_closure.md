# Tribunal closure and fault handling

This document closes the two tribunals against what we built and tested.
It quotes each tribunal requirement as given, then shows the original
position, the final implementation, the evidence and what is still open.
Section 3 traces every failure the car can meet from detection to a safe
response.

Sources: [AAI08_PostTri1.pdf](AAI08_PostTri1.pdf) (Tribunal 1, Week 8),
[AAI08_ImmediateTri2.pdf](AAI08_ImmediateTri2.pdf) (Tribunal 2 response),
[decisions.md](decisions.md), [data_widths.md](data_widths.md), the test
logs and the demo-day logs.

## 1. Tribunal 1 — architecture defence and the minimum-bit constraint

**Injected constraint (as given):** "The robot car firmware must minimise
the number of bits used to represent application data. Each data field
representing application information must use only the number of bits
required to represent its valid range, using C bit-fields or explicit masks
and shifts where appropriate."

### 1.1 The constraint

| Area | Tribunal 1 position | Final implementation | Evidence | Status |
|---|---|---|---|---|
| Telemetry (Buddy 1) | Binary summaries with agreed ranges, bit positions and byte order; raw data stays local | 32-byte little-endian frame (version 5), codes packed two per byte with masks and shifts, 5-bit letter + 3-bit command, validity flags (S1 d1, d5, d7) | [data_widths.md](data_widths.md) field table; S1-U05 golden frame; 98.6% of frames received on demo day | **Closed** |
| Internal records and shared state (Buddy 2) | Reduce fields while keeping range, STOP priority and safe concurrency | Kept whole-byte fields. Packing every record would save about 220 bytes (under 0.1% of SRAM) but add pack/unpack work in the 10 ms loops and lengthen mutex hold time | Calculated and measured sizes in [data_widths.md](data_widths.md) | **Closed as a justified deviation** |
| Barcode bar durations (Buddy 3) | 8-bit millisecond durations, if the car stays above about 50 mm/s | Rejected: the car crosses barcodes at 40 mm/s and runs reach 2.5 s, which needs 12 bits in ms. Kept as 32-bit µs; 16-bit ms is the measured-safe option | [data_widths.md](data_widths.md) | **Closed as a justified deviation** |
| Hump data (Buddy 4) | Height in mm with validity, range from tests | 8-bit mm, saturating at 255, with a validity flag; last hump and count added in v4 | S2-U71, S4-U40; demo readings | **Closed** |
| Obstacle data (Buddy 5) | Compact status and bypass results; scans stay local | Range, nearest, width (raw or shape), centre and result/side codes in 9 bytes of the frame; the 19-angle scan stays on the car | Frame table; SYS-05 | **Closed** |
| Shared fault bitmask (R6) | Individual fault bits with atomic or synchronised updates | Urgent events use separate kernel event-flag bits (`controller_flags`), set atomically by the kernel. Telemetry sends a single 4-bit fault class, not a bitmask | S2-U40, S2-U51 | **Partly closed**: the event bits are race-free; a published fault bitmask was not built (see Tribunal 2 Entries 11 and 19) |
| Measured savings | "Only claim improvements when the measurements support them" | Before/after sizes measured with the target compiler on 2026-11-20 and 2026-11-25 | [data_widths.md](data_widths.md) | **Closed** |

### 1.2 Risks R1–R6

| Risk | Tribunal 1 concern | Final position | Evidence | Status |
|---|---|---|---|---|
| R1 WiFi reconnection affects control timing | Reconnection might starve control | Motion Control runs from a 10 ms cyclic handler at priority 15. WiFi (priority 4) is above it in the kernel, but every application exchange with it is non-blocking (`TMO_POL`) | SYS-01: the car kept following with the link lost for over 4 min; S1-H03 | **Closed on observation**; control-loop jitter under reconnection was not measured |
| R2 Barcode timings wrong or encoder counts lost on a shared GPIO handler | Interrupt load; "fall back to 1 kHz timer polling if needed" | Encoders and ECHO use the shared dispatcher with short ISRs. The barcode is polled, not interrupt-driven. **The poll runs every 20 ms, not 10 ms** (all 953 logged bar durations are 20 ms multiples) | [software review](software_quality_review.md) F-01; S2-H03 (encoder counts) | **Open**: the 20 ms polling that Tribunal 1 warned about is what was built; small prints are affected, and one 20 ms blip can turn a code into a wrong letter (F-18). Proposals P1 and P2 |
| R3 Unstable power | Motors and servo might reset the board | Power-bank supply; HC-SR04 on 3V3 (S5 d13); a 0.5 s ultrasonic allowance after power-on (S5 d12) | SYS-11 (9 of 10 cold boots reached the broker; one unexplained); no reset seen in the demo-day logs apart from the operator's restarts | **Partly closed**; supply voltage under load was never measured |
| R4 Inconsistent data formats between subsystems | Different widths or units | One authoritative layout per interface in `TEAM.md` section 5 and the headers; record sizes checked by S2-U39 | INT-04 to INT-07 | **Closed** |
| R5 Inaccurate hump measurement from too few bits | Precision loss | 8 bits of whole mm is finer than the measurement error (±2 mm) | S4-H08, demo readings | **Closed** (the error comes from the method, not the bit width; [review](software_quality_review.md) F-04) |
| R6 Race on a shared fault bitmask | Lost updates | Kernel event flags (separate bits) | S2-U51 | **Closed** for the flags in use |

### 1.3 Week 6 assumptions

| Assumption | Final status | Evidence |
|---|---|---|
| A2 Telemetry and reconnection do not compromise control or STOP | Observed true | SYS-01 link loss; S2-H11 (STOP-latch-to-PWM-off worst 952 µs) |
| A4 Encoder turns are repeatable enough for navigation | Met on a limited sample | S2-H10 (S2 d32): 90° turns end just inside ±5° |
| A6 IR can read barcode bars at operating speed | **Partly**: large prints yes, small prints only with the sensor lowered; sampling is 20 ms | S3 drive logs; demo report D-01 |
| A8 Humps can be told from vibration | Yes, with a 6 mm rise and averaging (S4 d10, d11), at the cost of a 6 mm floor | S4-H07; S4-U41 |
| A11 Power stays stable with motors, servo and WiFi | Not measured | — |
| A12 Bypass clears the whole car and returns to the original line | Met for the supported box | SYS-07 3/3 |

## 2. Tribunal 2 — safe and predictable response to failure

**Injected requirement (as given):** "If any part of the system fails or
provides invalid data, the system must respond in a safe and predictable
way."

Tribunal 2 asked for analysis and decisions, not new code; the response
[AAI08_ImmediateTri2.pdf](AAI08_ImmediateTri2.pdf) records 22 entries. We
did not implement new behaviour for it. The table shows, for each entry,
whether the behaviour it relies on exists in the demonstrated firmware.

| Entry | Area | Decision | In the firmware? | Evidence | Remaining |
|---|---|---|---|---|---|
| 1 | Dependency-based fault policy | Accept | **Yes** as a design rule: movement data loss stops; telemetry loss does not | Section 3 below | Documented here |
| 2 | One authority; urgent STOP; single PWM writer | No change | Yes | S2-U45, U51; code review | — |
| 3 | Encoder validity, stall, timeouts | No change | Yes: fault after more than 3 invalid snapshots; stall at 500 ms | S2-U29 to U31 | Held-duty travel not measured on the car |
| 4 | Controller liveness; watchdog | Modify | **Yes, after the demo** (S2 d37): controller, S3 loop, scan and IMU tasks check in; a stall stops the motors (Motion Fault) and, with `S2_WATCHDOG=1`, stops feeding the watchdog | S2-U81, U82, INT-15; S2-H05 (watchdog on the bench) | Car run with `S2_WATCHDOG=1` pending; the watchdog stays off by default |
| 5 | Invalid or stale line data stops | No change | Yes: invalid report, no report in 200 ms after START, or none for 500 ms latch STOP | `task_controller.c` `line_step()`; S2-U57 | **Gap**: a car fully off the line reads as centred (F-17) |
| 6 | Barcode failure never turns the car | Modify | Partly: a failed decode gives no letter and no command | S3-U02, S2-U74 | The proposed stop before an unconfirmed route junction is not built |
| 7 | Forward invalid/stale counts as unsafe | No change | Yes | S5-U25, U55 | No car fault injection |
| 8 | Scan and bypass stop on failure | No change | Yes: FAULT, NO_ROUTE or a failed bypass latch STOP | Demo: `FAULT: scan` 14:49:54, `STOP latched: bypass` 14:35:35 | — |
| 9 | IMU health gates START; lift in bypass | Modify | **No**: START does not check the IMU; lift detection is off in BYPASS/REACQUIRE | S4 d16 | Proposal |
| 10 | Offline autonomy; STOP on its flag | No change | Yes | SYS-01, INT-04, S2-U52 | — |
| 11 | Independent concurrent fault bits in telemetry | Modify | **No**: one fault class | — | Version 5 frame proposal |
| 12 | Health checks before READY; clear stale work | Modify | Partly: START needs healthy motion and still wheels; queues are drained; IMU and range health not checked | S2-U41, U43 | Proposal |
| 13 | Fault-injection matrix | Accept | Partly: host fault injection exists (below); no car fault injection | Host tests | Car tests |
| 14 | Queue delivery failures | Modify | Partly: a failed line report latches STOP; STOP uses flags; scan sends are best effort | `s3_follow.c`, `task_controller.c` | — |
| 15 | No scan from a failed sensor | Modify | **No**: a hazard from invalid range still starts a scan; three invalid attempts then fault | S5-U24 | Proposal |
| 16 | IMU calibration before START | Modify | **No** | — | Proposal |
| 17 | Interrupts, scheduling, shared state | Modify | Partly: `TA_INHERIT` mutexes, `TMO_POL` reads, ISR sequence counters | Code review | Latency under combined load not measured |
| 18 | Boot and reset start safe | Modify | Yes for boot: INITIALISING with STOP latched; START required | SYS-11 | Brownout not tested |
| 19 | Fault bitmask size | Modify | **No** | — | As Entry 11 |
| 20 | Reject blanket STOP on any invalid value | Reject | Yes: WiFi or barcode loss does not stop the car | SYS-01 | — |
| 21 | Reject reuse of old line data | Reject | Yes: 100 ms freshness, 500 ms STOP | `S2_LINE_MAX_AGE_MS` | — |
| 22 | Reject automatic restart | Reject | Yes | S2-U52; demo: every fault cleared only by RESET | — |

Correction to the Tribunal 2 record: Entries 11 and 19 cite "the current
29-byte version 3 frame". Since 2026-11-24 the frame is 31 bytes
(version 4, S1 d7). The two added bytes carry hump data; they are **not**
the fault mask those entries proposed. Since 2026-11-26 version 5 (32
bytes, S1 d8) adds the motion fault detail (stall, encoder, timeout, task
stall...), the held junction command and a junction count; still one
fault at a time, not a bitmask.

## 3. Fault handling

Each row traces: failure → how it is detected → what the code does → state
→ safe response → how the car recovers. "Detectable" says whether the
current firmware can notice the failure at all.

| Failure | Detectable? | Detection | Handling (source) | State and response | Recovery | Evidence |
|---|---|---|---|---|---|---|
| WiFi or broker lost | Yes | Link status in the WiFi task | Rejoin every 5 s, MQTT every 3 s (kernel WiFi task) | Car keeps driving; telemetry `link up` flag clear | Automatic reconnection; never restarts motion | SYS-01 (link lost 14:58:30, car kept following); S1-H03; S2-U52 |
| Invalid or failed IR read (front) | Yes | ADC read error; malformed report | `s3_follow.c` requests `s2_line_stop()`; the controller rejects a malformed report (`line_step()`) | STOPPED, cause line | RESET then START | S2-U57 |
| No or stale line reports | Yes | No report within 200 ms of START, or none for 500 ms | `line_step()` | STOPPED, cause line | RESET then START | 2026-11-23 dual-core stall: every START ended "no report after START; STOP" |
| Car fully off the line | **No** | Both sensors white reads as centred | Drives straight; only a crossing over 1.5 s or exit over 2.5 s stops it | — | — | S2 d21 note; [review](software_quality_review.md) F-17 |
| Barcode decode fails | Yes | Capture rejected (bad sentinel, ambiguous letter, run too long, too many runs) | `s3_scan_poll()` returns ERROR; no letter is sent | Keeps following; no command | Next barcode | S3-U02; demo console log (45 rejections, no wrong letter) |
| Wrong or unmapped letter | Yes | Letter outside A..Z, command out of range, reserved bytes set | `nav_receive()` drops it; an unmapped letter is reported only | Keeps following | — | S2-U74 |
| IMU invalid or stale | Yes | Consecutive invalid samples; sample gap over the limit | `terrain_logic.c` faults; `task_imu.c` retries calibration in place, keeping the run maximum | Hump and lift detection unavailable; **the controller is not told**; the car keeps driving | Automatic local recovery | S4-U08, U11, U13 |
| Car picked up | Yes (FOLLOW, NAVIGATE) | Upward acceleration or held tilt over thresholds | `CTRL_FLG_PICKUP` | STOPPED, cause pick-up; motors off | RESET then START | SYS-10 5/5; 15 pick-up stops on demo day |
| Ultrasonic echo missing or late | Yes | No echo, stuck high, or reading older than 150 ms | Forward monitor gives INVALID or STALE → HAZARD; three invalid attempts → FAULT | Stops and scans; then FAULT if still invalid | RESET then START | S5-U24, U25, U55 |
| Encoder stops counting | Yes | More than 3 invalid snapshots, or decode-error burst | `motion_logic.c` → Fault(ENCODER) | FAULT, STOP latched | RESET with wheels still | S2-U31 |
| Motor stall | Yes | Full duty and no counts for 500 ms | Fault(STALL) | FAULT, STOP latched | RESET with wheels still | S2-U29; four motion faults on demo day (detail not logged) |
| Manoeuvre too slow | Yes | MOVE progress or stop confirmation past its limit | Fault(TIMEOUT); nav phase limit 12 s | FAULT or STOP | RESET | S2-U30, S2-U79 |
| Message buffer full | Yes for line and motion; best effort for scan | Send with `TMO_POL` fails | Line report or line-following command fails → line STOP; a controller motion command fails → FAULT, delivery (`ctrl_motion_sent()`); STOP itself uses event flags, never a queue | STOPPED or FAULT where movement depends on it | RESET then START | S2-U51 (STOP with `remote_q` full) |
| Stale data generally | Yes | Age checks: line 100 ms (fresh) / 500 ms (stop); range 150 ms; IMU gap; remote commands 2 s | Each owner rejects or stops | As rows above | — | S2-U50; S5-U25; S4-U11 |
| Conflicting requests | Yes | All movement goes through the Vehicle Controller | Urgent flags handled before queues; a scan abandons a junction manoeuvre; only Motion Control writes PWM | Highest-priority request wins: STOP over all | — | S2-U45, U46 |
| Controller, S3, scan or IMU task stops (blocks) | **Yes, after the demo** | No check-in for 500 ms (S3 loop 1000, IMU 1500) | Motion Control → Fault(TASK), duties 0 that period | FAULT, STOP latched | RESET with wheels still | S2-U81, U82, INT-15 (car test pending) |
| A task never yields, or Motion Control stops | Only with `S2_WATCHDOG=1` | Watchdog not fed for 200 ms | Chip reset; PWM back in reset | Boots with STOP latched | START after boot | S2-H05 (bench); car run pending |

### Host fault injection

| Fault injected | Test | Result |
|---|---|---|
| Invalid and stale IMU samples | S4-U08, U09, U11 | Pass |
| Bad echo: fall without rise, unsolicited edges, stuck high, oversize pulse | S5-U08, U21, U24 | Pass |
| Encoder decode errors, stalled wheel, slow move | S2-U29, U30, U31 | Pass |
| Remote STOP with `remote_q` full | S2-U51 | Pass |
| Expired and repeated remote commands | S2-U50 | Pass |
| Malformed line reports | S2-U57 | Pass |
| Corrupted barcode element, speed change within a code | S3-U02 | Pass |
| Noise blip or stray mark in a barcode (post-demo, F-18 fix) | S3-U03, U04, U05, U07 | Pass; one double fault remains for 25-run captures (F-25) |
| Junction manoeuvre with no response or no stop | S2-U77, U79 | Pass |

These prove the logic responds correctly to the injected input. They do
not prove the time from a real sensor fault to motors off on the car; only
the latch-to-PWM-off part is measured (S2-H11, worst 952 µs).

## 4. Undetectable failures and corrections

| Failure | Why it is not detected | Correction and status |
|---|---|---|
| Car fully off the line | Two sensors straddle the tape, so "both white" also means centred | A centre sensor (P4); a distance rule alone would also fire on a long centred straight |
| A noise blip turns a barcode into a different letter (F-18) | In the demo firmware the corrupted capture decoded cleanly | **Fixed after the demo** (S3 decision 9): in a 31-run capture the shortest run decides between a stray mark and a blip, and a blip is repaired (S3-U03). A capture with a damaged end *and* a blip could still give a wrong letter (F-25); since 2026-11-26 the damaged end is checked (S3 decision 11), which leaves this only for 25-run captures (S3-U07 known defect). Car test not possible: the robot has been returned |
| Controller, S3 loop or IMU task stalls | Before the demo: watchdog off, and only Motion Control fed it | **Applied after the demo** (S2 d37): check-ins and a liveness guard on Motion Control; the watchdog is fed only while all are healthy. Arming the watchdog by default awaits one car run |
| Stack overflow in a task | No high-water check; the controller's static margin is about 400 bytes once `tm_printf` is counted | Measure the high-water marks on the car (review F-24) |
| IMU fault while driving | `task_imu.c` recovers locally and does not raise a flag | Raise an IMU-fault flag; gate START on IMU ready (Entries 9, 16) |
| Simultaneous faults | One fault class in telemetry | **Fault detail applied after the demo** (version 5, S1 d8: motion fault detail, held command, junction count); one fault at a time, a full bitmask only if needed (Entries 11, 19) |
