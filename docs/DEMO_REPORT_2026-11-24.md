# Assessed demonstration — 24 November 2026

## 1. Purpose and evidence

This report records what happened during our assessed demonstration,
explains the software behaviour behind each result and links the evidence.
Every result below comes from the dashboard logs saved on the day, unless it
is marked as a team observation. We include the runs that failed.

| Evidence | What it is | Where |
|---|---|---|
| Practice session log | Dashboard log, 13:14–14:06, version 3 frames | [`2026-11-24_demo-dashboard-1.jsonl`](../testing/system/logs/2026-11-24_demo-dashboard-1.jsonl), SHA-256 `2d8456242367b7f011a81f1df6d5eb6ce1ab495fd6c094fef64c803c0bf2cf5c` |
| Assessed session log | Dashboard log, 14:17–15:03, version 4 frames; all three official attempts | [`2026-11-24_demo-dashboard-2.jsonl`](../testing/system/logs/2026-11-24_demo-dashboard-2.jsonl), SHA-256 `4e45008633ee0749180ffca8f17bb0391597f1152dd417d43d95cb0a3f5b3441` |
| Barcode console log | USB console of the side barcode sensor before the assessment | [`2026-11-24_s3-small-barcode.txt`](../testing/subsystem3/logs/2026-11-24_s3-small-barcode.txt) |
| Video | Team recording of the working system a few hours before the demo | <https://www.youtube.com/watch?v=B1mWqMT2HIk> (section 7) |
| Host tests | All five suites on the demonstrated source | [host review](../testing/host/HOST_REVIEW_2026-11-24.md) |

The dashboard ([`server.py`](../testing/subsystem1/dashboard/server.py),
`Store.record()`) appends one JSON line per message with the laptop's time,
so the times below are laptop times. The logs were copied into the
repository unchanged on the evening of 24 November; the SHA-256 values above
were taken then, not during the demo. "Line" means the line number in the
log file.

**Evidence integrity (checked 2026-11-25).** A SHA-256 digest changes
completely if even one byte of a file changes. So anyone can check that a
log is still the one we recorded: run `shasum -a 256 <file>` (or
`sha256sum`) and compare the result with the value in the table above. On
2026-11-25 we checked that:

- both logs in the repository still give the digests recorded on 24
  November;
- the assessed session log is byte-for-byte identical (`cmp`) to the
  original file the dashboard wrote on the laptop
  (`dashboard-20261124-141757.jsonl`) and to the copy the team shared on
  the day.

A hash does not stop a file being changed, and it cannot prove what
happened before the digest was taken. It shows that the file has not
changed since the digest was recorded, and git keeps the commit history of
both the logs and the digests.

**Firmware.** The assessed runs used the telemetry version 4 firmware (S1
decision 7), built at 14:17 with `make NET=lwip MQTT=1 S3_NAV_F=0 -j8`
(single core, S2 decision 33). Its UF2 SHA-256 is
`8cf2c419ae181224b0c3c7ec78f7bf651e6c17350860a7ce3198b198f416bf3f`. Its
source is commit `a41c19c`, tag `demo-2026-11-24-flashed`. A clean rebuild of
that source (`make -B`, 2026-11-25) produces the same checksum. The tag
`demo-2026-11-24` (`4a53c23`) marks the image planned the day before; the
practice session's version 3 frames are consistent with a rebuild of it for
the hotspot's broker address. Between the two images, S1 decision 7 added
two telemetry bytes: the subsystem 4 code now also counts humps and keeps
the last one, and subsystem 5 now fills in a raw width when an edge is
missing, for display only. No line-following, barcode, navigation, hump
detection or obstacle-planning decision changed
([`git diff 4a53c23 a41c19c`](https://github.com/timothyckl/INF2004-P2-08-RobotCar/compare/4a53c23...a41c19c)).

## 2. Timeline of the assessed attempts

Barcode letter table on the car: LEFT M, RIGHT N, STRAIGHT O, U-TURN P from
14:32:51 (line 1026); RIGHT O, STRAIGHT N from 14:48:34 (line 2487). The
course barcode was the letter O.

| Attempt | Run (log lines) | Barcode | Obstacle | Hump |
|---|---|---|---|---|
| 1 | START 14:34:01 (1130) to pick-up 14:37:08 (1405) | Not decoded | Scan, left bypass, then `STOP latched: bypass` during REACQUIRE at 14:35:35 (1279) | **6 mm** at 14:34:40 (1191, frame 1193) |
| 2 | START 14:48:36 (2492) to pick-up 14:50:40 | Not decoded | `FAULT: scan` at 14:49:54 (2615) | **6 mm** at 14:49:16 (2554, frame 2556) |
| 3 | Running 14:51:32 (2776) to remote STOP 14:56:30 (3251); lifted at 14:54:17 (3037) and restarted at 14:54:43 (3077) | **O decoded as RIGHT** at 14:51:46 (2800) and again at 14:54:57 (3100) | Three left bypasses, each back to FOLLOW: 14:53:08 (2926), 14:54:03 (3010), 14:56:25 (3240); no contact (team) | **7 mm** at 14:55:21 (3138, frame 3140) |
| Repeat scan (obstacle removed) | Runs from 14:56:50, 14:57:24 and 14:58:31 | O decoded as RIGHT at 14:56:59 (3303), 14:57:39 (3390), 14:58:46 (3538) | — | — |

No `NAVIGATE` mode appears anywhere in the assessed session log, so no
barcode turn was started during the assessment.

## 3. D-01 — Barcode detection

**Requirement.** MR2; SYS-02 needs at least 4 correct reads in 5 passes per
barcode and never a wrong letter.

**What happened.** The barcode was not decoded in attempts 1 and 2. In
attempt 3 it was decoded correctly as O, mapped to RIGHT, twice. On the
repeat scan requested by the assessor, the team remembers the scan as
unsuccessful; the log shows O decoded as RIGHT on three short runs after
attempt 3, but none of them went on to a turn (section 4). We cannot tell
from the log which of these runs the assessor watched. No wrong letter was
decoded in the assessed session. Host tests found after the demo that the
same firmware can give a wrong letter when a single 20 ms noise blip lands
inside the code (F-18, section 6 of the review); this was not seen on the
car.

**Practice before the assessment.** The practice log records correct
decodes of A (LEFT), Q, Z and F, including F at 13:42:33 (line 2512)
followed by a completed U-turn: `NAVIGATE (UTURN)` at 13:42:42 (2526) and
back to `FOLLOW` at 13:42:50 (2540). The barcode console log records 45
rejected captures and 6 decodes (Q, A, A, A, Q, F) while the sensor was
being set up. The team observed that the small A print only decoded after
the side sensor was lowered closer to the floor.

**One finding from the log.** Between attempts, O was decoded twice as
STRAIGHT (14:39:19, line 1601; 14:47:39, line 2390), because the letter
table sent at 14:32:51 put O under STRAIGHT. The table was corrected to
RIGHT O at 14:48:34, before attempt 2. This did not affect an official
attempt, because O was not decoded in attempt 1.

**How the decoder works.** The side sensor on ADC2/GP28 is separate from the
two front line sensors and never steers.
[`s3_barcode.c`](../app_program/subsystem3/s3_barcode.c) classifies black
and white with a margin above an adaptive white baseline;
[`s3_scan.c`](../app_program/subsystem3/s3_scan.c) captures bar timings,
[`s3_bar.c`](../app_program/subsystem3/s3_bar.c) splits them into narrow and
wide, and [`s3_code39.c`](../app_program/subsystem3/s3_code39.c) decodes
Code 39 with start/stop checks. A capture that does not decode is rejected
rather than guessed, which is why the failures on the day showed as no
letter, not a wrong letter. Because the decoder works on bar *timing*, a wrinkled or
lifted print, a change in sensor height, or the car slowing or turning over
the code changes the measured widths. These are plausible causes of the
missed reads; the demo logs do not show which one applied.

Since the demo we found two code-level factors (see the
[software review](software_quality_review.md), F-01 and F-09). The barcode
is sampled every 20 ms, not the intended 10 ms: every one of the 324 bar
durations in the demo-day console log is a multiple of 20 ms. That is
fine for the large print but coarse for the small print: 99 of the 324
logged runs were under 100 ms, and 30 lasted only one or two samples. Also, the barcode white limits were measured
before the sensor was lowered. Host tests (S3-U02) show the decoder itself
copes with a steady speed change of 0.4 to 2.5 times across one code and
never turns one misread element into a wrong letter. S3-U03 found one input
it does not handle: a single 20 ms blip inside the code gave a wrong letter
in 25 of 234 synthetic cases (F-18). That is a count of test cases, not a
real-world error rate. It was fixed on `main` after the demo (S3 decision
9); the fix is not in the demonstrated firmware.

**Earlier evidence.**

- 17 Nov, `8232e75`: [4/4 F reads while driving](../testing/subsystem3/logs/2026-11-17_s3-bar-drive-5.txt).
- 17 Nov, `e8cc529`: a loose barcode sensor cable was refitted, then [2/2
  F reads](../testing/subsystem3/logs/2026-11-17_s3-bar-drive-6.txt). This
  was an earlier fault; we do not claim it happened in the demo.
- 17 Nov, `3e9fafc`: [stationary readings](../testing/subsystem3/logs/2026-11-17_s3-bar-stationary.txt)
  of 156–204 on white and 2559–2602 on black. Good static contrast did not
  guarantee reliable moving reads on that build.

**Status.** Decoding works on the car but is not reliable enough to pass
SYS-02. Next test: tape both prints flat, record the sensor height, and run
five passes per print, keeping the console log.

## 4. D-02 — Obstacle bypass, line recovery and the right turn

**Requirement.** MR6–MR9 (SYS-05, SYS-07) and MR3 (SYS-03).

**What happened.** In attempt 3 the car stopped before the obstacle,
scanned, bypassed it on the left without contact and returned to `FOLLOW`
three times (table above). The team found returning to the original line
difficult with this obstacle position and track layout; the car was lifted
once (14:54:17) and restarted, and it then recovered the line. We received
the obstacle avoidance marks. Attempt 1 ended with `STOP latched: bypass`
during reacquisition, and attempt 2 with a scan fault, so neither completed
the bypass.

The right turn was not demonstrated. RIGHT was decoded and held, but the car
never entered `NAVIGATE` in the assessed session. The turn starts only when
the controller confirms a junction (both front sensors on black) after the
command is held ([`task_controller.c`](../app_program/subsystem2/task_controller.c),
[`nav_control.c`](../app_program/subsystem2/nav_control.c)). The junction
and hold messages go only to the USB console, which was not captured, so we
cannot say why no turn started. One clue is in the telemetry: in the 1 Hz frames
after each RIGHT decode the line position never read BOTH (both sensors on
black), while BOTH frames appear elsewhere in the logs. Frames are 1 s
apart, so this suggests but does not prove that no junction was confirmed.
Junction confirmation needs both front sensors on black for 100 ms; a
branch that leaves on one side only would read as a curve (review F-02).
The shape of the demo junction was not recorded. The two short repeat runs
ended with a pick-up stop 3 to 5 s after the decode, so they were too short
to show a turn either way.

**Earlier evidence.**

- 17 Nov, `85bdf28`: [right turn](../testing/subsystem3/logs/2026-11-17_s3-nav-right.txt):
  F decoded, `RIGHT held for the next junction` (lines 150–151), then the
  junction, spin and following on the new line.
- 20 Nov, `29bceff`: SYS-07 passed 3 of 3 bypasses and reacquisitions
  ([system test log](../testing/system/TEST_LOG.md)).
- 24 Nov practice (above): barcode F followed by a complete U-turn at a
  junction.

**Width (SYS-06).** Still not met; the width accuracy is unverified.

- Confirmed widths (both edges seen), practice, version 3 firmware: 315,
  327, 219 and 220 mm. The demo obstacle was 15 × 20 cm and its
  orientation for each scan was not recorded. 315 and 327 mm are too wide
  for either face. 219 and 220 mm are within ±20 mm only if the 200 mm
  face was towards the car, which we cannot confirm.
- Raw widths (one edge missing), assessed session, version 4 firmware: 166
  to 332 mm. These are for display only and are not width measurements,
  so they cannot show accuracy either way.

Bypass planning does not use the width, so this did not affect avoidance.

## 5. D-03 — Hump measurement and marks

**Requirement.** MR4–MR5; SYS-04 requires the reported highest hump to be
within 2 mm of the tallest hump crossed (our internal engineering
criterion, agreed on 20 November before the demo, not the assessor's
marking scheme), plus detection of every hump and no hump on a flat lap.
The course hump was 5 mm (0.5 cm), measured by the team.

| Attempt | Logged reading | Laptop time | Event line | Telemetry frame line | Error vs 5 mm | Within our ±2 mm criterion | Mark (team record) |
|---|---:|---|---:|---:|---:|---|---|
| 1 | 6 mm | 14:34:40.957 | 1191 | 1193 | +1 mm | Yes | 0.5 / 1 |
| 2 | 6 mm | 14:49:16.062 | 2554 | 2556 | +1 mm | Yes | 0.5 / 1 |
| 3 | 7 mm | 14:55:21.821 | 3138 | 3140 | +2 mm | Yes (at the limit) | 0.5 not awarded, see below |

The event line is the dashboard's `hump 1 measured` message; the frame
line is the first version 4 telemetry frame carrying the same value
(`hump_last_mm` and `hump_max_mm`), which then repeats in every frame until
the run ends.

A hump was detected and reported in each attempt, and all three readings
are within our pre-existing criterion. This is not a full SYS-04 pass,
because no flat lap was run and the number of physical crossings was not
independently checked against the logged hump count.

**Why the car reads 6 mm or more for a 5 mm hump.** In
[`terrain_logic.c`](../app_program/subsystem4/terrain_logic.c), the height
estimate is scaled by 0.851 (`S4_HEIGHT_SCALE_PERMILLE`, calibrated on a
10 mm hump, S4 decision 12) and a hump is only recorded once that estimate
reaches 6 mm (`S4_RISE_UM` in
[`task_imu.c`](../app_program/subsystem4/task_imu.c)). The threshold was
chosen because floor undulations read up to about 4 mm while driving (S4
decision 11). `s4_hump_complete()` then rounds the peak to whole millimetres
(line 555 in the demo source `a41c19c`; line 566 after later comment-only
changes); `s2_telemetry_add_hump()` in
[`robot_state.c`](../app_program/subsystem2/robot_state.c) copies it into
an 8-bit field, and the dashboard shows it as `N mm`
([`app.js`](../testing/subsystem1/dashboard/static/app.js), lines 56–57).
So the smallest height the car can report is 6 mm, and a 5 mm hump can
never read as 5. The 6 mm readings are the closest value the design can
give for this hump, and the 7 mm reading is one step above it. Measuring
humps below 6 mm accurately would need a lower threshold, which we would
have to re-test against false humps from the floor.

**Assessor's feedback (team recollection).** During the demonstration, the
assessor explained that functions not successfully demonstrated could
potentially still receive marks after review of the source code,
documentation and supporting evidence. We understood this as an
opportunity to submit technical evidence and ask for reconsideration where
appropriate, not as a promise of extra marks.

**Request.** Attempt 3 read 7 mm, within the same ±2 mm engineering
criterion that attempts 1 and 2 met, and it is recorded in the log above.
When the attempt 3 value was reported to the assessor, our groupmate
accidentally said 0.7 mm instead of 0.7 cm (7 mm), according to our team's
recollection. When we tried to correct this verbal mistake, we were told it
was already too late, and that attempt was not given the 0.5 mark awarded
for attempts 1 and 2. The value said aloud and the value the car recorded
should therefore be kept apart: the car detected the hump and recorded
7 mm in both `hump_max_mm` and `hump_last_mm` (line 3140). We respectfully
ask that attempt 3 be assessed from this logged reading on the same basis
as attempts 1 and 2. We note that its error (2 mm) is larger than theirs
(1 mm), and we do not know the exact partial-mark criterion.

We were also told that another group reported 0.8 cm (8 mm) and received
the full mark. We have not seen their hump height, conditions or reading, so
we cannot compare the results directly. If the criterion depends on the
reference height, we would appreciate clarification of how it was applied,
so that all results, including ours, are assessed the same way.

If the same reference height and assessment criteria were applied, we
would respectfully appreciate reconsideration of our hump measurement
marks in the assessed demonstration grade: whether our recorded
measurements of 6, 6 and 7 mm qualify for the full 1 mark, or at least
the 0.5 mark for attempt 3.

Our ±2 mm figure is the team's own engineering criterion for SYS-04, set
on 20 November. It is not the assessor's marking scheme, which we have not
seen; the request above asks only that the same scheme be applied to every
group.

**Earlier accuracy evidence.** On 13 November (`03237ef`) four crossings of
a 10 mm hump read 11, 10, 10 and 9 mm
([S4-H08](../testing/subsystem4/TEST_LOG.md),
[log](../testing/subsystem4/logs/2026-11-13_s4-h08-gated.txt)). In practice
on the day, the car reported 10 mm twice (practice log lines 758 and 1139)
and 10 and 11 mm at 14:23 (assessed session log lines 365 and 410), on a
hump we did not measure.

## 6. Summary

| Mission requirement | Assessed result (from the logs) |
|---|---|
| MR1 line following | Followed the line in every attempt |
| MR2 barcode | Decoded correctly in attempt 3 (O = RIGHT); missed in attempts 1 and 2 |
| MR3 barcode command | Not demonstrated: no turn started; U-turn shown in practice |
| MR4–MR5 hump | Detected in all three attempts: 6, 6, 7 mm for a 5 mm hump |
| MR6 obstacle detection | Stopped and scanned in all three attempts |
| MR7 profiling | Side chosen; width accuracy not shown (SYS-06 not met) |
| MR8–MR9 bypass and recovery | Attempt 3: bypassed without contact and recovered the line |
| MR10 telemetry | Telemetry, heartbeat and dashboard worked throughout; both sessions logged |

The full course was not completed in one run (SYS-09 not passed).

## 7. Video evidence

<https://www.youtube.com/watch?v=B1mWqMT2HIk>, "Evidence for fully
functional system few hrs before the demo", uploaded by our team member
Khang.

This is our backup evidence that the integrated system worked on the car
shortly before the assessment. The video is unlisted, so anyone with the
link can watch it. It was recorded before the demo, so it is supporting
evidence of what the system could do (MR1–MR10), not a record of the
assessed attempts: their results and readings come from the telemetry logs
in section 1. The firmware build is not shown in the video. It is listed in
the [video index](../videos/README.md).

## 8. Follow-up tests (not yet run)

- SYS-02: five passes per barcode with flat, taped prints and a recorded
  sensor height; keep the USB console.
- SYS-03: barcode then junction with the console captured, to see whether
  the junction is confirmed and the held command is acted on.
- SYS-04: a flat lap, and humps of several known heights including one
  below 6 mm.
- SYS-06: width against the 15 × 20 cm box in both orientations.
