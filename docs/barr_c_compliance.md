# BARR-C:2018 compliance

This report checks the application code (`app_program/`, 128 `.c` and `.h`
files including on-target test files) against the Barr Group *Embedded C
Coding Standard* (BARR-C:2018,
[barr_c_coding_standard_2018.pdf](barr_c_coding_standard_2018.pdf)). Kernel,
SDK, lwIP and CYW43 code is third-party and out of scope.

We do **not** claim full compliance. Rules a script can check were checked
on every file; rules that need judgement were checked by review and
compiler warnings, and are marked as such. Deviations are those agreed in
S2 decision 30 ([decisions.md](decisions.md)) unless stated.

The standard has eight rule chapters: general rules, comments, white
space, modules, data types, procedures, variables and statements. Rules on
expressions sit inside chapter 1 (1.4 parentheses, 1.6 casts) and chapter 8
(8.6 equivalence tests).

## 1. How it was checked

| Method | Scope | Output |
|---|---|---|
| `tools/barr_c_report.py` (written for this review; report only) | All 128 files: line width, tabs, trailing space, forbidden keywords, braces, constant position, include guards, end-of-file comment, function length, variable naming, uncommented casts | [2026-11-25_barr-c-report.txt](../testing/host/logs/2026-11-25_barr-c-report.txt) |
| `tools/check_subsystem5_barr_c.py` (enforcing, run by every build) | Subsystem 5, 46 files | "BARR-C check passed" |
| Strict compile with the target compiler (`testing/host/target_static_check.py`): `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wmissing-prototypes -Wswitch-default -Wnull-dereference` and others | 49 production `.c` files | 0 warnings in application code |
| Build flags (S2 d34) | All application objects | Built with `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror` |
| Code review | Rules that need judgement | This report |

The script uses simple text patterns, so its counts are close but not
exact (for example, it cannot tell a cast inside a long expression from a
comment on the line above).

**Statuses.** Compliant: checked, no violation found. Partial: mostly met,
with counted exceptions. Deviation: agreed exception with a reason.
Non-compliant: not met. Not verified: not fully checked.

## 2. Rule-by-rule results

### Chapter 1 — General rules

| Rule | In our words | Status | Evidence / count | Note |
|---|---|---|---|---|
| 1.1a | Write C99 | **Deviation** | Host tests compile every logic file with `-std=c99 -pedantic`; subsystem 5 builds with `-std=c99` on the target; the rest builds as `gnu11` | The Pico SDK headers need C11 static assertions, so the kernel boundary uses `gnu11` |
| 1.1c | Keep extensions and inline assembly to a few drivers | Compliant | Only `__asm volatile ("dmb")` in `drv_encoder.c` and `drv_ultrasonic.c` | Memory barriers for ISR data |
| 1.1d | Do not rename keywords with `#define` | Compliant | Review | — |
| 1.2a | Lines at most 80 characters | Compliant | 0 | — |
| 1.3a | Braces round every `if`, `else`, `for`, `while`, `do`, `switch` body | Compliant | 0 | — |
| 1.3b | Left brace on its own line | Compliant | 0 | — |
| 1.4a, b | Parenthesise to show order; operands of `&&` and `\|\|` in brackets | Compliant (review) | `-Wparentheses` clean; review of the files read for the software review | Not counted by script |
| 1.5a, b | Avoid unclear abbreviations; keep a project abbreviations table | **Compliant from 2026-11-25** | Section 4 of this report is that table | Previously missing |
| 1.6a | A comment on every cast saying why it is safe | **Non-compliant** | 479 casts with no comment on their line | Open since S2 d30. The strict compile (`-Wconversion`, `-Wsign-conversion`) shows no implicit conversion, so each cast is deliberate, but its range argument is not written down |
| 1.7a, b | No `auto`, no `register` | Compliant | 0, 0 | — |
| 1.7c | Avoid `goto` | Compliant | 0 | — |
| 1.7d | Avoid `continue` (preferred practice) | **Deviation** | 7 (`drv_encoder.c` 2, `drv_gpio_irq.c` 1, `s3_follow.c` 3, `task_imu.c` 1) | Each skips to the next poll period in a tested loop (S2 d30) |
| 1.8a | `static` for everything not used outside its module | Compliant | `-Wmissing-prototypes` clean: every non-static function has a header prototype | — |
| 1.8b | `const` where appropriate | Partial (review) | Used for parameters (`p_cfg const *`), tables and locals | Not counted |
| 1.8c | `volatile` for data shared with ISRs | Compliant | Encoder counters, sequence word and echo ring buffer are `volatile` | `volatile` does not give atomicity; the sequence counter and ring buffer do (section 3) |

### Chapter 2 — Comments

| Rule | In our words | Status | Note |
|---|---|---|---|
| 2.1 | Use only the accepted comment formats | Compliant (review) | `/* */` and Doxygen `/** */` |
| 2.2 | File, function and decision comments in the right places | Partial (review) | Every file has a `@file` header and an end-of-file comment (0 missing); most functions have a header comment; magic values carry the test or decision that set them, and 33 are marked UNMEASURED or PROVISIONAL |

### Chapter 3 — White space

| Rule | In our words | Status | Evidence |
|---|---|---|---|
| 3.1 | Spaces round operators and after keywords | Compliant (review) | S2 d30 fixed the starter files (3.1j) |
| 3.2, 3.3 | Alignment and blank lines | Not verified | Style follows the existing files |
| 3.4 | Four-space indentation | Compliant (review) | No tabs anywhere |
| 3.5a | No tab characters | Compliant | 0 |
| 3.6 | No non-printing characters or trailing spaces | Compliant | 0 |

### Chapter 4 — Modules

| Rule | In our words | Status | Note |
|---|---|---|---|
| 4.1 | Module names lower case, unique in 8 characters | **Deviation** | `subsystemN.c/.h`, `app_main.c` come from the course template; `telemetry_snapshot.h`/`telemetry_encoder` and `motion_config.h`/`motion_cmd.h` share 8 characters (S2 d30) |
| 4.2 | Header guard; header holds only the public interface | Compliant | 0 headers without a guard |
| 4.3 | Each source file includes its own header | Compliant (S2 d30 check) | — |
| 4.4 | Follow the file templates | Compliant | `@file` header and end-of-file comment in every file |

### Chapter 5 — Data types

| Rule | In our words | Status | Note |
|---|---|---|---|
| 5.1 | Type names end in `_t` | Compliant (review) | All application types (`motion_t`, `s2_robot_state_t`, …) |
| 5.2a | Fixed-width integer types; no `short` or `long` | **Deviation** | 0 `short` or `long`; T-Kernel types (`INT`, `UINT`, `ER`, `ID`, `UB`, `UW`) where a kernel call needs them (S2 d30) |
| 5.3 | Do not mix signed and unsigned without care | Compliant | `-Wsign-conversion` and `-Wconversion` give 0 warnings |
| 5.4 | Avoid floating point | Compliant | No `float` or `double` in the application: all maths is integer or fixed point (µm, mg, mdeg, per mille) |
| 5.5 | Structures and unions: no overlays relying on layout | Compliant | The telemetry frame is written byte by byte with shifts (`telemetry_encoder.c`), never by a struct overlay |
| 5.6 | Booleans from `stdbool.h` | Compliant | `bool` throughout |

### Chapter 6 — Procedures

| Rule | In our words | Status | Note |
|---|---|---|---|
| 6.1 | Function names: lower case, module prefix, verb-like | Compliant (review) | Module prefixes (`s3_scan_`, `nav_control_`, `motion_`) |
| 6.2a | Functions at most 100 lines | **Deviation** | 13 functions are longer: `s3_follow_run` (172), `line_step` (151), `obstacle_scan_task` (152), `nav_control_step` (126), `motion_command` (114), `control` (115), `controller_task` (104), `s4_imu_task` (110) and five test functions. Splitting tested control loops after the demo is a behaviour risk (S2 d30) |
| 6.3 | Prefer functions to function-like macros | Partial | 5 function-like macros: `CHECK`, `TEST_INIT`, `TEST_CHECK`, `TEST_SUMMARY` need the caller's line or `return`; `S3_LINE_LOOP_DONE` and `S4_DIAG_STAGE` compile to nothing in the mission image |
| 6.4a | Task functions named `..._task` | Partial | `controller_task`, `motion_task`, `s4_imu_task`, `obstacle_scan_task`; the S3 loop `s3_follow_run()` runs in the kernel's initial task and keeps its name |
| 6.5 | ISRs named `..._isr`, static, short | Compliant | `drv_gpio_irq_isr`, `drv_encoder_isr`, `drv_ultrasonic_isr`, all `static`; each does a few loads and stores |

### Chapter 7 — Variables

| Rule | In our words | Status | Count / note |
|---|---|---|---|
| 7.1a–d | No keyword or library names, no leading underscore, at most 31 characters | Compliant (review) | `test_pass_` in `TEST_INIT` has a trailing, not leading, underscore |
| 7.1e | Names at least 3 characters, including loop counters | **Partial** | 66 short names (mostly `i` 19, `id` 10, `v` 6) |
| 7.1f, h | Lower case with underscores | Compliant (review) | — |
| 7.1j | Globals start with `g` | **Deviation** | The interface objects (`controller_flags`, `safety_flags`, `remote_q`, `line_q`, `scan_q`, `nav_q`, `motion_q`) keep their agreed design names (S2 d30); all other file-scope variables use `g_` |
| 7.1k | Pointers start with `p` | Partial | 7 exceptions, all name or string tables (`names`, `reasons`, `name`) |
| 7.1m | Booleans start with `b` | **Partial** | 100 local Booleans without `b_` (for example `valid`, `ok`, `accepted`); structure members and globals mostly comply (`b_valid`, `gb_us_overrun`) |
| 7.2a | Initialise every variable before use | Compliant | `-Wall` (`-Wmaybe-uninitialized`) and `-fanalyzer` report none |
| 7.2c | File-scope variables grouped at the top | Compliant (review) | — |

### Chapter 8 — Statements

| Rule | In our words | Status | Note |
|---|---|---|---|
| 8.1 | One declaration per line | Compliant | S2 d30 fixed the one comma declaration |
| 8.2 | Conditionals: no assignments inside, else-if chains end in `else` | Compliant (review) | Long `else if` chains end in a commented `else` |
| 8.3 | Every `switch` has a `default` | Compliant | `-Wswitch-default` gives 0 warnings |
| 8.4 | Loops: `for (;;)` for infinite loops, no empty bodies | Compliant (S2 d30 check) | — |
| 8.5 | Jumps | Compliant | No `goto`; early `return` used for argument checks |
| 8.6a | Constant on the left of `==` and `!=` | Compliant | 0 found by script (S2 d30 fixed the starter files) |

## 3. Interrupts, shared data and `volatile`

BARR-C 1.8c asks for `volatile` on ISR data, but `volatile` only stops the
compiler caching a value; it does not make a multi-word update atomic. We
handle that separately:

| Data | Writer | Reader | Protection |
|---|---|---|---|
| Encoder counts, edge times, errors (`drv_encoder.c`) | Encoder ISR | Motion Control | `volatile`, plus a sequence counter incremented before and after each update with `dmb` barriers; the reader retries if it changed |
| Echo edges (`drv_ultrasonic.c`) | Echo ISR | Obstacle scan task | `volatile` ring buffer; only the ISR moves `head` and only the task moves `tail`; `dmb` before publishing `head`; overrun flagged |
| Shared snapshots (robot state, motion status, S4 result, S5 status) | One task each | Other tasks | Kernel mutexes with `TA_INHERIT`, read with `TMO_POL` |
| Urgent events | Several tasks | Controller | Kernel event-flag bits (set and cleared atomically by the kernel) |
| Barcode letter table (`s3_nav.c`) | WiFi task | S3 loop | Single-byte `volatile` entries; a read can never be torn (see the review, F-13) |

## 4. Project abbreviations (rule 1.5b)

| Abbreviation | Meaning | Abbreviation | Meaning |
|---|---|---|---|
| ADC | analogue-to-digital converter | mm, um | millimetres, micrometres |
| AO | analogue output | mdeg, cdeg | thousandths, hundredths of a degree |
| cfg | configuration | mg | thousandths of g (acceleration) |
| cmd | command | ms, us | milliseconds, microseconds (`_us` suffix) |
| ctrl | Vehicle Controller | MC | Motion Control |
| cyc | cyclic handler | mbf | message buffer |
| drv | driver | mtx | mutex |
| flg | event flag | nav | barcode navigation |
| GP*n* | RP2040 GPIO pin *n* | odom | odometry |
| hb | heartbeat | PID | proportional-integral-derivative control |
| idx | index | ptn | event-flag pattern |
| IF-*nn* | interface number in `TEAM.md` section 5 | PWM | pulse-width modulation |
| IMU | inertial measurement unit (LSM303DLHC) | S1..S5 | subsystems 1 to 5 |
| IRQ, ISR | interrupt request, interrupt service routine | seq | sequence counter |
| LSM | LSM303DLHC accelerometer/magnetometer | US (`DRV_US_`) | ultrasonic sensor (not microseconds) |
| MQTT | Message Queuing Telemetry Transport | tsk | task |

## 5. Summary

| Status | Rules |
|---|---|
| Compliant | 1.1c, 1.1d, 1.2a, 1.3a, 1.3b, 1.4, 1.5 (from this report), 1.7a–c, 1.8a, 1.8c, 2.1, 3.1, 3.4, 3.5a, 3.6, 4.2, 4.3, 4.4, 5.1, 5.3–5.6, 6.1, 6.5, 7.1a–d, 7.1f–h, 7.2, 8.1–8.6 |
| Partial | 1.8b, 2.2, 6.3, 6.4a, 7.1e, 7.1k, 7.1m |
| Deviation (agreed) | 1.1a, 1.7d, 4.1, 5.2a, 6.2a, 7.1j |
| Non-compliant | 1.6a (comments on casts) |
| Not verified | 3.2, 3.3 |

The largest open item is 1.6a. Adding a range comment to each of the
479 casts, and renaming the local Booleans and short loop counters (7.1e,
7.1m), changes no behaviour and can be done mechanically, but each file
would then need its host tests and a rebuild with a checksum comparison.
We list it as the next clean-up rather than doing it after the demo.
