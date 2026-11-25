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
| `tools/barr_c_report.py` (written for this review; report only; [first run](../testing/host/logs/2026-11-25_barr-c-report.txt), [after the narrowing-cast comments](../testing/host/logs/2026-11-25_barr-c-report-2.txt), [final](../testing/host/logs/2026-11-25_barr-c-report-3.txt)) | All 128 files: line width, tabs, trailing space, forbidden keywords, braces, constant position, include guards, end-of-file comment, function length, variable naming, uncommented casts | [2026-11-25_barr-c-report.txt](../testing/host/logs/2026-11-25_barr-c-report.txt) |
| `tools/check_subsystem5_barr_c.py` (enforcing, run by every build) | Subsystem 5, 46 files | "BARR-C check passed" |
| Strict compile with the target compiler (`testing/host/target_static_check.py`): `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wmissing-prototypes -Wswitch-default -Wnull-dereference` and others | 49 production `.c` files | 0 warnings in application code |
| Build flags (S2 d34) | All application objects | Built with `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror` |
| Code review | Rules that need judgement | This report |

The script uses simple text patterns, so its counts are close but not
exact (for example, it cannot tell a cast inside a long expression from a
comment on the line above).

**Statuses.** Compliant: checked, no violation found. Partial: mostly met,
with counted exceptions. Deviation: an exception kept on purpose, with its
reason; those marked "S2 d30" were agreed by the team, the others are our
recommendation and still need the team's approval. Outstanding: not met
and not justified. Not verified: not fully checked.

**How each status was reached.** "(review)" after a status means a person
read the code; no mark means the script or the compiler counted it.
A count of zero from the script is screening, not proof: it matches text
patterns. Zero compiler warnings under `-Wconversion -Wsign-conversion`
shows that no conversion is implicit; it does not show that an explicit
cast is in range, which is why every cast was also read by hand (1.6a).
No rule was found to be not applicable: rules about features we do not use
(`goto`, floating point, unions as overlays) are counted as compliant
because the checks found no use.

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
| 1.6a | Each cast has an associated comment showing it is safe for every possible value | **Compliant (review)** | Every cast now has its own comment: 514 in production code and 80 in tests (2026-11-26, after the post-demo changes; 478 and 63 on 2026-11-25). Narrowing casts were commented first (`032437c`); the other 210 production and 43 test casts were then read one by one and given a comment that names the source type, its range and why the destination holds it (section 3a). Two casts were found to need more than a comment (see section 3a) | Comments only: the mission UF2 stayed byte-identical. The script only shows that a comment is near each cast; the arguments were checked by hand |
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
| 6.2a | Functions at most 100 lines | **Deviation** | 12 functions are longer on 2026-11-26 (13 on 2026-11-25; line counts include the cast comments): `s3_follow_run` (175), `obstacle_scan_task` (154), `line_step` (153), `nav_control_step` (126), `control` (120), `motion_command` (117), `s4_imu_task` (110), `controller_task` (108) and five test functions. Each is one task loop or one state machine whose cases share local state; splitting them would change the code the compiler produces, so the mission image could no longer be proved identical to the demonstrated one, and the split code would need new car tests (S2 d30) |
| 6.3 | Prefer functions to function-like macros | Partial | 5 function-like macros: `CHECK`, `TEST_INIT`, `TEST_CHECK`, `TEST_SUMMARY` need the caller's line or `return`; `S3_LINE_LOOP_DONE` and `S4_DIAG_STAGE` compile to nothing in the mission image |
| 6.4a | Task functions named `..._task` | Partial | `controller_task`, `motion_task`, `s4_imu_task`, `obstacle_scan_task`; the S3 loop `s3_follow_run()` runs in the kernel's initial task and keeps its name |
| 6.5 | ISRs named `..._isr`, static, short | Compliant | `drv_gpio_irq_isr`, `drv_encoder_isr`, `drv_ultrasonic_isr`, all `static`; each does a few loads and stores |

### Chapter 7 — Variables

| Rule | In our words | Status | Count / note |
|---|---|---|---|
| 7.1a–d | No keyword or library names, no leading underscore, at most 31 characters | Compliant (review) | `test_pass_` in `TEST_INIT` has a trailing, not leading, underscore |
| 7.1e | Names at least 3 characters, including loop counters | **Partial** | Production code: 4 left, all structure members (`kp`, `ki`, `kd` in `pid.h`, the standard control-theory names, and `id` in `scan_look.h`); 72 variables and parameters were renamed on 2026-11-25 (for example `i` to `idx`, `er` to `ercd`, `in` to `inputs`). Test code: 143 short names left (2026-11-26) (mostly `i`, `a`, `b`, `in`, `id` in on-target and host test files) | The first checker missed indented local declarations, so the earlier count (39 in production) was too low. Production renames do not change the generated code: the mission UF2 stayed byte-identical and every build option compiled with 0 warnings. Test names were not renamed: a mechanical rename gave names no clearer than the originals (`val_a` for `a`), and each needs a person who knows the test to choose a good name |
| 7.1f, h | Lower case with underscores | Compliant (review) | — |
| 7.1j | Globals start with `g` | **Deviation** | The interface objects (`controller_flags`, `safety_flags`, `remote_q`, `line_q`, `scan_q`, `nav_q`, `motion_q`) keep their agreed design names (S2 d30); all other file-scope variables use `g_` |
| 7.1k | Pointers start with `p` | Compliant | The two pointer parameters (`name` in `app_start_task()`, `step` in two S2 hardware tests) were renamed `p_name` and `p_step`. The script still lists three string tables (`reasons`, `names`, `s_contact_names`); they are arrays, not pointer variables, so the rule does not apply to them |
| 7.1m | Booleans start with `b` | Compliant | All Boolean variables, parameters and structure members now start with `b_` (globals `g_b_`, `gb_`): 67 in production code and 23 in tests were renamed on 2026-11-25 (two more were `g_b_` globals the checker did not recognise), including the members of `telemetry_encoder_snapshot_t`, `s3_capture_t` and the S1 timer and bridge structures | Checked the same way as 7.1e |
| 7.2a | Initialise every variable before use | Compliant | `-Wall` (`-Wmaybe-uninitialized`) and `-fanalyzer` report none |
| 7.2c | File-scope variables grouped at the top | Compliant (review) | — |

### Chapter 8 — Statements

| Rule | In our words | Status | Note |
|---|---|---|---|
| 8.1 | One declaration per line | Compliant | Production code had none. The review found 17 lines in the on-target test files (`subsystem2_test.c`, `test_motion.c`) declaring several variables at once; the script does not look for them. They were split on 2026-11-25 |
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

## 3a. How each cast was justified (rule 1.6a)

Every cast was read with its source type and the range the code allows.
The comments fall into these groups:

| Group | Example | Argument in the comment |
|---|---|---|
| Narrowing after a check or clamp | `(int16_t)clamp32(...)`, `(uint32_t)limit` after clamping to `LIMIT_US_MAX` | The value has been limited to the destination's range first |
| Widening before arithmetic | `(int64_t)counts * wheel_circ_um` | The destination holds every source value; the product cannot overflow in 64 bits, and the result is narrowed only after `odom_fits_i32()` |
| Same width, change of signedness or kernel type | `(uint32_t)tim.lo`, `~(UINT)SAFETY_FLG_STOP` | Both types are 32-bit; no bit changes. Clock values wrap and are only subtracted |
| Console arguments | `(UINT)status.flags` for `%u` | The field is 8 to 32 bits and `INT`/`UINT` are 32-bit on the RP2040 |
| Message sizes | `(INT)sizeof(s2_line_report_t)` | A structure of a few bytes fits `INT`, the kernel's size type |
| Function pointers | `(FP)controller_task` | `FP` is the kernel's generic entry type; only the pointer type changes |

**What the review found.**

- `motion_logic.c`: the tolerances `move_tol_mm` and `turn_tol_mdeg` are
  `uint32_t` and are cast to `int32_t`. `motion_init()` does not check
  them, so a value above `INT32_MAX` would be converted in an
  implementation-defined way. Today they are build-time constants (5 and
  2000), so the cast is safe; the comment says so. A range check in
  `motion_init()` would make it safe by design, but changes the code, so
  it is noted for later rather than made now.
- `task_imu.c` (`S4_RAW_LOG=1` builds only): the stall report printed a
  `UW` with `%lx`, which expects `unsigned long`. Both are 32-bit on the
  RP2040, so the output was right, but the format did not match the type.
  It now uses `%x` and `UINT`. The mission image does not contain this code
  and stayed byte-identical.

The `(UB *)` casts on `tm_printf` format strings change only the pointer
type of a string literal (char to unsigned char), as the kernel's console
interface requires.

**Checker changes (2026-11-25).** The first version counted only a comment
on the same line, which overstated the problem (479). It now accepts a
comment in the same paragraph, or a comment earlier in the same function
that argues the range. Its 7.1e pattern also missed indented local
declarations, which hid most short names; it now finds them. It is still a
text-pattern tool: it shows that a comment is near each cast, not that the
argument is right, so the comments were written and checked by hand.

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
| Compliant | 1.1c, 1.1d, 1.2a, 1.3a, 1.3b, 1.4, 1.5 (from this report), 1.6a, 1.7a–c, 1.8a, 1.8c, 2.1, 3.1, 3.4, 3.5a, 3.6, 4.2, 4.3, 4.4, 5.1, 5.3–5.6, 6.1, 6.5, 7.1a–d, 7.1f–h, 7.1k, 7.1m, 7.2, 8.1–8.6 |
| Partial | 1.8b, 2.2, 6.3, 6.4a, 7.1e (4 structure members; 143 names in test code) |
| Deviation (agreed) | 1.1a, 1.7d, 4.1, 5.2a, 6.2a, 7.1j |
| Non-compliant | None found by the checks above; this is not a claim of full compliance |
| Not verified | 3.2, 3.3 |

After the demo, every cast was given its own range comment and the naming
rules 7.1e, 7.1k and 7.1m were brought into line by renaming variables,
parameters and Boolean structure members. None of this changes behaviour,
and we proved it: the mission UF2 stayed byte-identical
(`8cf2c419…`), every build option compiled with 0 warnings, and all host
tests passed. The open items are the agreed deviations, four short
structure-member names, the short names in test code and the partial
rules above.
