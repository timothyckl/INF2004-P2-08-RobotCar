# Application data widths

The Tribunal 1 curveball requires each application data field to use only
the bits its valid range needs, with C bit-fields or explicit masks and
shifts where appropriate (`docs/AAI08_PostTri1.pdf`, Constraint Injection;
`AGENTS.md`). This page records, for each interface record and the main
buffers, the valid range of every field, the bits it needs and the bits it
uses now, and the measured sizes of the build.

All sizes were measured on 2026-11-20 at `253a045`, mission image
(`make NET=lwip MQTT=1`), with `sizeof` compiled by `arm-none-eabi-gcc`
using the firmware's own flags, and `arm-none-eabi-size` on the objects and
the ELF. "Bits needed" is computed from the range; no packed version of the
internal records has been built, so the packed sizes below are calculated,
not measured.

## Summary

| Data | Packed now? | Size now | Minimum (calculated) |
|---|---|---|---|
| Telemetry frame v4 (MQTT, since 2026-11-24, S1 decision 7) | **Yes**, masks and shifts | 31 bytes per frame | 29 bytes (same method: two 12-bit pairs left byte-aligned) |
| Same data unpacked (`telemetry_encoder_snapshot_t`) | reference | 48 bytes (44 in v3) | — |
| Telemetry frame v3 (MQTT, until 2026-11-24) | **Yes** | 29 bytes | 27 bytes |
| Interface queue records (IF-06, 09, 11, 21, motion_q) | No, whole-byte fields | 8 to 16 bytes each, 368 bytes of queue memory | about 290 bytes of queue memory |
| `s2_robot_state_t` (shared state) | No | 32 bytes | 21 bytes |
| Barcode capture (`s3_capture_t`) | No | 264 bytes (on the initial task's stack) | 136 bytes with 16-bit ms runs |

The telemetry frame, the only data sent off the car, is packed: 31 bytes
against 48 for the same fields in their C types, 35% smaller (version 3:
29 against 44). The internal
records keep whole-byte fields; packing all of them would save about 220
bytes of RAM in total (below).

## Telemetry frame version 4 (31 bytes)

Version 4 is version 3 with two bytes appended and two flag bits used
(S1 decision 7); bytes 0 to 28 keep their meanings, so version 2 and 3
decoders still read the first part. Layout and packing are in
`app_program/subsystem1/telemetry_encoder.h`.

| Bytes | Field | Range | Bits needed | Bits used |
|---|---|---|---|---|
| 0 | version | 1..4 | 3 | 8 |
| 1 | flags0: 8 validity and state flags (bit 7 = last hump valid, new) | flags | 8 | 8 |
| 2-5 | `captured_ms` | controller clock, wraps | 32 | 32 |
| 6-7 | `distance_mm` | 0..65 535 per run | 16 | 16 |
| 8 | flags1: 5 flags (bit 4 = width raw, new) | flags | 5 | 8 |
| 9 | state, mode | 0..5 each | 3 + 3 | 4 + 4 |
| 10 | fault, line | 0..4 each | 3 + 3 | 4 + 4 |
| 11 | scan result, bypass side | 0..5, 0..2 | 3 + 2 | 4 + 4 |
| 12-15 | left, right wheel speed | ±400 mm/s | 10 + 10 (signed) | 16 + 16 |
| 16-17 | heading | ±18 000 cdeg | 16 (signed) | 16 |
| 18-21 | forward range, nearest point | 20..4000 mm | 12 + 12 | 16 + 16 |
| 22-23 | width (or raw span when width raw) | 0..5656 mm | 13 | 16 |
| 24-25 | centre offset | ±4000 mm | 13 (signed) | 16 |
| 26 | highest hump | 0..255 mm, saturated | 8 | 8 |
| 27 | barcode letter, command | 0..25 (31 invalid), 0..4 (7 invalid) | 5 + 3 | 5 + 3 |
| 28 | stop cause, barcode count | 0..11, low 4 bits | 4 + 4 | 4 + 4 |
| 29 | last hump (new) | 0..255 mm, saturated | 8 | 8 |
| 30 | hump count (new) | 0..255, saturates | 8 | 8 |

The new fields need their full byte: a hump height is whole millimetres up
to the 255 mm saturation, and the count saturates at 255. The raw width
reuses bytes 22-23 and needs only a flag bit. One frame per second is 31
bytes/s of payload, about 248 bit/s, against a WiFi link of several Mbit/s:
the frame is small for the data it carries, so we did not pack it further.

## Re-measured on 2026-11-25 (`a41c19c`, the demo source)

`sizeof` with `arm-none-eabi-gcc` 15.3.1 and the mission flags; sizes with
`arm-none-eabi-size` on the mission image. Unchanged sizes are not
repeated.

| Item | 2026-11-20 (`253a045`) | 2026-11-25 (`a41c19c`) | Why it changed |
|---|---|---|---|
| `telemetry_encoder_snapshot_t` | 44 bytes | 48 bytes | Last hump, hump count, two flags (S1 d7) |
| `s4_terrain_result_t` | 14 bytes | 16 bytes | Hump count (`uint8_t`) plus alignment padding |
| `s2_robot_state_t`, `s2_line_report_t`, `s2_remote_cmd_t`, `s2_scan_cmd_t`, `s2_nav_request_t`, `scan_telemetry_t`, `scan_task_status_t`, `s3_capture_t` | 32, 8, 12, 12, 12, 28, 120, 264 | same | — |
| Application objects: text (code and constants), data, bss | — | 48 024, 41, 1 976 bytes | — |
| Whole mission image: text, data, bss | — | 374 410, 132, 79 988 bytes | — |

The application is 12.8% of the image's code and constants and about 2.5%
of its static RAM; the network stack, USB and kernel make up the rest.

## Measured RAM

| Item | Bytes |
|---|---|
| Whole image `.data` + `.bss` (kernel, lwIP, CYW43, USB, application) | 116 + 84 124 |
| Application static data, all `app_program` objects | 15 + 1 970 |
| Largest application statics: `task_motion.o`, `task_controller.o`, `scan_task.o` | 549, 483, 472 |
| Application task stacks (controller, motion, IMU, obstacle scan; `TASK_STACK_SIZE` 2048 each) | 8 192 |
| Initial task stack (runs `s3_follow_run()`) | 4 096 |
| Message-buffer memory (motion_q 160, remote_q 32, line_q 48, scan_q 96, nav_q 32) | 368 |

**Stack use (static estimate, 2026-11-25).** GCC `-fstack-usage` with the
call graph gives the deepest chain of application frames from each task:
S3 loop 1,872 of 4,096 bytes; controller 1,016, motion 768, IMU 632 and
obstacle scan 392 of 2,048 bytes each. Every frame is fixed in size (no
variable-length arrays). Kernel, SDK and `tm_printf` frames are not in the
figure; `app.h` allows about 1 KB for `tm_printf`, which leaves the
controller the smallest margin (about 400 bytes). This is a calculation,
not a measurement on the car ([log](../testing/host/logs/2026-11-25_stack-usage.txt)).

The RP2040 has 264 KB of SRAM. The application's own data (statics, queues
and the barcode buffer, about 2.6 KB) is about 1% of it; stacks and the
network stack dominate.

## Interface records

Each kernel message-buffer slot costs 4 bytes of header plus the record
(the sizing comments in `subsystem2.h`). Ranges come from the `#define`s
in the headers named.

### `s2_line_report_t` (IF-06, `line_q`, 4 slots) — 8 bytes, 43 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `captured_ms` | controller clock, wraps | 32 | 32 |
| `position` | `S2_LINE_*` 0..4 | 3 | 8 |
| `strength` | 0..100 | 7 | 8 |
| `flags` | `S2_LINE_FLAG_BARCODE` | 1 | 8 |
| `reserved` | zero | 0 | 8 |

### `s2_remote_cmd_t` (IF-09, `remote_q`, 2 slots) — 12 bytes, 66 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `request_id` | nonzero, compared for repeats | 32 | 32 |
| `received_ms` | controller clock, wraps | 32 | 32 |
| `command` | START 1, RESET 3 | 2 | 8 |
| `reserved` | zero | 0 | 24 |

### `s2_scan_cmd_t` (IF-11, `scan_q`, 6 slots) — 12 bytes, 76 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `request_id` | matched by S5 | 32 | 32 |
| `time_ms` | controller clock, for the 2 s expiry | 32 | 32 |
| `kind` | `S2_SCAN_*` 1..8 | 4 | 8 |
| `b_asserted` | 0 or 1 | 1 | 8 |
| `angle_deg` | -45..45 | 7 (signed) | 8 |
| `reserved` | zero | 0 | 8 |

### `s2_nav_request_t` (IF-21, `nav_q`, 2 slots) — 12 bytes, 56 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `scan_id` | S3's decoded count, `uint16_t` | 16 | 32 |
| `staged_ms` | controller clock | 32 | 32 |
| `command` | `S2_NAV_*` 0..4 | 3 | 8 |
| `symbol` | `A`..`Z` | 5 | 8 |
| `reserved` | zero | 0 | 16 |

### `s2_motion_cmd_t` (`motion_q`, 8 slots) — 16 bytes, 80 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `time_ms` | controller clock | 32 | 32 |
| `cmd_id` | wraps, compared for newer | 16 | 16 |
| `cmd` | `S2_CMD_*` 1..6 | 3 | 8 |
| `reserved` | zero | 0 | 8 |
| payload, SPEED | two wheel speeds, ±400 mm/s (`MC_SPEED_MAX_MM_S`) | 20 | 32 |
| payload, MOVE | ±32 767 mm, speed 0..400 | 25 | 32 |
| payload, TURN | ±360 000 mdeg, speed 0..400 | 29 | 64 |

## Shared state

### `s2_robot_state_t` (`state_mutex`) — 32 bytes, 166 bits needed

| Field | Range | Bits needed | Bits used |
|---|---|---|---|
| `captured_ms` | controller clock | 32 | 32 |
| `run_distance_mm` | ±131 m covers any run | 18 (signed) | 32 |
| `last_request_id` | S1 request ID | 32 | 32 |
| `state` | `S2_SYS_*` 1..5 | 3 | 8 |
| `fault` | `S2_FAULT_*` 0..4 | 3 | 8 |
| `flags` | `S2_RS_*`, 7 bits | 7 | 8 |
| `last_command` | `S2_REMOTE_*` 0..3 | 2 | 8 |
| `last_result` | `S2_RESULT_*` 0..8 | 4 | 8 |
| `motion_state` | `motion_state_t` 0..6 | 3 | 8 |
| `mode` | `S2_MODE_*` 0..5 | 3 | 8 |
| `line` | `S2_LINE_*` 0..4 | 3 | 8 |
| `left_mm_s`, `right_mm_s` | ±400 | 10 each (signed) | 16 each |
| `heading_cdeg` | ±18 000 | 16 (signed) | 16 |
| `barcode` | `A`..`Z` | 5 | 8 |
| `barcode_command` | `S2_NAV_*` 0..4 | 3 | 8 |
| `barcode_count` | wraps at 256 | 8 | 8 |
| `stop_cause` | `S2_STOP_*` 0..11 | 4 | 8 |

Other measured snapshots, not packed: `motion_status_t` 28 bytes,
`s4_terrain_result_t` 14 bytes, `scan_telemetry_t` 28 bytes,
`scan_task_status_t` 120 bytes (88 of them the `bypass_plan_t`).

## Buffers

### `s3_capture_t` (barcode runs) — 264 bytes

`runs[64]` holds each bar or space duration as `uint32_t` microseconds.
The longest accepted run is 2.5 s (`S3_BARCODE_RUN_MAX_US`, S2 decision
26), which needs
22 bits in microseconds or 12 bits in milliseconds. A capture is a barcode
only once three runs reach 60 ms, so 1 ms resolution is better than 2% of
the narrowest run that matters. As 16-bit milliseconds the array would take
128 bytes instead of 256.

The 8-bit millisecond durations proposed at Tribunal 1 do not fit: they
assumed the car stays above about 50 mm/s over a barcode, but the mission
crosses barcodes at 40 mm/s and runs reach 2.5 s.

## Proposed changes (not agreed, not implemented)

| Change | RAM saved | Cost |
|---|---|---|
| Barcode runs as `uint16_t` milliseconds | 128 bytes of initial-task stack, more while a capture is copied for decoding | Decoder thresholds move from us to ms; re-run the S3 host cases and SYS-02 |
| Pack the five queue records with masks and shifts | about 80 bytes of queue memory | Pack and unpack in every send and receive, inside the 10 ms controller and motion loops; re-run S2-U39/U40 and the integration tests |
| Pack `s2_robot_state_t` | 11 bytes | Every reader unpacks under `state_mutex`, which lengthens the time the lock is held |

The telemetry frame is where packing pays: it is sent over WiFi every
second and decoded off the car. For the internal records, the whole of the
saving is about 220 bytes, under 0.1% of the RP2040's SRAM, against extra
work in the time-critical tasks. The kernel copies message-buffer records
whole, so packing does not remove any concurrency risk there. Separate
event-flag bits already avoid the shared-word race raised at Tribunal 1
(R6). The barcode buffer is the one internal change with a clear saving
and no effect on a control loop.
