# Proposed Subsystem 1 contracts for team review

Status: **UNAPPROVED PROPOSAL**. This file gives the team a concrete starting
point. It is not a validated result and is not an entry in
`docs/decisions.md`. After all affected owners agree, copy only the agreed
resolution into `docs/decisions.md` under Subsystem 1 and add the interfaces
to `TEAM.md`.

## Recommended decisions

The recommendations below preserve the existing architecture: the Vehicle
Controller authorises movement and recovery, only Motion Control writes motor
PWM, remote STOP bypasses the normal command queue, and all network API calls
remain in `cyw43_utk_task`.

### 1. MQTT topic layout

Use four separate topics under one configurable robot base:

```text
p2-08/<robot-id>/command
p2-08/<robot-id>/ack
p2-08/<robot-id>/telemetry
p2-08/<robot-id>/status
```

`<robot-id>` must be unique for the physical car and must also be included in
the MQTT client ID. Keep the full topic at or below 63 bytes so the current
bounded service bridge can copy it safely.

| Topic | Direction | QoS | Retain | Reason |
|---|---|---:|---:|---|
| `command` | Remote to robot | 1 | No | Delivery retry is useful; request ID handles duplicates |
| `ack` | Robot to remote | 1 | No | Command outcome should not be silently lost or replayed later |
| `telemetry` | Robot to remote | 0 | No | A later snapshot supersedes an older one |
| `status` | Robot to remote | 0 | Yes | New subscribers can see the latest online/offline state |

Configure an MQTT Last Will on `status` with an `offline` payload. Publish
`online` after the command subscription succeeds. Reconnection must publish
status and resume fresh telemetry; it must not issue START, clear STOP or
replay application commands.

The locally installed Pico SDK 1.5.1 lwIP MQTT implementation always requests
a clean session. Confirm that behaviour in the SDK version selected for the
project build before approval. Commands must still use request IDs because
QoS 1 may redeliver an individual message.

### 2. Remote command wire format

Replace the temporary plain-text tokens with this fixed six-byte version 1
packet after the team approves it:

| Byte | Field | Width | Meaning |
|---:|---|---:|---|
| 0 | `version` | 8 bits | Must equal 1 |
| 1 | `command` | 8 bits | 1 = START, 2 = STOP, 3 = RESET |
| 2–5 | `request_id` | 32 bits | Nonzero identifier, little-endian |

Exact payload length is six bytes. Reject every other version, command value,
zero request ID or length. Do not accept JSON, whitespace, case variants or a
terminating null byte on this topic.

The packet intentionally has no sender timestamp. Comparing a remote clock
with Pico uptime would require clock synchronisation that the design does not
currently provide. Freshness is instead enforced as follows:

- `command` publications are never retained;
- the selected build must use a clean MQTT session to prevent broker-side
  session backlog;
- Buddy 2 records processed request IDs and rejects duplicates;
- Buddy 1 records local `received_ms` immediately after the complete payload
  is received;
- START/RESET must reach the controller within a team-agreed local deadline
  or be rejected, never queued for later execution;
- STOP remains safe to repeat and is always routed urgently.

If commands may originate from more than one remote publisher, the team must
add an authenticated sender identifier or define how request IDs are unique
across publishers before approving this format.

### 3. Command acknowledgement wire format

Use this fixed eight-byte version 1 packet on `ack`:

| Byte | Field | Width | Meaning |
|---:|---|---:|---|
| 0 | `version` | 8 bits | Must equal 1 |
| 1 | `command` | 8 bits | Mirrors the command value |
| 2 | `result` | 8 bits | Result code below |
| 3 | `controller_state` | 8 bits | Buddy 2-owned compact state code |
| 4–7 | `request_id` | 32 bits | Mirrors request ID, little-endian |

Proposed result codes:

| Value | Meaning |
|---:|---|
| 1 | Received and forwarded to the controller |
| 2 | Rejected as malformed or unsupported |
| 3 | Rejected as duplicate |
| 4 | Rejected as stale or expired locally |
| 5 | Rejected because the controller interface was unavailable/full |
| 6 | Rejected by controller state or safety rules |
| 7 | Completed |
| 8 | Failed after acceptance |

An acknowledgement saying "forwarded" does not mean motion was authorised or
completed. STOP acknowledgement must distinguish flag acceptance from wheels
confirmed stationary. Buddy 2 decides when `completed` is valid.

### 4. Buddy 1 to Buddy 2 interfaces

Buddy 2 owns all controller objects. Use these public names unless Buddy 2 has
already allocated conflicting names:

| Purpose | Proposed name | Owner | Behaviour |
|---|---|---|---|
| Urgent remote STOP event | `CTRL_FLG_REMOTE_STOP` in `controller_flags` | S2 | Nonblocking event bit; bypasses normal queue |
| START/RESET input | `mbf_remote_commands` | S2 | Message buffer; sender uses `TMO_POL` |
| Controller outcome | `mbf_remote_results` | S1 | Message buffer; S2 uses `TMO_POL` |

Buddy 2 must select the STOP bit value after allocating a distinct bit for
every controller event. Buddy 1 must not guess it.

Proposed local START/RESET message, copied through `mbf_remote_commands`:

```c
typedef struct
{
    uint32_t request_id;
    uint32_t received_ms;
    uint8_t  command;
    uint8_t  reserved[3];
} remote_command_t;
```

The reserved bytes are explicitly zeroed so the message is deterministic and
has no implicit tail bytes on the RP2040 compiler. The local structure is not
sent over MQTT. Configure and test the message buffer to hold exactly two
records initially; its byte capacity must include the kernel's per-message
overhead rather than assuming `2 * sizeof(remote_command_t)`. A full queue
causes immediate rejection and an acknowledgement; it never blocks the WiFi
service task.

STOP request ID and receipt time cannot travel in an event bit. Buddy 2 must
provide a nonblocking adapter that stores this metadata using its own safe
controller mechanism and then sets `CTRL_FLG_REMOTE_STOP`. The existing
`service_bridge` callback hides that implementation from networking.

### 5. Timing and reconnect defaults

Use the existing starter timings as provisional starting values because they
already compile and no measured requirement supersedes them:

| Item | Proposed initial value | Validation needed |
|---|---:|---|
| MQTT heartbeat/status period | 5 s | Broker capture and disconnect detection |
| MQTT reconnect delay | 3 s | Control timing during broker outage |
| WiFi rejoin delay | 5 s | Control timing during AP outage |
| Telemetry snapshot period | 1 s | Remote usefulness and control interference |
| START/RESET local forwarding deadline | 2 s | Team review; stale-command injection |

These values are proposals, not validated timing results. Reconnect timers
must not busy-loop. Telemetry missed while offline is dropped; after reconnect
publish the newest snapshot only. Discrete completion/fault outcomes may be
queued only within an agreed bounded queue and expiry.

### 6. Telemetry envelope

Do not freeze sensor field widths until each owner supplies valid range and
precision. Agree the outer envelope now and add only reviewed sections later.

Proposed common header:

| Byte | Field | Width | Meaning |
|---:|---|---:|---|
| 0 | `version` | 8 bits | Must equal 1 |
| 1 | `message_type` | 8 bits | 1 = periodic snapshot, 2 = discrete event |
| 2–5 | `sequence` | 32 bits | Publish sequence, little-endian |
| 6–9 | `capture_ms` | 32 bits | Pico uptime at snapshot, little-endian |
| 10–11 | `valid_sections` | 16 bits | One validity bit per approved section |

Proposed section ownership:

| Section bit | Provider | Content still required from owner |
|---:|---|---|
| 0 | S2 | Controller state, STOP/fault state, motion summary |
| 1 | S3 | Line validity, barcode/navigation result |
| 2 | S4 | Terrain state, hump/highest-peak result |
| 3 | S5 | Obstacle state, essential range/profile/bypass result |
| 4 | S1 | WiFi/MQTT state and communication counters |

Each section begins with a one-byte section length followed by its agreed
fixed fields. Missing or stale sections clear their validity bit and have zero
length; do not reuse zero as an invalid sensor reading. The total payload must
remain bounded by a team-agreed maximum, proposed initially as 64 bytes.

The owner of each section must fill in this table before Buddy 1 encodes it:

| Field | Unit | Valid range | Resolution | Width | Invalid/stale rule | Max age | Protection |
|---|---|---|---|---:|---|---:|---|
| _Owner to complete_ | | | | | | | |

Detailed IR samples, raw IMU samples and ultrasonic scan arrays remain local.
Only state, essential measurements and discrete outcomes go into telemetry.

### 7. Deployment values that cannot be chosen in software design

Keep these in the gitignored `config/wifi_secrets.h`; do not put secrets in
this proposal or source control:

- WiFi SSID and password;
- broker IPv4 address and port; hostname support needs a separate DNS design;
- broker username/password if required;
- TLS requirement and certificate provisioning;
- the physical car's unique robot/client ID.

The current build only configures an IPv4 literal and does not establish TLS.
Do not claim broker compatibility until the actual course/network service is
known and tested.

## Review checklist

| Reviewer | Must approve | Status |
|---|---|---|
| Buddy 1 | MQTT topics, wire codecs, timing and failure handling | Pending |
| Buddy 2 | STOP adapter, event bit, queues, command/result records | Pending |
| Buddy 3 | Line/barcode telemetry fields and ranges | Pending |
| Buddy 4 | Terrain/hump telemetry fields and ranges | Pending |
| Buddy 5 | Obstacle/bypass telemetry fields and ranges | Pending |
| Team | Shared names, object budget, payload maximum and test plan | Pending |
| Instructor | Change inside shared WiFi/MQTT service task | Pending |

Suggested review outcome: approve, approve with listed changes, or reject with
a replacement value for each disputed item. Once approved, update
`docs/decisions.md`, `TEAM.md`, the codec tests and golden packets together.

## Filled configuration worksheet

Use the status column literally:

- **Proposed** means the team may approve or change the supplied value.
- **Owner input** means the named subsystem owner must provide the value.
- **Deployment input** means it comes from the actual network or broker and
  must not be invented or committed if secret.

### Network and MQTT

| Value | Filled value | Status / source |
|---|---|---|
| WiFi SSID | `TBD_NETWORK_SSID` | Deployment input: hotspot/router owner |
| WiFi password | `TBD_NETWORK_PASSWORD` | Deployment input; keep only in gitignored `wifi_secrets.h` |
| WiFi authentication | Current code defaults to WPA/WPA2 mixed PSK | Deployment input: verify actual network compatibility |
| Broker IPv4 address | `TBD_BROKER_IPV4` | Deployment input: instructor or broker operator |
| Broker port | `TBD_BROKER_PORT` | Deployment input; do not assume 1883 if authentication/TLS differs |
| Broker username | `TBD` or `NULL` if broker confirms none | Deployment input |
| Broker password | `TBD` or `NULL` if broker confirms none | Deployment input; never commit |
| TLS | `TBD_REQUIRED_OR_NOT` | Deployment input; current `mqtt_utk.c` does not configure TLS |
| Robot ID | `p2-08-car-01` | Proposed by Buddy 1 |
| MQTT client ID | `p2-08-car-01` | Proposed; only one connected client may use it |
| Base topic | `p2-08/p2-08-car-01` | Proposed |
| Command topic | `p2-08/p2-08-car-01/command` | Proposed |
| Acknowledgement topic | `p2-08/p2-08-car-01/ack` | Proposed |
| Telemetry topic | `p2-08/p2-08-car-01/telemetry` | Proposed |
| Status topic | `p2-08/p2-08-car-01/status` | Proposed |
| Command QoS / retain | QoS 1 / retain false | Proposed |
| Acknowledgement QoS / retain | QoS 1 / retain false | Proposed |
| Telemetry QoS / retain | QoS 0 / retain false | Proposed |
| Status QoS / retain | QoS 0 / retain true | Proposed |
| Last Will | Retained `offline` on status topic | Proposed |
| Online status | Retained `online` after command subscription succeeds | Proposed |
| Maximum topic length | 63 bytes excluding terminator | Proposed, matches bridge buffer |
| Maximum telemetry payload | 64 bytes initially | Proposed; recheck after field tables |

For developer testing, use a different client ID such as `p2-08-hy-dev` so a
developer connection cannot disconnect the physical car by reusing its ID.

### Command and controller integration

| Value | Filled value | Status / source |
|---|---|---|
| Wire command packet | 6 bytes: version, command, 32-bit request ID | Proposed |
| Protocol version | 1 | Proposed |
| Command values | 1 START, 2 STOP, 3 RESET | Proposed; matches current local command IDs |
| Multi-byte order | Little-endian | Required by project decisions |
| Request ID | Nonzero unsigned 32-bit value | Proposed |
| Remote command retain | Always false | Proposed safety rule |
| Duplicate handling | Reject an already processed request ID | Proposed; Buddy 2 stores processed IDs |
| START/RESET forwarding deadline | 2000 ms from local receipt | Proposed, unvalidated |
| STOP event object | `controller_flags` | Planned architecture; Buddy 2 owns it |
| STOP event-bit name | `CTRL_FLG_REMOTE_STOP` | Proposed name |
| STOP event-bit value | `TBD_BY_BUDDY_2` | Owner input: must not conflict with other controller bits |
| START/RESET object | `mbf_remote_commands` | Proposed name; Buddy 2 owns it |
| START/RESET capacity | Exactly two 12-byte records plus kernel overhead | Proposed; verify with full-buffer test |
| START/RESET send timeout | `TMO_POL` | Proposed nonblocking rule |
| Controller-result object | `mbf_remote_results` | Proposed name; Buddy 1 owns it |
| Controller-result capacity | Four result records plus kernel overhead | Proposed: two outcomes for each of two pending commands |
| Full/unavailable interface | Reject immediately and publish result code 5 | Proposed |
| Controller state codes | `TBD_BY_BUDDY_2` | Owner input |
| States permitting START | `TBD_BY_BUDDY_2` | Owner input |
| States permitting RESET | `TBD_BY_BUDDY_2` | Owner input |
| STOP metadata handoff | Nonblocking S2 adapter stores request ID/time, then sets flag | Proposed mechanism; implementation owned by Buddy 2 |

The proposed local START/RESET record is 12 bytes:

| Field | Type | Filled rule |
|---|---|---|
| `request_id` | `uint32_t` | Copied from wire packet |
| `received_ms` | `uint32_t` | Pico uptime when complete command arrived |
| `command` | `uint8_t` | START or RESET only |
| `reserved[3]` | `uint8_t[3]` | Always zero |

The proposed local controller-result record is eight bytes:

| Field | Type | Filled rule |
|---|---|---|
| `request_id` | `uint32_t` | Matches the command being reported |
| `command` | `uint8_t` | START, STOP or RESET |
| `result` | `uint8_t` | Uses the acknowledgement result table |
| `controller_state` | `uint8_t` | Buddy 2 supplies the enum value |
| `reserved` | `uint8_t` | Always zero |

Buddy 2 sends this record to `mbf_remote_results` with `TMO_POL`. Buddy 1
publishes the corresponding eight-byte acknowledgement packet. Buddy 2 must
agree that the proposed fields are sufficient before the record is frozen.

### Timing and failure handling

| Value | Filled value | Status / source |
|---|---:|---|
| Heartbeat/status period | 5000 ms | Proposed from starter; validate |
| Telemetry period | 1000 ms | Proposed; validate under combined load |
| MQTT reconnect delay | 3000 ms | Proposed from starter; validate |
| WiFi rejoin delay | 5000 ms | Proposed from starter; validate |
| Offline periodic telemetry | Drop it | Proposed; publish newest snapshot after reconnect |
| Offline discrete outcomes | Bounded queue with expiry | Proposed; expiry still needs team agreement |
| Reconnection effect on motion | None | Required: never START or clear STOP |
| Malformed/oversized command | Reject and count | Proposed and implemented in service bridge |
| Controller queue full | Reject; never wait or retry automatically | Proposed |
| Counter overflow | Saturate at maximum | Proposed and implemented for bridge counters |

### Telemetry fields by owner

The outer packet is defined in section 6. The fields below are the proposed
minimum summaries. `TBD` entries must come from the owner before widths are
frozen.

#### Buddy 1 communication section

| Field | Proposed representation | Status |
|---|---|---|
| WiFi ready | One status flag bit | Proposed |
| IP acquired | One status flag bit | Proposed |
| MQTT connected | One status flag bit | Proposed |
| Command subscription ready | One status flag bit | Proposed |
| Reconnection in progress | One status flag bit | Proposed |
| Valid command count | Saturating `uint32_t` | Proposed; may reduce only after range testing |
| Invalid command count | Saturating `uint32_t` | Proposed |
| Controller delivery failures | Saturating `uint32_t` | Proposed |
| Ignored topic count | Saturating `uint32_t` | Proposed |
| Last command receipt | `uint32_t` milliseconds since boot | Proposed |
| Last successful publish | `uint32_t` milliseconds since boot | Proposed |

#### Buddy 2 controller/motion section

| Field | Proposed representation | Status |
|---|---|---|
| Controller state | `uint8_t` enum | Owner input: Buddy 2 supplies values |
| STOP latched | One flag bit | Owner input: Buddy 2 confirms source |
| Wheels stationary | One flag bit | Owner input: Buddy 2 defines confirmed meaning |
| Moving | One flag bit | Owner input |
| Controller fault | One flag plus fault code | Owner input: enumerate codes |
| Active operation | `uint8_t` enum | Owner input |
| Active request ID | `uint32_t` | Owner input: define when valid |
| Completed request ID | Discrete result, `uint32_t` | Owner input |
| Speed/distance summary | `TBD` | Owner input: unit, range, resolution and necessity |

Proposed external controller-state values are:

| Value | Meaning |
|---:|---|
| 0 | Unknown or invalid |
| 1 | Initialising |
| 2 | Ready |
| 3 | Running |
| 4 | Stopped |
| 5 | Complete |
| 6 | Recovery |
| 7 | Fault |

Proposed external controller-status flag bits are:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x01` | STOP latched |
| 1 | `0x02` | Wheels confirmed stationary |
| 2 | `0x04` | Moving |
| 3 | `0x08` | Controller fault active |

These are telemetry wire values, not Buddy 2's internal enum or event-bit
allocation. Buddy 2 maps its implementation to these values after confirming
that the meanings cover its controller statechart.

#### Buddy 3 line/barcode section

| Field | Proposed representation | Status |
|---|---|---|
| Nominal line width | 18 mm | Supplied Barcode & Line Specification |
| Nominal barcode-to-line gap | 20 mm | Supplied Barcode & Line Specification |
| Nominal barcode length | Approximately 141 mm | Supplied specification; no tolerance given |
| Line valid | One flag bit | Owner input: Buddy 3 defines validity |
| Line state | `uint8_t` enum | Owner input: enumerate values |
| Line position/correction | `TBD` | Owner input: include only if remotely useful |
| Barcode valid | One flag bit | Owner input |
| Barcode value | `TBD` | Owner/instructor input: encoding and range determine width |
| Navigation instruction | `uint8_t` enum | Proposed representation; command mapping remains TBD |
| Line reacquisition state | `uint8_t` enum or flag | Owner input |
| Barcode event time | `uint32_t` milliseconds since boot | Proposed; owner confirms max age |

The supplied Barcode & Line Specification supports only these physical
nominal values:

```c
#define LINE_WIDTH_NOMINAL_MM       18u
#define BARCODE_LINE_GAP_MM         20u
#define BARCODE_LENGTH_NOMINAL_MM  141u
```

These constants are proposed for Buddy 3's module after Buddy 3 approves
them. `BARCODE_LENGTH_NOMINAL_MM` is not an exact acceptance limit because
the source calls the length approximate and supplies no tolerance.

Proposed external line-state values for Buddy 3 to map to are:

| Value | Meaning |
|---:|---|
| 0 | Unknown or no valid observation |
| 1 | Valid line |
| 2 | Line lost |
| 3 | Reacquiring line |
| 4 | Ambiguous line/branch |
| 5 | Sensor fault |

Tentative external navigation values are:

| Value | Meaning |
|---:|---|
| 0 | No navigation command |
| 1 | Left |
| 2 | Right |
| 3 | Straight |
| 4 | U-turn |
| 5 | Stop |

Do not approve/freeze these navigation values yet. The supplied PDF does not define the
barcode symbology, narrow/wide bar dimensions, start/stop pattern, reading
direction, checksum, valid value range or mapping to LEFT/RIGHT/STRAIGHT/
U-turn/STOP. Buddy 3 or the instructor must provide those rules. It also
does not provide dimensional tolerances or a sampling-rate requirement;
Buddy 3 must derive and validate timing using measured vehicle speed.

#### Buddy 4 terrain/hump section

| Field | Proposed representation | Status |
|---|---|---|
| Terrain state | `uint8_t` enum | Owner input: Buddy 4 enumerates values |
| Hump phase/result | `uint8_t` enum | Owner input |
| Highest peak | `TBD` | Owner input: meaning, unit, range and resolution |
| Peak valid | One flag bit | Owner input |
| Peak capture time | `uint32_t` milliseconds since boot | Proposed |
| Calibration/fault state | Flag plus `uint8_t` code if needed | Owner input |
| Highest-peak reset rule | `TBD` | Owner input: new run, RESET or power-up |

Proposed external terrain-state values are:

| Value | Meaning |
|---:|---|
| 0 | Unknown or invalid |
| 1 | Level |
| 2 | Ascending |
| 3 | Peak |
| 4 | Descending |
| 5 | Hump complete |
| 6 | Terrain/IMU fault |

Represent peak height provisionally as `uint16_t` millimetres with a separate
validity flag and `uint32_t` capture time. Its 0–65,535 mm representable range
is not a claim about measurable range or accuracy; Buddy 4 must replace it
with validated application limits and a reset rule.

#### Buddy 5 obstacle/bypass section

| Field | Proposed representation | Status |
|---|---|---|
| Obstacle/scan state | `uint8_t` enum | Owner input: Buddy 5 enumerates values |
| Forward distance | `uint16_t` millimetres | Agreed nominal range 20–4000 mm; usable range still requires physical validation |
| Distance valid/fresh | Validity flag plus capture time | Agreed approach; stale after 150 ms per S5 decision |
| Active request ID | `uint32_t` | Agreed S5 message requirement |
| Bypass available | One flag bit | Owner input: confirm source |
| Selected side | `uint8_t` enum | Owner input: enumerate none/left/right |
| Minimum clearance | `TBD` millimetres | Owner input: valid range determines width |
| Bypass outcome | `uint8_t` enum | Owner input: enumerate values |
| Failure reason | `uint8_t` enum | Owner input: enumerate values |
| Detailed scan/profile | Not transmitted | Agreed: remains local |

Proposed external bypass-side values are:

| Value | Meaning |
|---:|---|
| 0 | No side selected |
| 1 | Left |
| 2 | Right |

Proposed external bypass-outcome values are:

| Value | Meaning |
|---:|---|
| 0 | No result |
| 1 | Plan ready |
| 2 | No safe route |
| 3 | Cancelled |
| 4 | Expired |
| 5 | Completed |
| 6 | Failed |

Proposed external obstacle-failure values are:

| Value | Meaning |
|---:|---|
| 0 | No failure |
| 1 | Invalid distance |
| 2 | Echo timeout |
| 3 | Echo stuck |
| 4 | Stop-confirmation timeout |
| 5 | Scan timeout |
| 6 | No safe route |
| 7 | Cancelled |
| 8 | Result-delivery failure |
| 9 | Original-line return failed |

Represent minimum clearance provisionally as `uint16_t` millimetres with a
separate validity flag. Buddy 5 must confirm its valid application range and
the final threshold derived from 50 mm physical clearance plus measured
sensing and motion margins.

### Values still required before coding the final codec

1. Instructor/broker operator: actual WiFi and broker configuration.
2. Buddy 2: STOP bit, controller states, command acceptance rules and result
   record.
3. Buddy 3/instructor: barcode encoding, command mapping, tolerances, line
   validity/freshness and any telemetry fields beyond the documented nominal
   geometry.
4. Buddy 4: hump/peak meaning, unit, range, resolution and reset rule.
5. Buddy 5: obstacle/bypass enums and minimum-clearance range.
6. Team: approve or replace all rows marked Proposed.
7. Instructor: approve modification of the shared MQTT service task.

