/**
 * @file subsystem2.h
 * @brief Subsystem 2 (Vehicle Controller + Motion Control) public
 *        interface. Owner: Dalton, 2504003.
 *
 * PROVISIONAL. Nothing here is agreed with the other owners yet; every
 * name, bit and field is a proposal for TEAM.md section 5. Other
 * subsystems include it as "../subsystem2/subsystem2.h".
 *
 * Subsystem 2 creates every object below in subsystem2_init(). Until then
 * every ID is 0 and every function here fails safely (returns false or
 * E_OBJ). This matters because the WiFi service task starts before
 * usermain().
 *
 * Units: mm, millidegrees (left positive, as subsystem 5), ms from
 * tk_get_tim() (low 32 bits, compare by unsigned subtraction).
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "motion_api.h"     /* motion API and motion_get_status() */
#include "s2_s1_adapter.h"  /* S1-facing, kernel-free */

#define S2_PRI_CONTROLLER   14      /* highest application priority */
#define S2_PRI_MOTION       15

/* ------------------------------------------------------------------------
 *  controller_flags (event flag, owned by S2): urgent events.
 *  One bit per event. Setters use tk_set_flg(controller_flags, bit) and
 *  never wait. Only the Vehicle Controller clears, and only the bit it
 *  handled: tk_clr_flg(controller_flags, ~bit). Names for S1 follow
 *  hy_wifi_telemetry PROPOSAL.md section 4.
 * ------------------------------------------------------------------------ */
#define CTRL_FLG_HAZARD         (1u << 0)   /* S5: unsafe clearance */
#define CTRL_FLG_SCAN_FAULT     (1u << 1)   /* S5: scanner fault */
#define CTRL_FLG_MOTION_FAULT   (1u << 2)   /* S2 Motion Control fault */
#define CTRL_FLG_REMOTE_STOP    (1u << 3)   /* S1: via the STOP adapter */
#define CTRL_FLG_LOCAL_STOP     (1u << 4)   /* on-board STOP (test, demo) */
#define CTRL_FLG_ALL            (0x1Fu)

/* ------------------------------------------------------------------------
 *  safety_flags (event flag, owned by S2): latched STOP. Set at boot,
 *  set and cleared only by the Vehicle Controller. Anyone may read it
 *  with tk_ref_flg(); nothing may move while it is set.
 * ------------------------------------------------------------------------ */
#define SAFETY_FLG_STOP         (1u << 0)

IMPORT ID controller_flags;
IMPORT ID safety_flags;

/* ------------------------------------------------------------------------
 *  remote_q (message buffer, owned by S2): START and RESET from S1.
 *  Capacity 2 records (S1 PROPOSAL section 4): 2 x (4 + 12) = 32 bytes,
 *  maximum message 12 bytes. Send only through
 *  controller_send_remote_command(). STOP never uses this queue.
 *  Command codes S2_REMOTE_* are in s2_s1_adapter.h.
 * ------------------------------------------------------------------------ */
#define S2_REMOTE_MAX_AGE_MS    2000u   /* S1 PROPOSAL section 5 */
#define S2_REMOTE_Q_BYTES       32

/* Same layout as remote_command_t in S1 PROPOSAL section 4. */
typedef struct
{
    uint32_t request_id;    /* nonzero; S2 rejects a repeat of the last */
    uint32_t received_ms;   /* S1 receipt time; older than max age = stale */
    uint8_t  command;       /* S2_REMOTE_START or S2_REMOTE_RESET */
    uint8_t  reserved[3];   /* zero */
} s2_remote_cmd_t;

IMPORT ID remote_q;

/* S1 adapters (controller_raise_remote_stop, controller_send_remote_command,
   robot_read_telemetry) are declared in s2_s1_adapter.h, which has no
   kernel include so S1's code can use it next to its own headers: the
   kernel's size_t (long) and <stddef.h>'s (unsigned int) conflict. */

/* Build one remote_q record (pure; used by the adapter and tests).
   false, leaving *p_msg unchanged, unless command is START or RESET and
   request_id is nonzero. */
bool s2_remote_make (uint8_t command, uint32_t request_id,
                     uint32_t received_ms, s2_remote_cmd_t * p_msg);

/* ------------------------------------------------------------------------
 *  robot_state (owned by S2, written only by the Vehicle Controller under
 *  state_mutex, priority inheritance). Read a copy with
 *  s2_get_robot_state(); the mutex is held only for the copy.
 * ------------------------------------------------------------------------ */
#define S2_SYS_INITIALISING     1u
#define S2_SYS_READY            2u
#define S2_SYS_RUNNING          3u
#define S2_SYS_STOPPED          4u
#define S2_SYS_FAULT            5u

#define S2_FAULT_NONE           0u
#define S2_FAULT_INIT           1u      /* start-up checks failed */
#define S2_FAULT_MOTION         2u      /* detail: motion_status_t.fault */
#define S2_FAULT_SCAN           3u      /* CTRL_FLG_SCAN_FAULT */
#define S2_FAULT_DELIVERY       4u      /* a required send failed */
#define S2_FAULT_LINE           5u      /* line sensor fault or stale IF-06,
                                           or reacquisition failed */

/* Remote command results, using the S1 PROPOSAL section 3 ack codes. */
#define S2_RESULT_NONE          0u
#define S2_RESULT_DUPLICATE     3u
#define S2_RESULT_EXPIRED       4u
#define S2_RESULT_REJECTED      6u      /* not allowed in this state */
#define S2_RESULT_COMPLETED     7u      /* took effect */
#define S2_RESULT_FAILED        8u      /* accepted, then failed */

/* s2_robot_state_t.mode: the Running substate (design review statechart).
   S2_MODE_NONE outside Running. Subsystem 5 raises CTRL_FLG_HAZARD only
   in the moving modes FOLLOW, BYPASS and REACQUIRE. */
#define S2_MODE_NONE            0u
#define S2_MODE_FOLLOW          1u      /* line following */
#define S2_MODE_NAVIGATE        2u      /* barcode manoeuvre at a junction */
#define S2_MODE_SCAN            3u      /* stopped; S5 scans and plans */
#define S2_MODE_BYPASS          4u      /* executing S5's plan segments */
#define S2_MODE_REACQUIRE       5u      /* bounded search for the line */

/* s2_robot_state_t.flags */
#define S2_RS_STOP_LATCHED      0x01u
#define S2_RS_STOPPED           0x02u   /* wheels still >= 100 ms */
#define S2_RS_DISTANCE_VALID    0x04u
#define S2_RS_NAV_AUTHORISED    0x08u   /* S3 may deliver a junction turn */

typedef struct
{
    uint32_t captured_ms;
    int32_t  run_distance_mm;   /* signed travel since START */
    uint32_t last_request_id;   /* last remote command handled */
    uint8_t  state;             /* S2_SYS_* */
    uint8_t  fault;             /* S2_FAULT_* */
    uint8_t  flags;             /* S2_RS_* */
    uint8_t  last_command;      /* S2_REMOTE_* */
    uint8_t  last_result;       /* S2_RESULT_* */
    uint8_t  motion_state;      /* motion_state_t */
    uint8_t  mode;              /* S2_MODE_*: Running substate */
    uint8_t  reserved;
} s2_robot_state_t;

ER s2_get_robot_state (s2_robot_state_t * p_state);

/* Telemetry fields from a robot_state copy (pure; s2_s1_adapter.h). */
void s2_telemetry_from_state (s2_robot_state_t const * p_state,
                              s2_telemetry_t * p_out);

/* ------------------------------------------------------------------------
 *  Distance for S4 (s4_terrain_process wants a uint16_t delta every
 *  10 ms). Every 10 ms: call motion_get_status(), then
 *      b_ok = s2_distance_delta(&prev, &now, &delta_mm);
 *  and pass delta_mm, b_ok. When now.run_id != prev.run_id, call
 *  s4_terrain_start_new_run() (the controller starts a new run at START).
 * ------------------------------------------------------------------------ */
bool s2_distance_delta (motion_status_t const * p_prev,
                        motion_status_t const * p_now,
                        uint16_t * p_delta_mm);

/* ------------------------------------------------------------------------
 *  scan_q (message buffer, owned by S2), Controller to Obstacle Scan.
 *  Created by subsystem2_init(); subsystem 5's scan task receives. Kinds
 *  mirror scan_engine_event_kind_t (feat/subsystem5-obstacle-scanning);
 *  TICK and ECHO_EDGE are internal to S5. request_id matches
 *  scan_engine_event_t (uint32_t) so no conversion is needed. The
 *  controller sends STOP_CONFIRM(asserted) for the active request once
 *  motion_stop_confirmed() is true (both wheels still for 100 ms).
 * ------------------------------------------------------------------------ */
#define S2_SCAN_REQUEST         1u
#define S2_SCAN_AUTHORISE       2u  /* b_asserted 0 withdraws */
#define S2_SCAN_STOP_CONFIRM    3u  /* b_asserted 0 withdraws */
#define S2_SCAN_CANCEL          4u
#define S2_SCAN_STOP            5u  /* a new STOP event */
#define S2_SCAN_MOVEMENT        6u  /* the vehicle moved */
#define S2_SCAN_RESET           7u

typedef struct
{
    uint32_t request_id;
    uint32_t time_ms;       /* send time, for S5's 2 s request expiry */
    uint8_t  kind;          /* S2_SCAN_* */
    uint8_t  b_asserted;
    uint8_t  reserved[2];
} s2_scan_cmd_t;

#define S2_SCAN_Q_BYTES         96      /* 6 x (4 + 12): one scan's traffic */

IMPORT ID scan_q;

/* ------------------------------------------------------------------------
 *  Navigation commands from S3. S3 calls s3_nav_take() each sample with
 *  stopped = SAFETY_FLG_STOP set, authorised = S2_RS_NAV_AUTHORISED in
 *  robot_state, junction = its own detection, and publishes an offered
 *  command in IF-06 (subsystem3.h: nav_cmd with a unique nav_id). The
 *  controller runs each nav_id at most once.
 * ------------------------------------------------------------------------ */
#define S2_NAV_LEFT             1u  /* = S3_NAV_LEFT: turn left 90 deg */
#define S2_NAV_RIGHT            2u  /* = S3_NAV_RIGHT: turn right 90 deg */
#define S2_NAV_STRAIGHT         3u  /* = S3_NAV_STRAIGHT: continue */
#define S2_NAV_UTURN            4u  /* = S3_NAV_UTURN: turn 180 deg */

#endif /* SUBSYSTEM2_H */
