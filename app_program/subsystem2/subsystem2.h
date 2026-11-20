/**
 * @file subsystem2.h
 * @brief Subsystem 2 (Vehicle Controller + Motion Control) public
 *        interface. Owner: Dalton, 2504003.
 *
 * Most interfaces remain provisional. The S3 line-report interface is
 * agreed as TEAM.md IF-06; other names, bits and fields remain proposals.
 * Other subsystems include it as "../subsystem2/subsystem2.h".
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
#include "../subsystem5/include/scan_telemetry.h"   /* kernel-free */

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
#define CTRL_FLG_LOCAL_STOP     (1u << 4)   /* on-board STOP: GP20, or a
                                               controller-task failure */
#define CTRL_FLG_PICKUP         (1u << 5)   /* S4: car lifted (IF-19) */
#define CTRL_FLG_LINE_STOP      (1u << 6)   /* S2/S3: line sensing failed */
#define CTRL_FLG_ALL            (0x7Fu)

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
 *  line_q (message buffer, owned by S2): the two front sensors report the
 *  latest steering observation. The Vehicle Controller alone translates
 *  it into Motion Control commands. Capacity is four 8-byte records.
 * ------------------------------------------------------------------------ */
#define S2_LINE_CENTER          0u
#define S2_LINE_LEFT            1u
#define S2_LINE_RIGHT           2u
#define S2_LINE_BOTH_BLACK      3u
#define S2_LINE_STOP            4u
#define S2_LINE_STRENGTH_MAX    UINT8_C(100)
#define S2_LINE_MAX_AGE_MS      100u
#define S2_LINE_Q_BYTES         48

/* Report flags. BARCODE: the side sensor has a barcode capture in
   progress, so the controller softens strong curves (S2 decision 25). */
#define S2_LINE_FLAG_BARCODE    0x01u
#define S2_LINE_FLAGS_VALID     S2_LINE_FLAG_BARCODE

typedef struct
{
    uint32_t captured_ms;
    uint8_t  position;       /* one of S2_LINE_* */
    uint8_t  strength;       /* 0..100; meaningful for LEFT/RIGHT only */
    uint8_t  flags;          /* S2_LINE_FLAG_* */
    uint8_t  reserved;       /* zero */
} s2_line_report_t;

IMPORT ID line_q;

bool s2_line_make (uint8_t position, uint32_t captured_ms,
                   s2_line_report_t * p_report);
bool s2_line_submit (uint8_t position, uint32_t captured_ms);
bool s2_line_make_scaled (uint8_t position, uint8_t strength,
                          uint8_t flags, uint32_t captured_ms,
                          s2_line_report_t * p_report);
bool s2_line_submit_scaled (uint8_t position, uint8_t strength,
                            uint8_t flags, uint32_t captured_ms);
bool s2_line_start (void);   /* queued request; controller authorises it */
bool s2_line_stop (void);    /* urgent STOP flag, CTRL_FLG_LINE_STOP */

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
#define S2_RS_MOTION_VALID      0x10u   /* speeds and heading are current */
#define S2_RS_LINE_FRESH        0x20u   /* line: report <= S2_LINE_MAX_AGE_MS */
#define S2_RS_BARCODE_VALID     0x40u   /* barcode fields: one was decoded */

/* s2_robot_state_t.stop_cause: what latched STOP, while S2_RS_STOP_LATCHED
   (S2 decision 27). The first cause after STOP was clear is kept; a fault
   replaces it. START clears it. */
#define S2_STOP_NONE            0u      /* STOP not latched */
#define S2_STOP_BOOT            1u      /* since power-up or a fault RESET */
#define S2_STOP_REMOTE          2u      /* MQTT STOP (IF-08) */
#define S2_STOP_BUTTON          3u      /* GP20 button */
#define S2_STOP_PICKUP          4u      /* car lifted (IF-19) */
#define S2_STOP_HAZARD          5u      /* S5 hazard not handled by a scan */
#define S2_STOP_LINE            6u      /* line sensing or report failed */
#define S2_STOP_SCAN            7u      /* scan failed, timed out, no route */
#define S2_STOP_BYPASS          8u      /* bypass failed */
#define S2_STOP_NAV             9u      /* junction manoeuvre failed */
#define S2_STOP_FAULT           10u     /* see s2_robot_state_t.fault */
#define S2_STOP_DONE            11u     /* calibration script finished */

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
    uint8_t  line;              /* S2_LINE_*, while S2_RS_LINE_FRESH */
    int16_t  left_mm_s;         /* wheel speeds, while S2_RS_MOTION_VALID */
    int16_t  right_mm_s;
    int16_t  heading_cdeg;      /* centidegrees, left positive,
                                   -18000..18000; S2_RS_MOTION_VALID */
    char     barcode;           /* last letter decoded since power-up,
                                   'A'..'Z'; S2_RS_BARCODE_VALID */
    uint8_t  barcode_command;   /* its S2_NAV_*, 0 if none is mapped */
    uint8_t  barcode_count;     /* letters decoded, wraps at 256 */
    uint8_t  stop_cause;        /* S2_STOP_* */
} s2_robot_state_t;

ER s2_get_robot_state (s2_robot_state_t * p_state);

/* Telemetry snapshot fields from a robot_state copy (pure; replaces every
   S2 field, leaves link_up and the S4/S5 fields alone). */
void s2_telemetry_from_state (s2_robot_state_t const * p_state,
                              telemetry_encoder_snapshot_t * p_out);

/* Hump fields from subsystem 4's run maximum (pure; IF-18). Saturates at
   255 mm; an invalid maximum gives 0 and hump_valid false. */
void s2_telemetry_add_hump (telemetry_encoder_snapshot_t * p_out,
                            uint16_t max_mm, bool b_valid);

/* Obstacle fields from subsystem 5's decoded status (pure). A NULL status
   (S5 unavailable or busy) clears the obstacle fields and their flags. */
void s2_telemetry_add_scan (telemetry_encoder_snapshot_t * p_out,
                            scan_telemetry_t const * p_scan);

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
#define S2_SCAN_LOOK            8u  /* one side look, S2 decision 19 */

typedef struct
{
    uint32_t request_id;
    uint32_t time_ms;       /* send time, for S5's 2 s request expiry */
    uint8_t  kind;          /* S2_SCAN_* */
    uint8_t  b_asserted;
    int8_t   angle_deg;     /* LOOK only: positive = left */
    uint8_t  reserved;
} s2_scan_cmd_t;

#define S2_SCAN_Q_BYTES         96      /* 6 x (4 + 12): one scan's traffic */

IMPORT ID scan_q;

/* ------------------------------------------------------------------------
 *  nav_q (message buffer, owned by S2; IF-21): S3 sends one record per
 *  decoded barcode, with its letter and mapped command (S2_NAV_NONE if
 *  the letter has none). The Vehicle Controller reports every letter in
 *  telemetry (S2 decision 27). It holds a command and acts at the next
 *  junction it confirms (S2 decision 26, nav_control.h); a command that
 *  arrives while not running is dropped.
 * ------------------------------------------------------------------------ */
#define S2_NAV_NONE             0u  /* = S3_NAV_NONE: letter not mapped */
#define S2_NAV_LEFT             1u  /* = S3_NAV_LEFT: turn left 90 deg */
#define S2_NAV_RIGHT            2u  /* = S3_NAV_RIGHT: turn right 90 deg */
#define S2_NAV_STRAIGHT         3u  /* = S3_NAV_STRAIGHT: continue */
#define S2_NAV_UTURN            4u  /* = S3_NAV_UTURN: turn 180 deg */

typedef struct
{
    uint32_t scan_id;       /* S3's unique barcode scan ID */
    uint32_t staged_ms;     /* when S3 staged it, for expiry */
    uint8_t  command;       /* S2_NAV_* */
    char     symbol;        /* decoded letter, 'A'..'Z' */
    uint8_t  reserved[2];
} s2_nav_request_t;

#define S2_NAV_Q_BYTES          32      /* 2 x (4 + 12) */

IMPORT ID nav_q;

/** Queue a decoded letter and its command (S2_NAV_NONE..S2_NAV_UTURN)
 *  without blocking; false if either is invalid or the queue is full. */
bool s2_nav_submit (uint8_t command, char symbol, uint32_t scan_id,
                    uint32_t staged_ms);

#endif /* SUBSYSTEM2_H */
