/**
 * @file controller_logic.h
 * @brief Subsystem 2 minimal Vehicle Controller (pure logic).
 *
 * Week 10 scope: Initialising, Ready, Running, Stopped, Fault from the
 * system statechart (design review section 5). No kernel calls: the task
 * wrapper (task_controller.c) feeds flags, remote commands, Motion
 * Control status and time, then applies the output.
 *
 * Rules:
 * - STOP is latched at boot and stays latched in every state except
 *   Running. Only START in Ready clears it, after motion_q is drained.
 * - START is accepted only in Ready with Motion Control healthy (Idle,
 *   valid, no fault) and the wheels confirmed stopped.
 * - REMOTE_STOP, LOCAL_STOP and HAZARD latch STOP and wake Motion
 *   Control; from Running they go to Stopped. MOTION_FAULT and
 *   SCAN_FAULT latch STOP and go to Fault.
 * - RESET from Stopped (to Ready) or Fault (to Initialising) only with
 *   the wheels confirmed stopped; motion_q is drained.
 * - Nothing else clears STOP: WiFi reconnection produces no input here.
 *
 * Running substates (mission mode, design review statechart), used when
 * no demo script is configured:
 * - FOLLOW: renew a leased motion_drive() every cycle from S3's steering
 *   (IF-06). A stale snapshot holds the wheels still; stale for
 *   line_fail_ms, or an S3 sensor fault, is Fault(LINE). LOST starts
 *   REACQUIRE. A new barcode command (nav_id) starts NAVIGATE. HAZARD
 *   starts SCAN instead of latching STOP.
 * - NAVIGATE: one turn (left/right 90, U-turn 180) or a short forward move
 *   for STRAIGHT; DONE returns to FOLLOW.
 * - SCAN: motion_stop(), REQUEST to S5, then STOP_CONFIRM and AUTHORISE
 *   once the wheels are confirmed still. PLAN starts BYPASS, CLEAR
 *   returns to FOLLOW, anything else (or the time limit) is Fault(SCAN).
 * - BYPASS: S5's segments one by one, then its approach turn.
 * - REACQUIRE: rotate towards the side the line was last seen (after
 *   loss) or drive forward up to the plan's search distance (after a
 *   bypass) until a front sensor sees black for found_ms; then (after a
 *   bypass) the align turn and a heading check. Bounded by
 *   search_limit_ms; failure is Fault(LINE).
 * HAZARD outside FOLLOW and SCAN latches STOP as before. START also needs
 * a fresh, valid line in mission mode. Any STOP during SCAN or BYPASS
 * tells S5 (S2_SCAN_STOP).
 */
#ifndef CONTROLLER_LOGIC_H
#define CONTROLLER_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_api.h"
#include "subsystem2.h"

/** What the wrapper must ask Motion Control to do (motion API call). */
typedef enum
{
    CTRL_ACT_NONE = 0,
    CTRL_ACT_NEW_RUN,
    CTRL_ACT_RESET,
    CTRL_ACT_FORWARD,       /* value = mm */
    CTRL_ACT_BACKWARD,      /* value = mm */
    CTRL_ACT_LEFT,          /* value = mdeg */
    CTRL_ACT_RIGHT,         /* value = mdeg */
    CTRL_ACT_DRIVE,         /* drive_l, drive_r: motion_drive() */
    CTRL_ACT_STOP           /* motion_stop(): brake, STOP not latched */
} ctrl_action_t;

/* Copies of IF-06 values (subsystem3.h); task_controller.c checks at
   compile time that they match. */
#define CTRL_LINE_VALID         0x01u
#define CTRL_LINE_LOST          0x02u
#define CTRL_LINE_FAULT         0x08u
#define CTRL_LINE_SEEN          0x10u
#define CTRL_SIDE_LEFT          1u
#define CTRL_SIDE_RIGHT         2u

/* Copies of scan_task.h / bypass_plan.h values (checked the same way). */
#define CTRL_SCAN_PLAN          1u
#define CTRL_SCAN_CLEAR         2u
#define CTRL_SEG_TURN           1u
#define CTRL_SEG_MOVE           2u
#define CTRL_PLAN_SEGMENTS      4u
#define CTRL_SCAN_MSGS_MAX      3u

/** Line Sensor snapshot as the controller needs it (from IF-06). */
typedef struct
{
    bool     b_ok;              /* s3_line_get() succeeded */
    uint32_t captured_ms;
    int16_t  steer_permille;    /* + = left */
    uint8_t  flags;             /* CTRL_LINE_* */
    uint8_t  last_side;         /* CTRL_SIDE_* or 0 */
    uint8_t  nav_cmd;           /* S2_NAV_* or 0 */
    uint32_t nav_id;
} ctrl_line_t;

typedef struct
{
    uint8_t kind;               /* CTRL_SEG_* */
    int32_t value;              /* mdeg (+ left) or mm (+ forward) */
} ctrl_segment_t;

/** Obstacle Scan result as the controller needs it (from scan_task_get). */
typedef struct
{
    bool           b_ok;
    uint32_t       request_id;
    uint8_t        result;      /* CTRL_SCAN_* or another nonzero code */
    uint8_t        count;
    ctrl_segment_t seg[CTRL_PLAN_SEGMENTS];
    int32_t        approach_mdeg;
    uint16_t       search_mm;
    int32_t        align_mdeg;
} ctrl_scan_t;

/** Mission tuning (controller_config.h). */
typedef struct
{
    int32_t  follow_mm_s;
    int32_t  search_mm_s;
    int32_t  speed_max_mm_s;
    uint32_t junction_mm;
    uint32_t line_max_age_ms;
    uint32_t line_fail_ms;
    uint32_t found_ms;
    uint32_t search_limit_ms;
    uint32_t scan_limit_ms;
    int32_t  heading_tol_mdeg;
} ctrl_mission_cfg_t;

/** One scan_q record for the wrapper to send (s2_scan_cmd_t). */
typedef struct
{
    uint8_t  kind;              /* S2_SCAN_* */
    bool     b_asserted;
    uint32_t request_id;
} ctrl_scan_msg_t;

typedef struct
{
    ctrl_action_t action;
    uint32_t      value;
} ctrl_step_t;

typedef struct
{
    uint32_t           init_limit_ms;   /* Initialising to Fault(INIT) */
    uint32_t           result_limit_ms; /* wait for a motion result */
    uint32_t           remote_max_age_ms;
    ctrl_step_t const * p_script;       /* Running demo steps, or NULL */
    uint8_t            script_len;
    ctrl_mission_cfg_t mission;         /* used when p_script is NULL */
} ctrl_cfg_t;

typedef struct
{
    uint32_t        now_ms;
    bool            b_status_ok;        /* motion_get_status() succeeded */
    motion_status_t status;
    ctrl_line_t     line;
    ctrl_scan_t     scan;
} ctrl_input_t;

/** Applied by the wrapper in this order. */
typedef struct
{
    bool          b_latch_stop;     /* set SAFETY_FLG_STOP, wake motion */
    bool          b_drain_motion;   /* empty motion_q */
    bool          b_clear_stop;     /* clear SAFETY_FLG_STOP */
    ctrl_action_t action;           /* then call the motion API ... */
    uint32_t      value;
    int16_t       drive_l;          /* CTRL_ACT_DRIVE wheel speeds */
    int16_t       drive_r;
    uint32_t      clear_flags;      /* ... and clear these handled bits */
    ctrl_scan_msg_t scan[CTRL_SCAN_MSGS_MAX];  /* then send to scan_q */
    uint8_t       n_scan;
} ctrl_output_t;

/** Controller state; private, use only through ctrl_*(). */
typedef struct
{
    ctrl_cfg_t cfg;
    uint8_t    state;           /* S2_SYS_* */
    uint8_t    fault;           /* S2_FAULT_* */
    bool       b_stop;          /* mirror of the latch */
    uint32_t   since_ms;        /* entry time of the current state */
    bool       b_have_request;
    uint32_t   last_request_id;
    uint8_t    last_command;
    uint8_t    last_result;
    bool       b_run_valid;
    int32_t    run_start_mm;
    uint8_t    step;            /* next script step */
    bool       b_waiting;       /* a motion command is outstanding */
    uint16_t   wait_id;
    uint32_t   wait_since_ms;

    /* mission (Running substates) */
    uint8_t    mode;            /* S2_MODE_* */
    uint32_t   mode_since_ms;
    uint32_t   last_nav_id;
    bool       b_have_nav;
    uint32_t   line_good_ms;    /* last time IF-06 was fresh */
    uint32_t   scan_id;         /* active request */
    bool       b_scan_granted;  /* STOP_CONFIRM + AUTHORISE sent */
    ctrl_scan_t plan;           /* copy of the accepted plan */
    uint8_t    seg;             /* next plan step (count = approach) */
    bool       b_after_bypass;  /* REACQUIRE kind */
    uint8_t    search_side;     /* CTRL_SIDE_* to rotate towards */
    int32_t    search_start_mm;
    uint32_t   seen_since_ms;
    bool       b_seen;
    bool       b_aligning;      /* align turn outstanding */
    int32_t    heading_ref_mdeg; /* heading when the scan started */
    bool       b_no_wait;       /* this cycle's action has no result */
} ctrl_t;

void ctrl_init (ctrl_t * p_c, ctrl_cfg_t const * p_cfg, uint32_t now_ms);
void ctrl_output_clear (ctrl_output_t * p_out);
void ctrl_flags (ctrl_t * p_c, uint32_t flags, ctrl_input_t const * p_in,
                 ctrl_output_t * p_out);
void ctrl_remote (ctrl_t * p_c, s2_remote_cmd_t const * p_cmd,
                  ctrl_input_t const * p_in, ctrl_output_t * p_out);
void ctrl_tick (ctrl_t * p_c, ctrl_input_t const * p_in,
                ctrl_output_t * p_out);
void ctrl_motion_sent (ctrl_t * p_c, bool b_ok, uint16_t cmd_id,
                       uint32_t now_ms, ctrl_output_t * p_out);
void ctrl_get_state (ctrl_t const * p_c, ctrl_input_t const * p_in,
                     s2_robot_state_t * p_state);

/* Mission tuning from controller_config.h. */
void ctrl_mission_defaults (ctrl_mission_cfg_t * p_mission);

#endif /* CONTROLLER_LOGIC_H */

/* End of file. */
