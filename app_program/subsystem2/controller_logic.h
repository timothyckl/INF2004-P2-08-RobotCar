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
 * Later (not built, see the markers in controller_logic.c): line
 * following, barcode manoeuvre, scan and plan, bypass, line reacquisition
 * and Complete.
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
    CTRL_ACT_RIGHT          /* value = mdeg */
} ctrl_action_t;

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
} ctrl_cfg_t;

typedef struct
{
    uint32_t        now_ms;
    bool            b_status_ok;        /* motion_get_status() succeeded */
    motion_status_t status;
} ctrl_input_t;

/** Applied by the wrapper in this order. */
typedef struct
{
    bool          b_latch_stop;     /* set SAFETY_FLG_STOP, wake motion */
    bool          b_drain_motion;   /* empty motion_q */
    bool          b_clear_stop;     /* clear SAFETY_FLG_STOP */
    ctrl_action_t action;           /* then call the motion API ... */
    uint32_t      value;
    uint32_t      clear_flags;      /* ... and clear these handled bits */
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

#endif /* CONTROLLER_LOGIC_H */

/* End of file. */
