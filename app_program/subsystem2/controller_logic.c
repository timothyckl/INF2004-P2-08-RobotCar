/**
 * @file controller_logic.c
 * @brief Subsystem 2 minimal Vehicle Controller (see controller_logic.h).
 */
#include "controller_logic.h"

#include "motion_logic.h"   /* motion_state_t values */

#define CTRL_STOP_BITS  (CTRL_FLG_REMOTE_STOP | CTRL_FLG_LOCAL_STOP \
                         | CTRL_FLG_HAZARD)

static void enter (ctrl_t * p_c, uint8_t state, uint32_t now_ms);
static void latch (ctrl_t * p_c, ctrl_output_t * p_out);
static void fault (ctrl_t * p_c, uint8_t code, uint32_t now_ms,
                   ctrl_output_t * p_out);
static void stop_running (ctrl_t * p_c, uint32_t now_ms,
                          ctrl_output_t * p_out);
static bool motion_healthy (ctrl_input_t const * p_in);
static bool wheels_stopped (ctrl_input_t const * p_in);
static void run_step (ctrl_t * p_c, ctrl_input_t const * p_in,
                      ctrl_output_t * p_out);
static void do_start (ctrl_t * p_c, ctrl_input_t const * p_in,
                      ctrl_output_t * p_out);
static uint8_t do_reset (ctrl_t * p_c, ctrl_input_t const * p_in,
                         ctrl_output_t * p_out);

/** @brief Boot state: Initialising with STOP latched. */
void
ctrl_init (ctrl_t * p_c, ctrl_cfg_t const * p_cfg, uint32_t now_ms)
{
    ctrl_t const zero = {0};

    if ((NULL == p_c) || (NULL == p_cfg))
    {
        return;
    }

    *p_c        = zero;
    p_c->cfg    = *p_cfg;
    p_c->b_stop = true;
    enter(p_c, S2_SYS_INITIALISING, now_ms);
}

void
ctrl_output_clear (ctrl_output_t * p_out)
{
    ctrl_output_t const zero = {0};

    if (NULL != p_out)
    {
        *p_out = zero;
    }
}

/**
 * @brief Handle controller_flags bits. Call first each cycle, so a STOP
 *        is handled before any queued command.
 */
void
ctrl_flags (ctrl_t * p_c, uint32_t flags, ctrl_input_t const * p_in,
            ctrl_output_t * p_out)
{
    uint32_t const bits = flags & CTRL_FLG_ALL;

    if ((NULL == p_c) || (NULL == p_in) || (NULL == p_out) || (0u == bits))
    {
        return;
    }

    if (0u != (bits & CTRL_STOP_BITS))
    {
        /* HAZARD stops for now; Scan & Plan will handle it later. */
        latch(p_c, p_out);

        if (S2_SYS_RUNNING == p_c->state)
        {
            stop_running(p_c, p_in->now_ms, p_out);
        }
    }

    if (0u != (bits & CTRL_FLG_MOTION_FAULT))
    {
        fault(p_c, S2_FAULT_MOTION, p_in->now_ms, p_out);
    }
    else if (0u != (bits & CTRL_FLG_SCAN_FAULT))
    {
        fault(p_c, S2_FAULT_SCAN, p_in->now_ms, p_out);
    }
    else
    {
        /* No fault bit. */
    }

    p_out->clear_flags |= bits;     /* each handled bit, nothing else */
}

/** @brief Handle one remote_q record (after ctrl_flags in the cycle). */
void
ctrl_remote (ctrl_t * p_c, s2_remote_cmd_t const * p_cmd,
             ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    if ((NULL == p_c) || (NULL == p_cmd) || (NULL == p_in) || (NULL == p_out))
    {
        return;
    }

    uint8_t result = S2_RESULT_REJECTED;

    if (p_c->b_have_request && (p_cmd->request_id == p_c->last_request_id))
    {
        result = S2_RESULT_DUPLICATE;
    }
    else if ((p_in->now_ms - p_cmd->received_ms) > p_c->cfg.remote_max_age_ms)
    {
        result = S2_RESULT_EXPIRED;
    }
    else if (S2_REMOTE_START == p_cmd->command)
    {
        if ((S2_SYS_READY == p_c->state) && motion_healthy(p_in)
            && wheels_stopped(p_in))
        {
            do_start(p_c, p_in, p_out);
            result = S2_RESULT_COMPLETED;
        }
    }
    else if (S2_REMOTE_RESET == p_cmd->command)
    {
        result = do_reset(p_c, p_in, p_out);
    }
    else if (S2_REMOTE_STOP == p_cmd->command)
    {
        /* Not expected here (STOP uses the flag), but obey it. */
        latch(p_c, p_out);

        if (S2_SYS_RUNNING == p_c->state)
        {
            stop_running(p_c, p_in->now_ms, p_out);
        }

        result = S2_RESULT_COMPLETED;
    }
    else
    {
        /* Unknown command: REJECTED. */
    }

    p_c->last_command = p_cmd->command;
    p_c->last_result  = result;

    if (S2_RESULT_DUPLICATE != result)
    {
        p_c->last_request_id = p_cmd->request_id;
        p_c->b_have_request  = true;
    }
}

/** @brief Periodic work (at least every 20 ms). */
void
ctrl_tick (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    if ((NULL == p_c) || (NULL == p_in) || (NULL == p_out))
    {
        return;
    }

    if (S2_SYS_INITIALISING == p_c->state)
    {
        if (motion_healthy(p_in))
        {
            enter(p_c, S2_SYS_READY, p_in->now_ms);
        }
        else if ((p_in->now_ms - p_c->since_ms) > p_c->cfg.init_limit_ms)
        {
            fault(p_c, S2_FAULT_INIT, p_in->now_ms, p_out);
        }
        else
        {
            /* Still waiting for Motion Control. */
        }
    }
    else if (S2_SYS_RUNNING == p_c->state)
    {
        run_step(p_c, p_in, p_out);
    }
    else
    {
        /* Ready, Stopped, Fault: wait for commands. */
    }
}

/**
 * @brief Report the motion API result for this cycle's action.
 *        A failed send is a delivery fault.
 */
void
ctrl_motion_sent (ctrl_t * p_c, bool b_ok, uint16_t cmd_id, uint32_t now_ms,
                  ctrl_output_t * p_out)
{
    if ((NULL == p_c) || (NULL == p_out))
    {
        return;
    }

    if (!b_ok)
    {
        fault(p_c, S2_FAULT_DELIVERY, now_ms, p_out);
        return;
    }

    p_c->b_waiting     = true;
    p_c->wait_id       = cmd_id;
    p_c->wait_since_ms = now_ms;
}

/** @brief Fill robot_state for telemetry. */
void
ctrl_get_state (ctrl_t const * p_c, ctrl_input_t const * p_in,
                s2_robot_state_t * p_state)
{
    s2_robot_state_t const zero = {0};

    if ((NULL == p_c) || (NULL == p_in) || (NULL == p_state))
    {
        return;
    }

    bool const b_valid = p_in->b_status_ok
                         && (0u != (p_in->status.flags & MOTION_ST_VALID));
    uint8_t    flags   = p_c->b_stop ? S2_RS_STOP_LATCHED : 0u;

    *p_state = zero;

    if (wheels_stopped(p_in))
    {
        flags |= S2_RS_STOPPED;
    }

    if (p_c->b_run_valid && b_valid)
    {
        flags |= S2_RS_DISTANCE_VALID;
        p_state->run_distance_mm = (int32_t)((int64_t)p_in->status.travel_mm
                                             - p_c->run_start_mm);
    }

    p_state->captured_ms     = p_in->now_ms;
    p_state->last_request_id = p_c->last_request_id;
    p_state->state           = p_c->state;
    p_state->fault           = p_c->fault;
    p_state->flags           = flags;
    p_state->last_command    = p_c->last_command;
    p_state->last_result     = p_c->last_result;
    p_state->motion_state    = p_in->b_status_ok ? p_in->status.state : 0u;
}

/* ------------------------------------------------------------------------
 *  Private
 * ------------------------------------------------------------------------ */

static void
enter (ctrl_t * p_c, uint8_t state, uint32_t now_ms)
{
    p_c->state    = state;
    p_c->since_ms = now_ms;
}

static void
latch (ctrl_t * p_c, ctrl_output_t * p_out)
{
    p_c->b_stop         = true;
    p_out->b_latch_stop = true;
    p_out->b_clear_stop = false;    /* a latch always beats a clear */
}

static void
fault (ctrl_t * p_c, uint8_t code, uint32_t now_ms, ctrl_output_t * p_out)
{
    latch(p_c, p_out);
    p_c->b_waiting = false;
    p_out->action  = CTRL_ACT_NONE;

    if (S2_SYS_FAULT != p_c->state)
    {
        p_c->fault = code;
        enter(p_c, S2_SYS_FAULT, now_ms);
    }
}

static void
stop_running (ctrl_t * p_c, uint32_t now_ms, ctrl_output_t * p_out)
{
    p_c->b_waiting = false;
    p_out->action  = CTRL_ACT_NONE;
    enter(p_c, S2_SYS_STOPPED, now_ms);
}

static bool
motion_healthy (ctrl_input_t const * p_in)
{
    return (p_in->b_status_ok
            && (0u != (p_in->status.flags & MOTION_ST_VALID))
            && ((uint8_t)MOTION_IDLE == p_in->status.state)
            && (MOTION_FAULT_NONE == p_in->status.fault));
}

static bool
wheels_stopped (ctrl_input_t const * p_in)
{
    return (p_in->b_status_ok
            && (0u != (p_in->status.flags & MOTION_ST_STOPPED)));
}

/*
 * Running. Only the demo script lives here for Week 10. Later substates
 * go in this function:
 *   TODO(line following): steer from S3's line position.
 *   TODO(barcode manoeuvre): s2_nav_request_t to motion_turn_left/right.
 *   TODO(scan and plan): HAZARD, scan_q REQUEST/AUTHORISE/STOP_CONFIRM.
 *   TODO(bypass): execute S5's plan segments.
 *   TODO(reacquire line): bounded search, then line following.
 */
static void
run_step (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    if (p_c->b_waiting)
    {
        bool const b_ours = p_in->b_status_ok
                            && (p_in->status.last_id == p_c->wait_id);
        uint8_t const res = p_in->status.last_result;

        if (b_ours && (MOTION_EVT_DONE == res))
        {
            p_c->b_waiting = false;     /* send the next step now */
        }
        else if (b_ours && ((MOTION_EVT_REJECTED == res)
                            || (MOTION_EVT_FAULT == res)
                            || (MOTION_EVT_STOPPED == res)))
        {
            fault(p_c, S2_FAULT_MOTION, p_in->now_ms, p_out);
            return;
        }
        else if ((p_in->now_ms - p_c->wait_since_ms)
                 > p_c->cfg.result_limit_ms)
        {
            fault(p_c, S2_FAULT_MOTION, p_in->now_ms, p_out);
            return;
        }
        else
        {
            return;                     /* still waiting */
        }
    }

    if ((NULL != p_c->cfg.p_script) && (p_c->step < p_c->cfg.script_len))
    {
        p_out->action = p_c->cfg.p_script[p_c->step].action;
        p_out->value  = p_c->cfg.p_script[p_c->step].value;
        p_c->step++;
    }
    else if (p_c->cfg.script_len > 0u)
    {
        /* Script finished: hold still. (Complete comes later.) */
        latch(p_c, p_out);
        enter(p_c, S2_SYS_STOPPED, p_in->now_ms);
    }
    else
    {
        /* No script: Running with nothing to do yet. */
    }
}

static void
do_start (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    p_out->b_drain_motion = true;
    p_out->b_clear_stop   = true;
    p_out->action         = CTRL_ACT_NEW_RUN;
    p_c->b_stop           = false;
    p_c->step             = 0u;
    p_c->b_waiting        = false;
    p_c->run_start_mm     = p_in->status.travel_mm;
    p_c->b_run_valid      = true;
    enter(p_c, S2_SYS_RUNNING, p_in->now_ms);
}

static uint8_t
do_reset (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    uint8_t result = S2_RESULT_REJECTED;

    if (!wheels_stopped(p_in))
    {
        return (result);
    }

    if (S2_SYS_STOPPED == p_c->state)
    {
        if (motion_healthy(p_in))
        {
            p_out->b_drain_motion = true;
            enter(p_c, S2_SYS_READY, p_in->now_ms);
            result = S2_RESULT_COMPLETED;
        }
    }
    else if (S2_SYS_FAULT == p_c->state)
    {
        p_out->b_drain_motion = true;
        p_c->fault            = S2_FAULT_NONE;
        p_c->b_waiting        = false;
        enter(p_c, S2_SYS_INITIALISING, p_in->now_ms);

        if ((uint8_t)MOTION_FAULT == p_in->status.state)
        {
            p_out->action = CTRL_ACT_RESET;
        }

        result = S2_RESULT_COMPLETED;
    }
    else
    {
        /* RESET is not allowed in Initialising, Ready or Running. */
    }

    return (result);
}

/* End of file. */
