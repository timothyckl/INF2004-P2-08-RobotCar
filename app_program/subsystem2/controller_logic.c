/**
 * @file controller_logic.c
 * @brief Subsystem 2 Vehicle Controller (see controller_logic.h).
 */
#include "controller_logic.h"

#include "controller_config.h"
#include "motion_logic.h"   /* motion_state_t values */

#define CTRL_STOP_BITS  (CTRL_FLG_REMOTE_STOP | CTRL_FLG_LOCAL_STOP)
#define CTRL_QUARTER_MDEG   90000u
#define CTRL_HALF_MDEG      180000u
#define CTRL_FULL_MDEG      360000

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

/* Mission (Running substates). */
static bool is_mission (ctrl_t const * p_c);
static bool line_fresh (ctrl_t const * p_c, ctrl_input_t const * p_in);
static void enter_mode (ctrl_t * p_c, uint8_t mode, uint32_t now_ms);
static void issue (ctrl_t * p_c, ctrl_output_t * p_out, ctrl_action_t action,
                   uint32_t value, bool b_wait);
static void drive (ctrl_t * p_c, ctrl_output_t * p_out, int32_t left,
                   int32_t right);
static void push_scan (ctrl_output_t * p_out, uint8_t kind, bool b_asserted,
                       uint32_t request_id);
static void mission_step (ctrl_t * p_c, ctrl_input_t const * p_in,
                          ctrl_output_t * p_out);
static void mission_done (ctrl_t * p_c, ctrl_input_t const * p_in,
                          ctrl_output_t * p_out);
static void follow (ctrl_t * p_c, ctrl_input_t const * p_in,
                    ctrl_output_t * p_out);
static void navigate (ctrl_t * p_c, uint8_t command, uint32_t now_ms,
                      ctrl_output_t * p_out);
static void begin_scan (ctrl_t * p_c, ctrl_input_t const * p_in,
                        ctrl_output_t * p_out);
static void scan_step (ctrl_t * p_c, ctrl_input_t const * p_in,
                       ctrl_output_t * p_out);
static void bypass_next (ctrl_t * p_c, ctrl_input_t const * p_in,
                         ctrl_output_t * p_out);
static void reacquire (ctrl_t * p_c, ctrl_input_t const * p_in,
                       bool b_after_bypass, ctrl_output_t * p_out);
static void reacquire_step (ctrl_t * p_c, ctrl_input_t const * p_in,
                            ctrl_output_t * p_out);
static void turn (ctrl_t * p_c, ctrl_output_t * p_out, int32_t mdeg);
static int32_t clamp_speed (ctrl_t const * p_c, int32_t speed);
static int32_t wrap_mdeg (int32_t mdeg);
static bool line_reset (ctrl_t * p_c, uint32_t now_ms);
static int32_t line_correction_speed (uint8_t strength);
static bool line_apply (ctrl_t * p_c, ctrl_output_t * p_out,
                        line_control_action_t action, uint8_t strength);

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
    (void)line_reset(p_c, now_ms);
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

void
ctrl_mission_defaults (ctrl_mission_cfg_t * p_mission)
{
    if (NULL != p_mission)
    {
        p_mission->follow_mm_s      = CTRL_FOLLOW_SPEED_MM_S;
        p_mission->search_mm_s      = CTRL_SEARCH_SPEED_MM_S;
        p_mission->speed_max_mm_s   = MC_SPEED_MAX_MM_S;
        p_mission->junction_mm      = CTRL_JUNCTION_CROSS_MM;
        p_mission->line_max_age_ms  = CTRL_LINE_MAX_AGE_MS;
        p_mission->line_fail_ms     = CTRL_LINE_FAIL_MS;
        p_mission->found_ms         = CTRL_FOUND_MS;
        p_mission->search_limit_ms  = CTRL_SEARCH_LIMIT_MS;
        p_mission->scan_limit_ms    = CTRL_SCAN_LIMIT_MS;
        p_mission->heading_tol_mdeg = CTRL_HEADING_TOL_MDEG;
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
        latch(p_c, p_out);

        if (S2_SYS_RUNNING == p_c->state)
        {
            stop_running(p_c, p_in->now_ms, p_out);
        }
    }

    if (0u != (bits & CTRL_FLG_HAZARD))
    {
        bool const b_running = (S2_SYS_RUNNING == p_c->state);

        if (b_running && is_mission(p_c) && (S2_MODE_FOLLOW == p_c->mode))
        {
            begin_scan(p_c, p_in, p_out);   /* Scan & Plan, no latch */
        }
        else if (b_running && (S2_MODE_SCAN == p_c->mode))
        {
            /* Already stopped and scanning. */
        }
        else
        {
            latch(p_c, p_out);

            if (b_running)
            {
                stop_running(p_c, p_in->now_ms, p_out);
            }
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
        /* Design review: START needs a healthy system and a valid line. */
        bool const b_line_ok = (!is_mission(p_c))
                               || (line_fresh(p_c, p_in)
                                   && (0u != (p_in->line.flags
                                              & CTRL_LINE_VALID)));

        if ((S2_SYS_READY == p_c->state) && motion_healthy(p_in)
            && wheels_stopped(p_in) && b_line_ok)
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
 *        A failed send is a delivery fault. DRIVE and STOP produce no
 *        result, so they leave nothing outstanding.
 */
void
ctrl_motion_sent (ctrl_t * p_c, bool b_ok, uint16_t cmd_id, uint32_t now_ms,
                  ctrl_output_t * p_out)
{
    if ((NULL == p_c) || (NULL == p_out))
    {
        return;
    }

    bool const b_no_wait = p_c->b_no_wait;

    p_c->b_no_wait = false;

    if (!b_ok)
    {
        fault(p_c, S2_FAULT_DELIVERY, now_ms, p_out);
        return;
    }

    if (!b_no_wait)
    {
        p_c->b_waiting     = true;
        p_c->wait_id       = cmd_id;
        p_c->wait_since_ms = now_ms;
    }
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

    bool const b_valid   = p_in->b_status_ok
                           && (0u != (p_in->status.flags & MOTION_ST_VALID));
    bool const b_running = (S2_SYS_RUNNING == p_c->state);
    uint8_t    flags     = p_c->b_stop ? S2_RS_STOP_LATCHED : 0u;

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

    /* S3 may offer a junction command only while following. */
    if (b_running && (S2_MODE_FOLLOW == p_c->mode))
    {
        flags |= S2_RS_NAV_AUTHORISED;
    }

    p_state->captured_ms     = p_in->now_ms;
    p_state->last_request_id = p_c->last_request_id;
    p_state->state           = p_c->state;
    p_state->fault           = p_c->fault;
    p_state->flags           = flags;
    p_state->last_command    = p_c->last_command;
    p_state->last_result     = p_c->last_result;
    p_state->motion_state    = p_in->b_status_ok ? p_in->status.state : 0u;
    p_state->mode            = b_running ? p_c->mode : S2_MODE_NONE;
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
    if ((S2_SYS_RUNNING == p_c->state)
        && ((S2_MODE_SCAN == p_c->mode) || (S2_MODE_BYPASS == p_c->mode)))
    {
        push_scan(p_out, S2_SCAN_STOP, true, p_c->scan_id);
    }

    latch(p_c, p_out);
    p_c->b_waiting = false;
    p_out->action  = CTRL_ACT_NONE;
    p_c->mode      = S2_MODE_NONE;

    if (S2_SYS_FAULT != p_c->state)
    {
        p_c->fault = code;
        enter(p_c, S2_SYS_FAULT, now_ms);
    }
}

static void
stop_running (ctrl_t * p_c, uint32_t now_ms, ctrl_output_t * p_out)
{
    if ((S2_MODE_SCAN == p_c->mode) || (S2_MODE_BYPASS == p_c->mode))
    {
        push_scan(p_out, S2_SCAN_STOP, true, p_c->scan_id);
    }

    p_c->b_waiting = false;
    p_out->action  = CTRL_ACT_NONE;
    p_c->mode      = S2_MODE_NONE;
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

/* Running: wait for an outstanding motion result, then the demo script
   (if one is configured) or the mission substates. */
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
            p_c->b_waiting = false;

            if (is_mission(p_c))
            {
                mission_done(p_c, p_in, p_out);

                if ((S2_SYS_RUNNING != p_c->state) || p_c->b_waiting
                    || (CTRL_ACT_NONE != p_out->action))
                {
                    return;
                }
            }
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

    if (is_mission(p_c))
    {
        mission_step(p_c, p_in, p_out);
    }
    else if (p_c->step < p_c->cfg.script_len)
    {
        p_out->action = p_c->cfg.p_script[p_c->step].action;
        p_out->value  = p_c->cfg.p_script[p_c->step].value;
        p_c->step++;
    }
    else
    {
        /* Script finished: hold still. (Complete comes later.) */
        latch(p_c, p_out);
        enter(p_c, S2_SYS_STOPPED, p_in->now_ms);
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
    p_c->mode             = S2_MODE_NONE;   /* FOLLOW once NEW_RUN is done */
    p_c->line_good_ms     = p_in->now_ms;
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

/* ------------------------------------------------------------------------
 *  Mission: Running substates
 * ------------------------------------------------------------------------ */

static bool
is_mission (ctrl_t const * p_c)
{
    return ((NULL == p_c->cfg.p_script) || (0u == p_c->cfg.script_len));
}

static bool
line_fresh (ctrl_t const * p_c, ctrl_input_t const * p_in)
{
    return (p_in->line.b_ok
            && ((p_in->now_ms - p_in->line.captured_ms)
                <= p_c->cfg.mission.line_max_age_ms));
}

static void
enter_mode (ctrl_t * p_c, uint8_t mode, uint32_t now_ms)
{
    p_c->mode          = mode;
    p_c->mode_since_ms = now_ms;

    if (S2_MODE_FOLLOW == mode)
    {
        (void)line_reset(p_c, now_ms);
    }
}

static void
issue (ctrl_t * p_c, ctrl_output_t * p_out, ctrl_action_t action,
       uint32_t value, bool b_wait)
{
    p_out->action  = action;
    p_out->value   = value;
    p_c->b_no_wait = !b_wait;
}

static void
drive (ctrl_t * p_c, ctrl_output_t * p_out, int32_t left, int32_t right)
{
    /* The casts are safe: clamp_speed bounds both to speed_max_mm_s,
       which motion_drive() itself limits to MC_SPEED_MAX_MM_S. */
    issue(p_c, p_out, CTRL_ACT_DRIVE, 0u, false);
    p_out->drive_l = (int16_t)clamp_speed(p_c, left);
    p_out->drive_r = (int16_t)clamp_speed(p_c, right);
}

static void
push_scan (ctrl_output_t * p_out, uint8_t kind, bool b_asserted,
           uint32_t request_id)
{
    if (p_out->n_scan < CTRL_SCAN_MSGS_MAX)
    {
        p_out->scan[p_out->n_scan].kind       = kind;
        p_out->scan[p_out->n_scan].b_asserted = b_asserted;
        p_out->scan[p_out->n_scan].request_id = request_id;
        p_out->n_scan++;
    }
}

static void
mission_step (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    if (line_fresh(p_c, p_in))
    {
        p_c->line_good_ms = p_in->now_ms;
    }

    switch (p_c->mode)
    {
        case S2_MODE_FOLLOW:
            follow(p_c, p_in, p_out);
            break;
        case S2_MODE_SCAN:
            scan_step(p_c, p_in, p_out);
            break;
        case S2_MODE_REACQUIRE:
            reacquire_step(p_c, p_in, p_out);
            break;
        default:
            /* NONE, NAVIGATE, BYPASS: driven by motion results. */
            break;
    }
}

/* A waited command finished (DONE). */
static void
mission_done (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    switch (p_c->mode)
    {
        case S2_MODE_NONE:          /* NEW_RUN acknowledged */
        case S2_MODE_NAVIGATE:      /* manoeuvre complete */
            enter_mode(p_c, S2_MODE_FOLLOW, p_in->now_ms);
            break;
        case S2_MODE_BYPASS:
            bypass_next(p_c, p_in, p_out);
            break;
        case S2_MODE_REACQUIRE:
            if (p_c->b_aligning)
            {
                int32_t const err = wrap_mdeg(p_in->status.heading_mdeg
                                              - p_c->heading_ref_mdeg);
                int32_t const tol = p_c->cfg.mission.heading_tol_mdeg;

                p_c->b_aligning = false;

                if ((err <= tol) && (err >= -tol))
                {
                    enter_mode(p_c, S2_MODE_FOLLOW, p_in->now_ms);
                }
                else
                {
                    fault(p_c, S2_FAULT_LINE, p_in->now_ms, p_out);
                }
            }
            break;
        default:
            /* FOLLOW and SCAN issue no waited commands. */
            break;
    }
}

static void
follow (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    ctrl_line_t const * p_line  = &p_in->line;
    bool const          b_fresh = line_fresh(p_c, p_in);

    if ((p_line->b_ok && (0u != (p_line->flags & CTRL_LINE_FAULT)))
        || ((p_in->now_ms - p_c->line_good_ms)
            > p_c->cfg.mission.line_fail_ms))
    {
        fault(p_c, S2_FAULT_LINE, p_in->now_ms, p_out);
    }
    else if (b_fresh && (0u != p_line->nav_cmd)
             && ((!p_c->b_have_nav) || (p_line->nav_id != p_c->last_nav_id)))
    {
        p_c->last_nav_id = p_line->nav_id;      /* each command once */
        p_c->b_have_nav  = true;
        navigate(p_c, p_line->nav_cmd, p_in->now_ms, p_out);
    }
    else if (b_fresh && (0u != (p_line->flags & CTRL_LINE_LOST)))
    {
        reacquire(p_c, p_in, false, p_out);
    }
    else if (b_fresh && (0u != (p_line->flags & CTRL_LINE_VALID)))
    {
        /* task_controller.c verifies that IF-06 and line-control position
         * values match before this bounded enum conversion is compiled. */
        line_control_position_t const position =
            (line_control_position_t)p_line->position;
        line_control_action_t const action = line_control_step(
            &p_c->line_control, position, p_in->now_ms);

        if (!line_apply(p_c, p_out, action, p_line->strength))
        {
            fault(p_c, S2_FAULT_LINE, p_in->now_ms, p_out);
        }
    }
    else
    {
        drive(p_c, p_out, 0, 0);    /* unsettled or stale: hold still */
    }
}

static bool
line_reset (ctrl_t * p_c, uint32_t now_ms)
{
    line_control_cfg_t const cfg =
    {
        CTRL_LINE_CURVE_ENTRY_MS,
        CTRL_LINE_LEFT_HOLD_MS,
        CTRL_LINE_RIGHT_HOLD_MS,
        CTRL_LINE_JUNCTION_MS,
        CTRL_LINE_CROSSING_MS,
        CTRL_LINE_EXIT_MIN_MS,
        CTRL_LINE_EXIT_CENTER_MS,
        CTRL_LINE_EXIT_SIDE_MS,
        CTRL_LINE_EXIT_LIMIT_MS
    };

    return line_control_init(&p_c->line_control, &cfg, now_ms);
}

static int32_t
line_correction_speed (uint8_t strength)
{
    uint32_t const bounded = (strength <= CTRL_LINE_STRENGTH_MAX)
                             ? strength : CTRL_LINE_STRENGTH_MAX;
    uint32_t const range = CTRL_LINE_CORRECT_MAX_MM_S
                           - CTRL_LINE_CORRECT_MIN_MM_S;
    uint32_t const reduction = ((bounded * range)
                                + (CTRL_LINE_STRENGTH_MAX / 2U))
                               / CTRL_LINE_STRENGTH_MAX;

    return CTRL_LINE_CORRECT_MAX_MM_S - (int32_t)reduction;
}

static bool
line_apply (ctrl_t * p_c, ctrl_output_t * p_out,
            line_control_action_t action, uint8_t strength)
{
    bool b_valid = true;
    int32_t const correction = line_correction_speed(strength);

    switch (action)
    {
        case LINE_CONTROL_ACTION_STRAIGHT:
            drive(p_c, p_out, CTRL_FOLLOW_SPEED_MM_S,
                  CTRL_FOLLOW_SPEED_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_LEFT:
            drive(p_c, p_out, correction, CTRL_FOLLOW_SPEED_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_RIGHT:
            drive(p_c, p_out, CTRL_FOLLOW_SPEED_MM_S, correction);
        break;

        case LINE_CONTROL_ACTION_CURVE_LEFT:
            drive(p_c, p_out, CTRL_LINE_CURVE_INNER_MM_S,
                  CTRL_LINE_CURVE_OUTER_MM_S);
        break;

        case LINE_CONTROL_ACTION_CURVE_RIGHT:
            drive(p_c, p_out, CTRL_LINE_CURVE_OUTER_MM_S,
                  CTRL_LINE_CURVE_INNER_MM_S);
        break;

        case LINE_CONTROL_ACTION_STOP:
        default:
            b_valid = false;
        break;
    }

    return b_valid;
}

static void
navigate (ctrl_t * p_c, uint8_t command, uint32_t now_ms,
          ctrl_output_t * p_out)
{
    enter_mode(p_c, S2_MODE_NAVIGATE, now_ms);

    switch (command)
    {
        case S2_NAV_LEFT:
            issue(p_c, p_out, CTRL_ACT_LEFT, CTRL_QUARTER_MDEG, true);
            break;
        case S2_NAV_RIGHT:
            issue(p_c, p_out, CTRL_ACT_RIGHT, CTRL_QUARTER_MDEG, true);
            break;
        case S2_NAV_UTURN:
            issue(p_c, p_out, CTRL_ACT_LEFT, CTRL_HALF_MDEG, true);
            break;
        case S2_NAV_STRAIGHT:
            issue(p_c, p_out, CTRL_ACT_FORWARD, p_c->cfg.mission.junction_mm,
                  true);
            break;
        default:
            enter_mode(p_c, S2_MODE_FOLLOW, now_ms);    /* unknown: ignore */
            break;
    }
}

static void
begin_scan (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    p_c->scan_id++;

    if (0u == p_c->scan_id)
    {
        p_c->scan_id = 1u;          /* request IDs are never 0 */
    }

    p_c->b_scan_granted   = false;
    p_c->heading_ref_mdeg = p_in->status.heading_mdeg;
    issue(p_c, p_out, CTRL_ACT_STOP, 0u, false);
    push_scan(p_out, S2_SCAN_REQUEST, true, p_c->scan_id);
    enter_mode(p_c, S2_MODE_SCAN, p_in->now_ms);
}

static void
scan_step (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    ctrl_scan_t const * p_scan = &p_in->scan;

    if ((!p_c->b_scan_granted) && wheels_stopped(p_in))
    {
        push_scan(p_out, S2_SCAN_STOP_CONFIRM, true, p_c->scan_id);
        push_scan(p_out, S2_SCAN_AUTHORISE, true, p_c->scan_id);
        p_c->b_scan_granted = true;
    }

    if (p_scan->b_ok && (p_scan->request_id == p_c->scan_id)
        && (0u != p_scan->result))
    {
        if ((CTRL_SCAN_PLAN == p_scan->result)
            && (p_scan->count <= CTRL_PLAN_SEGMENTS))
        {
            p_c->plan = *p_scan;
            p_c->seg  = 0u;
            enter_mode(p_c, S2_MODE_BYPASS, p_in->now_ms);
            bypass_next(p_c, p_in, p_out);
        }
        else if (CTRL_SCAN_CLEAR == p_scan->result)
        {
            enter_mode(p_c, S2_MODE_FOLLOW, p_in->now_ms);
        }
        else
        {
            fault(p_c, S2_FAULT_SCAN, p_in->now_ms, p_out);
        }
    }
    else if ((p_in->now_ms - p_c->mode_since_ms)
             > p_c->cfg.mission.scan_limit_ms)
    {
        push_scan(p_out, S2_SCAN_CANCEL, true, p_c->scan_id);
        fault(p_c, S2_FAULT_SCAN, p_in->now_ms, p_out);
    }
    else
    {
        /* Waiting for stillness or for S5's result. */
    }
}

/* Send the next plan segment, then the approach turn, then search. */
static void
bypass_next (ctrl_t * p_c, ctrl_input_t const * p_in, ctrl_output_t * p_out)
{
    bool b_sent = false;

    while ((!b_sent) && (p_c->seg < p_c->plan.count))
    {
        ctrl_segment_t const * p_seg = &p_c->plan.seg[p_c->seg];

        p_c->seg++;

        if ((CTRL_SEG_TURN == p_seg->kind) && (0 != p_seg->value))
        {
            turn(p_c, p_out, p_seg->value);
            b_sent = true;
        }
        else if ((CTRL_SEG_MOVE == p_seg->kind) && (p_seg->value > 0))
        {
            issue(p_c, p_out, CTRL_ACT_FORWARD, (uint32_t)p_seg->value, true);
            b_sent = true;
        }
        else if ((CTRL_SEG_MOVE == p_seg->kind) && (p_seg->value < 0))
        {
            issue(p_c, p_out, CTRL_ACT_BACKWARD, (uint32_t)(-p_seg->value),
                  true);
            b_sent = true;
        }
        else
        {
            /* Empty segment: skip. */
        }
    }

    if ((!b_sent) && (p_c->seg == p_c->plan.count))
    {
        p_c->seg++;                         /* approach turn, once */

        if (0 != p_c->plan.approach_mdeg)
        {
            turn(p_c, p_out, p_c->plan.approach_mdeg);
            b_sent = true;
        }
    }

    if (!b_sent)
    {
        reacquire(p_c, p_in, true, p_out);
    }
}

static void
reacquire (ctrl_t * p_c, ctrl_input_t const * p_in, bool b_after_bypass,
           ctrl_output_t * p_out)
{
    enter_mode(p_c, S2_MODE_REACQUIRE, p_in->now_ms);
    p_c->b_after_bypass  = b_after_bypass;
    p_c->search_side     = b_after_bypass ? 0u : p_in->line.last_side;
    p_c->search_start_mm = p_in->status.travel_mm;
    p_c->b_seen          = false;
    p_c->b_aligning      = false;
    reacquire_step(p_c, p_in, p_out);
}

static void
reacquire_step (ctrl_t * p_c, ctrl_input_t const * p_in,
                ctrl_output_t * p_out)
{
    int32_t const s       = p_c->cfg.mission.search_mm_s;
    int64_t const travel  = (int64_t)p_in->status.travel_mm
                            - p_c->search_start_mm;
    bool const    b_black = line_fresh(p_c, p_in)
                            && (0u != (p_in->line.flags & CTRL_LINE_SEEN));

    if (p_c->b_aligning)
    {
        return;                             /* align turn outstanding */
    }

    if (((p_in->now_ms - p_c->mode_since_ms)
         > p_c->cfg.mission.search_limit_ms)
        || (p_c->b_after_bypass && (travel > (int64_t)p_c->plan.search_mm))
        || (p_in->line.b_ok && (0u != (p_in->line.flags & CTRL_LINE_FAULT))))
    {
        fault(p_c, S2_FAULT_LINE, p_in->now_ms, p_out);
        return;
    }

    if (b_black && (!p_c->b_seen))
    {
        p_c->b_seen        = true;
        p_c->seen_since_ms = p_in->now_ms;
    }
    else if (!b_black)
    {
        p_c->b_seen = false;
    }
    else
    {
        /* Still seeing the line. */
    }

    if (p_c->b_seen
        && ((p_in->now_ms - p_c->seen_since_ms) >= p_c->cfg.mission.found_ms))
    {
        if (p_c->b_after_bypass && (0 != p_c->plan.align_mdeg))
        {
            p_c->b_aligning = true;
            turn(p_c, p_out, p_c->plan.align_mdeg);
        }
        else
        {
            enter_mode(p_c, S2_MODE_FOLLOW, p_in->now_ms);
            drive(p_c, p_out, 0, 0);
        }
    }
    else if (p_c->b_after_bypass)
    {
        drive(p_c, p_out, s, s);            /* forward towards the line */
    }
    else if (CTRL_SIDE_RIGHT == p_c->search_side)
    {
        drive(p_c, p_out, s, -s);           /* rotate right */
    }
    else
    {
        drive(p_c, p_out, -s, s);           /* rotate left (or unknown) */
    }
}

static void
turn (ctrl_t * p_c, ctrl_output_t * p_out, int32_t mdeg)
{
    if (mdeg >= 0)
    {
        issue(p_c, p_out, CTRL_ACT_LEFT, (uint32_t)mdeg, true);
    }
    else
    {
        issue(p_c, p_out, CTRL_ACT_RIGHT, (uint32_t)(-mdeg), true);
    }
}

static int32_t
clamp_speed (ctrl_t const * p_c, int32_t speed)
{
    int32_t const max = p_c->cfg.mission.speed_max_mm_s;

    return ((speed > max) ? max : ((speed < -max) ? -max : speed));
}

static int32_t
wrap_mdeg (int32_t mdeg)
{
    int32_t a = mdeg % CTRL_FULL_MDEG;

    if (a > (CTRL_FULL_MDEG / 2))
    {
        a -= CTRL_FULL_MDEG;
    }
    else if (a <= -(CTRL_FULL_MDEG / 2))
    {
        a += CTRL_FULL_MDEG;
    }
    else
    {
        /* Already in (-180, 180] degrees. */
    }

    return (a);
}

/* End of file. */
