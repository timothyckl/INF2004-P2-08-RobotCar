/**
 * @file motion_logic.c
 * @brief Subsystem 2 Motion Control state machine (see motion_logic.h).
 *
 * Times are unsigned microsecond differences, valid for intervals under
 * 35 minutes. Distances come from counts since a reference, converted
 * once, so truncation does not accumulate. Pose integration uses a 1
 * degree Q15 sine table with linear interpolation (no floating point).
 */
#include "motion_logic.h"


#define US_PER_MS           1000u
#define US_PER_S            1000000
#define UM_PER_MM           1000
#define Q15_ONE             32767
#define MDEG_PER_DEG        1000
#define MDEG_90             90000
#define MDEG_180            180000
#define MDEG_360            360000
#define PI_NUM              355         /* pi ~= 355 / 113 */
#define PI_DEN              113
#define LIMIT_US_MAX        0x7FFFFFFFu /* keeps unsigned compares valid */
#define ID_HALF_RANGE       0x8000u

/* sin(0..90 degrees) x 32767, 1 degree steps. */
static const int16_t g_sin_q15[91] =
{
    0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,
    5690, 6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668,
    11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886,
    16383, 16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621,
    21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964, 24351, 24730,
    25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087,
    28377, 28659, 28932, 29196, 29451, 29697, 29934, 30162, 30381, 30591,
    30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762,
    32767
};

static void    add_event (motion_output_t * p_out, uint8_t type,
                          uint8_t reason, uint16_t id);
static void    note_result (motion_t * p_m, uint8_t type, uint8_t reason,
                            uint16_t id);
static void    emit (motion_t * p_m, motion_output_t * p_out, uint8_t type,
                     uint8_t reason, uint16_t id);
static void    enter_fault (motion_t * p_m, uint8_t reason,
                            motion_output_t * p_out);
static void    enter_stopping (motion_t * p_m, uint8_t type, uint16_t id,
                               uint32_t now_us);
static void    start_motion (motion_t * p_m, s2_motion_cmd_t const * p_cmd,
                             motion_input_t const * p_in);
static bool    params_valid (motion_t const * p_m,
                             s2_motion_cmd_t const * p_cmd);
static bool    id_newer (uint16_t id, uint16_t last);
static bool    is_moving (motion_state_t state);
static void    measure (motion_t * p_m, motion_input_t const * p_in,
                        motion_output_t * p_out);
static void    integrate_pose (motion_t * p_m, int32_t dl, int32_t dr);
static void    control (motion_t * p_m, motion_input_t const * p_in,
                        motion_output_t * p_out);
static void    check_stall (motion_t * p_m, uint32_t now_us,
                            motion_output_t * p_out);
static int32_t mm_since (motion_t const * p_m, uint32_t now, uint32_t start);
static int32_t counts_mm (motion_t const * p_m, int32_t counts);
static int32_t abs32 (int32_t v);
static int32_t clamp32 (int32_t v, int32_t lo, int32_t hi);
static int32_t wrap_mdeg (int64_t a);
static int32_t sin_q15 (int32_t mdeg);

/**
 * @brief Configure and enter Init (zero duty until motion_start()).
 * @return false, leaving *p_m unchanged, on invalid tuning.
 */
bool
motion_init (motion_t * p_m, motion_cfg_t const * p_cfg)
{
    motion_t m = {0};

    if ((NULL == p_m) || (NULL == p_cfg) || (!odom_cfg_valid(&p_cfg->odom))
        || (p_cfg->speed_window < 1u)
        || (p_cfg->speed_window > MOTION_WINDOW_MAX)
        || (p_cfg->speed_max_mm_s <= 0) || (p_cfg->duty_step_max <= 0)
        || (!pid_init(&m.pid_l, &p_cfg->pid))
        || (!pid_init(&m.pid_r, &p_cfg->pid))
        || (!pid_init(&m.pid_diff, &p_cfg->straight_pid)))
    {
        return (false);
    }

    m.cfg   = *p_cfg;
    m.state = MOTION_INIT;
    *p_m    = m;

    return (true);
}

void
motion_output_clear (motion_output_t * p_out)
{
    if (NULL != p_out)
    {
        p_out->duty_l        = 0;
        p_out->duty_r        = 0;
        p_out->n_events      = 0u;
        p_out->b_raise_fault = false;
    }
}

/** @brief Leave Init: Idle + READY if the hardware started, else Fault. */
void
motion_start (motion_t * p_m, bool b_hw_ok, motion_output_t * p_out)
{
    if ((NULL == p_m) || (NULL == p_out) || (MOTION_INIT != p_m->state))
    {
        return;
    }

    if (b_hw_ok)
    {
        p_m->state = MOTION_IDLE;
        emit(p_m, p_out, MOTION_EVT_READY, 0u, 0u);
    }
    else
    {
        enter_fault(p_m, MOTION_FAULT_INIT, p_out);
    }
}

/**
 * @brief Handle one motion_q command. Call before motion_step() in the
 *        same period, with the same input.
 */
void
motion_command (motion_t * p_m, s2_motion_cmd_t const * p_cmd,
                motion_input_t const * p_in, motion_output_t * p_out)
{
    if ((NULL == p_m) || (NULL == p_cmd) || (NULL == p_in) || (NULL == p_out))
    {
        return;
    }

    uint16_t const id = p_cmd->cmd_id;

    /* STOP is always obeyed, whatever its ID; others must be newer. */
    if ((S2_CMD_STOP != p_cmd->cmd) && p_m->b_have_id
        && (!id_newer(id, p_m->last_id)))
    {
        emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_STALE, id);
        return;
    }

    if ((!p_m->b_have_id) || id_newer(id, p_m->last_id))
    {
        p_m->last_id   = id;
        p_m->b_have_id = true;
    }

    switch (p_cmd->cmd)
    {
        case S2_CMD_STOP:
            if (MOTION_FAULT == p_m->state)
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, p_m->fault, id);
            }
            else if (MOTION_INIT == p_m->state)
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_STATE, id);
            }
            else if (MOTION_STOPPING == p_m->state)
            {
                p_m->pending_type = MOTION_EVT_STOPPED;
                p_m->pending_id   = id;
            }
            else
            {
                p_m->cmd_id = id;
                enter_stopping(p_m, MOTION_EVT_STOPPED, id, p_in->now_us);
            }
            break;

        case S2_CMD_RESET:
            if (MOTION_FAULT == p_m->state)
            {
                if (p_m->b_still && p_m->b_valid)
                {
                    p_m->state = MOTION_IDLE;
                    p_m->fault = MOTION_FAULT_NONE;
                    emit(p_m, p_out, MOTION_EVT_DONE, 0u, id);
                }
                else
                {
                    emit(p_m, p_out, MOTION_EVT_REJECTED, p_m->fault, id);
                }
            }
            else if (MOTION_IDLE == p_m->state)
            {
                emit(p_m, p_out, MOTION_EVT_DONE, 0u, id);
            }
            else
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_STATE, id);
            }
            break;

        case S2_CMD_NEW_RUN:
            if (MOTION_IDLE == p_m->state)
            {
                p_m->run_l0       = p_m->tot_l;
                p_m->run_r0       = p_m->tot_r;
                p_m->x_um         = 0;
                p_m->y_um         = 0;
                p_m->heading_mdeg = 0;
                p_m->run_id       = (uint8_t)(p_m->run_id + 1u);
                emit(p_m, p_out, MOTION_EVT_DONE, 0u, id);
            }
            else
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_STATE, id);
            }
            break;

        case S2_CMD_SPEED:
        case S2_CMD_MOVE:
        case S2_CMD_TURN:
            if ((MOTION_INIT == p_m->state) || (MOTION_FAULT == p_m->state))
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED,
                          (MOTION_FAULT == p_m->state) ? p_m->fault
                                                       : MOTION_REJ_STATE, id);
            }
            else if (p_in->b_stop)
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_STOP, id);
            }
            else if (!params_valid(p_m, p_cmd))
            {
                emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_PARAM, id);
            }
            else
            {
                start_motion(p_m, p_cmd, p_in);
            }
            break;

        default:
            emit(p_m, p_out, MOTION_EVT_REJECTED, MOTION_REJ_PARAM, id);
            break;
    }
}

/** @brief Run one control period: measure, apply STOP, control. */
void
motion_step (motion_t * p_m, motion_input_t const * p_in,
             motion_output_t * p_out)
{
    if ((NULL == p_m) || (NULL == p_in) || (NULL == p_out))
    {
        return;
    }

    measure(p_m, p_in, p_out);

    bool const b_stop_rise = p_in->b_stop && (!p_m->b_stop_prev);

    p_m->b_stop_prev = p_in->b_stop;

    if (is_moving(p_m->state) && p_in->b_stop)
    {
        enter_stopping(p_m, MOTION_EVT_STOPPED, p_m->cmd_id, p_in->now_us);
    }
    else if ((MOTION_IDLE == p_m->state) && b_stop_rise)
    {
        enter_stopping(p_m, MOTION_EVT_STOPPED, p_m->last_id,
                       p_in->now_us);
    }
    else
    {
        /* No STOP transition. */
    }

    control(p_m, p_in, p_out);
}

/** @brief Enter Fault from outside (e.g. an event could not be sent). */
void
motion_fault (motion_t * p_m, uint8_t reason, motion_output_t * p_out)
{
    if ((NULL != p_m) && (NULL != p_out) && (MOTION_FAULT != p_m->state))
    {
        enter_fault(p_m, reason, p_out);
        p_out->duty_l = 0;
        p_out->duty_r = 0;
    }
}

/** @brief Fill a status snapshot (all but captured_ms). */
void
motion_get_status_logic (motion_t const * p_m, motion_status_t * p_status)
{
    if ((NULL == p_m) || (NULL == p_status))
    {
        return;
    }

    int64_t const travel = ((int64_t)counts_mm(p_m, p_m->tot_l)
                            + counts_mm(p_m, p_m->tot_r)) / 2;

    p_status->travel_mm    = (int32_t)travel;
    p_status->x_mm         = (int16_t)clamp32(p_m->x_um / UM_PER_MM,
                                              INT16_MIN, INT16_MAX);
    p_status->y_mm         = (int16_t)clamp32(p_m->y_um / UM_PER_MM,
                                              INT16_MIN, INT16_MAX);
    p_status->heading_mdeg = p_m->heading_mdeg;
    p_status->left_mm_s    = (int16_t)clamp32(p_m->speed_l, INT16_MIN,
                                              INT16_MAX);
    p_status->right_mm_s   = (int16_t)clamp32(p_m->speed_r, INT16_MIN,
                                              INT16_MAX);
    p_status->last_id      = p_m->res_id;
    p_status->last_result  = p_m->res_type;
    p_status->last_reason  = p_m->res_reason;
    p_status->state        = (uint8_t)p_m->state;
    p_status->fault        = p_m->fault;
    p_status->run_id       = p_m->run_id;
    p_status->flags        = (uint8_t)((p_m->b_valid ? MOTION_ST_VALID : 0u)
                                       | (motion_is_stopped(p_m)
                                          ? MOTION_ST_STOPPED : 0u));
}

/** @brief Stop confirmation: both wheels still for still_ms, not driven. */
bool
motion_is_stopped (motion_t const * p_m)
{
    return ((NULL != p_m) && p_m->b_still && (!is_moving(p_m->state)));
}

motion_state_t
motion_state (motion_t const * p_m)
{
    return ((NULL == p_m) ? MOTION_FAULT : p_m->state);
}

/**
 * @brief Limit how fast |duty| may grow (pure).
 *
 * Toward zero (including target 0 for STOP and Fault) is immediate. A
 * sign change returns 0 for one step, then ramps from 0. An increase is
 * limited to step_max; step_max <= 0 disables the limit.
 */
int32_t
motion_ramp (int32_t prev, int32_t target, int32_t step_max)
{
    int32_t out = target;

    if ((0 == target) || (step_max <= 0))
    {
        /* STOP, Fault, or no limit: immediate. */
    }
    else if (((prev > 0) && (target < 0)) || ((prev < 0) && (target > 0)))
    {
        out = 0;                            /* through zero first */
    }
    else if (target > 0)
    {
        out = ((target - prev) > step_max) ? (prev + step_max) : target;
    }
    else
    {
        out = ((prev - target) > step_max) ? (prev - step_max) : target;
    }

    return (out);
}

uint8_t
motion_fault_code (motion_t const * p_m)
{
    return ((NULL == p_m) ? MOTION_FAULT_INIT : p_m->fault);
}

/* ------------------------------------------------------------------------
 *  Private
 * ------------------------------------------------------------------------ */

static void
add_event (motion_output_t * p_out, uint8_t type, uint8_t reason, uint16_t id)
{
    if (p_out->n_events < MOTION_EVENTS_MAX)
    {
        p_out->events[p_out->n_events].type   = type;
        p_out->events[p_out->n_events].reason = reason;
        p_out->events[p_out->n_events].cmd_id = id;
        p_out->n_events++;
    }
}

/* Remember the latest result for motion_status_t (READY is not one). */
static void
emit (motion_t * p_m, motion_output_t * p_out, uint8_t type, uint8_t reason,
      uint16_t id)
{
    add_event(p_out, type, reason, id);
    note_result(p_m, type, reason, id);
}

static void
note_result (motion_t * p_m, uint8_t type, uint8_t reason, uint16_t id)
{
    if (MOTION_EVT_READY != type)
    {
        p_m->res_type   = type;
        p_m->res_reason = reason;
        p_m->res_id     = id;
    }
}

static void
enter_fault (motion_t * p_m, uint8_t reason, motion_output_t * p_out)
{
    p_m->state  = MOTION_FAULT;
    p_m->fault  = reason;
    p_m->duty_l = 0;
    p_m->duty_r = 0;
    pid_reset(&p_m->pid_l);
    pid_reset(&p_m->pid_r);
    emit(p_m, p_out, MOTION_EVT_FAULT, reason, p_m->cmd_id);
    p_out->b_raise_fault = true;
}

static void
enter_stopping (motion_t * p_m, uint8_t type, uint16_t id, uint32_t now_us)
{
    p_m->state         = MOTION_STOPPING;
    p_m->pending_type  = type;
    p_m->pending_id    = id;
    p_m->stop_start_us = now_us;
    p_m->duty_l        = 0;
    p_m->duty_r        = 0;
}

static void
start_motion (motion_t * p_m, s2_motion_cmd_t const * p_cmd,
              motion_input_t const * p_in)
{
    int64_t expected_us = 0;

    if (!is_moving(p_m->state))
    {
        pid_reset(&p_m->pid_l);
        pid_reset(&p_m->pid_r);
    }

    p_m->cmd_id    = p_cmd->cmd_id;
    p_m->start_l   = p_in->count_l;
    p_m->start_r   = p_in->count_r;
    p_m->start_us  = p_in->now_us;
    p_m->b_stall_l = false;
    p_m->b_stall_r = false;

    if (S2_CMD_SPEED == p_cmd->cmd)
    {
        if (0 == p_cmd->u.speed.left_mm_s)
        {
            pid_reset(&p_m->pid_l);
            p_m->duty_l = 0;
        }

        if (0 == p_cmd->u.speed.right_mm_s)
        {
            pid_reset(&p_m->pid_r);
            p_m->duty_r = 0;
        }

        p_m->state = MOTION_SPEED;
        p_m->tgt_l = p_cmd->u.speed.left_mm_s;
        p_m->tgt_r = p_cmd->u.speed.right_mm_s;
    }
    else if (S2_CMD_MOVE == p_cmd->cmd)
    {
        pid_reset(&p_m->pid_diff);
        p_m->state  = MOTION_MOVE;
        p_m->goal   = p_cmd->u.move.distance_mm;
        p_m->speed  = p_cmd->u.move.speed_mm_s;
        expected_us = ((int64_t)abs32(p_m->goal) * US_PER_S) / p_m->speed;
    }
    else
    {
        /* Each wheel's arc = angle * track / 2 = mdeg * pi * track / 360000 */
        int64_t const arc_um = ((int64_t)abs32(p_cmd->u.turn.angle_mdeg)
                                * p_m->cfg.odom.track_mm * PI_NUM * UM_PER_MM)
                               / ((int64_t)PI_DEN * MDEG_360);

        p_m->state  = MOTION_TURN;
        p_m->goal   = p_cmd->u.turn.angle_mdeg;
        p_m->speed  = p_cmd->u.turn.speed_mm_s;
        expected_us = (arc_um * UM_PER_MM) / p_m->speed;
    }

    int64_t limit = (expected_us * p_m->cfg.timeout_factor)
                    + ((int64_t)p_m->cfg.timeout_extra_ms * US_PER_MS);

    if (limit > (int64_t)LIMIT_US_MAX)
    {
        limit = (int64_t)LIMIT_US_MAX;
    }

    p_m->limit_us = (uint32_t)limit;
}

static bool
params_valid (motion_t const * p_m, s2_motion_cmd_t const * p_cmd)
{
    int32_t const max = p_m->cfg.speed_max_mm_s;
    bool          b_ok;

    if (S2_CMD_SPEED == p_cmd->cmd)
    {
        b_ok = (abs32(p_cmd->u.speed.left_mm_s) <= max)
               && (abs32(p_cmd->u.speed.right_mm_s) <= max);
    }
    else if (S2_CMD_MOVE == p_cmd->cmd)
    {
        b_ok = (0 != p_cmd->u.move.distance_mm)
               && (p_cmd->u.move.speed_mm_s > 0)
               && (p_cmd->u.move.speed_mm_s <= max);
    }
    else
    {
        b_ok = (0 != p_cmd->u.turn.angle_mdeg)
               && (abs32(p_cmd->u.turn.angle_mdeg) <= MOTION_TURN_MAX_MDEG)
               && (p_cmd->u.turn.speed_mm_s > 0)
               && (p_cmd->u.turn.speed_mm_s <= max);
    }

    return (b_ok);
}

/* Serial-number comparison: newer if ahead by 1..32767 (wraps). */
static bool
id_newer (uint16_t id, uint16_t last)
{
    uint16_t const d = (uint16_t)(id - last);

    return ((0u != d) && (d < ID_HALF_RANGE));
}

static bool
is_moving (motion_state_t state)
{
    return ((MOTION_SPEED == state) || (MOTION_MOVE == state)
            || (MOTION_TURN == state));
}

/* Update counts, odometry, wheel speeds, stillness and encoder health. */
static void
measure (motion_t * p_m, motion_input_t const * p_in, motion_output_t * p_out)
{
    bool const b_can_fault = (MOTION_INIT != p_m->state)
                             && (MOTION_FAULT != p_m->state);

    if (!p_in->b_counts_valid)
    {
        p_m->b_valid     = false;
        p_m->b_still     = false;
        p_m->b_still_run = false;
        p_m->invalid_count++;

        if (b_can_fault && (p_m->invalid_count > p_m->cfg.enc_invalid_max))
        {
            enter_fault(p_m, MOTION_FAULT_ENCODER, p_out);
        }
        return;
    }

    p_m->invalid_count = 0u;
    p_m->b_valid       = true;

    if (p_m->b_have_prev)
    {
        int32_t const  dl = odom_count_delta(p_in->count_l, p_m->prev_l);
        int32_t const  dr = odom_count_delta(p_in->count_r, p_m->prev_r);
        uint32_t const new_errors = (p_in->errors_l - p_m->prev_err_l)
                                    + (p_in->errors_r - p_m->prev_err_r);

        p_m->tot_l += dl;
        p_m->tot_r += dr;
        integrate_pose(p_m, dl, dr);

        if (b_can_fault && (new_errors > p_m->cfg.enc_errors_max))
        {
            enter_fault(p_m, MOTION_FAULT_ENCODER, p_out);
        }
    }

    p_m->prev_l      = p_in->count_l;
    p_m->prev_r      = p_in->count_r;
    p_m->prev_err_l  = p_in->errors_l;
    p_m->prev_err_r  = p_in->errors_r;
    p_m->b_have_prev = true;

    /* Speed over the last speed_window periods. */
    uint32_t const size = p_m->cfg.speed_window + 1u;

    p_m->win_l[p_m->win_head]  = p_in->count_l;
    p_m->win_r[p_m->win_head]  = p_in->count_r;
    p_m->win_us[p_m->win_head] = p_in->now_us;
    p_m->win_head = (p_m->win_head + 1u) % size;

    if (p_m->win_fill < size)
    {
        p_m->win_fill++;
    }

    uint32_t const oldest = (p_m->win_head + size - p_m->win_fill) % size;
    uint32_t const dt_us  = p_in->now_us - p_m->win_us[oldest];
    int32_t        sl     = 0;
    int32_t        sr     = 0;

    if ((p_m->win_fill < 2u)
        || (!odom_speed_mm_s(&p_m->cfg.odom,
                             odom_count_delta(p_in->count_l, p_m->win_l[oldest]),
                             dt_us, &sl))
        || (!odom_speed_mm_s(&p_m->cfg.odom,
                             odom_count_delta(p_in->count_r, p_m->win_r[oldest]),
                             dt_us, &sr)))
    {
        sl = 0;
        sr = 0;
    }

    p_m->speed_l = sl;
    p_m->speed_r = sr;

    if (((uint32_t)abs32(sl) <= p_m->cfg.still_mm_s)
        && ((uint32_t)abs32(sr) <= p_m->cfg.still_mm_s))
    {
        if (!p_m->b_still_run)
        {
            p_m->b_still_run    = true;
            p_m->still_since_us = p_in->now_us;
        }

        p_m->b_still = ((p_in->now_us - p_m->still_since_us)
                        >= (p_m->cfg.still_ms * US_PER_MS));
    }
    else
    {
        p_m->b_still_run = false;
        p_m->b_still     = false;
    }
}

/* Dead reckoning: heading from the run's wheel difference, position
   advanced along the mid-step heading. */
static void
integrate_pose (motion_t * p_m, int32_t dl, int32_t dr)
{
    int32_t h_new = 0;

    if (!odom_turn_mdeg(&p_m->cfg.odom,
                        counts_mm(p_m, p_m->tot_l - p_m->run_l0),
                        counts_mm(p_m, p_m->tot_r - p_m->run_r0), &h_new))
    {
        return;
    }

    h_new = wrap_mdeg(h_new);

    int32_t const h_mid = wrap_mdeg((int64_t)p_m->heading_mdeg
                                    + (wrap_mdeg((int64_t)h_new
                                                 - p_m->heading_mdeg) / 2));
    int64_t const d_um  = (((int64_t)dl + dr) * p_m->cfg.odom.wheel_circ_um)
                          / ((int64_t)2 * p_m->cfg.odom.counts_per_rev);

    p_m->x_um += (int32_t)((d_um * sin_q15(h_mid + MDEG_90)) / Q15_ONE);
    p_m->y_um += (int32_t)((d_um * sin_q15(h_mid)) / Q15_ONE);
    p_m->heading_mdeg = h_new;
}

/* Per-state outputs, target checks, timeout and stall. */
static void
control (motion_t * p_m, motion_input_t const * p_in, motion_output_t * p_out)
{
    uint32_t const now = p_in->now_us;

    if ((MOTION_MOVE == p_m->state) || (MOTION_TURN == p_m->state))
    {
        int32_t const ml = mm_since(p_m, p_in->count_l, p_m->start_l);
        int32_t const mr = mm_since(p_m, p_in->count_r, p_m->start_r);
        int32_t const sign = (p_m->goal > 0) ? 1 : -1;
        int32_t       done_at;
        int32_t       progress = 0;

        if (MOTION_MOVE == p_m->state)
        {
            /* Setpoint 0 difference: left ahead gives a negative output,
               slowing left and speeding right, in either direction. */
            int32_t corr = 0;

            if (p_in->b_counts_valid)
            {
                (void)pid_update(&p_m->pid_diff, 0, clamp32(ml - mr,
                                 -PID_INPUT_MAX, PID_INPUT_MAX), &corr);
            }

            corr       = clamp32(corr, -p_m->speed, p_m->speed);
            progress   = (int32_t)(((int64_t)ml + mr) / 2);
            done_at    = abs32(p_m->goal) - (int32_t)p_m->cfg.move_tol_mm;
            p_m->tgt_l = (sign * p_m->speed) + corr;
            p_m->tgt_r = (sign * p_m->speed) - corr;
        }
        else
        {
            (void)odom_turn_mdeg(&p_m->cfg.odom, ml, mr, &progress);
            done_at    = abs32(p_m->goal) - (int32_t)p_m->cfg.turn_tol_mdeg;
            p_m->tgt_l = -sign * p_m->speed;
            p_m->tgt_r = sign * p_m->speed;
        }

        if (p_in->b_counts_valid && ((sign * progress) >= done_at))
        {
            enter_stopping(p_m, MOTION_EVT_DONE, p_m->cmd_id, now);
        }
        else if ((now - p_m->start_us) > p_m->limit_us)
        {
            enter_fault(p_m, MOTION_FAULT_TIMEOUT, p_out);
        }
        else
        {
            /* Still travelling. */
        }
    }

    if (is_moving(p_m->state))
    {
        /* Hold the last duty through a bad sample (fault after a few). */
        if (p_in->b_counts_valid)
        {
            int32_t dl = p_m->duty_l;
            int32_t dr = p_m->duty_r;

            (void)pid_update(&p_m->pid_l, p_m->tgt_l, p_m->speed_l, &dl);
            (void)pid_update(&p_m->pid_r, p_m->tgt_r, p_m->speed_r, &dr);
            p_m->duty_l = motion_ramp(p_m->duty_l, dl,
                                      p_m->cfg.duty_step_max);
            p_m->duty_r = motion_ramp(p_m->duty_r, dr,
                                      p_m->cfg.duty_step_max);
            check_stall(p_m, now, p_out);   /* on the ramped duty */
        }
    }
    else if (MOTION_STOPPING == p_m->state)
    {
        if (p_m->b_still)
        {
            p_m->state = MOTION_IDLE;
            emit(p_m, p_out, p_m->pending_type, 0u, p_m->pending_id);
        }
        else if ((now - p_m->stop_start_us)
                 > (p_m->cfg.stop_limit_ms * US_PER_MS))
        {
            enter_fault(p_m, MOTION_FAULT_STOP_TIMEOUT, p_out);
        }
        else
        {
            /* Braking, waiting for stillness. */
        }
    }
    else
    {
        /* Init, Idle, Fault: no drive. */
    }

    if (!is_moving(p_m->state))
    {
        p_m->duty_l = 0;
        p_m->duty_r = 0;
    }

    p_out->duty_l = p_m->duty_l;
    p_out->duty_r = p_m->duty_r;
}

/* A wheel driven hard that does not move for stall_ms has stalled. */
static void
check_stall (motion_t * p_m, uint32_t now_us, motion_output_t * p_out)
{
    bool const b_stuck_l = (abs32(p_m->duty_l) >= p_m->cfg.stall_duty)
                           && ((uint32_t)abs32(p_m->speed_l)
                               <= p_m->cfg.still_mm_s);
    bool const b_stuck_r = (abs32(p_m->duty_r) >= p_m->cfg.stall_duty)
                           && ((uint32_t)abs32(p_m->speed_r)
                               <= p_m->cfg.still_mm_s);
    uint32_t const limit = p_m->cfg.stall_ms * US_PER_MS;

    if (b_stuck_l && (!p_m->b_stall_l))
    {
        p_m->stall_since_l = now_us;
    }

    if (b_stuck_r && (!p_m->b_stall_r))
    {
        p_m->stall_since_r = now_us;
    }

    p_m->b_stall_l = b_stuck_l;
    p_m->b_stall_r = b_stuck_r;

    if ((b_stuck_l && ((now_us - p_m->stall_since_l) >= limit))
        || (b_stuck_r && ((now_us - p_m->stall_since_r) >= limit)))
    {
        enter_fault(p_m, MOTION_FAULT_STALL, p_out);
    }
}

static int32_t
mm_since (motion_t const * p_m, uint32_t now, uint32_t start)
{
    return (counts_mm(p_m, odom_count_delta(now, start)));
}

/* Counts to mm; saturates instead of failing (limits make it unlikely). */
static int32_t
counts_mm (motion_t const * p_m, int32_t counts)
{
    int32_t mm = 0;

    if (!odom_counts_to_mm(&p_m->cfg.odom, counts, &mm))
    {
        mm = (counts < 0) ? INT32_MIN : INT32_MAX;
    }

    return (mm);
}

static int32_t
abs32 (int32_t v)
{
    return ((v < 0) ? ((INT32_MIN == v) ? INT32_MAX : -v) : v);
}

static int32_t
clamp32 (int32_t v, int32_t lo, int32_t hi)
{
    return ((v < lo) ? lo : ((v > hi) ? hi : v));
}

/* Wrap to -180000 < a <= 180000. */
static int32_t
wrap_mdeg (int64_t a)
{
    int64_t r = a % MDEG_360;

    if (r > MDEG_180)
    {
        r -= MDEG_360;
    }
    else if (r <= -MDEG_180)
    {
        r += MDEG_360;
    }
    else
    {
        /* In range. */
    }

    return ((int32_t)r);
}

/* sin(mdeg) x 32767 by quarter-wave table and linear interpolation. */
static int32_t
sin_q15 (int32_t mdeg)
{
    int32_t a = (int32_t)(((int64_t)mdeg % MDEG_360 + MDEG_360) % MDEG_360);
    int32_t sign = 1;

    if (a >= MDEG_180)
    {
        a -= MDEG_180;
        sign = -1;
    }

    if (a > MDEG_90)
    {
        a = MDEG_180 - a;
    }

    int32_t const idx  = a / MDEG_PER_DEG;
    int32_t const frac = a % MDEG_PER_DEG;
    int32_t       s    = g_sin_q15[idx];

    if (idx < MDEG_90 / MDEG_PER_DEG)
    {
        s += ((g_sin_q15[idx + 1] - g_sin_q15[idx]) * frac) / MDEG_PER_DEG;
    }

    return (sign * s);
}

/* End of file. */
