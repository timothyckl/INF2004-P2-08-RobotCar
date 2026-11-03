/**
 * @file test_motion.c
 * @brief Motion Control unit cases S2-U20..S2-U35 with a synthetic wheel
 *        model.
 *
 * Model: each wheel's speed (mm/s) = duty x 500 / DRV_MOTOR_DUTY_MAX,
 * instantly, unless the case weakens a wheel, makes it stuck, caps its
 * speed or forces a speed. Geometry and gains are synthetic, not the robot's.
 * Steps are 10 ms, the planned control period.
 */
#include "test_motion.h"

#if APP_TEST

#include "motion_logic.h"
#include "motion_config.h"
#include "drv_motor.h"

#define SIM_DT_US       10000u
#define SIM_V_FULL      500         /* mm/s at full duty */
#define SIM_LOG_MAX     64u
#define SIM_CPR         1000u
#define SIM_CIRC_UM     200000u     /* 0.2 mm per count */

typedef struct
{
    motion_event_t ev;
    uint32_t       t_us;
} sim_log_t;

typedef struct
{
    motion_t        m;
    motion_output_t out;
    motion_input_t  in;
    int64_t         pos_um_l;
    int64_t         pos_um_r;
    bool            b_stuck_l;
    int32_t         left_pct;       /* left wheel strength, 100 = normal */
    bool            b_force;
    int32_t         force_mm_s;
    int32_t         cap_mm_s;       /* 0 = no cap */
    sim_log_t       log[SIM_LOG_MAX];
    uint32_t        n_log;
    uint16_t        next_id;
} sim_t;

/* One shared instance keeps the large state off the test task's stack. */
static sim_t g_sim;

static motion_cfg_t const g_cfg =
{
    {SIM_CPR, SIM_CIRC_UM, 150u},           /* odom */
    {6400, 1280, 0, -DRV_MOTOR_DUTY_MAX, DRV_MOTOR_DUTY_MAX},
    {2048, 102, 0, -400, 400},              /* straight_pid */
    400,        /* speed_max_mm_s */
    10u,        /* still_mm_s */
    100u,       /* still_ms */
    2000u,      /* stop_limit_ms */
    (DRV_MOTOR_DUTY_MAX * 9) / 10,
    500u,       /* stall_ms */
    2u,         /* timeout_factor */
    1000u,      /* timeout_extra_ms */
    5u,         /* move_tol_mm */
    2000u,      /* turn_tol_mdeg */
    2u,         /* enc_errors_max */
    3u,         /* enc_invalid_max */
    2u,         /* speed_window */
    DRV_MOTOR_DUTY_MAX / 20     /* duty_step_max, as MC_DUTY_STEP_MAX */
};

static void
sim_record (sim_t * p_s)
{
    for (uint32_t i = 0u; i < p_s->out.n_events; i++)
    {
        if (p_s->n_log < SIM_LOG_MAX)
        {
            p_s->log[p_s->n_log].ev   = p_s->out.events[i];
            p_s->log[p_s->n_log].t_us = p_s->in.now_us;
            p_s->n_log++;
        }
    }
}

static void
sim_counts (sim_t * p_s)
{
    p_s->in.count_l = (uint32_t)((p_s->pos_um_l * SIM_CPR) / SIM_CIRC_UM);
    p_s->in.count_r = (uint32_t)((p_s->pos_um_r * SIM_CPR) / SIM_CIRC_UM);
}

/* A fresh model; b_start false leaves Motion Control in Init. */
static sim_t *
sim_fresh (void)
{
    sim_t * p_s = &g_sim;
    sim_t const zero = {0};

    *p_s = zero;
    p_s->left_pct = 100;
    p_s->next_id = 1u;
    p_s->in.b_counts_valid = true;
    p_s->in.now_us = 1000000u;
    (void)motion_init(&p_s->m, &g_cfg);

    return (p_s);
}

static sim_t *
sim_new (bool b_start)
{
    sim_t * p_s = &g_sim;
    sim_t const zero = {0};

    *p_s = zero;
    p_s->left_pct = 100;
    p_s->next_id = 1u;
    p_s->in.b_counts_valid = true;
    p_s->in.now_us = 1000000u;
    (void)motion_init(&p_s->m, &g_cfg);
    motion_output_clear(&p_s->out);
    motion_start(&p_s->m, b_start, &p_s->out);
    sim_record(p_s);

    return (p_s);
}

static int32_t
sim_wheel (sim_t const * p_s, int32_t duty, bool b_stuck)
{
    int32_t v = (duty * SIM_V_FULL) / DRV_MOTOR_DUTY_MAX;

    if (b_stuck)
    {
        v = 0;
    }
    else if (p_s->b_force)
    {
        v = p_s->force_mm_s;
    }
    else if ((p_s->cap_mm_s > 0) && (v > p_s->cap_mm_s))
    {
        v = p_s->cap_mm_s;
    }
    else if ((p_s->cap_mm_s > 0) && (v < -p_s->cap_mm_s))
    {
        v = -p_s->cap_mm_s;
    }
    else
    {
        /* Free wheel. */
    }

    return (v);
}

/* One 10 ms period: measure, control, then move the wheels. */
static void
sim_step (sim_t * p_s)
{
    sim_counts(p_s);
    motion_output_clear(&p_s->out);
    motion_step(&p_s->m, &p_s->in, &p_s->out);
    sim_record(p_s);

    int32_t const vl = (sim_wheel(p_s, p_s->out.duty_l, p_s->b_stuck_l)
                        * p_s->left_pct) / 100;
    int32_t const vr = sim_wheel(p_s, p_s->out.duty_r, false);

    p_s->pos_um_l += ((int64_t)vl * SIM_DT_US) / 1000;
    p_s->pos_um_r += ((int64_t)vr * SIM_DT_US) / 1000;
    p_s->in.now_us += SIM_DT_US;
}

static void
sim_run (sim_t * p_s, uint32_t steps)
{
    for (uint32_t i = 0u; i < steps; i++)
    {
        sim_step(p_s);
    }
}

static uint16_t
sim_cmd (sim_t * p_s, uint8_t cmd, int32_t a, int32_t b)
{
    s2_motion_cmd_t c = {0};

    c.cmd    = cmd;
    c.cmd_id = p_s->next_id;
    p_s->next_id++;

    if (S2_CMD_SPEED == cmd)
    {
        c.u.speed.left_mm_s  = (int16_t)a;
        c.u.speed.right_mm_s = (int16_t)b;
    }
    else if (S2_CMD_MOVE == cmd)
    {
        c.u.move.distance_mm = (int16_t)a;
        c.u.move.speed_mm_s  = (int16_t)b;
    }
    else if (S2_CMD_TURN == cmd)
    {
        c.u.turn.angle_mdeg = a;
        c.u.turn.speed_mm_s = (int16_t)b;
    }
    else
    {
        /* STOP, RESET, NEW_RUN carry no payload. */
    }

    sim_counts(p_s);
    motion_output_clear(&p_s->out);
    motion_command(&p_s->m, &c, &p_s->in, &p_s->out);
    sim_record(p_s);

    return (c.cmd_id);
}

/* Index of the first logged event of this type (and id, if not 0), or -1. */
static int32_t
sim_find (sim_t const * p_s, uint8_t type, uint16_t id)
{
    for (uint32_t i = 0u; i < p_s->n_log; i++)
    {
        if ((p_s->log[i].ev.type == type)
            && ((0u == id) || (p_s->log[i].ev.cmd_id == id)))
        {
            return ((int32_t)i);
        }
    }

    return (-1);
}

/* Run until an event of this type (and id, if not 0) is logged. */
static int32_t
sim_until (sim_t * p_s, uint8_t type, uint16_t id, uint32_t max)
{
    for (uint32_t i = 0u; (i < max) && (sim_find(p_s, type, id) < 0); i++)
    {
        sim_step(p_s);
    }

    return (sim_find(p_s, type, id));
}

static bool
sim_rejected (sim_t const * p_s, uint16_t id, uint8_t reason)
{
    int32_t const i = sim_find(p_s, MOTION_EVT_REJECTED, id);

    return ((i >= 0) && (p_s->log[i].ev.reason == reason));
}

static bool
sim_fault (sim_t const * p_s, uint8_t reason)
{
    int32_t const i = sim_find(p_s, MOTION_EVT_FAULT, 0u);

    return ((i >= 0) && (p_s->log[i].ev.reason == reason)
            && (MOTION_FAULT == motion_state(&p_s->m)));
}

static int32_t
abs_i (int32_t v)
{
    return ((v < 0) ? -v : v);
}

/* ------------------------------------------------------------------------ */

bool
test_motion_start (void)
{
    sim_t * p_s = sim_new(true);
    bool const b_ok = (sim_find(p_s, MOTION_EVT_READY, 0u) == 0)
                      && (MOTION_IDLE == motion_state(&p_s->m));

    p_s = sim_new(false);
    bool const b_flag = p_s->out.b_raise_fault;

    sim_step(p_s);

    return (b_ok && b_flag && sim_fault(p_s, MOTION_FAULT_INIT)
            && (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r));
}

bool
test_motion_speed (void)
{
    sim_t *    p_s = sim_new(true);
    motion_status_t  o;

    (void)sim_cmd(p_s, S2_CMD_SPEED, 200, 100);
    sim_step(p_s);

    bool const b_sign = (p_s->out.duty_l > 0) && (p_s->out.duty_r > 0);

    sim_run(p_s, 100);
    motion_get_status_logic(&p_s->m, &o);

    return (b_sign && (MOTION_SPEED == motion_state(&p_s->m))
            && (abs_i(o.left_mm_s - 200) <= 20)
            && (abs_i(o.right_mm_s - 100) <= 20));
}

bool
test_motion_move (void)
{
    sim_t * p_s = sim_new(true);
    uint32_t t_stop = 0u;

    (void)sim_cmd(p_s, S2_CMD_MOVE, 300, 200);

    for (uint32_t i = 0u; (i < 400u) && (0u == t_stop); i++)
    {
        sim_step(p_s);

        if (MOTION_STOPPING == motion_state(&p_s->m))
        {
            t_stop = p_s->in.now_us - SIM_DT_US;
        }
    }

    int32_t const d = sim_until(p_s, MOTION_EVT_DONE, 0u, 100u);
    int64_t const mm = (p_s->pos_um_l + p_s->pos_um_r) / 2000;

    return ((t_stop > 0u) && (d >= 0)
            && ((p_s->log[d].t_us - t_stop) >= 100000u)
            && (mm >= 295) && (mm <= 330)
            && (MOTION_IDLE == motion_state(&p_s->m)));
}

bool
test_motion_turn (void)
{
    sim_t *   p_s = sim_new(true);
    motion_status_t o;

    (void)sim_cmd(p_s, S2_CMD_TURN, 90000, 150);
    sim_step(p_s);

    bool const b_sign = (p_s->out.duty_l < 0) && (p_s->out.duty_r > 0);
    int32_t const d = sim_until(p_s, MOTION_EVT_DONE, 0u, 500u);

    motion_get_status_logic(&p_s->m, &o);

    return (b_sign && (d >= 0) && (o.heading_mdeg > 0)
            && (abs_i(o.heading_mdeg - 90000) <= 5000));
}

/* STOP in one motion command; true if duty 0 at once and STOPPED later. */
static bool
stop_during (uint8_t cmd, int32_t a, int32_t b)
{
    sim_t *        p_s = sim_new(true);
    uint16_t const id  = sim_cmd(p_s, cmd, a, b);

    sim_run(p_s, 20);
    p_s->in.b_stop = true;
    sim_step(p_s);

    bool const     b_zero = (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r);
    uint32_t const t_stop = p_s->in.now_us - SIM_DT_US;
    int32_t const  s      = sim_until(p_s, MOTION_EVT_STOPPED, 0u, 300u);

    return (b_zero && (s >= 0) && (p_s->log[s].ev.cmd_id == id)
            && ((p_s->log[s].t_us - t_stop) >= 100000u)
            && (MOTION_IDLE == motion_state(&p_s->m)));
}

static bool
stop_motion_states (void)
{
    return (stop_during(S2_CMD_SPEED, 200, 200)
            && stop_during(S2_CMD_MOVE, 2000, 200)
            && stop_during(S2_CMD_TURN, 180000, 150));
}

static bool
stop_idle_and_fault (void)
{
    sim_t * p_s = sim_new(true);

    sim_run(p_s, 20);
    p_s->in.b_stop = true;

    bool const b_idle = (sim_until(p_s, MOTION_EVT_STOPPED, 0u, 50u) >= 0);

    p_s = sim_new(true);
    motion_fault(&p_s->m, MOTION_FAULT_EXTERNAL, &p_s->out);
    p_s->in.b_stop = true;
    sim_run(p_s, 50);

    return (b_idle && (sim_find(p_s, MOTION_EVT_STOPPED, 0u) < 0)
            && (MOTION_FAULT == motion_state(&p_s->m)));
}

bool
test_motion_reject_stop (void)
{
    sim_t * p_s = sim_new(true);

    p_s->in.b_stop = true;
    sim_run(p_s, 20);

    uint16_t const a = sim_cmd(p_s, S2_CMD_SPEED, 100, 100);
    uint16_t const b = sim_cmd(p_s, S2_CMD_MOVE, 100, 100);
    uint16_t const c = sim_cmd(p_s, S2_CMD_TURN, 90000, 100);

    sim_step(p_s);

    return (sim_rejected(p_s, a, MOTION_REJ_STOP)
            && sim_rejected(p_s, b, MOTION_REJ_STOP)
            && sim_rejected(p_s, c, MOTION_REJ_STOP)
            && (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r));
}

static bool
stale_ids (void)
{
    sim_t *        p_s = sim_new(true);
    uint16_t const a   = sim_cmd(p_s, S2_CMD_SPEED, 100, 100);

    sim_run(p_s, 5);
    p_s->next_id = a;           /* reuse the same ID */

    uint16_t const dup = sim_cmd(p_s, S2_CMD_MOVE, 100, 100);

    p_s->next_id = (uint16_t)(a - 1u);  /* an older ID */

    uint16_t const old = sim_cmd(p_s, S2_CMD_MOVE, 100, 100);

    p_s->next_id = (uint16_t)(a + 1u);
    p_s->in.b_stop = true;
    (void)sim_until(p_s, MOTION_EVT_STOPPED, 0u, 300u);
    p_s->in.b_stop = false;
    sim_step(p_s);
    p_s->next_id = (uint16_t)(a - 2u);  /* queued before the STOP */

    uint16_t const late = sim_cmd(p_s, S2_CMD_SPEED, 100, 100);

    p_s->next_id = (uint16_t)(a + 5u);

    uint16_t const fresh = sim_cmd(p_s, S2_CMD_SPEED, 100, 100);

    return ((dup == a) && sim_rejected(p_s, dup, MOTION_REJ_STALE)
            && sim_rejected(p_s, old, MOTION_REJ_STALE)
            && sim_rejected(p_s, late, MOTION_REJ_STALE)
            && (sim_find(p_s, MOTION_EVT_REJECTED, fresh) < 0)
            && (MOTION_SPEED == motion_state(&p_s->m)));
}

static bool
stop_timeout (void)
{
    sim_t * p_s = sim_new(true);

    (void)sim_cmd(p_s, S2_CMD_SPEED, 100, 100);
    sim_run(p_s, 10);
    p_s->b_force    = true;     /* wheels keep turning while braked */
    p_s->force_mm_s = 100;
    (void)sim_cmd(p_s, S2_CMD_STOP, 0, 0);
    sim_run(p_s, 190);          /* 1.9 s */

    bool const b_waiting = (MOTION_STOPPING == motion_state(&p_s->m))
                           && (sim_find(p_s, MOTION_EVT_STOPPED, 0u) < 0);

    sim_run(p_s, 20);           /* past 2 s */

    return (b_waiting && sim_fault(p_s, MOTION_FAULT_STOP_TIMEOUT)
            && (sim_find(p_s, MOTION_EVT_STOPPED, 0u) < 0));
}

bool
test_motion_stall (void)
{
    sim_t * p_s = sim_new(true);

    p_s->b_stuck_l = true;

    uint32_t const t0 = p_s->in.now_us;

    (void)sim_cmd(p_s, S2_CMD_SPEED, 300, 300);
    sim_run(p_s, 45);           /* under 500 ms */

    bool const b_early = (sim_find(p_s, MOTION_EVT_FAULT, 0u) < 0);
    int32_t const f = sim_until(p_s, MOTION_EVT_FAULT, 0u, 150u);

    sim_step(p_s);

    return (b_early && (f >= 0) && sim_fault(p_s, MOTION_FAULT_STALL)
            && ((p_s->log[f].t_us - t0) >= 500000u)
            && (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r));
}

bool
test_motion_timeout (void)
{
    sim_t * p_s = sim_new(true);

    p_s->cap_mm_s = 50;         /* moving, but far slower than asked */
    (void)sim_cmd(p_s, S2_CMD_MOVE, 1000, 200);
    (void)sim_until(p_s, MOTION_EVT_FAULT, 0u, 1300u);

    return (sim_fault(p_s, MOTION_FAULT_TIMEOUT));
}

bool
test_motion_encoder_fault (void)
{
    sim_t * p_s = sim_new(true);

    sim_run(p_s, 5);
    p_s->in.b_counts_valid = false;
    sim_run(p_s, 4);

    bool const b_invalid = sim_fault(p_s, MOTION_FAULT_ENCODER);

    p_s = sim_new(true);
    sim_run(p_s, 5);
    p_s->in.errors_l += 3u;
    sim_step(p_s);

    return (b_invalid && sim_fault(p_s, MOTION_FAULT_ENCODER));
}

bool
test_motion_recovery (void)
{
    sim_t * p_s = sim_new(true);

    motion_fault(&p_s->m, MOTION_FAULT_EXTERNAL, &p_s->out);

    uint16_t const a = sim_cmd(p_s, S2_CMD_SPEED, 100, 100);

    p_s->b_force    = true;
    p_s->force_mm_s = 100;
    sim_run(p_s, 20);

    uint16_t const r1 = sim_cmd(p_s, S2_CMD_RESET, 0, 0);

    p_s->b_force = false;
    sim_run(p_s, 20);

    uint16_t const r2 = sim_cmd(p_s, S2_CMD_RESET, 0, 0);
    bool const b_idle = (MOTION_IDLE == motion_state(&p_s->m));

    (void)sim_cmd(p_s, S2_CMD_SPEED, 100, 100);

    return (sim_rejected(p_s, a, MOTION_FAULT_EXTERNAL)
            && sim_rejected(p_s, r1, MOTION_FAULT_EXTERNAL)
            && (sim_find(p_s, MOTION_EVT_DONE, r2) >= 0) && b_idle
            && (MOTION_SPEED == motion_state(&p_s->m)));
}

bool
test_motion_params (void)
{
    sim_t * p_s = sim_new(true);
    uint16_t const a = sim_cmd(p_s, S2_CMD_SPEED, 401, 0);
    uint16_t const b = sim_cmd(p_s, S2_CMD_MOVE, 0, 100);
    uint16_t const c = sim_cmd(p_s, S2_CMD_MOVE, 100, 0);
    uint16_t const d = sim_cmd(p_s, S2_CMD_TURN, 0, 100);
    uint16_t const e = sim_cmd(p_s, S2_CMD_TURN, 360001, 100);
    uint16_t const f = sim_cmd(p_s, S2_CMD_TURN, 90000, 500);

    return (sim_rejected(p_s, a, MOTION_REJ_PARAM)
            && sim_rejected(p_s, b, MOTION_REJ_PARAM)
            && sim_rejected(p_s, c, MOTION_REJ_PARAM)
            && sim_rejected(p_s, d, MOTION_REJ_PARAM)
            && sim_rejected(p_s, e, MOTION_REJ_PARAM)
            && sim_rejected(p_s, f, MOTION_REJ_PARAM)
            && (MOTION_IDLE == motion_state(&p_s->m)));
}

bool
test_motion_odometry (void)
{
    sim_t *   p_s = sim_new(true);
    motion_status_t o;

    sim_run(p_s, 5);

    uint16_t const n1 = sim_cmd(p_s, S2_CMD_NEW_RUN, 0, 0);

    uint16_t const mv = sim_cmd(p_s, S2_CMD_MOVE, 500, 200);

    (void)sim_until(p_s, MOTION_EVT_DONE, mv, 600u);
    motion_get_status_logic(&p_s->m, &o);

    bool const b_line = (o.run_id == 1u)
                        && (sim_find(p_s, MOTION_EVT_DONE, n1) >= 0)
                        && (abs_i(o.travel_mm - 500) <= 25)
                        && (abs_i(o.x_mm - 500) <= 25)
                        && (abs_i(o.y_mm) <= 10)
                        && (abs_i(o.heading_mdeg) <= 2000)
                        && (0u != (o.flags & MOTION_ST_VALID))
                        && (0u != (o.flags & MOTION_ST_STOPPED));

    (void)sim_cmd(p_s, S2_CMD_NEW_RUN, 0, 0);
    motion_get_status_logic(&p_s->m, &o);

    return (b_line && (o.run_id == 2u) && (0 == o.x_mm)
            && (0 == o.y_mm) && (0 == o.heading_mdeg)
            && (abs_i(o.travel_mm - 500) <= 25));
}

/* STOP in Init: nothing moves, no confirmation, still Init. */
static bool
stop_in_init (void)
{
    sim_t * p_s = sim_fresh();

    p_s->in.b_stop = true;
    sim_run(p_s, 30);

    return ((MOTION_INIT == motion_state(&p_s->m))
            && (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r)
            && (sim_find(p_s, MOTION_EVT_STOPPED, 0u) < 0));
}

/* STOP while already Stopping after a MOVE: it still finishes. */
static bool
stop_in_stopping (void)
{
    sim_t * p_s = sim_new(true);

    (void)sim_cmd(p_s, S2_CMD_MOVE, 50, 200);

    for (uint32_t i = 0u; (i < 200u)
         && (MOTION_STOPPING != motion_state(&p_s->m)); i++)
    {
        sim_step(p_s);
    }

    bool const b_stopping = (MOTION_STOPPING == motion_state(&p_s->m));

    p_s->in.b_stop = true;
    sim_step(p_s);

    bool const b_zero = (0 == p_s->out.duty_l) && (0 == p_s->out.duty_r);

    (void)sim_until(p_s, MOTION_EVT_DONE, 0u, 100u);

    return (b_stopping && b_zero && (MOTION_IDLE == motion_state(&p_s->m)));
}

bool
test_motion_stop_all (void)
{
    return (stop_in_init() && stop_idle_and_fault() && stop_motion_states()
            && stop_in_stopping());
}

bool
test_motion_replace (void)
{
    sim_t * p_s = sim_new(true);

    (void)sim_cmd(p_s, S2_CMD_MOVE, 2000, 200);
    sim_run(p_s, 20);
    (void)sim_cmd(p_s, S2_CMD_TURN, 90000, 150);     /* STOP clear: replaces */

    bool const b_replaced = (MOTION_TURN == motion_state(&p_s->m));

    p_s->in.b_stop = true;
    sim_step(p_s);

    uint16_t const blocked = sim_cmd(p_s, S2_CMD_MOVE, 100, 100);
    bool const b_kept = (MOTION_STOPPING == motion_state(&p_s->m))
                        && sim_rejected(p_s, blocked, MOTION_REJ_STOP);

    return (b_replaced && b_kept && stale_ids());
}

bool
test_motion_stop_timing (void)
{
    sim_t * p_s = sim_new(true);
    bool    b_never_early = true;

    (void)sim_cmd(p_s, S2_CMD_SPEED, 200, 200);
    sim_run(p_s, 50);
    (void)sim_cmd(p_s, S2_CMD_STOP, 0, 0);

    uint32_t const t_stop = p_s->in.now_us;
    int32_t        s      = -1;

    for (uint32_t i = 0u; (i < 100u) && (s < 0); i++)
    {
        sim_step(p_s);
        s = sim_find(p_s, MOTION_EVT_STOPPED, 0u);

        /* The query must not claim a stop before the event can. */
        if ((s < 0) && motion_is_stopped(&p_s->m))
        {
            b_never_early = false;
        }
    }

    return (b_never_early && (s >= 0)
            && ((p_s->log[s].t_us - t_stop) >= 100000u)
            && motion_is_stopped(&p_s->m) && stop_timeout());
}

bool
test_motion_straight (void)
{
    sim_t *         p_s = sim_new(true);
    motion_status_t o;

    /* The left wheel slips for 200 ms (about 40 mm lost) and is weaker
       after: the speed loops alone cannot win back lost distance. */
    p_s->left_pct  = 80;
    p_s->b_stuck_l = true;
    (void)sim_cmd(p_s, S2_CMD_MOVE, 500, 200);
    sim_run(p_s, 20);
    p_s->b_stuck_l = false;
    (void)sim_until(p_s, MOTION_EVT_DONE, 0u, 600u);
    motion_get_status_logic(&p_s->m, &o);

    int64_t const diff_mm = (p_s->pos_um_l - p_s->pos_um_r) / 1000;

    return ((sim_find(p_s, MOTION_EVT_DONE, 0u) >= 0)
            && (diff_mm >= -5) && (diff_mm <= 5)
            && (abs_i(o.heading_mdeg) <= 2000));
}

bool
test_motion_api (void)
{
    s2_motion_cmd_t f, b, l, r, st, keep = {0};

    keep.cmd_id = 77u;

    bool const b_build =
        motion_api_make(S2_CMD_MOVE, 500, 1u, &f)
        && (S2_CMD_MOVE == f.cmd) && (500 == f.u.move.distance_mm)
        && (MC_MOVE_SPEED_MM_S == f.u.move.speed_mm_s) && (1u == f.cmd_id)
        && motion_api_make(S2_CMD_MOVE, -500, 2u, &b)
        && (-500 == b.u.move.distance_mm)
        && motion_api_make(S2_CMD_TURN, 90000, 3u, &l)
        && (S2_CMD_TURN == l.cmd) && (90000 == l.u.turn.angle_mdeg)
        && (MC_TURN_SPEED_MM_S == l.u.turn.speed_mm_s)
        && motion_api_make(S2_CMD_TURN, -90000, 4u, &r)
        && (-90000 == r.u.turn.angle_mdeg)
        && motion_api_make(S2_CMD_STOP, 0, 5u, &st) && (S2_CMD_STOP == st.cmd)
        && motion_api_make(S2_CMD_RESET, 0, 6u, &st)
        && motion_api_make(S2_CMD_NEW_RUN, 0, 7u, &st);

    bool const b_refuse =
        (!motion_api_make(S2_CMD_MOVE, 0, 8u, &keep))
        && (!motion_api_make(S2_CMD_MOVE, 32768, 8u, &keep))
        && (!motion_api_make(S2_CMD_TURN, 0, 8u, &keep))
        && (!motion_api_make(S2_CMD_TURN, 360001, 8u, &keep))
        && (!motion_api_make(S2_CMD_SPEED, 0, 8u, &keep))
        && (!motion_api_make(S2_CMD_MOVE, 100, 8u, NULL))
        && (77u == keep.cmd_id);

    bool const b_public =
        (E_PAR == motion_move_forward(0u))
        && (E_PAR == motion_move_backward(32768u))
        && (E_PAR == motion_turn_left(0u))
        && (E_PAR == motion_turn_right(360001u));

    return (b_build && b_refuse && b_public);
}

bool
test_motion_ramp (void)
{
    int32_t const st = 300;
    int32_t       d  = 0;
    bool          ok = true;

    /* Increase limited per step, both signs. */
    ok = ok && (300 == motion_ramp(0, 6250, st));
    ok = ok && (600 == motion_ramp(300, 6250, st));
    ok = ok && (6250 == motion_ramp(6100, 6250, st));
    ok = ok && (-300 == motion_ramp(0, -6250, st));
    ok = ok && (-1300 == motion_ramp(-1000, -6250, st));

    /* Sign change: 0 first, then ramps from 0. */
    ok = ok && (0 == motion_ramp(2000, -2000, st));
    ok = ok && (0 == motion_ramp(-2000, 2000, st));
    ok = ok && (-300 == motion_ramp(0, -2000, st));

    /* Decrease, STOP and Fault (target 0) are immediate. */
    ok = ok && (500 == motion_ramp(6250, 500, st));
    ok = ok && (-500 == motion_ramp(-6250, -500, st));
    ok = ok && (0 == motion_ramp(6250, 0, st));
    ok = ok && (0 == motion_ramp(-6250, 0, st));

    /* Full scale from 0 takes ceil(6250 / 300) = 21 steps. */
    for (uint32_t i = 0u; i < 20u; i++)
    {
        d = motion_ramp(d, 6250, st);
    }

    ok = ok && (6000 == d) && (6250 == motion_ramp(d, 6250, st));

    return (ok && (6250 == motion_ramp(0, 6250, 0)));   /* 0 = no limit */
}

bool
test_motion_ramp_stall (void)
{
    /* A very slow ramp (1 duty step per 10 ms): the free wheel stays
       below the still speed for about 1.25 s while the PID output is
       already saturated. A stall check on the un-ramped duty would fire
       at 500 ms; on the ramped duty it must not. */
    motion_cfg_t cfg = g_cfg;
    sim_t *      p_s = sim_fresh();

    cfg.duty_step_max = 1;
    (void)motion_init(&p_s->m, &cfg);
    motion_output_clear(&p_s->out);
    motion_start(&p_s->m, true, &p_s->out);
    (void)sim_cmd(p_s, S2_CMD_SPEED, 400, 400);
    sim_run(p_s, 150);                  /* 1.5 s, duty still ramping */

    bool const b_free = (sim_find(p_s, MOTION_EVT_FAULT, 0u) < 0)
                        && (MOTION_SPEED == motion_state(&p_s->m));

    /* Stuck wheel with the normal ramp: no stall before the ramped duty
       reaches the stall level, then a stall MC_STALL_MS after that. */
    p_s = sim_new(true);
    p_s->b_stuck_l = true;
    (void)sim_cmd(p_s, S2_CMD_SPEED, 400, 400);

    uint32_t t_level = 0u;

    for (uint32_t i = 0u; (i < 200u)
         && (sim_find(p_s, MOTION_EVT_FAULT, 0u) < 0); i++)
    {
        sim_step(p_s);

        if ((0u == t_level) && (p_s->out.duty_l >= g_cfg.stall_duty))
        {
            t_level = p_s->in.now_us - SIM_DT_US;
        }
    }

    int32_t const f = sim_find(p_s, MOTION_EVT_FAULT, 0u);

    return (b_free && (t_level > 0u) && (f >= 0)
            && sim_fault(p_s, MOTION_FAULT_STALL)
            && ((p_s->log[f].t_us - t_level) >= (g_cfg.stall_ms * 1000u)));
}

#endif /* APP_TEST */

/* End of file. */
