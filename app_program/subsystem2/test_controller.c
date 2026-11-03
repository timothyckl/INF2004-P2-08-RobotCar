/**
 * @file test_controller.c
 * @brief Vehicle Controller unit cases with injected motion status.
 *
 * The harness models the STOP latch from the outputs (set on latch,
 * cleared on clear) and records every motion action, standing in for
 * task_controller.c. Motion Control results are injected by setting
 * status.last_id / last_result.
 */
#include "test_controller.h"

#if APP_TEST

#include "controller_config.h"
#include "controller_logic.h"
#include "motion_logic.h"

#define H_T0_MS         1000u
#define H_ACT_MAX       16u
#define H_INIT_MS       3000u
#define H_RESULT_MS     20000u
#define H_AGE_MS        2000u

typedef struct
{
    ctrl_t        c;
    ctrl_input_t  in;
    ctrl_output_t out;
    bool          b_stop;       /* modelled safety_flags STOP */
    uint32_t      n_latch;
    uint32_t      n_clear;
    uint32_t      n_drain;
    uint32_t      cleared;      /* controller_flags bits cleared */
    ctrl_action_t act[H_ACT_MAX];
    uint32_t      n_act;
    uint16_t      next_id;
    bool          b_send_ok;
    uint32_t      next_req;
    ctrl_scan_msg_t scan[H_ACT_MAX];  /* scan_q records sent */
    uint32_t      n_scan;
    bool          b_line_auto;  /* refresh line.captured_ms every tick */
} h_t;

static h_t g_h;

static const ctrl_step_t g_script[] =
{
    {CTRL_ACT_FORWARD, 500u},
    {CTRL_ACT_LEFT,    90000u},
    {CTRL_ACT_FORWARD, 500u}
};

static void
h_apply (h_t * p_h)
{
    ctrl_output_t again;

    if (p_h->out.b_latch_stop)
    {
        p_h->b_stop = true;
        p_h->n_latch++;
    }

    if (p_h->out.b_drain_motion)
    {
        p_h->n_drain++;
    }

    if (p_h->out.b_clear_stop)
    {
        p_h->b_stop = false;
        p_h->n_clear++;
    }

    if (CTRL_ACT_NONE != p_h->out.action)
    {
        if (p_h->n_act < H_ACT_MAX)
        {
            p_h->act[p_h->n_act] = p_h->out.action;
            p_h->n_act++;
        }

        p_h->next_id++;
        ctrl_output_clear(&again);
        ctrl_motion_sent(&p_h->c, p_h->b_send_ok, p_h->next_id,
                         p_h->in.now_ms, &again);

        if (again.b_latch_stop)
        {
            p_h->b_stop = true;
            p_h->n_latch++;
        }
    }

    p_h->cleared |= p_h->out.clear_flags;

    for (uint32_t i = 0u; i < p_h->out.n_scan; i++)
    {
        if (p_h->n_scan < H_ACT_MAX)
        {
            p_h->scan[p_h->n_scan] = p_h->out.scan[i];
            p_h->n_scan++;
        }
    }
}

/* Healthy, stopped Motion Control. */
static void
h_motion_ok (h_t * p_h)
{
    p_h->in.b_status_ok  = true;
    p_h->in.status.state = (uint8_t)MOTION_IDLE;
    p_h->in.status.fault = MOTION_FAULT_NONE;
    p_h->in.status.flags = MOTION_ST_VALID | MOTION_ST_STOPPED;
}

static h_t *
h_new (bool b_script)
{
    h_t * p_h = &g_h;
    h_t const zero = {0};
    ctrl_cfg_t       cfg = {H_INIT_MS, H_RESULT_MS, H_AGE_MS,
                            b_script ? g_script : NULL,
                            b_script ? (uint8_t)3u : (uint8_t)0u, {0}};

    ctrl_mission_defaults(&cfg.mission);
    *p_h           = zero;
    p_h->b_stop    = true;      /* safety_flags is created latched */
    p_h->b_send_ok = true;
    p_h->next_req  = 1u;
    p_h->in.now_ms = H_T0_MS;

    /* A fresh, settled, centred line: the design review's START condition
       in mission mode. Cases that test the line override it. */
    p_h->b_line_auto            = true;
    p_h->in.line.b_ok           = true;
    p_h->in.line.captured_ms    = H_T0_MS;
    p_h->in.line.flags          = CTRL_LINE_VALID;
    ctrl_init(&p_h->c, &cfg, H_T0_MS);

    return (p_h);
}

static void
h_tick (h_t * p_h, uint32_t ms)
{
    p_h->in.now_ms += ms;

    if (p_h->b_line_auto)
    {
        p_h->in.line.captured_ms = p_h->in.now_ms;
    }

    ctrl_output_clear(&p_h->out);
    ctrl_tick(&p_h->c, &p_h->in, &p_h->out);
    h_apply(p_h);
}

static void
h_flags (h_t * p_h, uint32_t bits)
{
    ctrl_output_clear(&p_h->out);
    ctrl_flags(&p_h->c, bits, &p_h->in, &p_h->out);
    h_apply(p_h);
}

/* Remote command received age_ms ago with a fresh request ID. */
static uint8_t
h_remote (h_t * p_h, uint8_t command, uint32_t age_ms)
{
    s2_remote_cmd_t const m = {p_h->next_req, p_h->in.now_ms - age_ms,
                               command, {0u, 0u, 0u}};

    p_h->next_req++;
    ctrl_output_clear(&p_h->out);
    ctrl_remote(&p_h->c, &m, &p_h->in, &p_h->out);
    h_apply(p_h);

    return (p_h->c.last_result);
}

/* Motion Control reports a result for the last command sent. */
static void
h_result (h_t * p_h, uint8_t result)
{
    p_h->in.status.last_id     = p_h->next_id;
    p_h->in.status.last_result = result;
}

/* Boot to Ready. */
static h_t *
h_ready (bool b_script)
{
    h_t * p_h = h_new(b_script);

    h_motion_ok(p_h);
    h_tick(p_h, 10u);

    return (p_h);
}

/* Boot to Running (NEW_RUN acknowledged). */
static h_t *
h_running (bool b_script)
{
    h_t * p_h = h_ready(b_script);

    (void)h_remote(p_h, S2_REMOTE_START, 0u);
    h_result(p_h, MOTION_EVT_DONE);

    return (p_h);
}

static bool
h_state (h_t const * p_h, uint8_t state)
{
    return (p_h->c.state == state);
}

/* ------------------------------------------------------------------------ */

bool
test_ctrl_boot (void)
{
    h_t * p_h = h_new(false);

    h_tick(p_h, 10u);   /* motion status not available yet */

    bool const b_wait = h_state(p_h, S2_SYS_INITIALISING) && p_h->b_stop;

    h_motion_ok(p_h);
    h_tick(p_h, 10u);

    return (b_wait && h_state(p_h, S2_SYS_READY) && p_h->b_stop
            && (0u == p_h->n_clear));
}

bool
test_ctrl_init_timeout (void)
{
    h_t * p_h = h_new(false);

    h_tick(p_h, H_INIT_MS);     /* exactly the limit: still waiting */

    bool const b_early = h_state(p_h, S2_SYS_INITIALISING);

    h_tick(p_h, 10u);

    return (b_early && h_state(p_h, S2_SYS_FAULT)
            && (S2_FAULT_INIT == p_h->c.fault) && p_h->b_stop);
}

bool
test_ctrl_start (void)
{
    h_t *   p_h = h_new(false);
    bool    ok  = (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));

    h_motion_ok(p_h);
    h_tick(p_h, 10u);
    p_h->in.status.flags = MOTION_ST_VALID;     /* wheels not confirmed */
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));
    h_motion_ok(p_h);
    p_h->in.status.state = (uint8_t)MOTION_FAULT;   /* motion unhealthy */
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));
    h_motion_ok(p_h);

    uint8_t const r = h_remote(p_h, S2_REMOTE_START, 0u);

    ok = ok && (S2_RESULT_COMPLETED == r) && h_state(p_h, S2_SYS_RUNNING)
         && (!p_h->b_stop) && (1u == p_h->n_drain) && (1u == p_h->n_clear)
         && (1u == p_h->n_act) && (CTRL_ACT_NEW_RUN == p_h->act[0]);

    /* START again while Running, then in Stopped and Fault. */
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));
    h_flags(p_h, CTRL_FLG_LOCAL_STOP);
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));
    h_flags(p_h, CTRL_FLG_MOTION_FAULT);
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, S2_REMOTE_START, 0u));

    return (ok && p_h->b_stop && (1u == p_h->n_clear));
}

bool
test_ctrl_script (void)
{
    h_t * p_h = h_running(true);
    bool  ok  = true;

    for (uint32_t s = 0u; s < 3u; s++)
    {
        h_tick(p_h, 10u);   /* sends step s */
        ok = ok && (p_h->n_act == (2u + s));
        h_tick(p_h, 10u);   /* no result yet: nothing new */
        ok = ok && (p_h->n_act == (2u + s));
        h_result(p_h, MOTION_EVT_DONE);
        h_tick(p_h, 10u);   /* result seen */
    }

    bool const b_order = (CTRL_ACT_FORWARD == p_h->act[1])
                         && (CTRL_ACT_LEFT == p_h->act[2])
                         && (CTRL_ACT_FORWARD == p_h->act[3]);

    h_tick(p_h, 10u);       /* script finished */

    return (ok && b_order && h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop);
}

static bool
stop_bit_in_running (uint32_t bit)
{
    h_t * p_h = h_running(true);

    h_tick(p_h, 10u);       /* first step outstanding */

    uint32_t const acts = p_h->n_act;

    h_flags(p_h, bit);

    bool const b_now = h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop
                       && p_h->out.b_latch_stop && (p_h->cleared == bit);

    h_result(p_h, MOTION_EVT_DONE);
    h_tick(p_h, 10u);
    h_tick(p_h, 10u);

    return (b_now && (p_h->n_act == acts));     /* script halted */
}

bool
test_ctrl_stop_running (void)
{
    return (stop_bit_in_running(CTRL_FLG_REMOTE_STOP)
            && stop_bit_in_running(CTRL_FLG_LOCAL_STOP)
            && stop_bit_in_running(CTRL_FLG_HAZARD));
}

bool
test_ctrl_stop_other (void)
{
    h_t * p_h = h_new(false);
    bool  ok;

    h_flags(p_h, CTRL_FLG_REMOTE_STOP);
    ok = h_state(p_h, S2_SYS_INITIALISING) && p_h->out.b_latch_stop;
    h_motion_ok(p_h);
    h_tick(p_h, 10u);
    h_flags(p_h, CTRL_FLG_LOCAL_STOP);
    ok = ok && h_state(p_h, S2_SYS_READY) && p_h->out.b_latch_stop;
    (void)h_remote(p_h, S2_REMOTE_START, 0u);
    h_flags(p_h, CTRL_FLG_REMOTE_STOP);
    h_flags(p_h, CTRL_FLG_REMOTE_STOP);
    ok = ok && h_state(p_h, S2_SYS_STOPPED) && p_h->out.b_latch_stop;
    h_flags(p_h, CTRL_FLG_MOTION_FAULT);
    h_flags(p_h, CTRL_FLG_LOCAL_STOP);

    return (ok && h_state(p_h, S2_SYS_FAULT) && p_h->out.b_latch_stop
            && p_h->b_stop);
}

static bool
fault_is (h_t const * p_h, uint8_t code)
{
    return (h_state(p_h, S2_SYS_FAULT) && (code == p_h->c.fault)
            && p_h->b_stop);
}

bool
test_ctrl_faults (void)
{
    h_t * p_h = h_running(true);
    bool  ok;

    h_flags(p_h, CTRL_FLG_MOTION_FAULT);
    ok = fault_is(p_h, S2_FAULT_MOTION);
    p_h = h_ready(false);
    h_flags(p_h, CTRL_FLG_MOTION_FAULT);
    ok = ok && fault_is(p_h, S2_FAULT_MOTION);
    p_h = h_running(true);
    h_flags(p_h, CTRL_FLG_SCAN_FAULT);
    ok = ok && fault_is(p_h, S2_FAULT_SCAN);
    p_h = h_running(true);
    h_tick(p_h, 10u);
    h_result(p_h, MOTION_EVT_REJECTED);
    h_tick(p_h, 10u);
    ok = ok && fault_is(p_h, S2_FAULT_MOTION);
    p_h = h_running(true);
    h_tick(p_h, 10u);
    h_result(p_h, MOTION_EVT_FAULT);
    h_tick(p_h, 10u);
    ok = ok && fault_is(p_h, S2_FAULT_MOTION);
    p_h = h_running(true);
    p_h->b_send_ok = false;
    h_tick(p_h, 10u);
    ok = ok && fault_is(p_h, S2_FAULT_DELIVERY);
    p_h = h_running(true);
    h_tick(p_h, 10u);
    h_tick(p_h, H_RESULT_MS);
    ok = ok && h_state(p_h, S2_SYS_RUNNING);    /* at the limit: waiting */
    h_tick(p_h, 10u);

    return (ok && fault_is(p_h, S2_FAULT_MOTION));
}

bool
test_ctrl_reset_stopped (void)
{
    h_t * p_h = h_running(false);

    h_flags(p_h, CTRL_FLG_LOCAL_STOP);
    p_h->in.status.flags = MOTION_ST_VALID;     /* still rolling */

    bool const b_refused = (S2_RESULT_REJECTED
                            == h_remote(p_h, S2_REMOTE_RESET, 0u))
                           && h_state(p_h, S2_SYS_STOPPED);
    uint32_t const drains = p_h->n_drain;

    h_motion_ok(p_h);

    return (b_refused
            && (S2_RESULT_COMPLETED == h_remote(p_h, S2_REMOTE_RESET, 0u))
            && h_state(p_h, S2_SYS_READY) && p_h->b_stop
            && (p_h->n_drain == (drains + 1u)));
}

bool
test_ctrl_reset_fault (void)
{
    h_t * p_h = h_running(false);

    p_h->in.status.state = (uint8_t)MOTION_FAULT;
    p_h->in.status.fault = MOTION_FAULT_STALL;
    p_h->in.status.flags = MOTION_ST_VALID;     /* wheels not still */
    h_flags(p_h, CTRL_FLG_MOTION_FAULT);

    bool const b_refused = (S2_RESULT_REJECTED
                            == h_remote(p_h, S2_REMOTE_RESET, 0u))
                           && fault_is(p_h, S2_FAULT_MOTION);
    uint32_t const acts = p_h->n_act;

    p_h->in.status.flags = MOTION_ST_VALID | MOTION_ST_STOPPED;

    bool const b_accept = (S2_RESULT_COMPLETED
                           == h_remote(p_h, S2_REMOTE_RESET, 0u))
                          && h_state(p_h, S2_SYS_INITIALISING)
                          && (p_h->n_act == (acts + 1u))
                          && (CTRL_ACT_RESET == p_h->act[acts]);

    h_tick(p_h, 10u);       /* motion still in Fault */

    bool const b_wait = h_state(p_h, S2_SYS_INITIALISING);

    h_motion_ok(p_h);       /* motion RESET done */
    h_tick(p_h, 10u);

    return (b_refused && b_accept && b_wait && h_state(p_h, S2_SYS_READY)
            && p_h->b_stop && (S2_FAULT_NONE == p_h->c.fault));
}

bool
test_ctrl_remote_rules (void)
{
    h_t *            p_h = h_ready(false);
    s2_robot_state_t st;
    bool             ok;

    ok = (S2_RESULT_EXPIRED == h_remote(p_h, S2_REMOTE_START, H_AGE_MS + 1u))
         && h_state(p_h, S2_SYS_READY);
    ok = ok && (S2_RESULT_REJECTED == h_remote(p_h, 9u, 0u));

    /* Replay the previous request ID. */
    p_h->next_req--;
    ok = ok && (S2_RESULT_DUPLICATE == h_remote(p_h, S2_REMOTE_START, 0u))
         && h_state(p_h, S2_SYS_READY);

    uint32_t const id = p_h->next_req;

    ok = ok && (S2_RESULT_COMPLETED == h_remote(p_h, S2_REMOTE_START,
                                                H_AGE_MS));
    ctrl_get_state(&p_h->c, &p_h->in, &st);

    return (ok && (st.last_request_id == id)
            && (st.last_command == S2_REMOTE_START)
            && (st.last_result == S2_RESULT_COMPLETED));
}

bool
test_ctrl_stop_queue_full (void)
{
    h_t * p_h = h_running(false);

    /* One cycle as task_controller.c runs it: flags first, then the
       queued commands. */
    h_flags(p_h, CTRL_FLG_REMOTE_STOP);

    uint8_t const r1 = h_remote(p_h, S2_REMOTE_START, 0u);
    uint8_t const r2 = h_remote(p_h, S2_REMOTE_START, 0u);

    h_tick(p_h, 10u);

    return (h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop
            && (S2_RESULT_REJECTED == r1) && (S2_RESULT_REJECTED == r2)
            && (1u == p_h->n_clear));   /* only the original START */
}

bool
test_ctrl_only_start_clears (void)
{
    h_t * p_h = h_running(false);

    h_flags(p_h, CTRL_FLG_REMOTE_STOP);
    (void)h_remote(p_h, S2_REMOTE_RESET, 0u);     /* to Ready */

    for (uint32_t i = 0u; i < 100u; i++)
    {
        h_flags(p_h, 0u);                       /* reconnect: no input */
        h_tick(p_h, 10u);
    }

    p_h->next_req--;                            /* replay the RESET ID */

    bool const b_dup = (S2_RESULT_DUPLICATE
                        == h_remote(p_h, S2_REMOTE_START, 0u));
    bool const b_held = h_state(p_h, S2_SYS_READY) && p_h->b_stop
                        && (1u == p_h->n_clear);

    p_h->next_req++;

    return (b_dup && b_held
            && (S2_RESULT_COMPLETED == h_remote(p_h, S2_REMOTE_START, 0u))
            && (!p_h->b_stop) && (2u == p_h->n_clear));
}

bool
test_ctrl_robot_state (void)
{
    h_t *            p_h = h_ready(false);
    s2_robot_state_t st;

    p_h->in.status.travel_mm = 1000;
    ctrl_get_state(&p_h->c, &p_h->in, &st);

    bool const b_ready = (S2_SYS_READY == st.state)
                         && (0u != (st.flags & S2_RS_STOP_LATCHED))
                         && (0u != (st.flags & S2_RS_STOPPED))
                         && (0u == (st.flags & S2_RS_DISTANCE_VALID));

    (void)h_remote(p_h, S2_REMOTE_START, 0u);
    p_h->in.status.travel_mm = 1250;
    p_h->in.status.flags     = MOTION_ST_VALID;     /* moving */
    ctrl_get_state(&p_h->c, &p_h->in, &st);

    return (b_ready && (S2_SYS_RUNNING == st.state)
            && (0u == (st.flags & S2_RS_STOP_LATCHED))
            && (0u == (st.flags & S2_RS_STOPPED))
            && (0u != (st.flags & S2_RS_DISTANCE_VALID))
            && (250 == st.run_distance_mm)
            && (S2_FAULT_NONE == st.fault)
            && (st.captured_ms == p_h->in.now_ms));
}

/* ------------------------------------------------------------------------
 *  Mission substates (S2-U60..S2-U66)
 * ------------------------------------------------------------------------ */

/* Running in mission mode with NEW_RUN acknowledged: FOLLOW after a tick. */
static h_t *
h_mission (void)
{
    h_t * p_h = h_running(false);

    h_tick(p_h, 10u);

    return (p_h);
}

static bool
h_mode (h_t const * p_h, uint8_t mode)
{
    return (h_state(p_h, S2_SYS_RUNNING) && (mode == p_h->c.mode));
}

static ctrl_action_t
h_last (h_t const * p_h)
{
    return ((p_h->n_act > 0u) ? p_h->act[p_h->n_act - 1u] : CTRL_ACT_NONE);
}

static bool
h_scan_sent (h_t const * p_h, uint8_t kind)
{
    bool b_found = false;

    for (uint32_t i = 0u; i < p_h->n_scan; i++)
    {
        b_found = b_found || (kind == p_h->scan[i].kind);
    }

    return (b_found);
}

/* S2-U60: START needs a fresh, valid line in mission mode. */
bool
test_ctrl_start_line (void)
{
    h_t * p_h = h_ready(false);

    p_h->in.line.flags = 0u;                        /* not settled */

    bool const b_invalid = (S2_RESULT_REJECTED
                            == h_remote(p_h, S2_REMOTE_START, 0u));

    p_h->in.line.flags       = CTRL_LINE_VALID;
    p_h->in.line.captured_ms = p_h->in.now_ms - CTRL_LINE_MAX_AGE_MS - 1u;

    bool const b_stale = (S2_RESULT_REJECTED
                          == h_remote(p_h, S2_REMOTE_START, 0u));

    p_h->in.line.b_ok = false;

    bool const b_missing = (S2_RESULT_REJECTED
                            == h_remote(p_h, S2_REMOTE_START, 0u));

    p_h->in.line.b_ok        = true;
    p_h->in.line.captured_ms = p_h->in.now_ms;

    return (b_invalid && b_stale && b_missing && h_state(p_h, S2_SYS_READY)
            && (S2_RESULT_COMPLETED == h_remote(p_h, S2_REMOTE_START, 0u)));
}

/* S2-U61: FOLLOW renews a drive from S3's steering every cycle; a stale
   snapshot holds still, then faults; an S3 sensor fault faults. */
bool
test_ctrl_follow (void)
{
    h_t * p_h = h_mission();

    p_h->in.line.steer_permille = 500;              /* line to the left */
    h_tick(p_h, 10u);

    bool const b_left = h_mode(p_h, S2_MODE_FOLLOW)
                        && (CTRL_ACT_DRIVE == p_h->out.action)
                        && (75 == p_h->out.drive_l)
                        && (225 == p_h->out.drive_r)
                        && (!p_h->c.b_waiting);
    uint32_t const acts = p_h->n_act;

    p_h->in.line.steer_permille = -500;
    h_tick(p_h, 10u);

    bool const b_right = (225 == p_h->out.drive_l)
                         && (75 == p_h->out.drive_r)
                         && (p_h->n_act == (acts + 1u));
    s2_robot_state_t st;

    ctrl_get_state(&p_h->c, &p_h->in, &st);

    bool const b_auth = (S2_MODE_FOLLOW == st.mode)
                        && (0u != (st.flags & S2_RS_NAV_AUTHORISED));

    p_h->b_line_auto = false;                       /* snapshots stop */
    h_tick(p_h, CTRL_LINE_MAX_AGE_MS + 10u);

    bool const b_hold = h_mode(p_h, S2_MODE_FOLLOW)
                        && (CTRL_ACT_DRIVE == p_h->out.action)
                        && (0 == p_h->out.drive_l) && (0 == p_h->out.drive_r);

    h_tick(p_h, CTRL_LINE_FAIL_MS);

    bool const b_stale = fault_is(p_h, S2_FAULT_LINE);

    p_h = h_mission();
    p_h->in.line.flags = CTRL_LINE_FAULT;
    h_tick(p_h, 10u);

    return (b_left && b_right && b_auth && b_hold && b_stale
            && fault_is(p_h, S2_FAULT_LINE));
}

/* S2-U62: a barcode command runs once per nav_id at its junction. */
bool
test_ctrl_navigate (void)
{
    h_t * p_h = h_mission();

    p_h->in.line.nav_cmd = S2_NAV_LEFT;
    p_h->in.line.nav_id  = 7u;
    h_tick(p_h, 10u);

    s2_robot_state_t st;

    ctrl_get_state(&p_h->c, &p_h->in, &st);

    bool const b_turn = h_mode(p_h, S2_MODE_NAVIGATE)
                        && (CTRL_ACT_LEFT == h_last(p_h))
                        && (90000u == p_h->out.value) && p_h->c.b_waiting
                        && (0u == (st.flags & S2_RS_NAV_AUTHORISED));

    h_result(p_h, MOTION_EVT_DONE);
    h_tick(p_h, 10u);

    bool const b_back = h_mode(p_h, S2_MODE_FOLLOW)
                        && (CTRL_ACT_DRIVE == p_h->out.action);

    h_tick(p_h, 10u);                               /* same nav_id again */

    bool const b_once = h_mode(p_h, S2_MODE_FOLLOW);

    p_h->in.line.nav_cmd = S2_NAV_UTURN;
    p_h->in.line.nav_id  = 8u;
    h_tick(p_h, 10u);

    bool const b_uturn = (CTRL_ACT_LEFT == h_last(p_h))
                         && (180000u == p_h->out.value);

    h_result(p_h, MOTION_EVT_DONE);
    h_tick(p_h, 10u);
    p_h->in.line.nav_cmd = S2_NAV_STRAIGHT;
    p_h->in.line.nav_id  = 9u;
    h_tick(p_h, 10u);

    bool const b_straight = (CTRL_ACT_FORWARD == h_last(p_h))
                            && (CTRL_JUNCTION_CROSS_MM == p_h->out.value);

    h_result(p_h, MOTION_EVT_REJECTED);             /* manoeuvre refused */
    h_tick(p_h, 10u);

    return (b_turn && b_back && b_once && b_uturn && b_straight
            && fault_is(p_h, S2_FAULT_MOTION));
}

/* Drive one HAZARD -> SCAN -> PLAN -> BYPASS sequence up to REACQUIRE. */
static h_t *
h_to_reacquire (void)
{
    h_t * p_h = h_mission();

    h_flags(p_h, CTRL_FLG_HAZARD);
    h_tick(p_h, 10u);
    p_h->in.scan.b_ok          = true;
    p_h->in.scan.request_id    = p_h->c.scan_id;
    p_h->in.scan.result        = CTRL_SCAN_PLAN;
    p_h->in.scan.count         = 4u;
    p_h->in.scan.seg[0].kind   = CTRL_SEG_TURN;
    p_h->in.scan.seg[0].value  = -90000;
    p_h->in.scan.seg[1].kind   = CTRL_SEG_MOVE;
    p_h->in.scan.seg[1].value  = 256;
    p_h->in.scan.seg[2].kind   = CTRL_SEG_TURN;
    p_h->in.scan.seg[2].value  = 90000;
    p_h->in.scan.seg[3].kind   = CTRL_SEG_MOVE;
    p_h->in.scan.seg[3].value  = 670;
    p_h->in.scan.approach_mdeg = 90000;
    p_h->in.scan.search_mm     = 276u;
    p_h->in.scan.align_mdeg    = -90000;
    h_tick(p_h, 10u);                               /* plan accepted */

    for (uint32_t i = 0u; i < 5u; i++)              /* 4 segments, approach */
    {
        h_result(p_h, MOTION_EVT_DONE);
        h_tick(p_h, 10u);
    }

    return (p_h);
}

/* S2-U63: HAZARD in FOLLOW scans instead of latching STOP, then runs the
   plan, searches forward and aligns before following again. */
bool
test_ctrl_bypass (void)
{
    h_t * p_h = h_mission();

    p_h->in.status.flags = MOTION_ST_VALID;         /* still rolling */
    h_flags(p_h, CTRL_FLG_HAZARD);

    bool const b_scan = h_mode(p_h, S2_MODE_SCAN) && (!p_h->b_stop)
                        && (CTRL_ACT_STOP == h_last(p_h))
                        && (!p_h->c.b_waiting)
                        && h_scan_sent(p_h, S2_SCAN_REQUEST)
                        && (CTRL_FLG_HAZARD == (p_h->cleared
                                                & CTRL_FLG_HAZARD));

    h_tick(p_h, 10u);

    bool const b_wait_still = !h_scan_sent(p_h, S2_SCAN_AUTHORISE);

    h_motion_ok(p_h);                               /* wheels confirmed */
    h_tick(p_h, 10u);

    bool const b_grant = h_scan_sent(p_h, S2_SCAN_STOP_CONFIRM)
                         && h_scan_sent(p_h, S2_SCAN_AUTHORISE)
                         && (p_h->scan[1].request_id == p_h->c.scan_id);

    p_h = h_to_reacquire();

    /* The last six actions: four segments, the approach turn, a drive. A
       short record fails here rather than indexing outside it. */
    uint32_t const n = p_h->n_act;
    bool const b_order = (n >= 6u) && (n < H_ACT_MAX)
                         && (CTRL_ACT_RIGHT == p_h->act[n - 6u])
                         && (CTRL_ACT_FORWARD == p_h->act[n - 5u])
                         && (CTRL_ACT_LEFT == p_h->act[n - 4u])
                         && (CTRL_ACT_FORWARD == p_h->act[n - 3u])
                         && (CTRL_ACT_LEFT == p_h->act[n - 2u])
                         && (CTRL_ACT_DRIVE == p_h->act[n - 1u]);
    bool const b_search = h_mode(p_h, S2_MODE_REACQUIRE)
                          && (CTRL_SEARCH_SPEED_MM_S == p_h->out.drive_l)
                          && (CTRL_SEARCH_SPEED_MM_S == p_h->out.drive_r);

    p_h->in.line.flags = CTRL_LINE_SEEN;            /* line under a sensor */
    h_tick(p_h, 10u);
    h_tick(p_h, CTRL_FOUND_MS - 10u);

    bool const b_early = !p_h->c.b_aligning;

    h_tick(p_h, 10u);

    bool const b_align = p_h->c.b_aligning
                         && (CTRL_ACT_RIGHT == h_last(p_h))
                         && (90000u == p_h->out.value);

    p_h->in.status.heading_mdeg = 4000;             /* within 15 degrees */
    p_h->in.line.flags          = CTRL_LINE_VALID;
    h_result(p_h, MOTION_EVT_DONE);
    h_tick(p_h, 10u);

    return (b_scan && b_wait_still && b_grant && b_order && b_search
            && b_early && b_align && h_mode(p_h, S2_MODE_FOLLOW)
            && (!p_h->b_stop));
}

/* S2-U64: a clear result resumes; no route, a fault result or the time
   limit faults (and stops S5's scan). */
bool
test_ctrl_scan_outcomes (void)
{
    h_t * p_h = h_mission();

    h_flags(p_h, CTRL_FLG_HAZARD);
    p_h->in.scan.b_ok       = true;
    p_h->in.scan.request_id = p_h->c.scan_id;
    p_h->in.scan.result     = CTRL_SCAN_CLEAR;
    h_tick(p_h, 10u);

    bool const b_clear = h_mode(p_h, S2_MODE_FOLLOW);

    p_h->in.scan.request_id = p_h->c.scan_id;       /* old request */
    h_flags(p_h, CTRL_FLG_HAZARD);
    h_tick(p_h, 10u);

    bool const b_new_id = h_mode(p_h, S2_MODE_SCAN);

    p_h->in.scan.request_id = p_h->c.scan_id;
    p_h->in.scan.result     = 3u;                   /* NO_ROUTE */
    h_tick(p_h, 10u);

    bool const b_route = fault_is(p_h, S2_FAULT_SCAN)
                         && h_scan_sent(p_h, S2_SCAN_STOP);

    p_h = h_mission();
    h_flags(p_h, CTRL_FLG_HAZARD);
    h_tick(p_h, CTRL_SCAN_LIMIT_MS);

    bool const b_in_time = h_mode(p_h, S2_MODE_SCAN);

    h_tick(p_h, 10u);

    return (b_clear && b_new_id && b_route && b_in_time
            && fault_is(p_h, S2_FAULT_SCAN)
            && h_scan_sent(p_h, S2_SCAN_CANCEL));
}

/* S2-U65: reacquisition is bounded and checks the heading. */
bool
test_ctrl_reacquire (void)
{
    h_t * p_h = h_mission();

    p_h->in.line.last_side = CTRL_SIDE_RIGHT;
    p_h->in.line.flags     = CTRL_LINE_LOST;
    h_tick(p_h, 10u);

    bool const b_rotate = h_mode(p_h, S2_MODE_REACQUIRE)
                          && (CTRL_SEARCH_SPEED_MM_S == p_h->out.drive_l)
                          && (-CTRL_SEARCH_SPEED_MM_S == p_h->out.drive_r);

    p_h->in.line.flags = CTRL_LINE_SEEN;
    h_tick(p_h, 10u);
    h_tick(p_h, CTRL_FOUND_MS);

    bool const b_found = h_mode(p_h, S2_MODE_FOLLOW);

    p_h->in.line.flags = CTRL_LINE_LOST;
    h_tick(p_h, 10u);
    h_tick(p_h, CTRL_SEARCH_LIMIT_MS);

    bool const b_time_ok = h_mode(p_h, S2_MODE_REACQUIRE);

    h_tick(p_h, 10u);

    bool const b_timeout = fault_is(p_h, S2_FAULT_LINE);

    p_h = h_to_reacquire();
    p_h->in.status.travel_mm += 277;                /* past search_mm */
    h_tick(p_h, 10u);

    bool const b_distance = fault_is(p_h, S2_FAULT_LINE);

    p_h = h_to_reacquire();
    p_h->in.line.flags = CTRL_LINE_SEEN;
    h_tick(p_h, 10u);
    h_tick(p_h, CTRL_FOUND_MS);
    p_h->in.status.heading_mdeg = 20000;            /* 20 degrees off */
    h_result(p_h, MOTION_EVT_DONE);
    h_tick(p_h, 10u);

    return (b_rotate && b_found && b_time_ok && b_timeout && b_distance
            && fault_is(p_h, S2_FAULT_LINE));
}

/* S2-U66: STOP preempts every mission mode; HAZARD outside FOLLOW and
   SCAN latches STOP; STOP during a scan or bypass tells S5. */
bool
test_ctrl_mission_stop (void)
{
    h_t * p_h = h_mission();

    h_flags(p_h, CTRL_FLG_REMOTE_STOP);

    bool const b_follow = h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop
                          && (!h_scan_sent(p_h, S2_SCAN_STOP));

    p_h = h_mission();
    p_h->in.line.nav_cmd = S2_NAV_RIGHT;
    p_h->in.line.nav_id  = 3u;
    h_tick(p_h, 10u);
    h_flags(p_h, CTRL_FLG_LOCAL_STOP);

    bool const b_nav = h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop
                       && (!p_h->c.b_waiting);

    p_h = h_mission();
    h_flags(p_h, CTRL_FLG_HAZARD);
    h_flags(p_h, CTRL_FLG_REMOTE_STOP);

    bool const b_scan = h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop
                        && h_scan_sent(p_h, S2_SCAN_STOP);

    p_h = h_to_reacquire();
    p_h->n_scan = 0u;
    h_flags(p_h, CTRL_FLG_HAZARD);

    bool const b_reacq = h_state(p_h, S2_SYS_STOPPED) && p_h->b_stop;

    s2_robot_state_t st;

    ctrl_get_state(&p_h->c, &p_h->in, &st);

    return (b_follow && b_nav && b_scan && b_reacq
            && (S2_MODE_NONE == st.mode)
            && (0u == (st.flags & S2_RS_NAV_AUTHORISED)));
}

#endif /* APP_TEST */

/* End of file. */
