/**
 * @file task_controller.c
 * @brief Subsystem 2 Vehicle Controller task: owns system state and STOP.
 *
 * Priority S2_PRI_CONTROLLER (14), the highest application priority.
 * Pinned to core 1 with Motion Control, so a STOP latched here is acted
 * on by Motion Control as soon as this task blocks.
 *
 * Each cycle it waits on controller_flags for at most CTRL_WATCHDOG_MS
 * (10 ms, under the 20 ms limit; the kernel tick is 10 ms), then:
 *   1. reads Motion Control status;
 *   2. handles flags first (STOP before any queued command);
 *   3. handles every remote_q record (non-blocking);
 *   4. runs the periodic step (mission substates, or the demo script);
 *   5. publishes robot_state under state_mutex.
 * Inputs also include copies of the Line Sensor snapshot (IF-06,
 * s3_line_get) and the Obstacle Scan result (scan_task_get); neither call
 * waits. Scan messages go to scan_q without waiting.
 * Bits stay set until handled, so the wait returns at once for a bit
 * already pending.
 */
#include "task_controller.h"

#include "controller_logic.h"
#include "motion_cmd.h"
#include "robot_state.h"
#include "task_motion.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem3/s3_nav.h"
#include "../subsystem3/s3_track.h"
#include "../subsystem5/include/scan_task.h"

/* The controller logic copies these IF-06 and scan values (it includes no
   other subsystem's header); a mismatch fails the build here. */
typedef char ctrl_copies_match[((CTRL_LINE_VALID == S3_ST_VALID)
    && (CTRL_LINE_LOST == S3_ST_LOST) && (CTRL_LINE_FAULT == S3_ST_FAULT)
    && (CTRL_LINE_SEEN == S3_ST_SEEN)
    && (CTRL_SIDE_LEFT == S3_TRACK_SIDE_LEFT)
    && (CTRL_SIDE_RIGHT == S3_TRACK_SIDE_RIGHT)
    && (S2_NAV_LEFT == (uint8_t)S3_NAV_LEFT)
    && (S2_NAV_RIGHT == (uint8_t)S3_NAV_RIGHT)
    && (S2_NAV_STRAIGHT == (uint8_t)S3_NAV_STRAIGHT)
    && (S2_NAV_UTURN == (uint8_t)S3_NAV_UTURN)
    && (CTRL_SCAN_PLAN == SCAN_TASK_RESULT_PLAN)
    && (CTRL_SCAN_CLEAR == SCAN_TASK_RESULT_CLEAR)
    && (CTRL_SEG_TURN == BYPASS_PLAN_TURN)
    && (CTRL_SEG_MOVE == BYPASS_PLAN_MOVE)
    && (CTRL_PLAN_SEGMENTS == BYPASS_PLAN_SEGMENTS)) ? 1 : -1];

#define CTRL_TASK_CORE      TP_PRC2
#define CTRL_WATCHDOG_MS    10
#define CTRL_INIT_LIMIT_MS  3000u       /* UNMEASURED: motion start-up */
#define CTRL_RESULT_LIMIT_MS 20000u     /* UNMEASURED: longest manoeuvre */
#define CTRL_AUTOSTART_MS   5000u
#define CTRL_LOG_MS         500u

#if S2_DEMO_SCRIPT
static const ctrl_step_t g_demo[] =
{
    {CTRL_ACT_FORWARD, 500u},
    {CTRL_ACT_LEFT,    90000u},
    {CTRL_ACT_FORWARD, 500u}
};
#define CTRL_SCRIPT     g_demo
#define CTRL_SCRIPT_LEN ((uint8_t)(sizeof(g_demo) / sizeof(g_demo[0])))
#else
#define CTRL_SCRIPT     NULL
#define CTRL_SCRIPT_LEN 0u
#endif

static ctrl_t g_ctrl;               /* controller task only */

static void     controller_task (INT stacd, void * exinf);
static void     demo (s2_robot_state_t const * p_st, ctrl_input_t const * p_in);
static void     read_input (ctrl_input_t * p_in);
static void     apply (ctrl_output_t * p_out, ctrl_input_t const * p_in);
static ER       send_action (ctrl_output_t const * p_out);
static void     send_scan (ctrl_output_t const * p_out, uint32_t now);
static void     read_line (ctrl_line_t * p_line);
static void     read_scan (ctrl_scan_t * p_scan);
static uint32_t now_ms (void);

/**
 * @brief Create controller_flags, safety_flags (STOP set), remote_q,
 *        scan_q and state_mutex. Each global ID is set only once its object exists,
 *        because the WiFi task may already be calling the adapters.
 */
ER
s2_controller_create (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = SAFETY_FLG_STOP;     /* boot latched */

    ID const sflg = tk_cre_flg(&cflg);

    if (sflg < E_OK)
    {
        return (sflg);
    }

    safety_flags = sflg;
    cflg.flgatr  = TA_TFIFO | TA_WSGL;  /* only the controller waits */
    cflg.iflgptn = 0u;

    ID const cf = tk_cre_flg(&cflg);

    if (cf < E_OK)
    {
        return (cf);
    }

    controller_flags = cf;
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S2_REMOTE_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_remote_cmd_t);

    ID const q = tk_cre_mbf(&cmbf);

    if (q < E_OK)
    {
        return (q);
    }

    remote_q    = q;
    cmbf.bufsz  = S2_SCAN_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_scan_cmd_t);

    ID const sq = tk_cre_mbf(&cmbf);

    if (sq < E_OK)
    {
        return (sq);
    }

    scan_q = sq;

    return (s2_robot_state_create());
}

ER
s2_controller_start (void)
{
    ID const tid = app_start_task((FP)controller_task, S2_PRI_CONTROLLER,
                                  CTRL_TASK_CORE, "controller");

    return ((tid < E_OK) ? tid : E_OK);
}

static void
controller_task (INT stacd, void * exinf)
{
    ctrl_cfg_t       cfg = {CTRL_INIT_LIMIT_MS, CTRL_RESULT_LIMIT_MS,
                            S2_REMOTE_MAX_AGE_MS, CTRL_SCRIPT,
                            CTRL_SCRIPT_LEN, {0}};
    ctrl_input_t     in  = {0};
    ctrl_output_t    out;
    s2_remote_cmd_t  cmd;
    s2_robot_state_t st;
    UINT             ptn;

    (void)stacd;
    (void)exinf;

    ctrl_mission_defaults(&cfg.mission);
    ctrl_init(&g_ctrl, &cfg, now_ms());

    for (;;)
    {
        if (tk_wai_flg(controller_flags, CTRL_FLG_ALL, TWF_ORW, &ptn,
                       CTRL_WATCHDOG_MS) < E_OK)
        {
            ptn = 0u;   /* watchdog timeout */
        }

        read_input(&in);

        ctrl_output_clear(&out);
        ctrl_flags(&g_ctrl, (uint32_t)ptn, &in, &out);
        apply(&out, &in);

        while (tk_rcv_mbf(remote_q, &cmd, TMO_POL)
               == (INT)sizeof(s2_remote_cmd_t))
        {
            ctrl_output_clear(&out);
            ctrl_remote(&g_ctrl, &cmd, &in, &out);
            apply(&out, &in);
        }

        ctrl_output_clear(&out);
        ctrl_tick(&g_ctrl, &in, &out);
        apply(&out, &in);

        ctrl_get_state(&g_ctrl, &in, &st);
        s2_robot_state_publish(&st);
        demo(&st, &in);
    }
}

/* Optional demo helpers (S2_DEMO_AUTOSTART, S2_DEMO_LOG). */
static void
demo (s2_robot_state_t const * p_st, ctrl_input_t const * p_in)
{
#if S2_DEMO_AUTOSTART
    static bool     b_started  = false;
    static bool     b_in_ready = false;
    static uint32_t ready_ms   = 0u;

    if ((S2_SYS_READY == p_st->state) && (!b_started))
    {
        if (!b_in_ready)
        {
            b_in_ready = true;
            ready_ms   = p_in->now_ms;
        }
        else if ((p_in->now_ms - ready_ms) >= CTRL_AUTOSTART_MS)
        {
            b_started = s2_local_command(S2_REMOTE_START);
        }
        else
        {
            /* Waiting before the automatic START. */
        }
    }
#endif

#if S2_DEMO_LOG
    static uint32_t last_ms = 0u;
    uint32_t        latch_us;
    uint32_t        off_us;

    if ((p_in->now_ms - last_ms) >= CTRL_LOG_MS)
    {
        last_ms = p_in->now_ms;
        tm_printf((UB *)"[S2] state %d fault %d motion %d dist %d mm"
                  " flags 0x%x\n", (INT)p_st->state, (INT)p_st->fault,
                  (INT)p_st->motion_state, (INT)p_st->run_distance_mm,
                  (INT)p_st->flags);

        if (s2_stop_timing(&latch_us, &off_us))
        {
            tm_printf((UB *)"[S2] last STOP: latch to motor off %d us\n",
                      (INT)(off_us - latch_us));
        }
    }
#endif

    (void)p_st;
    (void)p_in;
}

static void
read_input (ctrl_input_t * p_in)
{
    p_in->now_ms      = now_ms();
    p_in->b_status_ok = (E_OK == motion_get_status(&p_in->status));
    read_line(&p_in->line);
    read_scan(&p_in->scan);
}

/* A busy or missing snapshot leaves b_ok false; the logic then treats the
   line as stale or the scan result as not yet available. */
static void
read_line (ctrl_line_t * p_line)
{
    s3_line_status_t st;

    p_line->b_ok = (E_OK == s3_line_get(&st));

    if (p_line->b_ok)
    {
        p_line->captured_ms    = st.captured_ms;
        p_line->steer_permille = st.steer_permille;
        p_line->flags          = st.flags;
        p_line->last_side      = st.last_side;
        p_line->nav_cmd        = st.nav_cmd;
        p_line->nav_id         = st.nav_id;
    }
}

static void
read_scan (ctrl_scan_t * p_scan)
{
    static scan_task_status_t st;   /* controller task only: off the stack */

    p_scan->b_ok = (E_OK == scan_task_get(&st));

    if (p_scan->b_ok)
    {
        p_scan->request_id    = st.request_id;
        p_scan->result        = st.result;
        p_scan->count         = (st.plan.count <= CTRL_PLAN_SEGMENTS)
                                ? st.plan.count : 0u;
        p_scan->approach_mdeg = st.plan.approach_mdeg;
        p_scan->search_mm     = st.plan.search_mm;
        p_scan->align_mdeg    = st.plan.align_mdeg;

        for (uint32_t i = 0u; i < CTRL_PLAN_SEGMENTS; i++)
        {
            p_scan->seg[i].kind  = st.plan.segments[i].kind;
            p_scan->seg[i].value = st.plan.segments[i].value;
        }
    }
}

/* Order matters: latch, drain, clear, then command, then clear bits. */
static void
apply (ctrl_output_t * p_out, ctrl_input_t const * p_in)
{
    s2_motion_cmd_t stale;

    if (p_out->b_latch_stop)
    {
        s2_motion_note_latch();
        (void)tk_set_flg(safety_flags, SAFETY_FLG_STOP);
        s2_motion_wake();
    }

    if (p_out->b_drain_motion)
    {
        while (tk_rcv_mbf(motion_q, &stale, TMO_POL) > 0)
        {
            /* Discard stale motion commands. */
        }
    }

    if (p_out->b_clear_stop)
    {
        (void)tk_clr_flg(safety_flags, ~(UINT)SAFETY_FLG_STOP);
    }

    if (CTRL_ACT_NONE != p_out->action)
    {
        ER const er = send_action(p_out);
        ctrl_output_t again;

        ctrl_output_clear(&again);
        ctrl_motion_sent(&g_ctrl, (E_OK == er), motion_last_command_id(),
                         p_in->now_ms, &again);

        if (again.b_latch_stop)
        {
            s2_motion_note_latch();
            (void)tk_set_flg(safety_flags, SAFETY_FLG_STOP);
            s2_motion_wake();
        }
    }

    send_scan(p_out, p_in->now_ms);

    if (0u != p_out->clear_flags)
    {
        (void)tk_clr_flg(controller_flags, ~(UINT)p_out->clear_flags);
    }
}

/* Best effort: a lost REQUEST or AUTHORISE ends in the controller's scan
   time limit (Fault), and S5 also aborts when robot_state leaves Running,
   so a lost STOP or CANCEL cannot leave a scan moving. */
static void
send_scan (ctrl_output_t const * p_out, uint32_t now)
{
    for (uint8_t i = 0u; (i < p_out->n_scan) && (i < CTRL_SCAN_MSGS_MAX); i++)
    {
        s2_scan_cmd_t const m =
        {
            p_out->scan[i].request_id, now, p_out->scan[i].kind,
            p_out->scan[i].b_asserted ? 1u : 0u, {0u, 0u}
        };

        if (scan_q > 0)
        {
            (void)tk_snd_mbf(scan_q, &m, (INT)sizeof(m), TMO_POL);
        }
    }
}

static ER
send_action (ctrl_output_t const * p_out)
{
    ctrl_action_t const action = p_out->action;
    uint32_t const      value  = p_out->value;
    ER                  er;

    switch (action)
    {
        case CTRL_ACT_NEW_RUN:
            er = motion_new_run();
            break;
        case CTRL_ACT_RESET:
            er = motion_reset();
            break;
        case CTRL_ACT_FORWARD:
            er = (value > MOTION_DISTANCE_MAX_MM) ? E_PAR
                 : motion_move_forward((uint16_t)value);
            break;
        case CTRL_ACT_BACKWARD:
            er = (value > MOTION_DISTANCE_MAX_MM) ? E_PAR
                 : motion_move_backward((uint16_t)value);
            break;
        case CTRL_ACT_LEFT:
            er = motion_turn_left(value);
            break;
        case CTRL_ACT_RIGHT:
            er = motion_turn_right(value);
            break;
        case CTRL_ACT_DRIVE:
            er = motion_drive(p_out->drive_l, p_out->drive_r);
            break;
        case CTRL_ACT_STOP:
            er = motion_stop();
            break;
        default:
            er = E_PAR;
            break;
    }

    return (er);
}

static uint32_t
now_ms (void)
{
    SYSTIM tim;

    return ((tk_get_tim(&tim) >= E_OK) ? (uint32_t)tim.lo : 0u);
}

/* End of file. */
