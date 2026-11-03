/**
 * @file task_motion.c
 * @brief Subsystem 2 Motion Control task: the only writer of motor PWM.
 *
 * Period: MC_PERIOD_MS (10 ms) from a cyclic handler. The kernel tick is
 * 10 ms, so a cyclic handler gives an exact period where a sleep would
 * drift; 10 ms rather than 20 ms halves the worst-case STOP latency and
 * gives subsystem 4 one distance update per 10 ms IMU sample.
 *
 * Each period, or at once on a STOP wake-up (s2_motion_wake):
 *   1. read the encoder snapshot, the 1 MHz timer and the latched STOP;
 *   2. handle every queued motion_q command (non-blocking);
 *   3. run motion_step() and write the duties to the motors;
 *   4. raise CTRL_FLG_MOTION_FAULT if Motion Control entered Fault;
 *   5. publish motion_status_t under its mutex.
 * The motors are written (3) before anything else is shared.
 *
 * Fail safe: if safety_flags cannot be read (e.g. not created yet), STOP
 * is treated as set, so the motors cannot move.
 *
 * Core: pinned to core 1. Core 0 runs usermain, the WiFi polling task and
 * the GPIO dispatcher interrupt; the encoder snapshot is safe across cores.
 */
#include "task_motion.h"

#include "drv_encoder.h"
#include "drv_motor.h"
#include "motion_config.h"
#include "motion_logic.h"
#include "subsystem2.h"

#define MC_TASK_CORE        TP_PRC2

#define MC_LOCK_TMO_MS      10      /* reader wait for the status lock */

EXPORT ID motion_q = 0;

static ID              g_tid_motion = 0;
static ID              g_cyc_motion = 0;
static ID              g_status_mtx = 0;
static motion_status_t g_status;            /* guarded by g_status_mtx */
static motion_t        g_motion;            /* motion task only */
static volatile uint32_t g_latch_us   = 0u; /* written by the controller */
static volatile uint32_t g_off_us     = 0u; /* written by this task */
static volatile bool     g_b_off_due  = false;

static const motion_cfg_t g_motion_cfg =
{
    {MC_COUNTS_PER_REV, MC_WHEEL_CIRC_UM, MC_TRACK_MM},
    {MC_PID_KP, MC_PID_KI, MC_PID_KD, -DRV_MOTOR_DUTY_MAX, DRV_MOTOR_DUTY_MAX},
    {MC_STRAIGHT_KP, MC_STRAIGHT_KI, MC_STRAIGHT_KD,
     -MC_SPEED_MAX_MM_S, MC_SPEED_MAX_MM_S},
    MC_SPEED_MAX_MM_S,
    MC_STILL_MM_S,
    MC_STILL_MS,
    MC_STOP_LIMIT_MS,
    MC_STALL_DUTY,
    MC_STALL_MS,
    MC_TIMEOUT_FACTOR,
    MC_TIMEOUT_EXTRA_MS,
    MC_MOVE_TOL_MM,
    MC_TURN_TOL_MDEG,
    MC_ENC_ERRORS_MAX,
    MC_ENC_INVALID_MAX,
    MC_SPEED_WINDOW,
    MC_DUTY_STEP_MAX
};

static void     motion_task (INT stacd, void * exinf);
static void     motion_cyclic (void * exinf);
static void     read_input (motion_input_t * p_in);
static void     apply_duty (int32_t duty_l, int32_t duty_r);
static void     raise_fault (motion_output_t const * p_out);
static void     publish_status (void);
static uint32_t now_ms (void);

/** @brief Create motion_q, the status mutex and the period handler. */
ER
s2_motion_create (void)
{
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};
    T_CCYC ccyc = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = MC_MOTION_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_motion_cmd_t);

    ID const q = tk_cre_mbf(&cmbf);

    if (q < E_OK)
    {
        return (q);
    }

    motion_q     = q;
    cmtx.mtxatr  = TA_INHERIT;
    g_status_mtx = tk_cre_mtx(&cmtx);

    if (g_status_mtx < E_OK)
    {
        return (g_status_mtx);
    }

    ccyc.cycatr  = TA_HLNG;         /* created stopped; started with task */
    ccyc.cychdr  = (FP)motion_cyclic;
    ccyc.cyctim  = MC_PERIOD_MS;
    ccyc.cycphs  = MC_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);

    return ((g_cyc_motion < E_OK) ? g_cyc_motion : E_OK);
}

/** @brief Start the Motion Control task, then its period. */
ER
s2_motion_start (void)
{
    ID const tid = app_start_task((FP)motion_task, S2_PRI_MOTION,
                                  MC_TASK_CORE, "motion");

    if (tid < E_OK)
    {
        return (tid);
    }

    g_tid_motion = tid;

    return (tk_sta_cyc(g_cyc_motion));
}

/** @brief Run a step now (the controller calls this after latching STOP). */
void
s2_motion_wake (void)
{
    if (g_tid_motion > 0)
    {
        (void)tk_wup_tsk(g_tid_motion);
    }
}

void
s2_motion_note_latch (void)
{
    g_latch_us  = in_w(TIMER_TIMERAWL);
    g_b_off_due = true;
}

/** @brief Latest latch and motor-off times; false until one is recorded. */
bool
s2_stop_timing (uint32_t * p_latch_us, uint32_t * p_motor_off_us)
{
    if ((NULL == p_latch_us) || (NULL == p_motor_off_us) || g_b_off_due
        || (0u == g_off_us))
    {
        return (false);
    }

    *p_latch_us     = g_latch_us;
    *p_motor_off_us = g_off_us;

    return (true);
}

/** @brief Copy the latest Motion Control status (any task). */
ER
motion_get_status (motion_status_t * p_status)
{
    if (NULL == p_status)
    {
        return (E_PAR);
    }

    if (g_status_mtx <= 0)
    {
        return (E_OBJ);
    }

    ER const er = tk_loc_mtx(g_status_mtx, MC_LOCK_TMO_MS);

    if (er < E_OK)
    {
        return (er);
    }

    *p_status = g_status;
    (void)tk_unl_mtx(g_status_mtx);

    return (E_OK);
}

/** @brief Stop confirmation: both wheels still for 100 ms. False if unknown. */
bool
motion_stop_confirmed (void)
{
    motion_status_t st;

    return ((E_OK == motion_get_status(&st))
            && (0u != (st.flags & MOTION_ST_STOPPED)));
}

static void
motion_task (INT stacd, void * exinf)
{
    motion_input_t  in  = {0};
    motion_output_t out;
    s2_motion_cmd_t cmd;

    (void)stacd;
    (void)exinf;

    bool const b_cfg_ok   = motion_init(&g_motion, &g_motion_cfg);
    bool const b_motor_ok = (drv_motor_init() >= E_OK);
    bool const b_enc_ok   = (drv_encoder_init() >= E_OK);

    motion_output_clear(&out);
    motion_start(&g_motion, b_cfg_ok && b_motor_ok && b_enc_ok, &out);
    raise_fault(&out);
    publish_status();

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);     /* drop wake-ups queued meanwhile */

        read_input(&in);
        motion_output_clear(&out);

        while (tk_rcv_mbf(motion_q, &cmd, TMO_POL)
               == (INT)sizeof(s2_motion_cmd_t))
        {
            motion_command(&g_motion, &cmd, &in, &out);
        }

        motion_step(&g_motion, &in, &out);
        apply_duty(out.duty_l, out.duty_r);

        if (g_b_off_due && in.b_stop && (0 == out.duty_l)
            && (0 == out.duty_r))
        {
            g_off_us    = in_w(TIMER_TIMERAWL);  /* duties now zero */
            g_b_off_due = false;
        }
        raise_fault(&out);
        publish_status();
    }
}

static void
motion_cyclic (void * exinf)
{
    (void)exinf;

    if (g_tid_motion > 0)
    {
        (void)tk_wup_tsk(g_tid_motion);
    }
}

static void
read_input (motion_input_t * p_in)
{
    drv_encoder_snapshot_t snap;
    T_RFLG                 rflg;

    p_in->now_us         = in_w(TIMER_TIMERAWL);
    p_in->b_counts_valid = (E_OK == drv_encoder_snapshot(&snap));

    if (p_in->b_counts_valid)
    {
        uint32_t const l = snap.count[MC_LEFT_ENCODER];
        uint32_t const r = snap.count[MC_RIGHT_ENCODER];

        /* Unsigned negation reverses the count direction modulo 2^32. */
        p_in->count_l  = (0 != MC_LEFT_ENC_INVERT) ? (0u - l) : l;
        p_in->count_r  = (0 != MC_RIGHT_ENC_INVERT) ? (0u - r) : r;
        p_in->errors_l = snap.errors[MC_LEFT_ENCODER];
        p_in->errors_r = snap.errors[MC_RIGHT_ENCODER];
    }

    p_in->b_stop = (tk_ref_flg(safety_flags, &rflg) < E_OK)
                   || (0u != (rflg.flgptn & SAFETY_FLG_STOP));
}

static void
apply_duty (int32_t duty_l, int32_t duty_r)
{
    (void)drv_motor_set(MC_LEFT_MOTOR,
                        (0 != MC_LEFT_MOTOR_INVERT) ? -duty_l : duty_l);
    (void)drv_motor_set(MC_RIGHT_MOTOR,
                        (0 != MC_RIGHT_MOTOR_INVERT) ? -duty_r : duty_r);
}

/* The controller treats this bit like STOP (Step 4). */
static void
raise_fault (motion_output_t const * p_out)
{
    if (p_out->b_raise_fault)
    {
        (void)tk_set_flg(controller_flags, CTRL_FLG_MOTION_FAULT);
    }
}

static void
publish_status (void)
{
    if (tk_loc_mtx(g_status_mtx, TMO_POL) >= E_OK)
    {
        motion_get_status_logic(&g_motion, &g_status);
        g_status.captured_ms = now_ms();
        (void)tk_unl_mtx(g_status_mtx);
    }
}

static uint32_t
now_ms (void)
{
    SYSTIM tim;

    return ((tk_get_tim(&tim) >= E_OK) ? (uint32_t)tim.lo : 0u);
}

/* End of file. */
