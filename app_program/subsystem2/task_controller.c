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
 *   4. runs the periodic step (demo script);
 *   5. publishes robot_state under state_mutex.
 * Bits stay set until handled, so the wait returns at once for a bit
 * already pending.
 */
#include "task_controller.h"

#include "bypass_control.h"
#include "controller_logic.h"
#include "line_control.h"
#include "scan_control.h"
#include "motion_cmd.h"
#include "robot_state.h"
#include "task_motion.h"
#include "../subsystem5/include/scan_task.h"

#define CTRL_TASK_CORE      TP_PRC2
#define CTRL_WATCHDOG_MS    10
#define CTRL_INIT_LIMIT_MS  3000u       /* UNMEASURED: motion start-up */
#define CTRL_RESULT_LIMIT_MS 20000u     /* UNMEASURED: longest manoeuvre */
#define CTRL_AUTOSTART_MS   5000u
#define CTRL_LOG_MS         500u

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

#ifndef S2_CAL_TEST
#define S2_CAL_TEST 0
#endif

/* The GP20 START/STOP button is used by the line and calibration images. */
#define CTRL_BUTTON         (S3_LINE_FOLLOW_TEST || S2_CAL_TEST)

#if CTRL_BUTTON
#define CTRL_BUTTON_PIN         20u
#define CTRL_BUTTON_MASK        (UINT32_C(1) << CTRL_BUTTON_PIN)
#define CTRL_BUTTON_DEBOUNCE_MS 30u
#endif

#if S2_CAL_TEST
#define CTRL_CAL_DISTANCE_MM    1000u
#define CTRL_CAL_ANGLE_MDEG     360000u
#endif

#if S3_LINE_FOLLOW_TEST
#define CTRL_LINE_BASE_MM_S        40
#define CTRL_LINE_CORRECT_MIN_MM_S 32
#define CTRL_LINE_CORRECT_MAX_MM_S 38
#define CTRL_LINE_CURVE_OUTER_MM_S 35
#define CTRL_LINE_LEFT_CURVE_MM_S   5
#define CTRL_LINE_RIGHT_CURVE_MM_S  5
#define CTRL_LINE_TIMEOUT_MS      500u
#define CTRL_LINE_START_GRACE_MS  200u
#define CTRL_LINE_CURVE_ENTRY_MS  200u
#define CTRL_LINE_LEFT_HOLD_MS    250u
#define CTRL_LINE_RIGHT_HOLD_MS   150u
#define CTRL_LINE_JUNCTION_MS     100u
#define CTRL_LINE_CROSSING_MS    1500u
#define CTRL_LINE_EXIT_MIN_MS     600u
#define CTRL_LINE_EXIT_CENTER_MS  100u
#define CTRL_LINE_EXIT_SIDE_MS    150u
#define CTRL_LINE_EXIT_LIMIT_MS  2500u
/* Scan & Plan: S5's 15 s scan limit plus the stop and confirmation
   before it (2 s) and a margin, as on full-system. */
#define CTRL_SCAN_LIMIT_MS      20000u
/* Bypass and reacquisition (S2 decision 14). Agreed in S5 decision 9:
   100 ms of stable line, 15 degrees of heading. The search runs at 45
   degrees, so its time limit is 7.5 s: the largest diagonal search,
   500 mm sideways (708 mm), at the search speed plus 6% (S2 decision 17).
   Subsystem 5 publishes the forward verdict every 65 ms and rejects data
   older than 150 ms (S5 decision 3); an older snapshot counts as a hazard.
   UNMEASURED: the search speed. Measured by ruler (Dalton, 2026-11-11):
   the front line sensors are 133 mm ahead of the drive-wheel axle. */
#define CTRL_BYPASS_SEARCH_MS       7500u
#define CTRL_BYPASS_FOUND_MS        100u
#define CTRL_BYPASS_HEADING_MDEG    15000
#define CTRL_BYPASS_FORWARD_AGE_MS  150u
#define CTRL_BYPASS_SEARCH_MM_S     100
#define CTRL_BYPASS_SENSOR_AHEAD_MM 133u

/* scan_control and bypass_control mirror subsystem 5's codes (IF-13). */
typedef char ctrl_scan_codes_match_t[
    ((SCAN_CONTROL_RESULT_NONE == SCAN_TASK_RESULT_NONE)
     && (SCAN_CONTROL_RESULT_PLAN == SCAN_TASK_RESULT_PLAN)
     && (SCAN_CONTROL_RESULT_CLEAR == SCAN_TASK_RESULT_CLEAR)
     && (BYPASS_CONTROL_TURN == BYPASS_PLAN_TURN)
     && (BYPASS_CONTROL_MOVE == BYPASS_PLAN_MOVE)
     && (BYPASS_CONTROL_SEGMENTS == BYPASS_PLAN_SEGMENTS)) ? 1 : -1];
#endif

#if S2_CAL_TEST == 1
static const ctrl_step_t g_demo[] =
{
    {CTRL_ACT_FORWARD, CTRL_CAL_DISTANCE_MM}
};
#define CTRL_SCRIPT     g_demo
#define CTRL_SCRIPT_LEN ((uint8_t)(sizeof(g_demo) / sizeof(g_demo[0])))
#elif S2_CAL_TEST == 2
static const ctrl_step_t g_demo[] =
{
    {CTRL_ACT_LEFT, CTRL_CAL_ANGLE_MDEG}
};
#define CTRL_SCRIPT     g_demo
#define CTRL_SCRIPT_LEN ((uint8_t)(sizeof(g_demo) / sizeof(g_demo[0])))
#elif S2_DEMO_SCRIPT
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

#if S2_CAL_TEST
static uint32_t g_cal_boot_us;      /* controller task only */
static uint32_t g_cal_boot_ms;
#endif

#if CTRL_BUTTON
typedef struct
{
    uint32_t candidate_since_ms;
    bool     b_candidate_pressed;
    bool     b_stable_pressed;
    bool     b_initialised;
    bool     b_armed;
    bool     b_restart_pending;
} ctrl_button_t;

static ctrl_button_t g_button;
#endif

#if S3_LINE_FOLLOW_TEST
typedef struct
{
    line_control_t        control;
    uint32_t              last_report_ms;
    uint32_t              running_since_ms;
    uint8_t               position;
    uint8_t               strength;
    int16_t               last_correction_mm_s;
    line_control_action_t last_action;
    bool                  b_have_report;
    bool                  b_running;
    bool                  b_action_sent;
} ctrl_line_runtime_t;

static ctrl_line_runtime_t g_line_runtime;
static bypass_control_cfg_t const g_bypass_cfg =
{
    CTRL_BYPASS_SEARCH_MS, CTRL_BYPASS_FOUND_MS, CTRL_BYPASS_HEADING_MDEG,
    CTRL_BYPASS_SENSOR_AHEAD_MM
};

static scan_control_t      g_scan;
static bypass_control_t    g_bypass;
static scan_task_status_t  g_scan_status;  /* off the stack */
#endif

static void     controller_task (INT stacd, void * exinf);
static void     demo (s2_robot_state_t const * p_st, ctrl_input_t const * p_in);
static void     read_input (ctrl_input_t * p_in);
static void     apply (ctrl_output_t * p_out, ctrl_input_t const * p_in);
static ER       send_action (ctrl_action_t action, uint32_t value);
static uint32_t now_ms (void);
#if CTRL_BUTTON
static ER       button_init (void);
static bool     button_press_event (uint32_t now);
static void     button_request_reset (s2_robot_state_t const * p_state);
static void     button_step (s2_robot_state_t const * p_state,
                             ctrl_input_t const * p_input);
#endif
#if S2_CAL_TEST
static void     cal_timer_start (void);
static void     cal_timer_print (uint32_t now);
static void     cal_report (s2_robot_state_t const * p_state,
                            ctrl_input_t const * p_input);
#endif
#if S3_LINE_FOLLOW_TEST
static void     line_step (s2_robot_state_t const * p_state,
                           ctrl_input_t const * p_input);
static void     line_reset (uint32_t now);
static int16_t  line_correction_speed (uint8_t strength);
static bool     line_apply_action (line_control_action_t action,
                                   int16_t correction_mm_s);
static uint32_t scan_hazard (uint32_t ptn, s2_robot_state_t const * p_state);
static void     scan_update (s2_robot_state_t * p_state,
                             ctrl_input_t const * p_input);
static void     scan_send (scan_control_output_t const * p_out,
                           uint32_t now);
static void     scan_report (scan_control_outcome_t outcome,
                             ctrl_input_t const * p_input);
static void     scan_profile_report (scan_profile_t const * p_profile);
static void     bypass_input (ctrl_input_t const * p_input,
                              bypass_control_input_t * p_in);
static void     bypass_send (bypass_control_output_t const * p_out);
static void     bypass_report (bypass_control_output_t const * p_out);
static void     line_log_mode (line_control_mode_t old_mode,
                               line_control_mode_t new_mode,
                               line_control_action_t action);
#endif

/**
 * @brief Create controller_flags, safety_flags (STOP set), remote_q, line_q,
 *        scan_q and state_mutex. Each global ID is set only once its object
 *        exists, because the WiFi task may already be calling the adapters.
 */
ER
s2_controller_create (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

#if CTRL_BUTTON
    ER const button_er = button_init();

    if (button_er < E_OK)
    {
        return (button_er);
    }
#endif

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

    remote_q = q;

    cmbf.bufsz  = S2_LINE_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_line_report_t);

    ID const lq = tk_cre_mbf(&cmbf);

    if (lq < E_OK)
    {
        return (lq);
    }

    line_q = lq;

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
    ctrl_cfg_t const cfg = {CTRL_INIT_LIMIT_MS, CTRL_RESULT_LIMIT_MS,
                            S2_REMOTE_MAX_AGE_MS, CTRL_SCRIPT,
                            CTRL_SCRIPT_LEN};
    ctrl_input_t     in  = {0};
    ctrl_output_t    out;
    s2_remote_cmd_t  cmd;
    s2_robot_state_t st;
    UINT             ptn;

    (void)stacd;
    (void)exinf;

#if S2_CAL_TEST
    cal_timer_start();
#endif

    ctrl_init(&g_ctrl, &cfg, now_ms());
#if S3_LINE_FOLLOW_TEST
    (void)scan_control_init(&g_scan, CTRL_SCAN_LIMIT_MS);
    (void)bypass_control_init(&g_bypass, &g_bypass_cfg);
#endif

#if CTRL_BUTTON
    tm_printf((UB *)"[S2] GP20 button ready: press to START or STOP\n");
#endif

    for (;;)
    {
        if (tk_wai_flg(controller_flags, CTRL_FLG_ALL, TWF_ORW, &ptn,
                       CTRL_WATCHDOG_MS) < E_OK)
        {
            ptn = 0u;   /* watchdog timeout */
        }

        read_input(&in);
#if S3_LINE_FOLLOW_TEST
        ctrl_get_state(&g_ctrl, &in, &st);
        ptn = scan_hazard(ptn, &st);    /* a hazard while following scans */
#endif

        if ((0u != (ptn & (CTRL_FLG_HAZARD | CTRL_FLG_SCAN_FAULT)))
            && (S2_SYS_RUNNING == g_ctrl.state))
        {
            /* A stop must never be silent; S5 repeats its flags while
               the cause persists, so report only the one that stops a
               run. */
            tm_printf((UB *)"[S2] S5 %s%s: STOP latched\n",
                      (0u != (ptn & CTRL_FLG_HAZARD)) ? "HAZARD" : "",
                      (0u != (ptn & CTRL_FLG_SCAN_FAULT)) ? " SCAN_FAULT"
                                                          : "");
        }

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
#if S3_LINE_FOLLOW_TEST
        /* Running is line following (or a scan) only in this image. */
        scan_update(&st, &in);
#endif
#if CTRL_BUTTON
        button_step(&st, &in);
#endif
#if S2_CAL_TEST
        cal_report(&st, &in);
#endif
#if S3_LINE_FOLLOW_TEST
        line_step(&st, &in);
#endif
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
        ER const er = send_action(p_out->action, p_out->value);
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

    if (0u != p_out->clear_flags)
    {
        (void)tk_clr_flg(controller_flags, ~(UINT)p_out->clear_flags);
    }
}

static ER
send_action (ctrl_action_t action, uint32_t value)
{
    ER er;

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

#if CTRL_BUTTON
static ER
button_init (void)
{
    ER const er = gpio_set_pin(CTRL_BUTTON_PIN, GPIO_MODE_IN);

    if (er >= E_OK)
    {
        clr_w(GPIO(CTRL_BUTTON_PIN), GPIO_PDE);
        set_w(GPIO(CTRL_BUTTON_PIN), GPIO_PUE);
    }

    return (er);
}

static bool
button_press_event (uint32_t now)
{
    bool const b_pressed =
        (0u == (in_w(GPIO_IN) & CTRL_BUTTON_MASK));
    bool b_event = false;

    if (!g_button.b_initialised)
    {
        g_button.b_candidate_pressed = b_pressed;
        g_button.candidate_since_ms = now;
        g_button.b_initialised = true;
    }
    else if (b_pressed != g_button.b_candidate_pressed)
    {
        g_button.b_candidate_pressed = b_pressed;
        g_button.candidate_since_ms = now;
    }
    else if ((now - g_button.candidate_since_ms)
             >= CTRL_BUTTON_DEBOUNCE_MS)
    {
        if (!b_pressed)
        {
            g_button.b_armed = true;
        }

        if (b_pressed != g_button.b_stable_pressed)
        {
            g_button.b_stable_pressed = b_pressed;

            if (b_pressed && g_button.b_armed)
            {
                g_button.b_armed = false;
                b_event = true;
            }
        }
    }
    else
    {
        /* The candidate level has not been stable for long enough. */
    }

    return (b_event);
}

static void
button_request_reset (s2_robot_state_t const * p_state)
{
    bool const b_wheels_stopped =
        (0u != (p_state->flags & S2_RS_STOPPED));

    if (!b_wheels_stopped)
    {
        tm_printf((UB *)"[S2] GP20: RESET waiting; wheels not stopped\n");
    }
    else if (s2_local_command(S2_REMOTE_RESET))
    {
        g_button.b_restart_pending = true;

        if (S2_SYS_FAULT == p_state->state)
        {
            tm_printf((UB *)"[S2] GP20: FAULT RESET requested before restart\n");
        }
        else
        {
            tm_printf((UB *)"[S2] GP20: RESET requested before restart\n");
        }
    }
    else
    {
        tm_printf((UB *)"[S2] GP20: RESET request failed\n");
    }
}

static void
button_step (s2_robot_state_t const * p_state,
                  ctrl_input_t const * p_input)
{
    bool const b_press = button_press_event(p_input->now_ms);

    if (g_button.b_restart_pending)
    {
        if (S2_SYS_READY == p_state->state)
        {
            if (s2_local_command(S2_REMOTE_START))
            {
                g_button.b_restart_pending = false;
                tm_printf((UB *)"[S2] GP20: START requested\n");
            }
        }
        else if (S2_SYS_FAULT == p_state->state)
        {
            g_button.b_restart_pending = false;
        }
        else
        {
            /* Wait for RESET to move Stopped to Ready. */
        }
    }

    if (b_press)
    {
        if (S2_SYS_READY == p_state->state)
        {
            if (s2_local_command(S2_REMOTE_START))
            {
                tm_printf((UB *)"[S2] GP20: START requested\n");
            }
            else
            {
                tm_printf((UB *)"[S2] GP20: START request failed\n");
            }
        }
        else if (S2_SYS_RUNNING == p_state->state)
        {
            if (s2_local_stop())
            {
                tm_printf((UB *)"[S2] GP20: STOP requested\n");
            }
            else
            {
                tm_printf((UB *)"[S2] GP20: STOP request failed\n");
            }
        }
        else if ((S2_SYS_STOPPED == p_state->state)
                 || (S2_SYS_FAULT == p_state->state))
        {
            button_request_reset(p_state);
        }
        else
        {
            tm_printf((UB *)"[S2] GP20: ignored; controller state=%u"
                      " fault=%u, status_ok=%u flags=0x%x,"
                      " motion state=%u fault=%u, speed=%d/%d mm/s\n",
                      (UINT)p_state->state, (UINT)p_state->fault,
                      (UINT)p_input->b_status_ok,
                      (UINT)p_input->status.flags,
                      (UINT)p_input->status.state,
                      (UINT)p_input->status.fault,
                      (INT)p_input->status.left_mm_s,
                      (INT)p_input->status.right_mm_s);
        }
    }
}
#endif

#if S2_CAL_TEST
/* Kernel time runs on SysTick (clk_sys); TIMERAWL runs on the watchdog
   tick (clk_ref). Both come from the crystal, so they agree only if the
   tick generator is set up (kernel startup_clock()). The reference is
   taken at boot and compared at every report, because after a cold
   power-up the USB terminal is usually opened after boot. TIMERAWL wraps
   after about 71 minutes. */
static void
cal_timer_start (void)
{
    g_cal_boot_us = in_w(TIMER_TIMERAWL);
    g_cal_boot_ms = now_ms();
}

static void
cal_timer_print (uint32_t now)
{
    tm_printf((UB *)"[S2] CAL timer: %u us in %u ms of kernel time"
              " (expect 1000 us per ms)\n",
              (UINT)(in_w(TIMER_TIMERAWL) - g_cal_boot_us),
              (UINT)(now - g_cal_boot_ms));
}

/* Print the odometry once when a run ends. Odometry drives the manoeuvre,
   so it always reads close to the command: the calibration value is the
   physical distance or angle measured on the floor. */
static void
cal_report (s2_robot_state_t const * p_state, ctrl_input_t const * p_input)
{
    static uint8_t last_state = 0u;

    if ((S2_SYS_RUNNING == last_state) && (S2_SYS_RUNNING != p_state->state))
    {
#if S2_CAL_TEST == 1
        tm_printf((UB *)"[S2] CAL forward %u mm finished\n",
                  (UINT)CTRL_CAL_DISTANCE_MM);
#else
        tm_printf((UB *)"[S2] CAL left %u mdeg finished\n",
                  (UINT)CTRL_CAL_ANGLE_MDEG);
#endif
        tm_printf((UB *)"[S2] CAL odometry: x=%d mm y=%d mm heading=%d mdeg;"
                  " result=%u reason=%u state=%u fault=%u\n",
                  (INT)p_input->status.x_mm, (INT)p_input->status.y_mm,
                  (INT)p_input->status.heading_mdeg,
                  (UINT)p_input->status.last_result,
                  (UINT)p_input->status.last_reason,
                  (UINT)p_state->state, (UINT)p_state->fault);
        cal_timer_print(p_input->now_ms);
    }

    last_state = p_state->state;
}
#endif

#if S3_LINE_FOLLOW_TEST
/* A hazard while following starts Scan & Plan instead of the latched
   STOP; the flag is consumed here. During a bypass the flag is consumed
   too: bypass_control checks the hazard level itself, because the bypass
   starts within the hazard distance. Returns the flags still to handle. */
static uint32_t
scan_hazard (uint32_t ptn, s2_robot_state_t const * p_state)
{
    scan_control_output_t out;

    if ((0u == (ptn & CTRL_FLG_HAZARD)) || (S2_SYS_RUNNING != p_state->state)
        || (0u != (p_state->flags & S2_RS_STOP_LATCHED)))
    {
        return (ptn);
    }

    if ((!scan_control_active(&g_scan)) && (!bypass_control_active(&g_bypass)))
    {
        scan_control_begin(&g_scan, p_state->captured_ms, &out);
        tm_printf((UB *)"[S2] S5 HAZARD: stopping to scan, request %u\n",
                  (UINT)g_scan.request_id);
        scan_send(&out, p_state->captured_ms);
    }

    (void)tk_clr_flg(controller_flags, ~(UINT)CTRL_FLG_HAZARD);

    return (ptn & ~(uint32_t)CTRL_FLG_HAZARD);
}

/* Publish the mode, end a scan or bypass on a stop elsewhere, otherwise
   advance the handshake or the bypass and act on the outcome. */
static void
scan_update (s2_robot_state_t * p_state, ctrl_input_t const * p_input)
{
    scan_control_output_t out;
    bool const b_running = (S2_SYS_RUNNING == p_state->state)
                           && (0u == (p_state->flags & S2_RS_STOP_LATCHED));

    if (!b_running)
    {
        scan_control_abort(&g_scan, &out);
        scan_send(&out, p_input->now_ms);
        bypass_control_abort(&g_bypass);
        p_state->mode = S2_MODE_NONE;
        return;
    }

    if (scan_control_active(&g_scan))
    {
        bool const b_ok = (E_OK == scan_task_get(&g_scan_status));

        scan_control_step(&g_scan, motion_stop_confirmed(), b_ok,
                          g_scan_status.request_id, g_scan_status.result,
                          p_input->now_ms, &out);
        scan_send(&out, p_input->now_ms);
        scan_report(out.outcome, p_input);
    }
    else if (bypass_control_active(&g_bypass))
    {
        bypass_control_input_t  in;
        bypass_control_output_t bout;

        (void)scan_task_get(&g_scan_status);  /* keeps the last on E_TMOUT */
        bypass_input(p_input, &in);
        bypass_control_step(&g_bypass, &in, &bout);
        bypass_send(&bout);
    }
    else
    {
        /* Following. */
    }

    if (scan_control_active(&g_scan))
    {
        p_state->mode = S2_MODE_SCAN;
    }
    else if (bypass_control_active(&g_bypass))
    {
        p_state->mode = bypass_control_mode(&g_bypass);
    }
    else
    {
        p_state->mode = S2_MODE_FOLLOW;
    }
}

static void
scan_send (scan_control_output_t const * p_out, uint32_t now)
{
    if (p_out->b_stop_motion)
    {
        (void)motion_stop();
    }

    for (uint8_t i = 0u; i < p_out->n_msg; i++)
    {
        s2_scan_cmd_t const msg =
        {
            g_scan.request_id, now, p_out->msg[i].kind,
            p_out->msg[i].b_asserted ? 1u : 0u, {0u, 0u}
        };

        /* Best effort: S5 also checks robot_state for authority. */
        (void)tk_snd_mbf(scan_q, &msg, (INT)sizeof(msg), TMO_POL);
    }
}

/* Print what the scan saw. Extents are sideways from the sensor axis
   (the car's centre line) at the obstacle's edge rays; width and centre
   are the observed span only (S5 decision 5), so print them only when
   both edges were seen. */
static void
scan_profile_report (scan_profile_t const * p_profile)
{
    if (0u == p_profile->forward_mm)
    {
        /* No profile: the scan faulted or was cancelled. */
    }
    else if (p_profile->b_shape_estimated)
    {
        tm_printf((UB *)"[S2] scan %u profile: ahead %u mm, nearest %u mm,"
                  " left edge %u mm, right edge %u mm, width %u mm,"
                  " centre %d mm (left +)\n",
                  (UINT)g_scan.request_id, (UINT)p_profile->forward_mm,
                  (UINT)p_profile->nearest_observed_mm,
                  (UINT)p_profile->left_extent_mm,
                  (UINT)p_profile->right_extent_mm,
                  (UINT)p_profile->observed_width_mm,
                  (INT)p_profile->centre_offset_mm);
    }
    else
    {
        tm_printf((UB *)"[S2] scan %u profile: ahead %u mm, nearest %u mm,"
                  " left edge %s %u mm, right edge %s %u mm\n",
                  (UINT)g_scan.request_id, (UINT)p_profile->forward_mm,
                  (UINT)p_profile->nearest_observed_mm,
                  p_profile->b_left_edge ? "seen" : "UNSEEN",
                  (UINT)p_profile->left_extent_mm,
                  p_profile->b_right_edge ? "seen" : "UNSEEN",
                  (UINT)p_profile->right_extent_mm);
    }
}

/* Print the result. CLEAR resumes following (S2 decision 11); PLAN starts
   the bypass (S2 decision 14); failures stop the run. */
static void
scan_report (scan_control_outcome_t outcome, ctrl_input_t const * p_input)
{
    bypass_plan_t const * p_plan = &g_scan_status.plan;

    if (SCAN_CONTROL_NONE != outcome)
    {
        scan_profile_report(&p_plan->profile);
    }

    if (SCAN_CONTROL_PLAN == outcome)
    {
        bypass_control_plan_t   plan;
        bypass_control_input_t  in;
        bypass_control_output_t out;

        tm_printf((UB *)"[S2] scan %u: PLAN side=%s obstacle=%u mm"
                  " lateral=%u mm, %u segments:\n",
                  (UINT)g_scan.request_id,
                  (BYPASS_PLAN_LEFT == p_plan->side) ? "LEFT" : "RIGHT",
                  (UINT)p_plan->obstacle_mm, (UINT)p_plan->lateral_mm,
                  (UINT)p_plan->count);

        for (uint8_t i = 0u; (i < p_plan->count)
                             && (i < BYPASS_PLAN_SEGMENTS); i++)
        {
            tm_printf((UB *)"[S2]   %u: %s %d %s\n", (UINT)i,
                      (BYPASS_PLAN_TURN == p_plan->segments[i].kind)
                          ? "TURN" : "MOVE",
                      (INT)p_plan->segments[i].value,
                      (BYPASS_PLAN_TURN == p_plan->segments[i].kind)
                          ? "mdeg" : "mm");
        }

        tm_printf((UB *)"[S2]   then approach %d mdeg, search %u mm,"
                  " align %d mdeg; bypass starts\n",
                  (INT)p_plan->approach_mdeg, (UINT)p_plan->search_mm,
                  (INT)p_plan->align_mdeg);

        /* Copy now: subsystem 5 clears an unaccepted plan after 2 s. */
        plan.count = p_plan->count;
        plan.approach_mdeg = p_plan->approach_mdeg;
        plan.align_mdeg = p_plan->align_mdeg;
        plan.search_mm = p_plan->search_mm;

        for (uint8_t i = 0u; i < BYPASS_CONTROL_SEGMENTS; i++)
        {
            plan.seg[i].kind = p_plan->segments[i].kind;
            plan.seg[i].value = p_plan->segments[i].value;
        }

        bypass_input(p_input, &in);
        bypass_control_begin(&g_bypass, &plan, &in, &out);
        bypass_send(&out);
    }
    else if (SCAN_CONTROL_CLEAR == outcome)
    {
        tm_printf((UB *)"[S2] scan %u: CLEAR, resuming line following\n",
                  (UINT)g_scan.request_id);
    }
    else if ((SCAN_CONTROL_FAILED == outcome)
             || (SCAN_CONTROL_TIMEOUT == outcome))
    {
        tm_printf((UB *)"[S2] scan %u: %s (S5 result %u); STOP\n",
                  (UINT)g_scan.request_id,
                  (SCAN_CONTROL_TIMEOUT == outcome) ? "TIMEOUT" : "FAILED",
                  (UINT)g_scan_status.result);
        (void)s2_local_stop();
    }
    else
    {
        /* Still scanning. */
    }
}

/* One cycle's view for bypass_control. A line report counts while fresh
   and with a sensor on black (CENTER is both white: straddling). The
   forward verdict counts while its snapshot is fresh; the snapshot may be
   taken after now_ms, so its age is checked both ways. */
static void
bypass_input (ctrl_input_t const * p_input, bypass_control_input_t * p_in)
{
    uint8_t const  position = g_line_runtime.position;
    uint32_t const now = p_input->now_ms;
    uint32_t const captured = g_scan_status.captured_ms;
    bool const     b_line_fresh = g_line_runtime.b_have_report
        && ((now - g_line_runtime.last_report_ms) <= S2_LINE_MAX_AGE_MS);
    bool const     b_forward_fresh =
        ((now - captured) <= CTRL_BYPASS_FORWARD_AGE_MS)
        || ((captured - now) <= CTRL_BYPASS_FORWARD_AGE_MS);

    p_in->now_ms = now;
    p_in->travel_mm = p_input->status.travel_mm;
    p_in->heading_mdeg = p_input->status.heading_mdeg;
    p_in->last_id = p_input->status.last_id;
    p_in->last_result = p_input->status.last_result;
    p_in->b_status_ok = p_input->b_status_ok;
    p_in->b_on_line = b_line_fresh
                      && ((S2_LINE_LEFT == position)
                          || (S2_LINE_RIGHT == position)
                          || (S2_LINE_BOTH_BLACK == position));
    p_in->b_hazard = (!b_forward_fresh)
                     || (0u != (g_scan_status.flags & SCAN_TASK_FLAG_HAZARD));
}

/* Send the requested motion command, report the send, then report the
   outcome of either. */
static void
bypass_send (bypass_control_output_t const * p_out)
{
    bypass_control_output_t sent;
    ER                      er = E_OK;

    switch (p_out->action)
    {
        case BYPASS_CONTROL_ACT_MOVE:
            er = (p_out->value >= 0)
                 ? motion_move_forward((uint16_t)p_out->value)
                 : motion_move_backward((uint16_t)(-p_out->value));
            tm_printf((UB *)"[S2] bypass: MOVE %d mm\n", (INT)p_out->value);
        break;

        case BYPASS_CONTROL_ACT_TURN:
            er = (p_out->value >= 0)
                 ? motion_turn_left((uint32_t)p_out->value)
                 : motion_turn_right((uint32_t)(-p_out->value));
            tm_printf((UB *)"[S2] bypass: TURN %d mdeg\n", (INT)p_out->value);
        break;

        case BYPASS_CONTROL_ACT_SEARCH:
            er = motion_set_speeds(CTRL_BYPASS_SEARCH_MM_S,
                                   CTRL_BYPASS_SEARCH_MM_S);
            tm_printf((UB *)"[S2] bypass: searching for the line at %d mm/s\n",
                      (INT)CTRL_BYPASS_SEARCH_MM_S);
        break;

        default:
            /* Nothing to send. */
        break;
    }

    if (BYPASS_CONTROL_ACT_NONE == p_out->action)
    {
        bypass_report(p_out);
    }
    else
    {
        bypass_control_sent(&g_bypass, (E_OK == er), motion_last_command_id(),
                            &sent);
        bypass_report(&sent);
    }
}

/* DONE resumes line following; a failure stops the run. */
static void
bypass_report (bypass_control_output_t const * p_out)
{
    static char const * const reasons[] =
    {
        "none", "invalid plan", "motion command failed", "forward hazard",
        "line not found", "heading off the original line"
    };

    if (BYPASS_CONTROL_DONE == p_out->outcome)
    {
        tm_printf((UB *)"[S2] bypass: back on the line, resuming line"
                  " following\n");
    }
    else if (BYPASS_CONTROL_FAILED == p_out->outcome)
    {
        tm_printf((UB *)"[S2] bypass: FAILED, %s; STOP\n",
                  ((uint32_t)p_out->fail
                   < (sizeof(reasons) / sizeof(reasons[0])))
                      ? reasons[p_out->fail] : "unknown");
        (void)s2_local_stop();
    }
    else
    {
        /* Still bypassing. */
    }
}

static void
line_step (s2_robot_state_t const * p_state, ctrl_input_t const * p_input)
{
    s2_line_report_t report;
    bool const b_system_running = (S2_SYS_RUNNING == p_state->state)
                                  && (S2_MODE_FOLLOW == p_state->mode);
    /* The bypass search reads the latest report too (bypass_input). */
    bool const b_line_wanted = b_system_running
                               || ((S2_SYS_RUNNING == p_state->state)
                                   && (S2_MODE_REACQUIRE == p_state->mode));
    bool b_report_valid = true;
    line_control_action_t action;
    line_control_mode_t old_mode;
    line_control_mode_t new_mode;
    int16_t correction_mm_s;
    bool b_correction_action;

    if (b_system_running && (!g_line_runtime.b_running))
    {
        line_reset(p_input->now_ms);
        g_line_runtime.b_running = true;
        g_line_runtime.running_since_ms = p_input->now_ms;
    }

    while (tk_rcv_mbf(line_q, &report, TMO_POL)
           == (INT)sizeof(s2_line_report_t))
    {
        bool const b_this_valid = b_line_wanted
            && (report.position <= S2_LINE_STOP)
            && (report.strength <= S2_LINE_STRENGTH_MAX)
            && (((S2_LINE_LEFT == report.position)
                 || (S2_LINE_RIGHT == report.position))
                || (0u == report.strength))
            && (0u == report.reserved[0])
            && (0u == report.reserved[1])
            && ((p_input->now_ms - report.captured_ms)
                <= S2_LINE_MAX_AGE_MS);

        if (b_line_wanted)
        {
            b_report_valid = b_report_valid && b_this_valid;

            if (b_this_valid)
            {
                g_line_runtime.last_report_ms = report.captured_ms;
                g_line_runtime.position = report.position;
                g_line_runtime.strength = report.strength;
                g_line_runtime.b_have_report = true;
            }
        }
    }

    if (b_line_wanted && (!b_report_valid))
    {
        tm_printf((UB *)"[S2] line: invalid report; STOP\n");
        (void)s2_line_stop();
        return;
    }

    if (!b_system_running)
    {
        if (g_line_runtime.b_running)
        {
            line_reset(p_input->now_ms);
        }
        return;
    }

    if (!g_line_runtime.b_have_report)
    {
        if ((p_input->now_ms - g_line_runtime.running_since_ms)
            > CTRL_LINE_START_GRACE_MS)
        {
            tm_printf((UB *)"[S2] line: no report after START; STOP\n");
            (void)s2_line_stop();
        }
        return;
    }

    if ((p_input->now_ms - g_line_runtime.last_report_ms)
        > CTRL_LINE_TIMEOUT_MS)
    {
        tm_printf((UB *)"[S2] line: stale report; STOP\n");
        (void)s2_line_stop();
        return;
    }

    old_mode = line_control_mode(&g_line_runtime.control);
    action = line_control_step(&g_line_runtime.control,
                               g_line_runtime.position,
                               p_input->now_ms);
    new_mode = line_control_mode(&g_line_runtime.control);
    line_log_mode(old_mode, new_mode, action);
    correction_mm_s = line_correction_speed(g_line_runtime.strength);
    b_correction_action = (LINE_CONTROL_ACTION_CORRECT_LEFT == action)
                       || (LINE_CONTROL_ACTION_CORRECT_RIGHT == action);

    if ((!g_line_runtime.b_action_sent)
        || (action != g_line_runtime.last_action)
        || (b_correction_action
            && (correction_mm_s
                != g_line_runtime.last_correction_mm_s)))
    {
        if (!line_apply_action(action, correction_mm_s))
        {
            tm_printf((UB *)"[S2] line: motion command failed; STOP\n");
            (void)s2_line_stop();
        }
        else
        {
            g_line_runtime.last_action = action;
            g_line_runtime.last_correction_mm_s = correction_mm_s;
            g_line_runtime.b_action_sent = true;
        }
    }
}

static void
line_reset (uint32_t now)
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

    (void)line_control_init(&g_line_runtime.control, &cfg, now);
    g_line_runtime.last_report_ms = now;
    g_line_runtime.running_since_ms = now;
    g_line_runtime.position = S2_LINE_STOP;
    g_line_runtime.strength = 0u;
    g_line_runtime.last_correction_mm_s = CTRL_LINE_CORRECT_MAX_MM_S;
    g_line_runtime.last_action = LINE_CONTROL_ACTION_STOP;
    g_line_runtime.b_have_report = false;
    g_line_runtime.b_running = false;
    g_line_runtime.b_action_sent = false;
}

static int16_t
line_correction_speed (uint8_t strength)
{
    uint16_t const speed_range = (uint16_t)(CTRL_LINE_CORRECT_MAX_MM_S
                               - CTRL_LINE_CORRECT_MIN_MM_S);
    uint16_t const reduction = (uint16_t)((((uint16_t)strength * speed_range)
                               + (S2_LINE_STRENGTH_MAX / 2u))
                               / S2_LINE_STRENGTH_MAX);

    return ((int16_t)(CTRL_LINE_CORRECT_MAX_MM_S - (int16_t)reduction));
}

static bool
line_apply_action (line_control_action_t action, int16_t correction_mm_s)
{
    ER er = E_PAR;

    switch (action)
    {
        case LINE_CONTROL_ACTION_STRAIGHT:
            er = motion_set_speeds(CTRL_LINE_BASE_MM_S,
                                   CTRL_LINE_BASE_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_LEFT:
            er = motion_set_speeds(correction_mm_s,
                                   CTRL_LINE_BASE_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_RIGHT:
            er = motion_set_speeds(CTRL_LINE_BASE_MM_S,
                                   correction_mm_s);
        break;

        case LINE_CONTROL_ACTION_CURVE_LEFT:
            er = motion_set_speeds(CTRL_LINE_LEFT_CURVE_MM_S,
                                   CTRL_LINE_CURVE_OUTER_MM_S);
        break;

        case LINE_CONTROL_ACTION_CURVE_RIGHT:
            er = motion_set_speeds(CTRL_LINE_CURVE_OUTER_MM_S,
                                   CTRL_LINE_RIGHT_CURVE_MM_S);
        break;

        case LINE_CONTROL_ACTION_STOP:
            er = s2_line_stop() ? E_OK : E_OBJ;
        break;

        default:
            er = E_PAR;
        break;
    }

    return (E_OK == er);
}

static void
line_log_mode (line_control_mode_t old_mode, line_control_mode_t new_mode,
               line_control_action_t action)
{
    if (old_mode != new_mode)
    {
        if (LINE_CONTROL_MODE_JUNCTION_CONFIRM == new_mode)
        {
            tm_printf((UB *)"[S2] line: BOTH BLACK; confirming junction\n");
        }
        else if (LINE_CONTROL_MODE_CROSSING == new_mode)
        {
            tm_printf((UB *)"[S2] line: junction confirmed; "
                              "crossing straight\n");
        }
        else if (LINE_CONTROL_MODE_EXIT == new_mode)
        {
            tm_printf((UB *)"[S2] line: junction cleared; "
                              "holding straight exit\n");
        }
        else if (LINE_CONTROL_MODE_FOLLOW == new_mode)
        {
            tm_printf((UB *)"[S2] line: normal following resumed\n");
        }
        else
        {
            tm_printf((UB *)"[S2] line: invalid recovery mode; STOP\n");
        }
    }

    if (((LINE_CONTROL_ACTION_CURVE_LEFT == action)
         || (LINE_CONTROL_ACTION_CURVE_RIGHT == action))
        && (LINE_CONTROL_MODE_FOLLOW == new_mode)
        && ((!g_line_runtime.b_action_sent)
            || (action != g_line_runtime.last_action)))
    {
        static UB const left[] = "LEFT";
        static UB const right[] = "RIGHT";
        UB const * p_direction =
            (LINE_CONTROL_ACTION_CURVE_LEFT == action) ? left : right;

        if (LINE_CONTROL_ACTION_CURVE_LEFT == action)
        {
            tm_printf((UB *)"[S2] line: sustained %s; "
                              "strong curve target=%d/%d mm/s\n",
                      p_direction, (INT)CTRL_LINE_LEFT_CURVE_MM_S,
                      (INT)CTRL_LINE_CURVE_OUTER_MM_S);
        }
        else
        {
            tm_printf((UB *)"[S2] line: sustained %s; "
                              "strong curve target=%d/%d mm/s\n",
                      p_direction, (INT)CTRL_LINE_CURVE_OUTER_MM_S,
                      (INT)CTRL_LINE_RIGHT_CURVE_MM_S);
        }
    }

    if ((LINE_CONTROL_ACTION_STOP == action)
        && ((!g_line_runtime.b_action_sent)
            || (LINE_CONTROL_ACTION_STOP != g_line_runtime.last_action)))
    {
        tm_printf((UB *)"[S2] line: junction/recovery bound exceeded; STOP\n");
    }
}
#endif

/* End of file. */
