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

#include "controller_logic.h"
#include "line_control.h"
#include "motion_cmd.h"
#include "robot_state.h"
#include "task_motion.h"

#define CTRL_TASK_CORE      TP_PRC2
#define CTRL_WATCHDOG_MS    10
#define CTRL_INIT_LIMIT_MS  3000u       /* UNMEASURED: motion start-up */
#define CTRL_RESULT_LIMIT_MS 20000u     /* UNMEASURED: longest manoeuvre */
#define CTRL_AUTOSTART_MS   5000u
#define CTRL_LOG_MS         500u

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

#if S3_LINE_FOLLOW_TEST
#define CTRL_LINE_BASE_MM_S        45
#define CTRL_LINE_CORRECT_MM_S     38
#define CTRL_LINE_LEFT_CURVE_MM_S -10
#define CTRL_LINE_RIGHT_CURVE_MM_S  0
#define CTRL_LINE_TIMEOUT_MS      500u
#define CTRL_LINE_START_GRACE_MS  200u
#define CTRL_LINE_CURVE_ENTRY_MS  250u
#define CTRL_LINE_LOST_LIMIT_MS   800u
#define CTRL_LINE_DIRECTION_MEMORY_MS 500u
#define CTRL_LINE_DIRECTION_CHANGE_MS 100u
#define CTRL_LINE_BUTTON_PIN       20u
#define CTRL_LINE_BUTTON_MASK      (UINT32_C(1) << CTRL_LINE_BUTTON_PIN)
#define CTRL_LINE_DEBOUNCE_MS       30u
#endif

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

#if S3_LINE_FOLLOW_TEST
typedef struct
{
    uint32_t candidate_since_ms;
    bool     b_candidate_pressed;
    bool     b_stable_pressed;
    bool     b_initialised;
    bool     b_armed;
    bool     b_restart_pending;
} ctrl_line_button_t;

typedef struct
{
    line_control_t        control;
    uint32_t              last_report_ms;
    uint32_t              running_since_ms;
    uint8_t               position;
    line_control_action_t last_action;
    bool                  b_have_report;
    bool                  b_running;
    bool                  b_action_sent;
} ctrl_line_runtime_t;

static ctrl_line_button_t g_line_button;
static ctrl_line_runtime_t g_line_runtime;
#endif

static void     controller_task (INT stacd, void * exinf);
static void     demo (s2_robot_state_t const * p_st, ctrl_input_t const * p_in);
static void     read_input (ctrl_input_t * p_in);
static void     apply (ctrl_output_t * p_out, ctrl_input_t const * p_in);
static ER       send_action (ctrl_action_t action, uint32_t value);
static uint32_t now_ms (void);
#if S3_LINE_FOLLOW_TEST
static ER       line_button_init (void);
static bool     line_button_press_event (uint32_t now);
static void     line_button_request_reset (s2_robot_state_t const * p_state);
static void     line_button_step (s2_robot_state_t const * p_state,
                                  ctrl_input_t const * p_input);
static void     line_step (s2_robot_state_t const * p_state,
                           ctrl_input_t const * p_input);
static void     line_reset (uint32_t now);
static bool     line_apply_action (line_control_action_t action);
static void     line_log_mode (line_control_mode_t old_mode,
                               line_control_mode_t new_mode,
                               line_control_action_t action);
#endif

/**
 * @brief Create controller_flags, safety_flags (STOP set), remote_q, line_q
 *        and state_mutex. Each global ID is set only once its object exists,
 *        because the WiFi task may already be calling the adapters.
 */
ER
s2_controller_create (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

#if S3_LINE_FOLLOW_TEST
    ER const button_er = line_button_init();

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

    ctrl_init(&g_ctrl, &cfg, now_ms());

#if S3_LINE_FOLLOW_TEST
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
        line_button_step(&st, &in);
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

#if S3_LINE_FOLLOW_TEST
static ER
line_button_init (void)
{
    ER const er = gpio_set_pin(CTRL_LINE_BUTTON_PIN, GPIO_MODE_IN);

    if (er >= E_OK)
    {
        clr_w(GPIO(CTRL_LINE_BUTTON_PIN), GPIO_PDE);
        set_w(GPIO(CTRL_LINE_BUTTON_PIN), GPIO_PUE);
    }

    return (er);
}

static bool
line_button_press_event (uint32_t now)
{
    bool const b_pressed =
        (0u == (in_w(GPIO_IN) & CTRL_LINE_BUTTON_MASK));
    bool b_event = false;

    if (!g_line_button.b_initialised)
    {
        g_line_button.b_candidate_pressed = b_pressed;
        g_line_button.candidate_since_ms = now;
        g_line_button.b_initialised = true;
    }
    else if (b_pressed != g_line_button.b_candidate_pressed)
    {
        g_line_button.b_candidate_pressed = b_pressed;
        g_line_button.candidate_since_ms = now;
    }
    else if ((now - g_line_button.candidate_since_ms)
             >= CTRL_LINE_DEBOUNCE_MS)
    {
        if (!b_pressed)
        {
            g_line_button.b_armed = true;
        }

        if (b_pressed != g_line_button.b_stable_pressed)
        {
            g_line_button.b_stable_pressed = b_pressed;

            if (b_pressed && g_line_button.b_armed)
            {
                g_line_button.b_armed = false;
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
line_button_request_reset (s2_robot_state_t const * p_state)
{
    bool const b_wheels_stopped =
        (0u != (p_state->flags & S2_RS_STOPPED));

    if (!b_wheels_stopped)
    {
        tm_printf((UB *)"[S2] GP20: RESET waiting; wheels not stopped\n");
    }
    else if (s2_local_command(S2_REMOTE_RESET))
    {
        g_line_button.b_restart_pending = true;

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
line_button_step (s2_robot_state_t const * p_state,
                  ctrl_input_t const * p_input)
{
    bool const b_press = line_button_press_event(p_input->now_ms);

    if (g_line_button.b_restart_pending)
    {
        if (S2_SYS_READY == p_state->state)
        {
            if (s2_local_command(S2_REMOTE_START))
            {
                g_line_button.b_restart_pending = false;
                tm_printf((UB *)"[S2] GP20: START requested\n");
            }
        }
        else if (S2_SYS_FAULT == p_state->state)
        {
            g_line_button.b_restart_pending = false;
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
            line_button_request_reset(p_state);
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

static void
line_step (s2_robot_state_t const * p_state, ctrl_input_t const * p_input)
{
    s2_line_report_t report;
    bool const b_system_running = (S2_SYS_RUNNING == p_state->state);
    bool b_report_valid = true;
    line_control_action_t action;
    line_control_mode_t old_mode;
    line_control_mode_t new_mode;

    if (b_system_running && (!g_line_runtime.b_running))
    {
        line_reset(p_input->now_ms);
        g_line_runtime.b_running = true;
        g_line_runtime.running_since_ms = p_input->now_ms;
    }

    while (tk_rcv_mbf(line_q, &report, TMO_POL)
           == (INT)sizeof(s2_line_report_t))
    {
        bool const b_this_valid = b_system_running
            && (report.position <= S2_LINE_STOP)
            && (0u == report.reserved[0])
            && (0u == report.reserved[1])
            && (0u == report.reserved[2])
            && ((p_input->now_ms - report.captured_ms)
                <= S2_LINE_MAX_AGE_MS);

        if (b_system_running)
        {
            b_report_valid = b_report_valid && b_this_valid;

            if (b_this_valid)
            {
                g_line_runtime.last_report_ms = report.captured_ms;
                g_line_runtime.position = report.position;
                g_line_runtime.b_have_report = true;
            }
        }
    }

    if (!b_system_running)
    {
        if (g_line_runtime.b_running)
        {
            line_reset(p_input->now_ms);
        }
        return;
    }

    if (!b_report_valid)
    {
        tm_printf((UB *)"[S2] line: invalid report; STOP\n");
        (void)s2_line_stop();
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

    if ((!g_line_runtime.b_action_sent)
        || (action != g_line_runtime.last_action))
    {
        if (!line_apply_action(action))
        {
            tm_printf((UB *)"[S2] line: motion command failed; STOP\n");
            (void)s2_line_stop();
        }
        else
        {
            g_line_runtime.last_action = action;
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
        CTRL_LINE_LOST_LIMIT_MS,
        CTRL_LINE_DIRECTION_MEMORY_MS,
        CTRL_LINE_DIRECTION_CHANGE_MS
    };

    (void)line_control_init(&g_line_runtime.control, &cfg, now);
    g_line_runtime.last_report_ms = now;
    g_line_runtime.running_since_ms = now;
    g_line_runtime.position = S2_LINE_STOP;
    g_line_runtime.last_action = LINE_CONTROL_ACTION_STOP;
    g_line_runtime.b_have_report = false;
    g_line_runtime.b_running = false;
    g_line_runtime.b_action_sent = false;
}

static bool
line_apply_action (line_control_action_t action)
{
    ER er = E_PAR;

    switch (action)
    {
        case LINE_CONTROL_ACTION_STRAIGHT:
            er = motion_set_speeds(CTRL_LINE_BASE_MM_S,
                                   CTRL_LINE_BASE_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_LEFT:
            er = motion_set_speeds(CTRL_LINE_CORRECT_MM_S,
                                   CTRL_LINE_BASE_MM_S);
        break;

        case LINE_CONTROL_ACTION_CORRECT_RIGHT:
            er = motion_set_speeds(CTRL_LINE_BASE_MM_S,
                                   CTRL_LINE_CORRECT_MM_S);
        break;

        case LINE_CONTROL_ACTION_CURVE_LEFT:
            er = motion_set_speeds(CTRL_LINE_LEFT_CURVE_MM_S,
                                   CTRL_LINE_BASE_MM_S);
        break;

        case LINE_CONTROL_ACTION_CURVE_RIGHT:
            er = motion_set_speeds(CTRL_LINE_BASE_MM_S,
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
        if (LINE_CONTROL_MODE_LOST_RECOVERY == new_mode)
        {
            static UB const left[] = "LEFT";
            static UB const right[] = "RIGHT";
            UB const * p_direction =
                (LINE_CONTROL_ACTION_CURVE_LEFT == action) ? left : right;

            tm_printf((UB *)"[S2] line: WHITE/WHITE; searching %s\n",
                      p_direction);
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

        tm_printf((UB *)"[S2] line: sustained %s; strong curve\n",
                  p_direction);
    }

    if ((LINE_CONTROL_ACTION_STOP == action)
        && ((!g_line_runtime.b_action_sent)
            || (LINE_CONTROL_ACTION_STOP != g_line_runtime.last_action)))
    {
        tm_printf((UB *)"[S2] line: lost/recovery failed; STOP\n");
    }
}
#endif

/* End of file. */
