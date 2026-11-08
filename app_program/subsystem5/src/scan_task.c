/**
 * @file scan_task.c
 * @brief Obstacle Scan task: forward monitoring, scans and bypass plans.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Every 10 ms a cyclic handler wakes the task, which:
 *   1. passes scan_q commands from the controller to the scan engine;
 *   2. passes queued ECHO edges to the engine while scanning, or to the
 *      forward capture otherwise;
 *   3. ticks the engine (settling, triggers, deadlines, 15 s limit);
 *   4. while not scanning, takes one forward reading every 65 ms;
 *   5. raises CTRL_FLG_HAZARD when the forward verdict turns to a stop in
 *      a moving mission mode, and CTRL_FLG_SCAN_FAULT on a scan fault;
 *   6. publishes the snapshot read by scan_task_get().
 * Every value marked UNMEASURED is a provisional starting point.
 */
#include "../include/scan_task.h"

#include "../include/drv_servo.h"
#include "../include/drv_ultrasonic.h"
#include "../include/forward_monitor.h"
#include "../include/range_measurement.h"
#include "../include/scan_engine.h"
#include "../include/scan_lifecycle.h"
#include "../../subsystem2/subsystem2.h"
#include "../../subsystem2/motion_api.h"

#define SCAN_TASK_PERIOD_MS     (10)
#define SCAN_TASK_PUBLISH_MS    (2)

/* Agreed (decision 3): trigger spacing, echo deadline, freshness. */
static uint32_t const g_task_retrigger_us = 65000u;
static uint32_t const g_task_deadline_us  = 30000u;
static uint32_t const g_task_stuck_us     = 130000u;
static uint32_t const g_task_fresh_us     = 150000u;
static uint32_t const g_task_result_limit_us = 2000000u;

/* UNMEASURED: worst servo settling + 25% (S5-U47) and the hazard
   distance from braking and response (S5-U38). */
static uint32_t const g_task_settling_us  = 300000u;
static uint16_t const g_task_hazard_mm    = 250u;

/* Bypass geometry: half width and margin UNMEASURED; 50 mm clearance and
   the 500 mm search bound agreed (decisions 5 and 9); assumed depth. */
static bypass_plan_config_t const g_task_plan_config =
{
    80u, 50u, 30u, 200u, 500u, 4000u
};

static ID                 g_task_tid = 0;
static ID                 g_task_cyc = 0;
static ID                 g_task_mtx = 0;
static scan_task_status_t g_task_status;    /* guarded by g_task_mtx */

/* Scan task only. */
static scan_task_status_t g_work;
static scan_engine_t      g_engine;
static uint16_t           g_profile[BYPASS_PLAN_FINE_ANGLES];
static forward_monitor_t  g_monitor;
static uint32_t           g_monitor_since_us;
static uint32_t           g_capture_us;
static uint32_t           g_capture_rise_us;
static bool               gb_capture_armed;
static bool               gb_capture_rise;
static bool               gb_capture_used;
static bool               gb_was_scanning;
static bool               gb_was_hazard;

/** Wake the task once per period (interrupt context). */
static void scan_task_cyclic (void * p_exinf);

/** The task body. */
static void scan_task_main (INT stacd, void * p_exinf);

/** Engine clock port. */
static uint32_t scan_task_now_us (void);

/** Engine output port: servo, trigger, profile and results. */
static bool scan_task_publish (scan_engine_output_t const * p_output);

/** Map one scan_q command onto the engine. */
static void scan_task_command (s2_scan_cmd_t const * p_cmd);

/** One ECHO edge for the forward capture. */
static void scan_task_forward_edge (bool b_high, uint32_t time_us);

/** Forward capture timing: deadline, spacing, stuck ECHO. */
static void scan_task_forward_step (uint32_t now_us);

/** Close the forward capture with one attempt's result. */
static void scan_task_forward_result (bool b_valid, uint16_t distance_mm,
                                      uint32_t now_us);

/** Latch a forward sensor fault and notify the controller once. */
static void scan_task_forward_fault (void);

/** True when the mission mode moves the vehicle (S5 hazards apply). */
static bool scan_task_moving (void);

/** Abort active work if the controller left SCAN or the wheels moved. */
static void scan_task_check_authority (void);

/** Invalidate an unaccepted result after its two-second lifetime. */
static void scan_task_expire_result (uint32_t now_us);

ER
scan_task_create (void)
{
    T_CMTX cmtx = {0};
    T_CCYC ccyc = {0};

    cmtx.mtxatr = TA_INHERIT;

    ID const mtx = tk_cre_mtx(&cmtx);

    if (mtx < E_OK)
    {
        return (mtx);
    }

    g_task_mtx  = mtx;
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP) scan_task_cyclic;
    ccyc.cyctim = SCAN_TASK_PERIOD_MS;
    ccyc.cycphs = SCAN_TASK_PERIOD_MS;
    g_task_cyc  = tk_cre_cyc(&ccyc);

    return ((g_task_cyc < E_OK) ? g_task_cyc : E_OK);
}

ER
scan_task_start (void)
{
    scan_engine_port_t const   port   = {scan_task_now_us,
                                         scan_task_publish};
    scan_engine_config_t const config = {g_task_settling_us,
                                         g_task_hazard_mm,
                                         g_task_plan_config.depth_mm};
    ER er = drv_servo_init();

    if (E_OK == er)
    {
        er = drv_ultrasonic_init();
    }

    if (E_OK != er)
    {
        return (er);
    }

    scan_engine_reset(&g_engine, &port, &config);
    forward_monitor_reset(&g_monitor);
    g_monitor_since_us = scan_task_now_us();

    ID const tid = app_start_task((FP) scan_task_main, OBSTACLE_SCAN_PRIORITY,
                                  0u, "obstacle_scan");

    if (tid < E_OK)
    {
        return (tid);
    }

    g_task_tid = tid;

    return (tk_sta_cyc(g_task_cyc));
}

ER
scan_task_get (scan_task_status_t * p_status)
{
    ID const mtx = g_task_mtx;
    ER       er  = E_PAR;

    if (NULL != p_status)
    {
        er = (mtx <= 0) ? E_OBJ : tk_loc_mtx(mtx, TMO_POL);

        if (E_OK == er)
        {
            *p_status = g_task_status;
            (void) tk_unl_mtx(mtx);

            if (((SCAN_TASK_RESULT_PLAN == p_status->result)
                 || (SCAN_TASK_RESULT_CLEAR == p_status->result))
                && ((scan_task_now_us() - p_status->result_us)
                    > g_task_result_limit_us))
            {
                p_status->result = SCAN_TASK_RESULT_NONE;
            }
        }

    }

    return (er);
}

static void
scan_task_cyclic (void * p_exinf)
{
    (void) p_exinf;

    if (g_task_tid > 0)
    {
        (void) tk_wup_tsk(g_task_tid);
    }

}

static void
scan_task_main (INT stacd, void * p_exinf)
{
    forward_monitor_config_t const monitor_config =
    {
        g_task_hazard_mm, g_task_fresh_us
    };

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        s2_scan_cmd_t cmd;
        bool          b_high  = false;
        uint32_t      edge_us = 0u;

        (void) tk_slp_tsk(TMO_FEVR);
        (void) tk_can_wup(TSK_SELF);

        /* 1. Controller commands. */
        while (tk_rcv_mbf(scan_q, &cmd, TMO_POL)
               == (INT) sizeof(s2_scan_cmd_t))
        {
            scan_task_command(&cmd);
        }

        if (g_engine.b_is_scanning)
        {
            gb_capture_armed = false;   /* discard the old forward capture */

            if (gb_was_scanning)
            {
                scan_task_check_authority();
            }
        }

        /* 2. ECHO edges in time order. */
        while (drv_ultrasonic_edge(&b_high, &edge_us))
        {
            if (g_engine.b_is_scanning)
            {
                scan_engine_event_t const echo =
                {
                    SCAN_ENGINE_ECHO_EDGE, g_engine.request_id, b_high,
                    edge_us
                };

                scan_engine_process(&g_engine, &echo);
            }
            else
            {
                scan_task_forward_edge(b_high, edge_us);
            }

        }

        if (drv_ultrasonic_overrun() && gb_capture_armed)
        {
            scan_task_forward_result(false, 0u, scan_task_now_us());
        }

        /* 3. Engine timing. */
        scan_engine_event_t const tick = {SCAN_ENGINE_TICK, 0u, false, 0u};

        scan_engine_process(&g_engine, &tick);

        uint32_t const now_us = scan_task_now_us();

        scan_task_expire_result(now_us);

        /* 4. Forward monitoring, restarted after each scan. */
        if (g_engine.b_is_scanning)
        {
            gb_capture_armed = false;
        }
        else
        {
            if (gb_was_scanning)
            {
                forward_monitor_reset(&g_monitor);
                g_monitor_since_us = now_us;
                if (g_engine.b_has_trigger)
                {
                    g_capture_us = g_engine.trigger_us;
                    gb_capture_used = true;
                }
            }

            scan_task_forward_step(now_us);
        }

        gb_was_scanning = g_engine.b_is_scanning;

        /* 5. Hazard on the transition to a stop verdict while moving. The
              150 ms freshness counts from a monitoring restart until the
              first reading arrives. */
        forward_monitor_verdict_t verdict =
            forward_monitor_check(&g_monitor, &monitor_config, now_us);

        if ((FORWARD_MONITOR_STALE == verdict) && (!g_monitor.b_has_sample)
            && ((now_us - g_monitor_since_us) <= g_task_fresh_us))
        {
            verdict = FORWARD_MONITOR_SAFE;
        }

        bool const b_hazard = (FORWARD_MONITOR_SAFE != verdict)
                              && (!g_engine.b_is_scanning)
                              && scan_task_moving();

        if (b_hazard && (!gb_was_hazard))
        {
            (void) tk_set_flg(controller_flags, CTRL_FLG_HAZARD);
        }

        gb_was_hazard = b_hazard;

        /* 6. Publish. */
        SYSTIM tim;

        g_work.captured_ms = (tk_get_tim(&tim) >= E_OK) ? tim.lo : 0u;
        g_work.verdict     = (uint8_t) verdict;
        g_work.forward_mm  = g_monitor.b_is_valid ? g_monitor.distance_mm
                                                  : 0u;
        g_work.flags       = (uint8_t) (
            (g_engine.b_is_scanning ? SCAN_TASK_FLAG_SCANNING
                                    : SCAN_TASK_FLAG_MONITORING)
            | ((FORWARD_MONITOR_SAFE != verdict) ? SCAN_TASK_FLAG_HAZARD
                                                 : 0u));

        if (tk_loc_mtx(g_task_mtx, SCAN_TASK_PUBLISH_MS) >= E_OK)
        {
            g_task_status = g_work;
            (void) tk_unl_mtx(g_task_mtx);
        }

    }

}

static uint32_t
scan_task_now_us (void)
{
    return (in_w(TIMER_TIMERAWL));
}

static bool
scan_task_publish (scan_engine_output_t const * p_output)
{
    bool b_accepted = true;

    switch (p_output->kind)
    {
        case SCAN_ENGINE_SERVO:
            b_accepted = (E_OK == drv_servo_set_angle(p_output->angle_deg));
        break;

        case SCAN_ENGINE_TRIGGER:
            (void) drv_ultrasonic_trigger();
        break;

        case SCAN_ENGINE_DISTANCE:
        {
            int32_t const angle = (int32_t)p_output->angle_deg + 60;
            int32_t const index = angle / 5;

            if ((angle >= 0) && ((angle % 5) == 0)
                && (index < (int32_t)BYPASS_PLAN_FINE_ANGLES))
            {
                g_profile[index] = p_output->distance_mm;
            }

        }
        break;

        case SCAN_ENGINE_PLAN:
            (void) bypass_plan_make_fine(g_profile, &g_task_plan_config,
                                         &g_work.plan);
            g_work.result_us = scan_task_now_us();

            if (BYPASS_PLAN_OK == g_work.plan.result)
            {
                g_work.result = SCAN_TASK_RESULT_PLAN;
            }
            else if (BYPASS_PLAN_NO_OBSTACLE == g_work.plan.result)
            {
                g_work.result = SCAN_TASK_RESULT_CLEAR;
            }
            else
            {
                g_work.result = SCAN_TASK_RESULT_NO_ROUTE;
            }

        break;

        case SCAN_ENGINE_FAULT:
            g_work.result = SCAN_TASK_RESULT_FAULT;
            (void) tk_set_flg(controller_flags, CTRL_FLG_SCAN_FAULT);
        break;

        case SCAN_ENGINE_CANCEL_ACK:
            g_work.result = SCAN_TASK_RESULT_CANCELLED;
        break;

        default:
            /* HAZARD and READY are not produced by this engine. */
        break;
    }

    return (b_accepted);
}

static void
scan_task_command (s2_scan_cmd_t const * p_cmd)
{
    scan_engine_event_t event =
    {
        SCAN_ENGINE_RESET, p_cmd->request_id, (0u != p_cmd->b_asserted), 0u
    };
    bool b_mapped = true;

    switch (p_cmd->kind)
    {
        case S2_SCAN_REQUEST:
            event.kind = SCAN_ENGINE_REQUEST;

            if (event.b_asserted && (!g_engine.b_has_request))
            {
                bypass_plan_t const cleared = {0};

                g_work.request_id = p_cmd->request_id;
                g_work.result     = SCAN_TASK_RESULT_NONE;
                g_work.plan       = cleared;

                for (uint32_t index = 0u; index < BYPASS_PLAN_FINE_ANGLES;
                     index++)
                {
                    g_profile[index] = 0u;
                }

            }

        break;

        case S2_SCAN_AUTHORISE:
            event.kind = SCAN_ENGINE_AUTHORISE;
        break;

        case S2_SCAN_STOP_CONFIRM:
            event.kind = SCAN_ENGINE_STOP_CONFIRM;
        break;

        case S2_SCAN_CANCEL:
            event.kind = SCAN_ENGINE_CANCEL;
        break;

        case S2_SCAN_STOP:
            event.kind = SCAN_ENGINE_STOP;
        break;

        case S2_SCAN_MOVEMENT:
            event.kind = SCAN_ENGINE_MOVEMENT;
        break;

        case S2_SCAN_RESET:
            event.kind = SCAN_ENGINE_RESET;
        break;

        default:
            b_mapped = false;
        break;
    }

    if (b_mapped)
    {
        scan_engine_process(&g_engine, &event);
    }

}

static void
scan_task_forward_edge (bool b_high, uint32_t time_us)
{
    uint32_t const age_us = time_us - g_capture_us;

    if (gb_capture_armed && b_high && (!gb_capture_rise)
        && (age_us < g_task_deadline_us))
    {
        gb_capture_rise   = true;
        g_capture_rise_us = time_us;
    }
    else if (gb_capture_armed && (!b_high) && gb_capture_rise)
    {
        uint16_t   distance_mm = 0u;
        bool const b_valid     = (age_us <= g_task_deadline_us)
            && range_measurement_from_echo(time_us - g_capture_rise_us,
                                           &distance_mm);

        scan_task_forward_result(b_valid, distance_mm, time_us);
    }
    else
    {
        /* Unsolicited or malformed edge: ignored. */
    }

}

static void
scan_task_forward_step (uint32_t now_us)
{
    uint32_t const since_us = now_us - g_capture_us;

    if (gb_capture_armed)
    {
        if (since_us >= g_task_deadline_us)
        {
            scan_task_forward_result(false, 0u, now_us);
        }

    }
    else if (drv_ultrasonic_echo_high())
    {
        /* Never retrigger while ECHO is high; stuck high is invalid. */
        if (gb_capture_used && (since_us >= g_task_stuck_us))
        {
            forward_monitor_sample(&g_monitor, false, 0u, now_us);
            g_work.invalid_count = g_monitor.invalid_count;
            scan_task_forward_fault();
        }

    }
    else if ((!gb_capture_used) || (since_us >= g_task_retrigger_us))
    {
        g_capture_us     = drv_ultrasonic_trigger();
        gb_capture_armed = true;
        gb_capture_rise  = false;
        gb_capture_used  = true;
    }
    else
    {
        /* Waiting out the 65 ms spacing. */
    }

}

static void
scan_task_forward_result (bool b_valid, uint16_t distance_mm,
                          uint32_t now_us)
{
    gb_capture_armed = false;
    forward_monitor_sample(&g_monitor, b_valid, distance_mm, now_us);
    g_work.invalid_count = g_monitor.invalid_count;

    if (forward_monitor_faulted(&g_monitor))
    {
        scan_task_forward_fault();
    }

}

static void
scan_task_forward_fault (void)
{
    g_work.result = SCAN_TASK_RESULT_FAULT;
    (void) tk_set_flg(controller_flags, CTRL_FLG_SCAN_FAULT);
}

static bool
scan_task_moving (void)
{
    s2_robot_state_t state;

    return ((E_OK == s2_get_robot_state(&state))
            && (S2_SYS_RUNNING == state.state)
            && ((S2_MODE_FOLLOW == state.mode)
                || (S2_MODE_BYPASS == state.mode)
                || (S2_MODE_REACQUIRE == state.mode)));
}

static void
scan_task_check_authority (void)
{
    s2_robot_state_t state;
    bool const b_state_ok = (E_OK == s2_get_robot_state(&state))
                            && (S2_SYS_RUNNING == state.state)
                            && (S2_MODE_SCAN == state.mode)
                            && (0u == (state.flags & S2_RS_STOP_LATCHED));

    if (!b_state_ok)
    {
        scan_engine_event_t const stop =
            {SCAN_ENGINE_STOP, g_engine.request_id, true, 0u};

        scan_engine_process(&g_engine, &stop);
    }
    else if (!motion_stop_confirmed())
    {
        scan_engine_event_t const moved =
            {SCAN_ENGINE_MOVEMENT, g_engine.request_id, true, 0u};

        scan_engine_process(&g_engine, &moved);
        scan_task_forward_fault();
    }
    else
    {
        /* The controller still owns a stationary scan. */
    }
}

static void
scan_task_expire_result (uint32_t now_us)
{
    if (((SCAN_TASK_RESULT_PLAN == g_work.result)
         || (SCAN_TASK_RESULT_CLEAR == g_work.result))
        && ((now_us - g_work.result_us) > g_task_result_limit_us))
    {
        bypass_plan_t const cleared = {0};

        g_work.result = SCAN_TASK_RESULT_NONE;
        g_work.plan   = cleared;
    }
}

/* End of file. */
