/**
 * @file s2_integration_main.c
 * @brief Host integration tests of the Vehicle Controller with the other
 *        subsystems' interfaces (INT-08 onwards, testing/integration).
 *
 * The real task_controller.c is compiled in (so its private state can be
 * checked), with the real controller_logic, line_control, scan_control,
 * bypass_control, nav_control, robot_state, s2_bridge (the S3 and S1 submit
 * calls) and motion_api. kernel_model.c stands in for the kernel. Around
 * them:
 *  - Motion Control: the real motion_logic.c, driven in the same order as
 *    motion_task() every 10 ms, with a first-order drivetrain that turns
 *    PWM duty into encoder counts (ideal wheels, no slip);
 *  - S3: a geometric track (18 mm tape, the two front sensors 30 mm apart
 *    and 133 mm ahead of the axle), filtered with the real s3_ir_filter()
 *    for 60 ms and reported as s3_follow.c does (on change, else every
 *    30 ms); barcode letters are submitted with the real s2_nav_submit();
 *  - S5: answers scan_q as scan_task.c does, with the result each test
 *    asks for, and raises CTRL_FLG_HAZARD at a set point on the track;
 *  - S4: raises CTRL_FLG_PICKUP when a test asks; terrain status is empty.
 * Every step is one 10 ms period: S3 report, Motion Control period, then
 * one controller period (controller_cycle()).
 *
 * This is synthetic: ideal wheels and sensors, no timing jitter. It checks
 * that the interfaces and state machines fit together; it is not evidence
 * that the car does (testing/README.md).
 */
#include "../../app_program/subsystem2/task_controller.c"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kernel_model.h"
#include "../../app_program/subsystem2/s2_bridge.h"
#include "../../app_program/subsystem2/drv_motor.h"
#include "../../app_program/subsystem2/motion_config.h"
#include "../../app_program/subsystem2/motion_logic.h"
#include "../../app_program/subsystem3/s3_ir.h"
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem5/include/scan_telemetry.h"

/* Interface objects, created by s2_controller_create() and here. */
EXPORT ID controller_flags = 0;
EXPORT ID safety_flags = 0;
EXPORT ID remote_q = 0;
EXPORT ID line_q = 0;
EXPORT ID scan_q = 0;
EXPORT ID nav_q = 0;
EXPORT ID motion_q = 0;

#define SIM_PERIOD_MS       10u
#define SIM_TAPE_HALF_MM    9       /* 18 mm tape */
#define SIM_SENSOR_HALF_MM  15      /* sensors 30 mm apart */
#define SIM_SENSOR_AHEAD_MM 133     /* CTRL_BYPASS_SENSOR_AHEAD_MM */
#define SIM_FILTER_SAMPLES  6u      /* 60 ms at 10 ms (S3 decision 10) */
#define SIM_REPORT_MS       30u     /* S3_LINE_REPORT_MS */
#define SIM_VMAX_MM_S       500     /* full duty, no load (model) */
#define SIM_TAU_MS          40      /* drivetrain time constant (model) */
#define SIM_SCAN_MS         5000u   /* S5 scan time (demo log: 4 to 6 s) */
#define SIM_COUNT_BUF       8

/* ------------------------------------------------------------------------
 *  Track: a straight line along x up to a junction at junction_mm, with
 *  optional branches to the left (+y) and right (-y) and an optional
 *  continuation. Coordinates are Motion Control's pose since NEW_RUN.
 * ------------------------------------------------------------------------ */
typedef struct
{
    int32_t junction_mm;    /* 0: no junction */
    bool    b_left;
    bool    b_right;
    bool    b_ahead;        /* the line carries on past the junction */
    int32_t hazard_mm;      /* S5 raises HAZARD here once; 0: none */
    uint8_t scan_result;    /* SCAN_TASK_RESULT_* S5 gives */
} sim_track_t;

typedef struct
{
    motion_t      motion;
    int32_t       speed_um_s[2];    /* actual wheel speed, um/s */
    int64_t       count_milli[2];   /* encoder counts x1000 */
    int32_t       duty[2];
    uint32_t      now_ms;
    sim_track_t   track;
    s3_filter_t   filter;
    uint32_t      last_report_ms;
    uint8_t       last_position;
    bool          b_reporting;      /* S3 loop running */
    bool          b_hazard_done;
    uint32_t      scan_request;     /* S5: request being served */
    bool          b_scan_authorised;
    uint32_t      scan_since_ms;
    uint8_t       scan_answer;      /* result for scan_request */
    uint32_t      scan_answer_id;
    uint32_t      scan_answer_ms;
    bool          b_still_at_authorise; /* stop-before-scan held */
    uint32_t      authorise_count;
    uint32_t      navigate_periods; /* periods seen in NAVIGATE */
    uint32_t      scan_periods;     /* periods seen in SCAN */
    int32_t       max_speed_after_stop;
} sim_t;

static sim_t g_sim;

/* The same settings, in the same order, as g_motion_cfg in task_motion.c
   (static there); keep the two in step. */
static motion_cfg_t const g_sim_motion_cfg =
{
    {MC_COUNTS_PER_REV, MC_WHEEL_CIRC_UM, MC_TRACK_MM},
    {MC_PID_KP, MC_PID_KI, MC_PID_KD,
     -MC_CONTROL_DUTY_MAX, MC_CONTROL_DUTY_MAX},
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
static unsigned int g_cases = 0u;
static unsigned int g_failed = 0u;

static void sim_check (bool b_ok, char const * p_id, char const * p_name);
static void sim_init (sim_track_t const * p_track);
static void sim_step (void);
static void sim_run_ms (uint32_t duration_ms);
static bool sim_run_until (bool (*p_done)(void), uint32_t limit_ms);
static void sim_motion_period (void);
static void sim_s3_period (void);
static void sim_s5_period (void);
static uint8_t sim_sensors (void);
static bool sim_on_tape (int32_t px_mm, int32_t py_mm);
static void sim_state (s2_robot_state_t * p_state);
static bool sim_running (void);
static bool sim_following (void);
static bool sim_navigating (void);
static int32_t sim_heading_mdeg (void);
static int32_t sim_x_mm (void);
static void sim_start (void);

/* ------------------------------------------------------------------------
 *  Stand-ins for the other tasks' calls the controller makes.
 * ------------------------------------------------------------------------ */
ER
motion_get_status (motion_status_t * p_status)
{
    motion_get_status_logic(&g_sim.motion, p_status);
    p_status->captured_ms = g_sim.now_ms;
    return (E_OK);
}

bool
motion_stop_confirmed (void)
{
    motion_status_t status;

    return ((E_OK == motion_get_status(&status))
            && (0u != (status.flags & MOTION_ST_STOPPED)));
}

void
s2_motion_wake (void)
{
    /* The next sim_motion_period() runs at once in the same period. */
}

void
s2_motion_note_latch (void)
{
}

ID
app_start_task (FP entry, PRI pri, UW prc, const char * p_name)
{
    (void)entry;
    (void)pri;
    (void)prc;
    (void)p_name;
    return (1);
}

ER
scan_task_get (scan_task_status_t * p_status)
{
    scan_task_status_t const zero = {0};

    *p_status = zero;
    p_status->captured_ms = g_sim.now_ms;
    p_status->request_id = g_sim.scan_answer_id;
    p_status->result = g_sim.scan_answer;
    return (E_OK);
}

bool
scan_telemetry_get (scan_telemetry_t * p_out)
{
    scan_telemetry_t const zero = {0};

    *p_out = zero;
    return (true);
}

ER
subsystem4_get_status (s4_terrain_result_t * p_result)
{
    s4_terrain_result_t const zero = {0};

    *p_result = zero;
    return (E_OK);
}

/* ------------------------------------------------------------------------
 *  Simulation.
 * ------------------------------------------------------------------------ */
static void
sim_init (sim_track_t const * p_track)
{
    motion_output_t out;
    T_CMBF          cmbf = {0};
    sim_t const     zero = {0};

    model_reset();
    g_sim = zero;
    g_sim.track = *p_track;
    g_sim.last_position = UINT8_MAX;
    g_sim.max_speed_after_stop = 0;
    g_sim.now_ms = 1000u;
    host_set_time_ms(g_sim.now_ms);

    (void)s2_controller_create();
    (void)s2_robot_state_create();
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = MC_MOTION_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_motion_cmd_t);
    motion_q = tk_cre_mbf(&cmbf);

    /* The controller's statics carry over between cases in one process,
       so each case starts them as a fresh boot would. */
    {
        ctrl_line_runtime_t const line_zero = {0};
        ctrl_barcode_t const barcode_zero = {0};
        ctrl_button_t const button_zero = {0};

        g_line_runtime = line_zero;
        g_barcode = barcode_zero;
        g_button = button_zero;
        g_local_cause = S2_STOP_NONE;
    }
    (void)motion_init(&g_sim.motion, &g_sim_motion_cfg);
    motion_output_clear(&out);
    motion_start(&g_sim.motion, true, &out);
    controller_setup();
}

static void
sim_check (bool b_ok, char const * p_id, char const * p_name)
{
    g_cases++;
    if (!b_ok)
    {
        g_failed++;
    }
    printf("[HOST] %s %s  %s\n", p_id, b_ok ? "PASS" : "FAIL", p_name);
}

/* One 10 ms period: S3, Motion Control, S5, then the controller. */
static void
sim_step (void)
{
    ctrl_input_t inputs = {0};
    UINT ptn = 0u;
    s2_robot_state_t state;

    g_sim.now_ms += SIM_PERIOD_MS;
    host_set_time_ms(g_sim.now_ms);

    sim_s3_period();
    sim_motion_period();
    sim_s5_period();

    if (tk_wai_flg(controller_flags, CTRL_FLG_ALL, TWF_ORW, &ptn,
                   CTRL_WATCHDOG_MS) < E_OK)
    {
        ptn = 0u;
    }
    controller_cycle(ptn, &inputs);

    sim_state(&state);
    if (S2_MODE_NAVIGATE == state.mode)
    {
        g_sim.navigate_periods++;
    }
    if (S2_MODE_SCAN == state.mode)
    {
        g_sim.scan_periods++;
    }
}

static void
sim_run_ms (uint32_t duration_ms)
{
    for (uint32_t elapsed = 0u; elapsed < duration_ms;
         elapsed += SIM_PERIOD_MS)
    {
        sim_step();
    }
}

static bool
sim_run_until (bool (*p_done)(void), uint32_t limit_ms)
{
    for (uint32_t elapsed = 0u; elapsed < limit_ms; elapsed += SIM_PERIOD_MS)
    {
        sim_step();
        if (p_done())
        {
            return (true);
        }
    }

    return (false);
}

/* motion_task(): commands, step, duty; then the drivetrain moves. */
static void
sim_motion_period (void)
{
    motion_input_t  in = {0};
    motion_output_t out;
    s2_motion_cmd_t cmd;
    T_RFLG          rflg;

    in.now_us = g_sim.now_ms * 1000u;
    in.count_l = (uint32_t)(g_sim.count_milli[0] / 1000);
    in.count_r = (uint32_t)(g_sim.count_milli[1] / 1000);
    in.b_counts_valid = true;
    in.b_stop = (tk_ref_flg(safety_flags, &rflg) < E_OK)
                || (0u != (rflg.flgptn & SAFETY_FLG_STOP));

    motion_output_clear(&out);
    while (tk_rcv_mbf(motion_q, &cmd, TMO_POL) == (INT)sizeof(cmd))
    {
        motion_command(&g_sim.motion, &cmd, &in, &out);
    }
    motion_step(&g_sim.motion, &in, &out);
    g_sim.duty[0] = out.duty_l;
    g_sim.duty[1] = out.duty_r;
    if (out.b_raise_fault)
    {
        (void)tk_set_flg(controller_flags, CTRL_FLG_MOTION_FAULT);
    }

    for (uint8_t wheel = 0u; wheel < 2u; wheel++)
    {
        /* First-order lag towards duty x VMAX / DUTY_MAX. */
        int64_t const target_um_s = ((int64_t)g_sim.duty[wheel]
                                     * SIM_VMAX_MM_S * 1000)
                                    / DRV_MOTOR_DUTY_MAX;
        int64_t const delta = ((target_um_s - g_sim.speed_um_s[wheel])
                               * SIM_PERIOD_MS) / SIM_TAU_MS;

        g_sim.speed_um_s[wheel] += (int32_t)delta;
        /* um travelled this period x counts per um, kept x1000. */
        g_sim.count_milli[wheel] += ((int64_t)g_sim.speed_um_s[wheel]
                                     * SIM_PERIOD_MS * MC_COUNTS_PER_REV)
                                    / (int64_t)MC_WHEEL_CIRC_UM;
    }

    if (in.b_stop)
    {
        int32_t const fastest =
            (abs(g_sim.speed_um_s[0]) > abs(g_sim.speed_um_s[1]))
            ? abs(g_sim.speed_um_s[0]) : abs(g_sim.speed_um_s[1]);

        if ((0 != g_sim.duty[0]) || (0 != g_sim.duty[1]))
        {
            /* Any duty while STOP is latched is recorded as a speed far
               above anything real, so the check fails loudly. */
            g_sim.max_speed_after_stop = INT32_MAX;
        }
        else if (fastest > g_sim.max_speed_after_stop)
        {
            g_sim.max_speed_after_stop = fastest;
        }
        else
        {
            /* Coasting down. */
        }
    }
}

/* s3_follow_run(): filter the two front sensors for 60 ms; report on a
   change, else every 30 ms. */
static void
sim_s3_period (void)
{
    uint8_t const mask = sim_sensors();

    if (!g_sim.b_reporting)
    {
        return;
    }
    if (s3_ir_filter(&g_sim.filter, mask, (uint8_t)SIM_FILTER_SAMPLES))
    {
        uint8_t position = S2_LINE_CENTER;

        if (S3_LINE_LEFT == s3_ir_follow_decision(g_sim.filter.stable))
        {
            position = S2_LINE_LEFT;
        }
        else if (S3_LINE_RIGHT == s3_ir_follow_decision(g_sim.filter.stable))
        {
            position = S2_LINE_RIGHT;
        }
        else if (S3_LINE_JUNCTION
                 == s3_ir_follow_decision(g_sim.filter.stable))
        {
            position = S2_LINE_BOTH_BLACK;
        }
        else
        {
            /* Both white: centred. */
        }

        if ((position != g_sim.last_position)
            || ((g_sim.now_ms - g_sim.last_report_ms) >= SIM_REPORT_MS))
        {
            if (s2_line_submit_scaled(position,
                                      ((S2_LINE_LEFT == position)
                                       || (S2_LINE_RIGHT == position))
                                      ? 50u : 0u, 0u, g_sim.now_ms))
            {
                g_sim.last_position = position;
                g_sim.last_report_ms = g_sim.now_ms;
            }
        }
    }
}

/* scan_task.c: raise HAZARD at the obstacle, answer the handshake. */
static void
sim_s5_period (void)
{
    s2_scan_cmd_t    cmd;
    s2_robot_state_t state;

    sim_state(&state);
    if ((0 != g_sim.track.hazard_mm) && (!g_sim.b_hazard_done)
        && (S2_MODE_FOLLOW == state.mode)
        && (sim_x_mm() >= g_sim.track.hazard_mm))
    {
        (void)tk_set_flg(controller_flags, CTRL_FLG_HAZARD);
        g_sim.b_hazard_done = true;
    }

    while (tk_rcv_mbf(scan_q, &cmd, TMO_POL) == (INT)sizeof(cmd))
    {
        if ((S2_SCAN_REQUEST == cmd.kind) && (0u != cmd.b_asserted))
        {
            g_sim.scan_request = cmd.request_id;
            g_sim.b_scan_authorised = false;
        }
        else if ((S2_SCAN_AUTHORISE == cmd.kind) && (0u != cmd.b_asserted)
                 && (cmd.request_id == g_sim.scan_request))
        {
            g_sim.b_scan_authorised = true;
            g_sim.scan_since_ms = g_sim.now_ms;
            g_sim.b_still_at_authorise = motion_stop_confirmed();
            g_sim.authorise_count++;
        }
        else
        {
            /* STOP_CONFIRM, CANCEL, STOP and the rest need no answer. */
        }
    }

    if (g_sim.b_scan_authorised
        && ((g_sim.now_ms - g_sim.scan_since_ms) >= SIM_SCAN_MS))
    {
        g_sim.scan_answer_id = g_sim.scan_request;
        g_sim.scan_answer = g_sim.track.scan_result;
        g_sim.scan_answer_ms = g_sim.now_ms;
        g_sim.b_scan_authorised = false;
    }
}

/* Front sensor mask (bit 0 left, bit 1 right) from the pose. */
static uint8_t
sim_sensors (void)
{
    motion_status_t status;
    uint8_t mask = 0u;

    (void)motion_get_status(&status);
    {
        /* heading in mdeg, left positive; the sensors sit ahead of the
           axle, one each side. Floating point is fine in a host model. */
        double const h = ((double)status.heading_mdeg / 1000.0)
                         * 3.14159265358979 / 180.0;
        double const ax = (double)status.x_mm;
        double const ay = (double)status.y_mm;
        double const fx = ax + (SIM_SENSOR_AHEAD_MM * cos(h));
        double const fy = ay + (SIM_SENSOR_AHEAD_MM * sin(h));
        /* Left of the heading is +90 degrees. */
        double const lx = fx - (SIM_SENSOR_HALF_MM * sin(h));
        double const ly = fy + (SIM_SENSOR_HALF_MM * cos(h));
        double const rx = fx + (SIM_SENSOR_HALF_MM * sin(h));
        double const ry = fy - (SIM_SENSOR_HALF_MM * cos(h));

        if (sim_on_tape((int32_t)lround(lx), (int32_t)lround(ly)))
        {
            mask |= S3_IR_LEFT_MASK;
        }
        if (sim_on_tape((int32_t)lround(rx), (int32_t)lround(ry)))
        {
            mask |= S3_IR_RIGHT_MASK;
        }
    }

    return (mask);
}

static bool
sim_on_tape (int32_t px_mm, int32_t py_mm)
{
    sim_track_t const * const p_t = &g_sim.track;
    int32_t const j = p_t->junction_mm;
    bool b_on = false;

    /* The start line: y = 0 from far behind to the junction (or on). */
    if ((abs(py_mm) <= SIM_TAPE_HALF_MM)
        && ((0 == j) || p_t->b_ahead || (px_mm <= (j + SIM_TAPE_HALF_MM))))
    {
        b_on = true;
    }
    if ((0 != j) && (abs(px_mm - j) <= SIM_TAPE_HALF_MM))
    {
        /* The cross line at the junction, to each side that exists. */
        b_on = b_on || (p_t->b_left && (py_mm >= -SIM_TAPE_HALF_MM))
               || (p_t->b_right && (py_mm <= SIM_TAPE_HALF_MM));
    }

    return (b_on);
}

static void
sim_state (s2_robot_state_t * p_state)
{
    (void)s2_get_robot_state(p_state);
}

static bool
sim_running (void)
{
    s2_robot_state_t state;

    sim_state(&state);
    return (S2_SYS_RUNNING == state.state);
}

static bool
sim_following (void)
{
    s2_robot_state_t state;

    sim_state(&state);
    return ((S2_SYS_RUNNING == state.state)
            && (S2_MODE_FOLLOW == state.mode));
}

static bool
sim_navigating (void)
{
    s2_robot_state_t state;

    sim_state(&state);
    return (S2_MODE_NAVIGATE == state.mode);
}

static int32_t
sim_heading_mdeg (void)
{
    motion_status_t status;

    (void)motion_get_status(&status);
    return (status.heading_mdeg);
}

static int32_t
sim_x_mm (void)
{
    motion_status_t status;

    (void)motion_get_status(&status);
    return (status.x_mm);
}

/* Power-up to RUNNING: S3 starts reporting, the remote START arrives. */
static void
sim_start (void)
{
    s2_remote_cmd_t cmd;
    static uint32_t request_id = 100u;

    g_sim.b_reporting = true;
    sim_run_ms(500u);       /* boot: INITIALISING to READY */
    request_id++;
    (void)s2_remote_make(S2_REMOTE_START, request_id, g_sim.now_ms, &cmd);
    (void)tk_snd_mbf(remote_q, &cmd, (INT)sizeof(cmd), TMO_POL);
    (void)sim_run_until(sim_running, 1000u);
}

#include "s2_integration_cases.c"

int
main (void)
{
    s2_integration_cases();
    printf("[HOST] s2 integration SUMMARY %u passed, %u failed\n",
           g_cases - g_failed, g_failed);
    return ((0u == g_failed) ? 0 : 1);
}

/* End of file. */
