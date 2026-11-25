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
 *    30 ms); barcode letters are submitted with the real s2_nav_submit()
 *    either directly or, when a case prints a code on the track, from the
 *    side sensor through the real s3_barcode_update() (adaptive level,
 *    capture and Code 39 decoder) and s3_nav_command(), with the
 *    barcode-active flag in each line report, as s3_follow.c does;
 *  - S5: answers scan_q as scan_task.c does, with the result each test
 *    asks for, and raises CTRL_FLG_HAZARD at a set point on the track; or,
 *    when a case places an obstacle box, with ideal ultrasonic rays from
 *    the pose: the real forward_monitor (250 mm, 150 ms, every 70 ms)
 *    raises the hazard, the 19-angle profile is planned by the real
 *    bypass_plan_make_fine() with scan_task.c's settings, and side looks
 *    are answered by a ray. Rays have no beam width (F-05), so this checks
 *    the plan and its execution, not the sensor;
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
#include "../../app_program/subsystem1/telemetry_encoder.h"
#include "../../app_program/subsystem2/s2_s1_adapter.h"
#include "../../app_program/subsystem2/s2_bridge.h"
/* Included, not linked, so a case can clear its records as a boot does. */
#include "../../app_program/subsystem2/task_health.c"
#include "../../app_program/subsystem2/drv_motor.h"
#include "../../app_program/subsystem2/motion_config.h"
#include "../../app_program/subsystem2/motion_logic.h"
#include "../../app_program/subsystem3/s3_ir.h"
#include "../../app_program/subsystem3/s3_barcode.h"
#include "../../app_program/subsystem3/s3_code39.h"
#include "../../app_program/subsystem3/s3_nav.h"
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem4/s4_config.h"
#include "../../app_program/subsystem5/include/scan_telemetry.h"
#include "../../app_program/subsystem5/include/bypass_plan.h"
#include "../../app_program/subsystem5/include/forward_monitor.h"

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
/* Side (barcode) sensor: harness geometry, not a car measurement; a code
   printed beside the line is seen at this distance ahead of the axle. */
#define SIM_SIDE_AHEAD_MM   133
/* ADC levels and print as the S3 host tests (TEST_ADC_WHITE/BLACK; 100
   and 250 ms runs at the 40 mm/s line speed). Synthetic, not measured. */
#define SIM_BAR_WHITE_ADC   170u
#define SIM_BAR_BLACK_ADC   1690u
#define SIM_NARROW_UM       4000u
#define SIM_WIDE_UM         10000u
#define SIM_PRINT_RUNS      40u
/* S5 model (scan_task.c values; keep in step): */
#define SIM_FWD_PERIOD_MS   70u     /* forward reading, triggers >= 65 ms */
#define SIM_LOOK_MS         300u    /* one side look (model) */
#define SIM_HAZARD_MM       250u    /* g_task_hazard_mm */
#define SIM_FRESH_US        150000u /* g_task_fresh_us */
#define SIM_CLEAR_MM        500u    /* g_task_clear_mm */
#define SIM_RANGE_MM        4000.0  /* no echo: beyond range */

/* A code printed beside the line from start_mm along x: run_um[0] black,
   then alternating. */
typedef struct
{
    int32_t  start_mm;
    uint8_t  runs;
    uint32_t run_um[SIM_PRINT_RUNS];
} sim_print_t;

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

/* g_task_plan_config in scan_task.c (static there); keep in step. */
static bypass_plan_config_t const g_sim_plan_cfg =
{
    82u, 63u, 132u, 56u, 81u, 50u, 30u, 200u, 150u, 500u, 4000u, 10u, 300u
};

/* An obstacle box (mm, track coordinates) and the S5 model's state. */
typedef struct
{
    bool               b_on;
    double             x0, x1, y0, y1;
    forward_monitor_t  monitor;
    uint32_t           next_fwd_ms;
    uint16_t           forward_mm;
    bool               b_hazard;     /* verdict is not SAFE */
    bool               b_was_hazard;
    bypass_plan_t      plan;
    uint32_t           look_id;
    uint32_t           look_due_ms;
    int8_t             look_angle;
    bool               b_look_pending;
    uint8_t            look_result;
    uint16_t           look_mm;
    double             min_clear_mm; /* body to box, whole run */
    uint8_t            modes_seen;   /* bit (1 << S2_MODE_*) */
} sim_box_t;

/* S4's detection gate (task_imu.c, about every 20 ms) on the published
   state: whether hump detection would be on in each mode. */
typedef struct
{
    uint8_t  settle;
    uint8_t  on_modes;          /* bit (1 << S2_MODE_*) while enabled */
    uint32_t follow_samples;    /* samples in FOLLOW */
    uint32_t follow_on;         /* of those, with detection on */
} sim_gate_t;

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
    bool          b_controller_blocked; /* controller task stalled */
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
    sim_print_t   print;            /* runs 0: nothing printed */
    s3_barcode_t  barcode;          /* the real S3 barcode reader */
    uint16_t      bar_decoded;      /* as s3_follow.c's decoded_count */
    uint16_t      bar_rejected;
    uint8_t       last_flags;       /* S2_LINE_FLAG_* last reported */
    bool          b_flag_seen;      /* barcode-active flag reported */
    sim_box_t     box;
    sim_gate_t    gate;
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
static void sim_bar_period (void);
static void sim_box_period (void);
static double sim_ray_mm (int32_t angle_deg);
static double sim_box_clearance (void);
static uint16_t sim_side_adc (void);
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

void
app_stack_report (void)
{
    /* The pooled stacks exist only on the target (app_main.c). */
}

ER
scan_task_get (scan_task_status_t * p_status)
{
    scan_task_status_t const zero = {0};

    *p_status = zero;
    p_status->captured_ms = g_sim.now_ms;
    p_status->request_id = g_sim.scan_answer_id;
    p_status->result = g_sim.scan_answer;
    if (g_sim.box.b_on)
    {
        p_status->forward_mm = g_sim.box.forward_mm;
        p_status->flags = g_sim.box.b_hazard ? SCAN_TASK_FLAG_HAZARD : 0u;
        p_status->plan = g_sim.box.plan;
        p_status->look_id = g_sim.box.look_id;
        p_status->look_result = g_sim.box.look_result;
        p_status->look_mm = g_sim.box.look_mm;
    }
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

    /* The clock only moves forward between cases, as it would across
       boots of a car that kept time. */
    uint32_t const start_ms = g_sim.now_ms + 1000u;

    model_reset();
    g_sim = zero;
    g_sim.track = *p_track;
    g_sim.last_position = UINT8_MAX;
    g_sim.max_speed_after_stop = 0;
    (void)s3_barcode_init(&g_sim.barcode);
    g_sim.gate.settle = S4_SETTLE_SAMPLES;
    g_sim.now_ms = start_ms;
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
        /* A boot clears the liveness records (task_health.c). */
        for (uint32_t task = 0u; task < (uint32_t)S2_HEALTH_TASKS; task++)
        {
            g_health_seen[task] = false;
            g_health_last_ms[task] = 0u;
        }
        g_barcode = barcode_zero;
        g_button = button_zero;
        g_local_cause = S2_STOP_NONE;
        g_junction_count = 0u;
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
    sim_box_period();
    if (0u == ((g_sim.now_ms / SIM_PERIOD_MS) % 2u))
    {
        s2_robot_state_t gate_state;
        bool b_following;

        sim_state(&gate_state);
        b_following = (S2_SYS_RUNNING == gate_state.state)
                      && (S2_MODE_FOLLOW == gate_state.mode);
        if (s4_terrain_gate_step(&g_sim.gate.settle, b_following,
                                 S4_SETTLE_SAMPLES))
        {
            g_sim.gate.on_modes = (uint8_t)(g_sim.gate.on_modes
                                            | (1u << gate_state.mode));
            g_sim.gate.follow_on++;
        }
        g_sim.gate.follow_samples += b_following ? 1u : 0u;
    }

    if (!g_sim.b_controller_blocked)
    {
        if (tk_wai_flg(controller_flags, CTRL_FLG_ALL, TWF_ORW, &ptn,
                       CTRL_WATCHDOG_MS) < E_OK)
        {
            ptn = 0u;
        }
        controller_cycle(ptn, &inputs);
    }

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
    /* As task_motion.c in the mission image (S2 decision 37). */
    (void)s2_health_guard(&g_sim.motion, &out, g_sim.now_ms);
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
    s2_health_check_in(S2_HEALTH_LINE);     /* as s3_follow.c, each pass */
    sim_bar_period();       /* every period, before the steering filter */
    if (s3_ir_filter(&g_sim.filter, mask, (uint8_t)SIM_FILTER_SAMPLES))
    {
        uint8_t position = S2_LINE_CENTER;
        /* S2 decision 25: the controller is told while a code is read. */
        uint8_t const flags = s3_barcode_active(&g_sim.barcode)
                              ? S2_LINE_FLAG_BARCODE : 0u;

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

        if ((position != g_sim.last_position) || (flags != g_sim.last_flags)
            || ((g_sim.now_ms - g_sim.last_report_ms) >= SIM_REPORT_MS))
        {
            if (s2_line_submit_scaled(position,
                                      ((S2_LINE_LEFT == position)
                                       || (S2_LINE_RIGHT == position))
                                      ? 50u : 0u, flags, g_sim.now_ms))
            {
                g_sim.last_position = position;
                g_sim.last_flags = flags;
                g_sim.last_report_ms = g_sim.now_ms;
                g_sim.b_flag_seen = g_sim.b_flag_seen || (0u != flags);
            }
        }
    }
}

/* s3_follow_barcode_sample(): one side-sensor sample through the real
   reader; a decoded letter goes to the controller with its command. */
static void
sim_bar_period (void)
{
    char symbol = '\0';
    /* The 1 MHz timer from the ms clock; it wraps as the real one does. */
    uint32_t const now_us = g_sim.now_ms * 1000u;
    s3_scan_result_t const result = s3_barcode_update(
        &g_sim.barcode, sim_side_adc(), now_us, &symbol);

    if (S3_SCAN_DECODED == result)
    {
        g_sim.bar_decoded++;
        (void)s2_nav_submit((uint8_t)s3_nav_command(symbol), symbol,
                            g_sim.bar_decoded, g_sim.now_ms);
    }
    else if (S3_SCAN_ERROR == result)
    {
        g_sim.bar_rejected++;
    }
    else
    {
        /* Nothing finished this period. */
    }
}

/* Side sensor ADC: black over a black run of the printed code. */
static uint16_t
sim_side_adc (void)
{
    motion_status_t status;
    bool b_black = false;

    (void)motion_get_status(&status);
    if (0u != g_sim.print.runs)
    {
        double const h = ((double)status.heading_mdeg / 1000.0)
                         * 3.14159265358979 / 180.0;
        double const sx_um = ((double)status.x_mm
                              + (SIM_SIDE_AHEAD_MM * cos(h))) * 1000.0;
        double edge_um = (double)g_sim.print.start_mm * 1000.0;

        for (uint8_t run = 0u; run < g_sim.print.runs; run++)
        {
            double const end_um = edge_um + (double)g_sim.print.run_um[run];

            if ((sx_um >= edge_um) && (sx_um < end_um))
            {
                b_black = (0u == (run % 2u));
            }
            edge_um = end_um;
        }
    }

    return (b_black ? SIM_BAR_BLACK_ADC : SIM_BAR_WHITE_ADC);
}

/* Print letter (A..Z) as a framed Code 39 code from start_mm. */
static void
sim_print_code (char letter, int32_t start_mm)
{
    uint16_t pattern = 0u;
    uint16_t order[3];
    uint8_t count = 0u;

    for (uint8_t idx = 0u; idx < S3_CODE39_SYMBOL_COUNT; idx++)
    {
        if (letter == g_s3_code39_alphabet[idx].symbol)
        {
            pattern = g_s3_code39_alphabet[idx].pattern;
        }
    }
    order[0] = S3_CODE39_SENTINEL;
    order[1] = pattern;
    order[2] = S3_CODE39_SENTINEL;
    for (uint8_t ch = 0u; ch < 3u; ch++)
    {
        for (uint8_t el = 0u; el < S3_BAR_ELEMENTS; el++)
        {
            g_sim.print.run_um[count] =
                (0u != (order[ch] & (1u << (8u - el))))
                ? SIM_WIDE_UM : SIM_NARROW_UM;
            count++;
        }
        if (ch < 2u)
        {
            g_sim.print.run_um[count] = SIM_NARROW_UM;
            count++;
        }
    }
    g_sim.print.runs = count;
    g_sim.print.start_mm = start_mm;
}

/* Split run of the print with a speck of the opposite level, speck_um
   long, in its middle (a noise blip as seen by the sensor). */
static void
sim_print_speck (uint8_t run, uint32_t speck_um)
{
    sim_print_t * const p_print = &g_sim.print;
    uint32_t const head = (p_print->run_um[run] - speck_um) / 2u;
    uint32_t const tail = p_print->run_um[run] - speck_um - head;

    for (uint8_t idx = (uint8_t)(p_print->runs - 1u); idx > run; idx--)
    {
        p_print->run_um[idx + 2u] = p_print->run_um[idx];
    }
    p_print->run_um[run] = head;
    p_print->run_um[run + 1u] = speck_um;
    p_print->run_um[run + 2u] = tail;
    p_print->runs = (uint8_t)(p_print->runs + 2u);
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
        else if ((S2_SCAN_LOOK == cmd.kind) && g_sim.box.b_on)
        {
            g_sim.box.b_look_pending = true;
            g_sim.box.look_id = cmd.request_id;
            g_sim.box.look_angle = cmd.angle_deg;
            g_sim.box.look_due_ms = g_sim.now_ms + SIM_LOOK_MS;
            g_sim.box.look_result = SCAN_TASK_LOOK_NONE;
        }
        else if ((S2_SCAN_CANCEL == cmd.kind) || (S2_SCAN_STOP == cmd.kind)
                 || (S2_SCAN_RESET == cmd.kind))
        {
            /* The scan task abandons a scan or look in progress. */
            g_sim.b_scan_authorised = false;
            g_sim.box.b_look_pending = false;
        }
        else
        {
            /* STOP_CONFIRM, MOVEMENT and the rest need no answer. */
        }
    }

    if (g_sim.b_scan_authorised
        && ((g_sim.now_ms - g_sim.scan_since_ms) >= SIM_SCAN_MS))
    {
        g_sim.scan_answer_id = g_sim.scan_request;
        g_sim.scan_answer = g_sim.track.scan_result;
        if (g_sim.box.b_on)
        {
            /* scan_task.c, SCAN_ENGINE_PLAN: plan from the 5-degree
               profile; clear, plan or no route. */
            uint16_t profile[BYPASS_PLAN_FINE_ANGLES];

            for (uint32_t idx = 0u; idx < BYPASS_PLAN_FINE_ANGLES; idx++)
            {
                profile[idx] = (uint16_t)lround(sim_ray_mm(
                    ((int32_t)idx * SCAN_PROFILE_STEP_DEG)
                    - SCAN_PROFILE_HALF_DEG));
            }
            (void)bypass_plan_make_fine(profile, &g_sim_plan_cfg,
                                        &g_sim.box.plan);
            if (scan_profile_clear(profile, SIM_CLEAR_MM))
            {
                g_sim.scan_answer = SCAN_TASK_RESULT_CLEAR;
            }
            else if (BYPASS_PLAN_OK == g_sim.box.plan.result)
            {
                g_sim.scan_answer = SCAN_TASK_RESULT_PLAN;
            }
            else
            {
                g_sim.scan_answer = SCAN_TASK_RESULT_NO_ROUTE;
            }
        }
        g_sim.scan_answer_ms = g_sim.now_ms;
        g_sim.b_scan_authorised = false;
    }
}

/* With an obstacle box: forward readings through the real forward
   monitor and the hazard edge as scan_task.c raises it; side looks;
   the closest approach of the body; the modes seen. */
static void
sim_box_period (void)
{
    sim_box_t * const p_box = &g_sim.box;
    s2_robot_state_t state;
    forward_monitor_config_t const cfg = {SIM_HAZARD_MM, SIM_FRESH_US};
    uint32_t const now_us = g_sim.now_ms * 1000u;
    double const clear_mm = sim_box_clearance();

    if (!p_box->b_on)
    {
        return;
    }
    sim_state(&state);
    p_box->modes_seen = (uint8_t)(p_box->modes_seen | (1u << state.mode));
    if (clear_mm < p_box->min_clear_mm)
    {
        p_box->min_clear_mm = clear_mm;
    }

    if (p_box->b_look_pending && (g_sim.now_ms >= p_box->look_due_ms))
    {
        p_box->look_mm = (uint16_t)lround(sim_ray_mm(p_box->look_angle));
        p_box->look_result = SCAN_TASK_LOOK_DONE;
        p_box->b_look_pending = false;
    }

    if ((!g_sim.b_scan_authorised) && (!p_box->b_look_pending)
        && (g_sim.now_ms >= p_box->next_fwd_ms))
    {
        bool const b_moving = (S2_SYS_RUNNING == state.state)
                              && ((S2_MODE_FOLLOW == state.mode)
                                  || (S2_MODE_BYPASS == state.mode)
                                  || (S2_MODE_REACQUIRE == state.mode));
        bool b_hazard;

        p_box->next_fwd_ms = g_sim.now_ms + SIM_FWD_PERIOD_MS;
        p_box->forward_mm = (uint16_t)lround(sim_ray_mm(0));
        forward_monitor_sample(&p_box->monitor, true, p_box->forward_mm,
                               now_us);
        p_box->b_hazard = (FORWARD_MONITOR_SAFE
                           != forward_monitor_check(&p_box->monitor, &cfg,
                                                    now_us));
        b_hazard = p_box->b_hazard && b_moving;
        if (b_hazard && (!p_box->b_was_hazard))
        {
            (void)tk_set_flg(controller_flags, CTRL_FLG_HAZARD);
        }
        p_box->b_was_hazard = b_hazard;
    }
}

/* Ideal ray from the ultrasonic sensor face (sensor_mm ahead of the axle)
   at angle_deg from the heading (left positive) to the box; SIM_RANGE_MM
   if it misses. Slab intersection with the axis-aligned box. */
static double
sim_ray_mm (int32_t angle_deg)
{
    motion_status_t status;
    double near_t = 0.0;
    double far_t = SIM_RANGE_MM;

    (void)motion_get_status(&status);
    {
        double const h = ((double)status.heading_mdeg / 1000.0)
                         * 3.14159265358979 / 180.0;
        double const a = h + (((double)angle_deg * 3.14159265358979) / 180.0);
        double const ox = (double)status.x_mm
                          + ((double)g_sim_plan_cfg.sensor_mm * cos(h));
        double const oy = (double)status.y_mm
                          + ((double)g_sim_plan_cfg.sensor_mm * sin(h));
        double const d[2] = {cos(a), sin(a)};
        double const o[2] = {ox, oy};
        double const lo[2] = {g_sim.box.x0, g_sim.box.y0};
        double const hi[2] = {g_sim.box.x1, g_sim.box.y1};

        for (uint32_t axis = 0u; axis < 2u; axis++)
        {
            if (fabs(d[axis]) < 1e-9)
            {
                if ((o[axis] < lo[axis]) || (o[axis] > hi[axis]))
                {
                    return (SIM_RANGE_MM);
                }
            }
            else
            {
                double t1 = (lo[axis] - o[axis]) / d[axis];
                double t2 = (hi[axis] - o[axis]) / d[axis];

                if (t1 > t2)
                {
                    double const swap = t1;

                    t1 = t2;
                    t2 = swap;
                }
                near_t = (t1 > near_t) ? t1 : near_t;
                far_t = (t2 < far_t) ? t2 : far_t;
            }
        }
    }

    return ((near_t <= far_t) ? near_t : SIM_RANGE_MM);
}

/* Distance from the car body (bypass_plan geometry: sides 82 and 63 mm,
   front 132 and rear 56 mm from the axle) to the box; negative inside.
   The outline is sampled every 2 mm and the box corners tested. */
static double
sim_box_clearance (void)
{
    motion_status_t status;
    double best = 1.0e9;

    if (!g_sim.box.b_on)
    {
        return (best);
    }
    (void)motion_get_status(&status);
    {
        double const h = ((double)status.heading_mdeg / 1000.0)
                         * 3.14159265358979 / 180.0;
        double const c = cos(h);
        double const s = sin(h);
        double const f = (double)g_sim_plan_cfg.front_mm;
        double const r = -(double)g_sim_plan_cfg.rear_mm;
        double const l = (double)g_sim_plan_cfg.left_side_mm;
        double const w = -(double)g_sim_plan_cfg.right_side_mm;
        double const corner[4][2] = {{f, l}, {f, w}, {r, w}, {r, l}};

        for (uint32_t side = 0u; side < 4u; side++)
        {
            double const * const p_a = corner[side];
            double const * const p_b = corner[(side + 1u) % 4u];
            double const len = hypot(p_b[0] - p_a[0], p_b[1] - p_a[1]);

            for (double u = 0.0; u <= len; u += 2.0)
            {
                double const bx = p_a[0] + (((p_b[0] - p_a[0]) * u) / len);
                double const by = p_a[1] + (((p_b[1] - p_a[1]) * u) / len);
                double const px = (double)status.x_mm + (bx * c) - (by * s);
                double const py = (double)status.y_mm + (bx * s) + (by * c);
                double const dx = fmax(fmax(g_sim.box.x0 - px, 0.0),
                                       px - g_sim.box.x1);
                double const dy = fmax(fmax(g_sim.box.y0 - py, 0.0),
                                       py - g_sim.box.y1);
                double const dist = hypot(dx, dy);

                best = (dist < best) ? dist : best;
                if ((px >= g_sim.box.x0) && (px <= g_sim.box.x1)
                    && (py >= g_sim.box.y0) && (py <= g_sim.box.y1))
                {
                    best = -1.0;
                }
            }
        }

        /* A box corner inside the body (the body is convex). */
        for (uint32_t k = 0u; k < 4u; k++)
        {
            double const qx = ((k & 1u) != 0u) ? g_sim.box.x1 : g_sim.box.x0;
            double const qy = ((k & 2u) != 0u) ? g_sim.box.y1 : g_sim.box.y0;
            double const ex = ((qx - (double)status.x_mm) * c)
                              + ((qy - (double)status.y_mm) * s);
            double const ey = (-(qx - (double)status.x_mm) * s)
                              + ((qy - (double)status.y_mm) * c);

            if ((ex <= f) && (ex >= r) && (ey <= l) && (ey >= w))
            {
                best = -1.0;
            }
        }
    }

    return (best);
}

/* Place a box width_mm across and depth_mm along the line, its near face
   at near_mm, centred on the line. */
static void
sim_place_box (int32_t near_mm, int32_t width_mm, int32_t depth_mm)
{
    sim_box_t const zero = {0};

    g_sim.box = zero;
    g_sim.box.b_on = true;
    g_sim.box.x0 = (double)near_mm;
    g_sim.box.x1 = (double)(near_mm + depth_mm);
    g_sim.box.y0 = -(double)width_mm / 2.0;
    g_sim.box.y1 = (double)width_mm / 2.0;
    g_sim.box.min_clear_mm = 1.0e9;
    forward_monitor_reset(&g_sim.box.monitor);
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
