/*
 *       subsystem2_test.c
 *       On-target tests for subsystem 2.  Built only with: make TEST=1
 *       Record results in testing/subsystem2/TEST_LOG.md.
 *
 *       S2-U03..S2-U14 test the pure PID and odometry logic with synthetic
 *       gains and geometry; they need no motors or encoders. S2-U20..S2-U35
 *       (test_motion.c) drive the Motion Control logic with a wheel model.
 *
 *       Bench mode, off by default: make TEST=1 S2_HW_TEST=n (1..4).
 *       The motion and controller tasks are then not started:
 *         1  encoder counts every 100 ms for 60 s; motors stay OFF (S2-H03)
 *         2  also SPINS THE MOTORS (S2-H01, S2-H02, S2-H04): wheels off the
 *            ground first
 *         3  only the S2-H04 full-speed diagnostic (SPINS THE MOTORS)
 *         4  only the S2-H05 watchdog check (SPINS MOTOR 1, then resets)
 */

#include "subsystem2.h"
#include "pid.h"
#include "odometry.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_gpio_irq.h"
#include "drv_watchdog.h"
#include "motion_config.h"
#include "test_motion.h"
#include "test_controller.h"
#include "test_line_control.h"
#include "test_scan_control.h"
#include "test_bypass_control.h"
#include "test_nav_control.h"
#include "s2_bridge.h"
#include "../test_report.h"

#include "task_motion.h"                /* S2_HW_TEST */

#if APP_TEST

#define SENTINEL        12345   /* output value a rejected call must keep */

/* Synthetic geometry: 20 counts per 200 mm revolution, 150 mm track */
static const odom_cfg_t g_odom_cfg = { 20u, 200000u, 150u };

/* Configure a controller with gains given as PID_GAIN_SCALE multiples */
static bool pid_setup(pid_ctrl_t *p_pid, int32_t kp, int32_t ki, int32_t kd,
                      int32_t out_min, int32_t out_max)
{
    pid_cfg_t cfg = { kp, ki, kd, out_min, out_max };
    return pid_init(p_pid, &cfg);
}

/* S2-U03: proportional term alone gives kp * error */
static bool test_pid_proportional (void)
{
    pid_ctrl_t pid;
    int32_t    out = 0;

    return pid_setup(&pid, 2 * PID_GAIN_SCALE, 0, 0, -1000, 1000) &&
           pid_update(&pid, 100, 40, &out) && (120 == out) &&
           pid_update(&pid, 40, 100, &out) && (-120 == out);
}

/* S2-U04: output never leaves out_min..out_max */
static bool test_pid_clamp (void)
{
    pid_ctrl_t pid;
    int32_t    hi = 0, lo = 0;

    return pid_setup(&pid, 100 * PID_GAIN_SCALE, 0, 0, -255, 255) &&
           pid_update(&pid, 1000, 0, &hi) && (255 == hi) &&
           pid_update(&pid, -1000, 0, &lo) && (-255 == lo);
}

/* S2-U05: integral term adds ki * error every sample */
static bool test_pid_integral (void)
{
    pid_ctrl_t pid;
    int32_t    a = 0, b = 0, c = 0;

    return pid_setup(&pid, 0, PID_GAIN_SCALE / 2, 0, -1000, 1000) &&
           pid_update(&pid, 10, 0, &a) &&
           pid_update(&pid, 10, 0, &b) &&
           pid_update(&pid, 10, 0, &c) &&
           (5 == a) && (10 == b) && (15 == c);
}

/* S2-U06: a long saturation (stalled wheel) does not wind up the integral,
   so the output reverses on the first sample after the error reverses */
static bool test_pid_anti_windup (void)
{
    pid_ctrl_t pid;
    int32_t    out = 0;
    bool       ok;
    int        i;

    ok = pid_setup(&pid, 0, PID_GAIN_SCALE, 0, -100, 100);
    for (i = 0; i < 50; i++)
    {
        ok = pid_update(&pid, 1000, 0, &out) && (100 == out) && ok;
    }
    return ok && pid_update(&pid, 0, 10, &out) && (-10 == out);
}

/* S2-U07: derivative acts on measurement change, not setpoint change */
static bool test_pid_derivative (void)
{
    pid_ctrl_t pid;
    int32_t    first = SENTINEL, step = SENTINEL, rise = SENTINEL;

    return pid_setup(&pid, 0, 0, PID_GAIN_SCALE, -1000, 1000) &&
           pid_update(&pid, 0, 0, &first) &&       /* no history yet */
           pid_update(&pid, 500, 0, &step) &&      /* setpoint step */
           pid_update(&pid, 500, 5, &rise) &&      /* measurement +5 */
           (0 == first) && (0 == step) && (-5 == rise);
}

/* S2-U08: invalid tuning and inputs are rejected without side effects */
static bool test_pid_invalid (void)
{
    pid_ctrl_t pid;
    int32_t    out = SENTINEL;

    return !pid_setup(&pid, 0, 0, 0, 10, 10) &&     /* empty range */
           !pid_setup(&pid, -1, 0, 0, -10, 10) &&  /* negative gain */
           !pid_setup(&pid, PID_GAIN_MAX + 1, 0, 0, -10, 10) &&
           !pid_init(NULL, NULL) &&
           pid_setup(&pid, PID_GAIN_SCALE, 0, 0, -10, 10) &&
           !pid_update(&pid, PID_INPUT_MAX + 1, 0, &out) &&
           !pid_update(&pid, 0, -PID_INPUT_MAX - 1, &out) &&
           !pid_update(&pid, 0, 0, NULL) &&
           (SENTINEL == out);
}

/* S2-U09: reset clears the integral and derivative history */
static bool test_pid_reset (void)
{
    pid_ctrl_t pid;
    int32_t    out = SENTINEL;

    return pid_setup(&pid, 0, PID_GAIN_SCALE, PID_GAIN_SCALE, -1000, 1000) &&
           pid_update(&pid, 50, 0, &out) &&
           pid_update(&pid, 50, 0, &out) && (100 == out) &&
           (pid_reset(&pid), pid_update(&pid, 7, 7, &out)) && (0 == out);
}

/* S2-U10: counts convert to millimetres, truncating toward zero */
static bool test_odom_distance (void)
{
    static const odom_cfg_t thirds = { 3u, 100000u, 150u };
    int32_t a = 0, b = 0, c = 0, d = 0;

    return odom_counts_to_mm(&g_odom_cfg, 20, &a) && (200 == a) &&
           odom_counts_to_mm(&g_odom_cfg, -10, &b) && (-100 == b) &&
           odom_counts_to_mm(&thirds, 1, &c) && (33 == c) &&
           odom_counts_to_mm(&thirds, -1, &d) && (-33 == d);
}

/* S2-U11: counter deltas are correct across uint32_t wraparound */
static bool test_odom_wrap (void)
{
    return (odom_count_delta(5u, 0xFFFFFFFBu) == 10) &&
           (odom_count_delta(0xFFFFFFFBu, 5u) == -10) &&
           (odom_count_delta(0x7FFFFFFFu, 0u) == INT32_MAX) &&
           (odom_count_delta(0x80000000u, 0u) == INT32_MIN) &&
           (odom_count_delta(42u, 42u) == 0);
}

/* S2-U12: speed from counts over an interval; zero interval rejected */
static bool test_odom_speed (void)
{
    int32_t fwd = 0, rev = 0, slow = 0, out = SENTINEL;

    return odom_speed_mm_s(&g_odom_cfg, 10, 100000u, &fwd) && (1000 == fwd) &&
           odom_speed_mm_s(&g_odom_cfg, -10, 100000u, &rev) && (-1000 == rev) &&
           odom_speed_mm_s(&g_odom_cfg, 1, 20000u, &slow) && (500 == slow) &&
           !odom_speed_mm_s(&g_odom_cfg, 10, 0u, &out) && (SENTINEL == out);
}

/* S2-U13: turn angle sign (left positive) and magnitude (1 rad) */
static bool test_odom_turn (void)
{
    int32_t left = 0, right = 0, straight = SENTINEL;

    return odom_turn_mdeg(&g_odom_cfg, 0, 150, &left) && (57295 == left) &&
           odom_turn_mdeg(&g_odom_cfg, 150, 0, &right) && (-57295 == right) &&
           odom_turn_mdeg(&g_odom_cfg, 80, 80, &straight) && (0 == straight);
}

/* S2-U14: invalid geometry and overflowing results are rejected */
static bool test_odom_invalid (void)
{
    static const odom_cfg_t no_cpr = { 0u, 200000u, 150u };
    static const odom_cfg_t no_track = { 20u, 200000u, 0u };
    static const odom_cfg_t coarse = { 1u, ODOM_CIRC_UM_MAX, 150u };
    int32_t out = SENTINEL;

    return !odom_cfg_valid(&no_cpr) && !odom_cfg_valid(&no_track) &&
           !odom_cfg_valid(NULL) && odom_cfg_valid(&coarse) &&
           !odom_counts_to_mm(&no_cpr, 1, &out) &&
           !odom_counts_to_mm(&coarse, INT32_MAX, &out) &&
           !odom_speed_mm_s(&coarse, INT32_MAX, 1u, &out) &&
           !odom_turn_mdeg(&no_track, 0, 1, &out) &&
           !odom_counts_to_mm(&g_odom_cfg, 1, NULL) &&
           (SENTINEL == out);
}

/* S2-U15: duty sign selects the A or B output; zero brakes */
static bool test_motor_map (void)
{
    drv_motor_cc_t fwd, rev, stop, full;

    return drv_motor_map(1000, &fwd) &&
           (1000u == fwd.cc_a) && (0u == fwd.cc_b) &&
           drv_motor_map(-1000, &rev) &&
           (0u == rev.cc_a) && (1000u == rev.cc_b) &&
           drv_motor_map(0, &stop) &&
           (0u == stop.cc_a) && (0u == stop.cc_b) &&
           drv_motor_map(-DRV_MOTOR_DUTY_MAX, &full) &&
           (full.cc_b == (uint16_t)DRV_MOTOR_DUTY_MAX);
}

/* S2-U16: out-of-range duty and NULL output are rejected, output kept */
static bool test_motor_reject (void)
{
    drv_motor_cc_t cc = { 7u, 9u };

    return !drv_motor_map(DRV_MOTOR_DUTY_MAX + 1, &cc) &&
           !drv_motor_map(-DRV_MOTOR_DUTY_MAX - 1, &cc) &&
           !drv_motor_map(INT32_MIN, &cc) &&
           !drv_motor_map(0, NULL) &&
           (7u == cc.cc_a) && (9u == cc.cc_b);
}

static void test_gpio_cb (uint32_t gpio, uint32_t events, uint32_t now_us)
{
    (void)gpio; (void)events; (void)now_us;
}

/* S2-U17: invalid registrations are refused before touching hardware */
static bool test_gpio_irq_reject (void)
{
    uint32_t const both = DRV_GPIO_IRQ_EDGE_RISE | DRV_GPIO_IRQ_EDGE_FALL;

    return (drv_gpio_irq_register(30u, both, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(23u, both, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(24u, both, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(25u, both, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(29u, both, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(5u, 0u, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(5u, 0x1u, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(5u, 0x2u, test_gpio_cb) == E_PAR) &&
           (drv_gpio_irq_register(5u, both, NULL) == E_PAR) &&
           (drv_gpio_irq_unregister(30u) == E_PAR);
}

/* S2-U18: x2 quadrature decode */
static bool test_encoder_step (void)
{
    uint32_t const rise = DRV_GPIO_IRQ_EDGE_RISE;
    uint32_t const fall = DRV_GPIO_IRQ_EDGE_FALL;

    return (drv_encoder_step(rise, true, false) == 1) &&
           (drv_encoder_step(rise, true, true) == -1) &&
           (drv_encoder_step(fall, false, true) == 1) &&
           (drv_encoder_step(fall, false, false) == -1) &&
           (drv_encoder_step(rise | fall, false, true) == 2) &&
           (drv_encoder_step(rise | fall, true, false) == 2) &&
           (drv_encoder_step(rise | fall, false, false) == -2) &&
           (drv_encoder_step(rise | fall, true, true) == -2) &&
           (drv_encoder_step(0u, false, true) == 0);
}

/* S2-U19: snapshot rejects NULL; idle counts do not change */
static bool test_encoder_snapshot (void)
{
    drv_encoder_snapshot_t a, b;
    UINT e;
    bool same = true;

    if ((drv_encoder_snapshot(NULL) != E_PAR) ||
        (drv_encoder_snapshot(&a) != E_OK) ||
        (drv_encoder_snapshot(&b) != E_OK))
        {
        return false;
    }
    for (e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++)
    {
        same = same && (a.count[e] == b.count[e]) &&
               (a.errors[e] == b.errors[e]);
    }
    return same;
}

/* S2-U36: remote_q record for START/RESET only, request ID nonzero */
static bool test_remote_make (void)
{
    s2_remote_cmd_t m, keep = { 99u, 98u, 97u, { 1u, 2u, 3u } };

    return s2_remote_make(S2_REMOTE_START, 5u, 1234u, &m) &&
           (5u == m.request_id) && (1234u == m.received_ms) &&
           (S2_REMOTE_START == m.command) && (0u == m.reserved[0]) &&
           (0u == m.reserved[1]) && (0u == m.reserved[2]) &&
           s2_remote_make(S2_REMOTE_RESET, 6u, 0u, &m) &&
           (S2_REMOTE_RESET == m.command) &&
           !s2_remote_make(S2_REMOTE_STOP, 7u, 0u, &keep) &&
           !s2_remote_make(0u, 7u, 0u, &keep) &&
           !s2_remote_make(4u, 7u, 0u, &keep) &&
           !s2_remote_make(S2_REMOTE_START, 0u, 0u, &keep) &&
           !s2_remote_make(S2_REMOTE_START, 7u, 0u, NULL) &&
           (99u == keep.request_id) && (3u == keep.reserved[2]);
}

/* S2-U57: line report accepts only the agreed positions. */
static bool test_line_make (void)
{
    s2_line_report_t report;
    s2_line_report_t keep = { 99u, 98u, 97u, 1u, 2u };
    UINT position;
    bool ok = true;

    for (position = S2_LINE_CENTER; position <= S2_LINE_STOP; position++)
    {
        ok = s2_line_make((uint8_t)position, 1234u, &report) && ok;
        ok = (1234u == report.captured_ms) &&
             (report.position == (uint8_t)position) &&
             (report.strength == (((S2_LINE_LEFT == position) ||
                                   (S2_LINE_RIGHT == position))
                                  ? S2_LINE_STRENGTH_MAX : 0u)) &&
             (0u == report.flags) &&
             (0u == report.reserved) && ok;
    }

    ok = s2_line_make_scaled(S2_LINE_LEFT, 35u, 0u, 4321u, &report) &&
         (4321u == report.captured_ms) &&
         (S2_LINE_LEFT == report.position) &&
         (35u == report.strength) && (0u == report.flags) && ok;

    /* S2 decision 25: the barcode flag is carried; others are refused. */
    ok = s2_line_make_scaled(S2_LINE_CENTER, 0u, S2_LINE_FLAG_BARCODE,
                             5u, &report) &&
         (S2_LINE_FLAG_BARCODE == report.flags) &&
         (0u == report.reserved) && ok;

    return ok && !s2_line_make_scaled(S2_LINE_CENTER, 1u, 0u, 0u, &keep) &&
           !s2_line_make_scaled(S2_LINE_LEFT, 101u, 0u, 0u, &keep) &&
           !s2_line_make_scaled(S2_LINE_CENTER, 0u, 0x02u, 0u, &keep) &&
           !s2_line_make(5u, 0u, &keep) &&
           !s2_line_make(S2_LINE_CENTER, 0u, NULL) &&
           (99u == keep.captured_ms) && (98u == keep.position) &&
           (97u == keep.strength) && (1u == keep.flags) &&
           (2u == keep.reserved);
}

/* S2-U37: telemetry fields from robot_state */
static bool test_telemetry_fill (void)
{
    s2_robot_state_t st = { 0u };
    telemetry_encoder_snapshot_t a = { 0u }, b = { 0u }, c = { 0u };
    telemetry_encoder_snapshot_t d = { 0u }, e = { 0u };

    st.captured_ms = 4321u;
    st.flags = S2_RS_DISTANCE_VALID | S2_RS_STOPPED | S2_RS_STOP_LATCHED;
    st.run_distance_mm = 500;
    st.state = S2_SYS_RUNNING;
    st.mode = S2_MODE_BYPASS;
    st.fault = S2_FAULT_SCAN;
    st.line = S2_LINE_LEFT;
    st.left_mm_s = -40;
    st.right_mm_s = 45;
    st.heading_cdeg = -9000;
    st.barcode = 'F';
    st.barcode_command = S2_NAV_LEFT;
    st.barcode_count = 3u;
    st.stop_cause = S2_STOP_PICKUP;
    s2_telemetry_from_state(&st, &a);
    st.run_distance_mm = 70000;
    st.flags = S2_RS_DISTANCE_VALID | S2_RS_MOTION_VALID
               | S2_RS_LINE_FRESH | S2_RS_BARCODE_VALID;
    s2_telemetry_from_state(&st, &b);
    st.run_distance_mm = -5;
    s2_telemetry_from_state(&st, &c);
    st.run_distance_mm = 300;
    st.flags = 0u;
    s2_telemetry_from_state(&st, &d);
    e.link_up = true;
    e.hump_valid = true;
    s2_telemetry_from_state(&st, &e);
    s2_telemetry_from_state(NULL, &e);

    /* a: no motion or line flag, so those fields are cleared */
    return (4321u == a.captured_ms) && (500u == a.distance_mm) &&
           a.distance_valid && a.stopped && a.stop_latched &&
           (S2_SYS_RUNNING == a.state) && (S2_MODE_BYPASS == a.mode) &&
           (S2_FAULT_SCAN == a.fault) &&
           !a.motion_valid && (0 == a.left_mm_s) &&
           (0 == a.heading_cdeg) && !a.line_fresh && (0u == a.line) &&
           /* a: STOP latched gives the cause; no barcode flag */
           (S2_STOP_PICKUP == a.stop_cause) && !a.barcode_valid &&
           (a.barcode == '\0') && (0u == a.barcode_count) &&
           /* b: barcode flag set, STOP clear so no cause */
           b.barcode_valid && (b.barcode == 'F') &&
           (S2_NAV_LEFT == b.barcode_command) &&
           (3u == b.barcode_count) && (S2_STOP_NONE == b.stop_cause) &&
           (65535u == b.distance_mm) && !b.distance_valid &&
           !b.stopped && !b.stop_latched &&
           b.motion_valid && (-40 == b.left_mm_s) &&
           (45 == b.right_mm_s) && (-9000 == b.heading_cdeg) &&
           b.line_fresh && (S2_LINE_LEFT == b.line) &&
           (0u == c.distance_mm) && !c.distance_valid &&
           (0u == d.distance_mm) && !d.distance_valid &&
           /* S1's link state and the S4 fields are left alone */
           e.link_up && e.hump_valid;
}

/* S2-U71: hump fields from subsystem 4's run maximum, last hump and
   count (IF-18) */
static bool test_telemetry_hump (void)
{
    telemetry_encoder_snapshot_t a = { 0u }, b = { 0u }, c = { 0u };

    s2_telemetry_add_hump(&a, 10u, true, 8u, true, 2u);
    s2_telemetry_add_hump(&b, 300u, true, 256u, true, 255u);
    c.hump_max_mm = 7u;
    c.hump_valid = true;
    c.hump_last_mm = 7u;
    c.hump_last_valid = true;
    s2_telemetry_add_hump(&c, 12u, false, 12u, false, 0u);
    s2_telemetry_add_hump(NULL, 10u, true, 10u, true, 1u);

    return (10u == a.hump_max_mm) && a.hump_valid &&
           (8u == a.hump_last_mm) && a.hump_last_valid &&
           (2u == a.hump_count) &&
           (255u == b.hump_max_mm) && b.hump_valid &&
           (255u == b.hump_last_mm) && b.hump_last_valid &&
           (255u == b.hump_count) &&
           (0u == c.hump_max_mm) && !c.hump_valid &&
           (0u == c.hump_last_mm) && !c.hump_last_valid &&
           (0u == c.hump_count);
}

/* S2-U72: obstacle fields from subsystem 5's decoded status */
static bool test_telemetry_scan (void)
{
    scan_telemetry_t scan = { 0u };
    telemetry_encoder_snapshot_t a = { 0u }, b = { 0u }, c = { 0u };
    telemetry_encoder_snapshot_t r = { 0u };

    scan.forward_mm = 350u;
    scan.nearest_observed_mm = 180u;
    scan.observed_width_mm = 210u;
    scan.centre_offset_mm = -30;
    scan.scan_result = SCAN_TELEMETRY_PLAN;
    scan.bypass_side = SCAN_TELEMETRY_SIDE_LEFT;
    scan.flags = SCAN_TELEMETRY_FORWARD_VALID | SCAN_TELEMETRY_PROFILE_SEEN
                 | SCAN_TELEMETRY_SHAPE_ESTIMATE;
    s2_telemetry_add_scan(&a, &scan);

    /* Flags clear: values present in the status are not copied */
    scan.flags = 0u;
    s2_telemetry_add_scan(&b, &scan);

    /* Raw width only: copied with its flag, no centre (S1 decision 7) */
    scan.flags = SCAN_TELEMETRY_PROFILE_SEEN | SCAN_TELEMETRY_WIDTH_RAW;
    s2_telemetry_add_scan(&r, &scan);
    scan.flags = 0u;

    /* S5 busy: an earlier obstacle report is cleared, not repeated */
    c = r;
    c.forward_valid = true;
    c.shape_valid = true;
    s2_telemetry_add_scan(&c, NULL);
    s2_telemetry_add_scan(NULL, &scan);

    return r.width_raw && !r.shape_valid && (210u == r.width_mm) &&
           (0 == r.centre_offset_mm) && !a.width_raw && !c.width_raw &&
           a.forward_valid && (350u == a.forward_mm) &&
           a.profile_valid && (180u == a.nearest_mm) &&
           a.shape_valid && (210u == a.width_mm) &&
           (-30 == a.centre_offset_mm) &&
           (SCAN_TELEMETRY_PLAN == a.scan_result) &&
           (SCAN_TELEMETRY_SIDE_LEFT == a.bypass_side) &&
           !b.forward_valid && (0u == b.forward_mm) &&
           !b.profile_valid && (0u == b.nearest_mm) &&
           !b.shape_valid && (0u == b.width_mm) &&
           (0 == b.centre_offset_mm) &&
           (SCAN_TELEMETRY_PLAN == b.scan_result) &&
           !c.forward_valid && !c.profile_valid && !c.shape_valid &&
           (0u == c.forward_mm) &&
           (SCAN_TELEMETRY_NONE == c.scan_result) &&
           (SCAN_TELEMETRY_SIDE_NONE == c.bypass_side);
}

/* S2-U38: subsystem 4 distance delta */
static bool test_distance_delta (void)
{
    motion_status_t p = { 0u }, n = { 0u };
    uint16_t dm = 7u;
    bool ok = true;

    p.flags = MOTION_ST_VALID;
    n.flags = MOTION_ST_VALID;
    p.travel_mm = 1000;
    n.travel_mm = 1012;
    ok = s2_distance_delta(&p, &n, &dm) && (12u == dm) && ok;
    n.travel_mm = 1000;
    ok = s2_distance_delta(&p, &n, &dm) && (0u == dm) && ok;
    n.travel_mm = 997;
    ok = !s2_distance_delta(&p, &n, &dm) && (0u == dm) && ok;
    n.travel_mm = 1000 + 65536;
    ok = !s2_distance_delta(&p, &n, &dm) && ok;
    n.travel_mm = 1010;
    n.run_id = 1u;
    ok = !s2_distance_delta(&p, &n, &dm) && ok;
    n.run_id = 0u;
    n.flags = 0u;
    ok = !s2_distance_delta(&p, &n, &dm) && ok;
    return ok && !s2_distance_delta(&p, &n, NULL);
}

/* S2-U39: record sizes used to size the queues */
static bool test_interface_sizes (void)
{
    return (sizeof(s2_remote_cmd_t) == 12u) &&
           (sizeof(s2_line_report_t) == 8u) &&
           (sizeof(s2_scan_cmd_t) == 12u) &&
           (sizeof(s2_nav_request_t) == 12u) &&
           (sizeof(s2_robot_state_t) == 32u);
}

/* True if v has exactly one bit set */
static bool one_bit (UW v)
{
    return (0u != v) && ((v & (v - 1u)) == 0u);
}

/* S2-U40: flag bits distinct; codes equal subsystem 1's */
static bool test_interface_codes (void)
{
    UW const bits[] = { CTRL_FLG_HAZARD, CTRL_FLG_SCAN_FAULT,
                        CTRL_FLG_MOTION_FAULT, CTRL_FLG_REMOTE_STOP,
                        CTRL_FLG_LOCAL_STOP, CTRL_FLG_PICKUP,
                        CTRL_FLG_LINE_STOP };
    UW seen = 0u;
    UINT i;
    bool ok = true;

    for (i = 0u; i < (UINT)(sizeof(bits) / sizeof(bits[0])); i++)
    {
        ok = ok && one_bit(bits[i]) && ((seen & bits[i]) == 0u) &&
             ((bits[i] & ~CTRL_FLG_ALL) == 0u);
        seen |= bits[i];
    }
    return ok && (CTRL_FLG_ALL == seen) &&
           (S2_REMOTE_START == 1u) && (S2_REMOTE_STOP == 2u) &&
           (S2_REMOTE_RESET == 3u) &&
           (S2_RESULT_DUPLICATE == 3u) && (S2_RESULT_EXPIRED == 4u) &&
           (S2_RESULT_REJECTED == 6u) && (S2_RESULT_COMPLETED == 7u) &&
           (S2_RESULT_FAILED == 8u) && one_bit(SAFETY_FLG_STOP);
}

/* S2-U51 kernel half: on a real queue the size of remote_q, the third
   START is refused while the STOP adapter still sets its bit */
static bool test_stop_adapter_full (void)
{
    T_CMBF cmbf = { 0 };
    T_CFLG cflg = { 0 };
    T_RFLG rflg;
    ID q, flg;
    bool ok;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = S2_REMOTE_Q_BYTES;
    cmbf.maxmsz = (INT)sizeof(s2_remote_cmd_t);
    cflg.flgatr = TA_TFIFO | TA_WSGL;
    q = tk_cre_mbf(&cmbf);
    flg = tk_cre_flg(&cflg);
    if ((q < E_OK) || (flg < E_OK))
    {
        if (q >= E_OK)
        {
            (void)tk_del_mbf(q);
        }
        if (flg >= E_OK)
        {
            (void)tk_del_flg(flg);
        }
        return false;
    }

    ok = s2_bridge_send_command(q, S2_REMOTE_START, 1u, 0u) &&
         s2_bridge_send_command(q, S2_REMOTE_START, 2u, 0u) &&
         !s2_bridge_send_command(q, S2_REMOTE_START, 3u, 0u) &&
         s2_bridge_raise_stop(flg, 1234u) &&
         (tk_ref_flg(flg, &rflg) == E_OK) &&
         ((rflg.flgptn & CTRL_FLG_REMOTE_STOP) != 0u) &&
         !s2_bridge_send_command(0, S2_REMOTE_START, 4u, 0u) &&
         !s2_bridge_raise_stop(0, 0u);

    (void)tk_del_mbf(q);
    (void)tk_del_flg(flg);
    return ok;
}

/* S2-U54: watchdog LOAD value, x2 for erratum RP2040-E1, range checked */
static bool test_watchdog_load (void)
{
    return (drv_watchdog_load_value(1u) == 2000u) &&
           (drv_watchdog_load_value(MC_WATCHDOG_MS) ==
            MC_WATCHDOG_MS * 2000u) &&
           (drv_watchdog_load_value(200u) == 400000u) &&
           (drv_watchdog_load_value(DRV_WATCHDOG_MAX_MS) == 16776000u) &&
           (drv_watchdog_load_value(DRV_WATCHDOG_MAX_MS) <= 0x00FFFFFFu) &&
           (drv_watchdog_load_value(DRV_WATCHDOG_MAX_MS + 1u) == 0u) &&
           (drv_watchdog_load_value(0u) == 0u);
}

#if S2_HW_TEST
/* Print both encoders every 100 ms for the given number of seconds */
static void hw_print_counts (const char *step, INT seconds)
{
    drv_encoder_snapshot_t s;
    INT i;

    for (i = 0; i < seconds * 10; i++)
    {
        if (drv_encoder_snapshot(&s) == E_OK)
        {
            tm_printf((UB*)"[HW] %s enc1 %d (err %d)  enc2 %d (err %d)\n",
                      step, (INT)s.count[0], (INT)s.errors[0],
                      (INT)s.count[1], (INT)s.errors[1]);
        }
        tk_dly_tsk(100);
    }
}

#if (S2_HW_TEST == 2) || (S2_HW_TEST == 3)
/* S2-H04, split up to find where the full-speed freeze happens: each
   motor alone at 100%, then both, printing every 100 ms so the log
   shows the last moment the CPU was alive.  Wheels OFF THE GROUND. */
static void hw_full_speed_step (const char *step, INT m1, INT m2, INT seconds)
{
    drv_encoder_snapshot_t s0, s1;

    tm_printf((UB*)"[HW] %s: m1 %d%% m2 %d%% for %d s\n", step,
              m1, m2, seconds);
    (void)drv_encoder_snapshot(&s0);
    (void)drv_motor_set(DRV_MOTOR_1, (DRV_MOTOR_DUTY_MAX * m1) / 100);
    (void)drv_motor_set(DRV_MOTOR_2, (DRV_MOTOR_DUTY_MAX * m2) / 100);
    hw_print_counts(step, seconds);
    (void)drv_encoder_snapshot(&s1);
    (void)drv_motor_brake(DRV_MOTOR_1);
    (void)drv_motor_brake(DRV_MOTOR_2);
    tm_printf((UB*)"[HW] %s done: counts/s enc1 %d enc2 %d, new errors %d %d\n",
              step,
              (INT)(odom_count_delta(s1.count[0], s0.count[0]) / seconds),
              (INT)(odom_count_delta(s1.count[1], s0.count[1]) / seconds),
              (INT)(s1.errors[0] - s0.errors[0]),
              (INT)(s1.errors[1] - s0.errors[1]));
    hw_print_counts("S2-H04 braked", 2);
}

static void hw_full_speed (void)
{
    hw_full_speed_step("S2-H04a motor 1 only", 100, 0, 3);
    hw_full_speed_step("S2-H04b motor 2 only", 0, 100, 3);
    hw_full_speed_step("S2-H04c both", 100, 100, 5);
    tm_printf((UB*)"[HW] S2-H04 finished, all motors braked\n");
}
#endif /* S2_HW_TEST 2 or 3 */

#if S2_HW_TEST == 4
/* S2-H05: arm the watchdog, run motor 1 at 30%, kick for 3 s, then stop
   kicking. The board should reset within MC_WATCHDOG_MS and the next boot
   print "[S2] previous reset: WATCHDOG". If no reset comes within 2 s,
   brake and report FAIL so the motor never runs on.  Wheels OFF THE
   GROUND. */
static void hw_watchdog_check (void)
{
    UINT t;

    tm_printf((UB*)"[HW] S2-H05 starts in 5 s: wheels OFF THE GROUND\n");
    hw_print_counts("S2-H05 wait", 5);
    CHECK(drv_watchdog_start(MC_WATCHDOG_MS), "drv_watchdog_start");
    (void)drv_motor_set(DRV_MOTOR_1, (DRV_MOTOR_DUTY_MAX * 3) / 10);
    for (t = 0u; t < 3000u; t += 20u)
    {
        drv_watchdog_kick();
        tk_dly_tsk(20);
    }
    tm_printf((UB*)"[HW] S2-H05 stopped kicking: reset expected within %d ms\n",
              (INT)MC_WATCHDOG_MS);
    for (t = 0u; t < 2000u; t += 20u)
    {
        tm_printf((UB*)"[HW] S2-H05 %d ms since last kick\n", (INT)t);
        tk_dly_tsk(20);
    }
    (void)drv_motor_brake(DRV_MOTOR_1);
    tm_printf((UB*)"[HW] S2-H05 FAIL: no watchdog reset after 2 s, braked\n");
}
#endif /* S2_HW_TEST == 4 */

#if S2_HW_TEST == 2
/* S2-H01, S2-H02, S2-H04.  Wheels OFF THE GROUND. */
static void hw_motor_sequence (void)
{
    const int32_t duty30 = (DRV_MOTOR_DUTY_MAX * 3) / 10;
    UINT m;

    for (m = 0u; m < (UINT)DRV_MOTOR_COUNT; m++)
    {
        tm_printf((UB*)"[HW] S2-H01 motor %d forward 30%%\n", m + 1u);
        (void)drv_motor_set((drv_motor_id_t)m, duty30);
        hw_print_counts("S2-H01 fwd", 2);
        tm_printf((UB*)"[HW] S2-H02 motor %d BRAKE: turn by hand\n", m + 1u);
        (void)drv_motor_brake((drv_motor_id_t)m);
        hw_print_counts("S2-H02 brake", 3);
        tm_printf((UB*)"[HW] S2-H01 motor %d backward 30%%\n", m + 1u);
        (void)drv_motor_set((drv_motor_id_t)m, -duty30);
        hw_print_counts("S2-H01 back", 2);
        tm_printf((UB*)"[HW] S2-H02 motor %d COAST: turn by hand\n", m + 1u);
        (void)drv_motor_coast((drv_motor_id_t)m);
        hw_print_counts("S2-H02 coast", 3);
    }

    hw_full_speed();
}
#endif /* S2_HW_TEST == 2 */

static void hw_test (void)
{
    CHECK(drv_gpio_irq_init(), "drv_gpio_irq_init");
    CHECK(drv_motor_init(), "drv_motor_init");      /* starts braked */
    CHECK(drv_encoder_init(), "drv_encoder_init");

#if S2_HW_TEST == 4
    hw_watchdog_check();
#elif S2_HW_TEST == 3
    /* Full-speed diagnostic only: skip the hand and 30% steps. */
    tm_printf((UB*)"[HW] S2-H04 starts in 5 s: wheels OFF THE GROUND\n");
    hw_print_counts("S2-H04 wait", 5);
    hw_full_speed();
#else
    /* S2-H03: motors coast so the wheels turn freely by hand */
    (void)drv_motor_coast(DRV_MOTOR_1);
    (void)drv_motor_coast(DRV_MOTOR_2);
    tm_printf((UB*)"[HW] S2-H03: turn each wheel by hand (60 s)\n");
    hw_print_counts("S2-H03", 60);
    (void)drv_motor_brake(DRV_MOTOR_1);
    (void)drv_motor_brake(DRV_MOTOR_2);

#if S2_HW_TEST >= 2
    hw_motor_sequence();
#endif
#endif
}
#endif /* S2_HW_TEST */

EXPORT void subsystem2_test (void)
{
    TEST_INIT();

    /* S2-U01 and S2-U02 tested the starter producer, removed by
       S2 decision 29. */
    TEST_CHECK("S2-U03", test_pid_proportional(),
               "PID proportional term is kp * error");
    TEST_CHECK("S2-U04", test_pid_clamp(),
               "PID output stays within its limits");
    TEST_CHECK("S2-U05", test_pid_integral(),
               "PID integral adds ki * error per sample");
    TEST_CHECK("S2-U06", test_pid_anti_windup(),
               "PID integral does not wind up while saturated");
    TEST_CHECK("S2-U07", test_pid_derivative(),
               "PID derivative ignores setpoint steps");
    TEST_CHECK("S2-U08", test_pid_invalid(),
               "PID rejects invalid tuning and inputs");
    TEST_CHECK("S2-U09", test_pid_reset(),
               "PID reset clears history");
    TEST_CHECK("S2-U10", test_odom_distance(),
               "counts convert to millimetres");
    TEST_CHECK("S2-U11", test_odom_wrap(),
               "count delta survives counter wraparound");
    TEST_CHECK("S2-U12", test_odom_speed(),
               "speed from counts over an interval");
    TEST_CHECK("S2-U13", test_odom_turn(),
               "turn angle sign and magnitude");
    TEST_CHECK("S2-U14", test_odom_invalid(),
               "odometry rejects invalid geometry and overflow");
    TEST_CHECK("S2-U15", test_motor_map(),
               "motor duty maps to A/B outputs");
    TEST_CHECK("S2-U16", test_motor_reject(),
               "motor rejects out-of-range duty");
    TEST_CHECK("S2-U17", test_gpio_irq_reject(),
               "GPIO IRQ registration rejects invalid requests");
    TEST_CHECK("S2-U18", test_encoder_step(),
               "quadrature x2 decode");
    TEST_CHECK("S2-U19", test_encoder_snapshot(),
               "encoder snapshot rejects NULL, stable when idle");
    TEST_CHECK("S2-U20", test_motion_start(),
               "motion start: Idle + READY, or Fault(INIT)");
    TEST_CHECK("S2-U21", test_motion_speed(),
               "Speed state reaches the wheel targets");
    TEST_CHECK("S2-U22", test_motion_move(),
               "MOVE stops at target, DONE after stop confirmed");
    TEST_CHECK("S2-U23", test_motion_turn(),
               "TURN sign and angle (left positive)");
    TEST_CHECK("S2-U24", test_motion_straight(),
               "straight PID recovers lost wheel distance");
    TEST_CHECK("S2-U25", test_motion_stop_all(),
               "latched STOP in every state");
    TEST_CHECK("S2-U26", test_motion_reject_stop(),
               "motion commands rejected while STOP latched");
    TEST_CHECK("S2-U27", test_motion_replace(),
               "replace only while STOP clear; stale IDs rejected");
    TEST_CHECK("S2-U28", test_motion_stop_timing(),
               "stop confirmation timing, query and 2 s limit");
    TEST_CHECK("S2-U29", test_motion_stall(),
               "stalled wheel gives Fault(STALL)");
    TEST_CHECK("S2-U30", test_motion_timeout(),
               "slow MOVE gives Fault(TIMEOUT)");
    TEST_CHECK("S2-U31", test_motion_encoder_fault(),
               "bad snapshots or decode errors give Fault(ENCODER)");
    TEST_CHECK("S2-U32", test_motion_recovery(),
               "Fault cleared only by RESET with wheels still");
    TEST_CHECK("S2-U33", test_motion_params(),
               "invalid motion parameters rejected");
    TEST_CHECK("S2-U34", test_motion_odometry(),
               "odometry travel, pose and NEW_RUN");
    TEST_CHECK("S2-U35", test_motion_api(),
               "motion API builds and range-checks commands");
    TEST_CHECK("S2-U36", test_remote_make(),
               "remote_q record for START/RESET only");
    TEST_CHECK("S2-U37", test_telemetry_fill(),
               "telemetry snapshot: clamps, flags, codes, motion, line");
    TEST_CHECK("S2-U38", test_distance_delta(),
               "S4 distance delta rules");
    TEST_CHECK("S2-U39", test_interface_sizes(),
               "interface record sizes");
    TEST_CHECK("S2-U40", test_interface_codes(),
               "flag bits distinct; codes match subsystem 1");
    TEST_CHECK("S2-U41", test_ctrl_boot(),
               "boot: Initialising, then Ready, STOP latched");
    TEST_CHECK("S2-U42", test_ctrl_init_timeout(),
               "Initialising times out to Fault(INIT)");
    TEST_CHECK("S2-U43", test_ctrl_start(),
               "START only in Ready with healthy motion");
    TEST_CHECK("S2-U44", test_ctrl_script(),
               "script steps wait for DONE");
    TEST_CHECK("S2-U45", test_ctrl_stop_running(),
               "STOP bits in Running go to Stopped");
    TEST_CHECK("S2-U46", test_ctrl_stop_other(),
               "STOP in other states re-latches only");
    TEST_CHECK("S2-U47", test_ctrl_faults(),
               "fault paths latch STOP with the right code");
    TEST_CHECK("S2-U48", test_ctrl_reset_stopped(),
               "RESET from Stopped needs wheels stopped");
    TEST_CHECK("S2-U49", test_ctrl_reset_fault(),
               "RESET from Fault re-initialises");
    TEST_CHECK("S2-U50", test_ctrl_remote_rules(),
               "remote expiry, duplicate and result");
    TEST_CHECK("S2-U51",
               test_ctrl_stop_queue_full() && test_stop_adapter_full(),
               "STOP while remote_q is full");
    TEST_CHECK("S2-U52", test_ctrl_only_start_clears(),
               "only START in Ready clears STOP");
    TEST_CHECK("S2-U53", test_ctrl_robot_state(),
               "robot_state snapshot fields");
    TEST_CHECK("S2-U78", test_ctrl_stop_cause(),
               "stop cause of each STOP latch");
    TEST_CHECK("S2-U54", test_watchdog_load(),
               "watchdog load value (E1 x2, range)");
    TEST_CHECK("S2-U55", test_motion_ramp(),
               "duty ramp limits increases only");
    TEST_CHECK("S2-U56", test_motion_ramp_stall(),
               "ramp cannot cause a false stall");
    TEST_CHECK("S2-U57", test_line_make(),
               "line report accepts only agreed positions");
    TEST_CHECK("S2-U58", test_line_control_junction(),
               "black-black junction and exit are bounded");
    TEST_CHECK("S2-U59", test_line_control_curve(),
               "persistent correction becomes a curve without pivoting");
    TEST_CHECK("S2-U73", test_line_control_soften(),
               "barcode softening maps only curves to corrections");
    TEST_CHECK("S2-U60", test_scan_control_begin(),
               "hazard scan: motion STOP and one REQUEST");
    TEST_CHECK("S2-U61", test_scan_control_grant(),
               "STOP_CONFIRM and AUTHORISE only once wheels are still");
    TEST_CHECK("S2-U62", test_scan_control_outcomes(),
               "scan result for this request ends the scan");
    TEST_CHECK("S2-U63", test_scan_control_end(),
               "scan timeout sends CANCEL, abort sends STOP");
    TEST_CHECK("S2-U64", test_bypass_control_segments(),
               "bypass drives the plan in order; a bad plan fails");
    TEST_CHECK("S2-U65", test_bypass_control_motion(),
               "bypass advances only on its own DONE");
    TEST_CHECK("S2-U66", test_bypass_control_hazard(),
               "bypass hazard ignored in turns, stops forward travel");
    TEST_CHECK("S2-U67", test_bypass_control_reacquire(),
               "bounded reacquisition, step onto the line, heading check");
    TEST_CHECK("S2-U68", test_bypass_control_pass(),
               "pass: look, step, extra distance once passed");
    TEST_CHECK("S2-U69", test_bypass_control_pass_fail(),
               "pass: look fault, timeout, hazard, depth limit");
    TEST_CHECK("S2-U70", test_bypass_control_look_turn(),
               "pass: turn to look, back, pass only after pass_min");
    TEST_CHECK("S2-U71", test_telemetry_hump(),
               "telemetry hump: maximum, last and count; 255 cap; invalid 0");
    TEST_CHECK("S2-U72", test_telemetry_scan(),
               "telemetry obstacle fields follow S5's validity flags");
    TEST_CHECK("S2-U74", test_nav_control_hold(),
               "barcode command held to a junction; STRAIGHT; expiry");
    TEST_CHECK("S2-U75", test_nav_control_turn(),
               "junction turn: stop, step, spin, stop on the line");
    TEST_CHECK("S2-U76", test_nav_control_uturn(),
               "U-turn counts past 180 degrees, ignores a side branch");
    TEST_CHECK("S2-U77", test_nav_control_fail(),
               "junction turn fails on overshoot, rejection, slow stop");
    TEST_CHECK("S2-U79", test_nav_control_phase_limits(),
               "junction step and settle fail at the phase limit");
    TEST_CHECK("S2-U80", test_nav_control_hold_window(),
               "held command: taken at the limit, expired after, refreshed");

#if S2_HW_TEST
    hw_test();
#endif

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

/* End of file. */
