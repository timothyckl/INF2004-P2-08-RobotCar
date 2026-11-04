/**
 * @file subsystem3_test.c
 * @brief On-target monitors, calibration, and tests for subsystem 3.
 */
#include "subsystem3.h"

#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "../test_report.h"

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

#if S3_LINE_FOLLOW_TEST
#include "../subsystem2/subsystem2.h"
#endif

#ifndef S3_HARDWARE_TEST
#define S3_HARDWARE_TEST 0
#endif

#ifndef S3_CALIBRATION_TEST
#define S3_CALIBRATION_TEST 0
#endif

#if APP_TEST

#if S3_CALIBRATION_TEST || S3_LINE_FOLLOW_TEST
#define S3_CAL_PREPARE_MS          3000u
#define S3_CAL_SAMPLE_PERIOD_MS    10u
#define S3_CAL_SAMPLE_COUNT        50u
#define S3_CAL_STABLE_MIN          45u
#define S3_CAL_RETRY_MS            2000u
#define S3_CAL_MONITOR_PERIOD_MS   20u
#define S3_CAL_FILTER_SAMPLES      3u
#if S3_CALIBRATION_TEST
#define S3_CAL_REPORT_SAMPLES      50u
#endif

#if S3_LINE_FOLLOW_TEST
#define S3_LINE_READY_LIMIT_MS     5000u
#define S3_LINE_REPORT_MS            50u
#define S3_SAVED_WHITE_LEVELS         0u
#define S3_SAVED_BLACK_LEVELS         3u
#endif

typedef struct
{
    uint16_t high_count;
    bool b_level_high;
    bool b_stable;
} s3_cal_observation_t;

static UB const g_s3_cal_heading[] =
    "s3: guided digital IR polarity calibration started\n";
static UB const g_s3_cal_prompt[] =
    "s3: place %s sensor over %s; sampling in 3 seconds\n";
static UB const g_s3_cal_result[] =
    "s3: %s/%s raw=%u; high count=%u of %u; stable=%s\n";
static UB const g_s3_cal_retry[] =
    "s3: calibration failed; adjust the sensor and retry the prompts\n";
#if S3_CALIBRATION_TEST
static UB const g_s3_cal_complete[] =
    "s3: calibration passed; classified monitor started\n";
static UB const g_s3_classified_result[] =
    "s3: left=%s right=%s; raw left=%u right=%u\n";
#endif
static UB const g_s3_left[] = "LEFT";
static UB const g_s3_right[] = "RIGHT";
static UB const g_s3_white[] = "WHITE";
static UB const g_s3_black[] = "BLACK";
static UB const g_s3_yes[] = "YES";
static UB const g_s3_no[] = "NO";

static bool s3_test_calibrate(s3_calibration_t * p_calibration);
static s3_cal_observation_t s3_test_observe(UB const * p_sensor_name,
                                             UB const * p_surface_name,
                                             uint8_t sensor_mask);
#if S3_CALIBRATION_TEST
static void s3_test_classified_monitor(
    s3_calibration_t const * p_calibration);
static void s3_test_print_classified(uint8_t black_mask,
                                     uint8_t raw_levels);
#endif
#if S3_LINE_FOLLOW_TEST
static void s3_test_line_trial(s3_calibration_t const * p_calibration);
static bool s3_test_wait_ready(void);
static uint32_t s3_test_now_ms(void);
static bool s3_test_line_position(s3_line_t line, uint8_t * p_position);
static UB const * s3_test_line_name(uint8_t position);
#endif
#elif S3_HARDWARE_TEST
#define S3_RAW_SAMPLE_MS  100u

static UB const g_s3_raw_heading[] =
    "s3: raw front-sensor monitor started; reset the Pico to stop\n";
static UB const g_s3_raw_result[] =
    "s3 raw: left=%u right=%u\n";

static void s3_test_raw_monitor(void);
#else
static UB const g_s3_blocked[] =
    "[TEST] S3-H01..H03 BLOCKED  use S3_HW_TEST=1 for raw input "
    "or S3_CAL_TEST=1 for calibration\n";

static APP_STATS s3_test_read_stats(void);
#endif

void
subsystem3_test (void)
{
#if S3_LINE_FOLLOW_TEST
    s3_calibration_t calibration = {0};

    if (!s3_ir_calibrate(&calibration, S3_SAVED_WHITE_LEVELS,
                         S3_SAVED_BLACK_LEVELS))
    {
        tm_printf((UB *)"s3: saved calibration is invalid; trial aborted\n");
        (void)s2_line_stop();
        return;
    }

    tm_printf((UB *)"s3: using saved calibration: WHITE=LOW, BLACK=HIGH\n");
    s3_test_line_trial(&calibration);
#elif S3_CALIBRATION_TEST
    s3_calibration_t calibration = {0};

    tm_printf(g_s3_cal_heading);
    while (!s3_test_calibrate(&calibration))
    {
        tm_printf(g_s3_cal_retry);
        (void) tk_dly_tsk(S3_CAL_RETRY_MS);
    }
    s3_test_classified_monitor(&calibration);
#elif S3_HARDWARE_TEST
    s3_test_raw_monitor();
#else
    APP_STATS before;
    APP_STATS after;

    TEST_INIT();

    tm_printf(g_s3_blocked);
    before = s3_test_read_stats();
    (void) tk_dly_tsk(2000u);
    after = s3_test_read_stats();
    TEST_CHECK("INT-01", after.received >= (before.received + 5u),
               "samples flow from subsystem 2 to subsystem 3");
    TEST_CHECK("S3-U01",
               (after.received > 0u) && (after.min <= after.max) &&
               (after.min >= 0) && (after.max <= 100),
               "legacy statistics remain self-consistent");

    TEST_SUMMARY("subsystem3");
#endif
}

#if S3_CALIBRATION_TEST || S3_LINE_FOLLOW_TEST
static bool
s3_test_calibrate (s3_calibration_t * p_calibration)
{
    s3_cal_observation_t left_white;
    s3_cal_observation_t left_black;
    s3_cal_observation_t right_white;
    s3_cal_observation_t right_black;
    uint8_t white_levels = 0u;
    uint8_t black_levels = 0u;
    bool b_left_valid;
    bool b_right_valid;
    bool b_calibration_valid;

    TEST_INIT();

    left_white = s3_test_observe(g_s3_left, g_s3_white,
                                 S3_IR_LEFT_MASK);
    left_black = s3_test_observe(g_s3_left, g_s3_black,
                                 S3_IR_LEFT_MASK);
    right_white = s3_test_observe(g_s3_right, g_s3_white,
                                  S3_IR_RIGHT_MASK);
    right_black = s3_test_observe(g_s3_right, g_s3_black,
                                  S3_IR_RIGHT_MASK);

    if (left_white.b_level_high)
    {
        white_levels |= S3_IR_LEFT_MASK;
    }
    if (right_white.b_level_high)
    {
        white_levels |= S3_IR_RIGHT_MASK;
    }
    if (left_black.b_level_high)
    {
        black_levels |= S3_IR_LEFT_MASK;
    }
    if (right_black.b_level_high)
    {
        black_levels |= S3_IR_RIGHT_MASK;
    }

    b_left_valid = left_white.b_stable && left_black.b_stable &&
                   (left_white.b_level_high != left_black.b_level_high);
    b_right_valid = right_white.b_stable && right_black.b_stable &&
                    (right_white.b_level_high != right_black.b_level_high);
    b_calibration_valid = b_left_valid && b_right_valid &&
        s3_ir_calibrate(p_calibration, white_levels, black_levels);

    TEST_CHECK("S3-H01", b_left_valid,
               "front-left GP6 distinguishes white and black");
    TEST_CHECK("S3-H02", b_right_valid,
               "front-right GP16 distinguishes white and black");
    TEST_CHECK("S3-H03", b_calibration_valid,
               "two-sensor digital polarity calibration accepted");
    TEST_SUMMARY("subsystem3-calibration");

    return (b_calibration_valid);
}

static s3_cal_observation_t
s3_test_observe (UB const * p_sensor_name,
                 UB const * p_surface_name,
                 uint8_t sensor_mask)
{
    s3_cal_observation_t observation = {0u, false, false};
    uint16_t const stable_low_max = S3_CAL_SAMPLE_COUNT -
                                    S3_CAL_STABLE_MIN;

    tm_printf(g_s3_cal_prompt, p_sensor_name, p_surface_name);
    (void) tk_dly_tsk(S3_CAL_PREPARE_MS);

    for (uint16_t sample = 0u; sample < S3_CAL_SAMPLE_COUNT; sample++)
    {
        if (0u != (s3_ir_gpio_read() & sensor_mask))
        {
            observation.high_count++;
        }
        (void) tk_dly_tsk(S3_CAL_SAMPLE_PERIOD_MS);
    }

    if (observation.high_count >= S3_CAL_STABLE_MIN)
    {
        observation.b_level_high = true;
        observation.b_stable = true;
    }
    else if (observation.high_count <= stable_low_max)
    {
        observation.b_stable = true;
    }
    else
    {
        /* Mixed readings do not form a stable calibration observation. */
    }

    /* Counts and Boolean values are representable by the target UINT type. */
    tm_printf(g_s3_cal_result, p_sensor_name, p_surface_name,
              (UINT) observation.b_level_high,
              (UINT) observation.high_count,
              (UINT) S3_CAL_SAMPLE_COUNT,
              observation.b_stable ? g_s3_yes : g_s3_no);

    return (observation);
}

#if S3_CALIBRATION_TEST
static void
s3_test_classified_monitor (s3_calibration_t const * p_calibration)
{
    s3_filter_t filter = {0};
    uint8_t last_black_mask = 0u;
    uint16_t report_count = 0u;
    bool b_reported = false;

    tm_printf(g_s3_cal_complete);

    for (;;)
    {
        uint8_t const raw_levels = s3_ir_gpio_read();
        uint8_t black_mask = 0u;

        if (s3_ir_normalize(p_calibration, raw_levels, &black_mask) &&
            s3_ir_filter(&filter, black_mask, S3_CAL_FILTER_SAMPLES))
        {
            report_count++;
            if ((!b_reported) || (filter.stable != last_black_mask) ||
                (report_count >= S3_CAL_REPORT_SAMPLES))
            {
                s3_test_print_classified(filter.stable, raw_levels);
                last_black_mask = filter.stable;
                report_count = 0u;
                b_reported = true;
            }
        }
        (void) tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
    }
}

static void
s3_test_print_classified (uint8_t black_mask, uint8_t raw_levels)
{
    UB const * p_left_surface = g_s3_white;
    UB const * p_right_surface = g_s3_white;

    if (0u != (black_mask & S3_IR_LEFT_MASK))
    {
        p_left_surface = g_s3_black;
    }
    if (0u != (black_mask & S3_IR_RIGHT_MASK))
    {
        p_right_surface = g_s3_black;
    }

    /* Raw Boolean levels are representable by the target UINT type. */
    tm_printf(g_s3_classified_result, p_left_surface, p_right_surface,
              (UINT) (0u != (raw_levels & S3_IR_LEFT_MASK)),
              (UINT) (0u != (raw_levels & S3_IR_RIGHT_MASK)));
}
#endif

#if S3_LINE_FOLLOW_TEST
static void
s3_test_line_trial (s3_calibration_t const * p_calibration)
{
    s3_filter_t filter = {0};
    uint32_t last_report_ms = 0u;
    uint8_t last_position = UINT8_MAX;
    bool b_delivery_failed = false;
    bool b_invalid_reading = false;

    if (!s3_test_wait_ready())
    {
        tm_printf((UB *)"s3 line: S2 did not reach READY; trial aborted\n");
        (void)s2_line_stop();
        return;
    }

    tm_printf((UB *)"s3 line: WHEELS MUST BE RAISED\n");
    tm_printf((UB *)"s3 line: place the car on the track, then press GP20 "
                    "to START or STOP\n");
    tm_printf((UB *)"s3 line: continuous supervised trial; no time limit\n");

    for (;;)
    {
        uint8_t black_mask = 0u;
        uint8_t position = S2_LINE_STOP;
        uint32_t const now_ms = s3_test_now_ms();

        if (!s3_ir_normalize(p_calibration, s3_ir_gpio_read(), &black_mask))
        {
            if (!b_invalid_reading)
            {
                tm_printf((UB *)"s3 line: invalid calibrated reading; STOP\n");
                (void)s2_line_stop();
                b_invalid_reading = true;
            }
            (void)tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
            continue;
        }
        b_invalid_reading = false;

        if (!s3_ir_filter(&filter, black_mask, S3_CAL_FILTER_SAMPLES))
        {
            (void)tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
            continue;
        }

        if (!s3_test_line_position(s3_ir_follow_decision(filter.stable),
                                   &position))
        {
            tm_printf((UB *)"s3 line: invalid line decision; STOP\n");
            (void)s2_line_stop();
            (void)tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
            continue;
        }

        if ((position != last_position)
            || ((now_ms - last_report_ms) >= S3_LINE_REPORT_MS))
        {
            if (!s2_line_submit(position, now_ms))
            {
                if (!b_delivery_failed)
                {
                    tm_printf((UB *)"s3 line: report delivery failed; STOP\n");
                }
                (void)s2_line_stop();
                b_delivery_failed = true;
            }
            else
            {
                b_delivery_failed = false;

                if (position != last_position)
                {
                    tm_printf((UB *)"s3 line: left=%s right=%s "
                                      "mask=%u command=%s\n",
                              (0u != (filter.stable & S3_IR_LEFT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (0u != (filter.stable & S3_IR_RIGHT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (UINT)filter.stable,
                              s3_test_line_name(position));
                }
                last_position = position;
                last_report_ms = now_ms;
            }
        }

        (void)tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
    }
}

static bool
s3_test_wait_ready (void)
{
    uint32_t const start_ms = s3_test_now_ms();
    s2_robot_state_t state;

    while ((s3_test_now_ms() - start_ms) < S3_LINE_READY_LIMIT_MS)
    {
        if ((E_OK == s2_get_robot_state(&state))
            && (S2_SYS_READY == state.state))
        {
            return (true);
        }
        (void)tk_dly_tsk(S3_CAL_MONITOR_PERIOD_MS);
    }

    return (false);
}

static uint32_t
s3_test_now_ms (void)
{
    SYSTIM time;

    return ((tk_get_tim(&time) >= E_OK) ? (uint32_t)time.lo : 0u);
}

static bool
s3_test_line_position (s3_line_t line, uint8_t * p_position)
{
    if (NULL == p_position)
    {
        return (false);
    }

    if (S3_LINE_STRAIGHT == line)
    {
        *p_position = S2_LINE_CENTER;
    }
    else if (S3_LINE_LEFT == line)
    {
        *p_position = S2_LINE_LEFT;
    }
    else if (S3_LINE_RIGHT == line)
    {
        *p_position = S2_LINE_RIGHT;
    }
    else if (S3_LINE_JUNCTION == line)
    {
        *p_position = S2_LINE_STOP;
    }
    else
    {
        return (false);
    }

    return (true);
}

static UB const *
s3_test_line_name (uint8_t position)
{
    static UB const straight[] = "STRAIGHT";
    static UB const left[] = "LEFT";
    static UB const right[] = "RIGHT";
    static UB const invalid[] = "INVALID";
    UB const * p_name = invalid;

    if (S2_LINE_CENTER == position)
    {
        p_name = straight;
    }
    else if (S2_LINE_LEFT == position)
    {
        p_name = left;
    }
    else if (S2_LINE_RIGHT == position)
    {
        p_name = right;
    }
    else if (S2_LINE_STOP == position)
    {
        static UB const stop[] = "STOP";

        p_name = stop;
    }
    else
    {
        /* Retain INVALID for values that cannot command motion. */
    }

    return (p_name);
}
#endif
#elif S3_HARDWARE_TEST
static void
s3_test_raw_monitor (void)
{
    tm_printf(g_s3_raw_heading);

    for (;;)
    {
        uint8_t const raw_levels = s3_ir_gpio_read();

        /* Raw Boolean levels are representable by the target UINT type. */
        tm_printf(g_s3_raw_result,
                  (UINT) (0u != (raw_levels & S3_IR_LEFT_MASK)),
                  (UINT) (0u != (raw_levels & S3_IR_RIGHT_MASK)));
        (void) tk_dly_tsk(S3_RAW_SAMPLE_MS);
    }
}
#else
static APP_STATS
s3_test_read_stats (void)
{
    APP_STATS statistics = {0};

    (void) tk_loc_mtx(mtx_stats, TMO_FEVR);
    statistics = app_stats;
    (void) tk_unl_mtx(mtx_stats);

    return (statistics);
}
#endif

#endif /* APP_TEST */

/* End of file. */
