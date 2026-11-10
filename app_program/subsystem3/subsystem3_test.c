/**
 * @file subsystem3_test.c
 * @brief On-target monitors, line-follow trial, and tests for subsystem 3.
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

#if APP_TEST

#if S3_LINE_FOLLOW_TEST
#define S3_LINE_READY_POLL_MS        20u
#define S3_LINE_FILTER_SAMPLES        3u
#define S3_LINE_READY_LIMIT_MS     5000u
#define S3_LINE_SAMPLE_MS            10u
#define S3_LINE_REPORT_MS            30u
#define S3_LINE_LOG_MS              500u
#define S3_LEFT_INITIAL_WHITE       136u
#define S3_LEFT_MINIMUM_WHITE        96u
#define S3_LEFT_MAXIMUM_WHITE       168u
#define S3_LEFT_BLACK_MARGIN         32u
#define S3_LEFT_WHITE_MARGIN         20u
#define S3_LEFT_BLACK_REFERENCE    1500u
#define S3_RIGHT_INITIAL_WHITE      180u
#define S3_RIGHT_MINIMUM_WHITE      100u
#define S3_RIGHT_MAXIMUM_WHITE      300u
#define S3_RIGHT_BLACK_MARGIN       500u
#define S3_RIGHT_WHITE_MARGIN       350u
#define S3_RIGHT_BLACK_REFERENCE   3300u

static UB const g_s3_white[] = "WHITE";
static UB const g_s3_black[] = "BLACK";

static void s3_test_line_trial(void);
static bool s3_test_wait_ready(void);
static uint32_t s3_test_now_ms(void);
static bool s3_test_line_position(s3_line_t line, uint8_t * p_position);
static UB const * s3_test_line_name(uint8_t position);
#elif S3_HARDWARE_TEST
#define S3_RAW_SAMPLE_MS  100u

static UB const g_s3_raw_heading[] =
    "s3: raw AO monitor: left=GP26 right=GP27 barcode-DO=GP16\n";
static UB const g_s3_raw_result[] =
    "s3 raw: left_adc=%u right_adc=%u barcode_do=%u\n";
static UB const g_s3_raw_error[] =
    "s3 raw: ADC read failed, error=%d\n";

static void s3_test_raw_monitor(void);
#else
static UB const g_s3_blocked[] =
    "[TEST] S3-H01..H03 BLOCKED  use S3_HW_TEST=1 for raw input "
    "or S3_LINE_TEST=1 for line following\n";

static APP_STATS s3_test_read_stats(void);
#endif

void
subsystem3_test (void)
{
#if S3_LINE_FOLLOW_TEST
    s3_test_line_trial();
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

#if S3_LINE_FOLLOW_TEST
static void
s3_test_line_trial (void)
{
    static s3_ir_adaptive_config_t const left_config =
    {
        S3_LEFT_INITIAL_WHITE,
        S3_LEFT_MINIMUM_WHITE,
        S3_LEFT_MAXIMUM_WHITE,
        S3_LEFT_BLACK_MARGIN,
        S3_LEFT_WHITE_MARGIN
    };
    static s3_ir_adaptive_config_t const right_config =
    {
        S3_RIGHT_INITIAL_WHITE,
        S3_RIGHT_MINIMUM_WHITE,
        S3_RIGHT_MAXIMUM_WHITE,
        S3_RIGHT_BLACK_MARGIN,
        S3_RIGHT_WHITE_MARGIN
    };
    s3_ir_adaptive_t left_state = {0};
    s3_ir_adaptive_t right_state = {0};
    s3_filter_t filter = {0};
    uint32_t last_report_ms = 0u;
    uint32_t last_log_ms = 0u;
    uint8_t last_position = UINT8_MAX;
    bool b_delivery_failed = false;
    bool b_read_failed = false;

    if ((!s3_ir_adaptive_init(&left_state, &left_config))
        || (!s3_ir_adaptive_init(&right_state, &right_config)))
    {
        tm_printf((UB *)"s3 line: invalid adaptive configuration; STOP\n");
        (void)s2_line_stop();
        return;
    }

    if (!s3_test_wait_ready())
    {
        tm_printf((UB *)"s3 line: S2 did not reach READY; trial aborted\n");
        (void)s2_line_stop();
        return;
    }

    tm_printf((UB *)"s3 line: WHEELS MUST BE RAISED\n");
    tm_printf((UB *)"s3 line: place the car on the track, then press GP20 "
                    "to START or STOP\n");
    tm_printf((UB *)"s3 line: adaptive AO left=GP26 right=GP27\n");
    tm_printf((UB *)"s3 line: continuous supervised trial; no time limit\n");

    for (;;)
    {
        s3_ir_adc_sample_t sample = {0u, 0u};
        uint8_t black_mask = 0u;
        uint8_t position = S2_LINE_STOP;
        uint8_t strength = 0u;
        uint32_t const now_ms = s3_test_now_ms();
        ER const read_error = s3_ir_gpio_read_analog(&sample);

        if (E_OK != read_error)
        {
            if (!b_read_failed)
            {
                tm_printf((UB *)"s3 line: ADC read failed, error=%d; STOP\n",
                          read_error);
                (void)s2_line_stop();
                b_read_failed = true;
            }
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
            continue;
        }
        b_read_failed = false;

        if (s3_ir_adaptive_update(&left_state, sample.left))
        {
            black_mask |= S3_IR_LEFT_MASK;
        }
        if (s3_ir_adaptive_update(&right_state, sample.right))
        {
            black_mask |= S3_IR_RIGHT_MASK;
        }

        if (!s3_ir_filter(&filter, black_mask, S3_LINE_FILTER_SAMPLES))
        {
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
            continue;
        }

        if (!s3_test_line_position(s3_ir_follow_decision(filter.stable),
                                   &position))
        {
            tm_printf((UB *)"s3 line: invalid line decision; STOP\n");
            (void)s2_line_stop();
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
            continue;
        }

        if (S2_LINE_LEFT == position)
        {
            strength = s3_ir_adaptive_strength(
                &left_state, S3_LEFT_BLACK_REFERENCE);
        }
        else if (S2_LINE_RIGHT == position)
        {
            strength = s3_ir_adaptive_strength(
                &right_state, S3_RIGHT_BLACK_REFERENCE);
        }
        else
        {
            /* Straight and crossing reports do not carry steering strength. */
        }

        if ((position != last_position)
            || ((now_ms - last_report_ms) >= S3_LINE_REPORT_MS))
        {
            if (!s2_line_submit_scaled(position, strength, now_ms))
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

                if ((position != last_position)
                    || ((now_ms - last_log_ms) >= S3_LINE_LOG_MS))
                {
                    tm_printf((UB *)"s3 line: adc=%u/%u filtered=%u/%u "
                                      "threshold=%u/%u left=%s right=%s "
                                      "mask=%u strength=%u command=%s\n",
                              (UINT)sample.left,
                              (UINT)sample.right,
                              (UINT)left_state.filtered,
                              (UINT)right_state.filtered,
                              (UINT)left_state.threshold,
                              (UINT)right_state.threshold,
                              (0u != (filter.stable & S3_IR_LEFT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (0u != (filter.stable & S3_IR_RIGHT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (UINT)filter.stable,
                              (UINT)strength,
                              s3_test_line_name(position));
                    last_log_ms = now_ms;
                }
                last_position = position;
                last_report_ms = now_ms;
            }
        }

        (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
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
        (void)tk_dly_tsk(S3_LINE_READY_POLL_MS);
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
        *p_position = S2_LINE_BOTH_BLACK;
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
    else if (S2_LINE_BOTH_BLACK == position)
    {
        static UB const crossing[] = "CROSSING";

        p_name = crossing;
    }
    else
    {
        /* Retain INVALID for values that cannot command motion. */
    }

    return (p_name);
}
#elif S3_HARDWARE_TEST
static void
s3_test_raw_monitor (void)
{
    tm_printf(g_s3_raw_heading);

    for (;;)
    {
        s3_ir_adc_sample_t sample = {0u, 0u};
        ER const error = s3_ir_gpio_read_analog(&sample);

        if (E_OK == error)
        {
            tm_printf(g_s3_raw_result,
                      (UINT)sample.left,
                      (UINT)sample.right,
                      (UINT)s3_ir_gpio_read_barcode());
        }
        else
        {
            tm_printf(g_s3_raw_error, error);
        }
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
