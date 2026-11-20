/**
 * @file subsystem3_test.c
 * @brief On-target monitors and tests for subsystem 3.
 */
#include "subsystem3.h"

#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "../test_report.h"

#ifndef S3_HARDWARE_TEST
#define S3_HARDWARE_TEST 0
#endif

#if APP_TEST

#if S3_HARDWARE_TEST
#define S3_RAW_SAMPLE_MS  100u

static UB const g_s3_raw_heading[] =
    "s3: raw AO monitor: left=GP26 right=GP27 barcode=GP28\n";
static UB const g_s3_raw_result[] =
    "s3 raw: left_adc=%u right_adc=%u barcode_adc=%u\n";
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
#if S3_HARDWARE_TEST
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
#if S3_HARDWARE_TEST
static void
s3_test_raw_monitor (void)
{
    tm_printf(g_s3_raw_heading);

    for (;;)
    {
        s3_ir_adc_sample_t sample = {0u, 0u};
        uint16_t barcode = 0u;
        ER error = s3_ir_gpio_read_analog(&sample);

        if (E_OK == error)
        {
            error = s3_ir_gpio_read_barcode(&barcode);
        }

        if (E_OK == error)
        {
            tm_printf(g_s3_raw_result,
                      (UINT)sample.left,
                      (UINT)sample.right,
                      (UINT)barcode);
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
