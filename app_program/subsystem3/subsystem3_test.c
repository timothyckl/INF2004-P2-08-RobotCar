/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	The logic cases run on the host (testing/host: test_main.c,
 *	test_track_scan.c). This checks the live Line Sensor task.
 *	Record results in testing/subsystem3/TEST_LOG.md and the INT- result
 *	in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "s3_config.h"
#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "../test_report.h"

#if APP_TEST

#ifndef S3_IR_DIAGNOSTIC
#define S3_IR_DIAGNOSTIC 0
#endif

#if S3_IR_DIAGNOSTIC
#define S3_IR_DIAG_PERIOD_MS        100U
#define S3_IR_DIAG_HEARTBEAT        10U
#define S3_IR_DIAG_SAMPLES          600U

static void s3_ir_diagnostic(void);
#endif

EXPORT void subsystem3_test(void)
{
	s3_line_status_t	before, after;
	SYSTIM			now;
	ER			er1, er2;

	TEST_INIT();

	/* INT-03 (IF-06): the Line Sensor task publishes a fresh snapshot
	   every 10 ms. In 1 s at least 80 new samples (allowing for test
	   load), and the latest is no older than 50 ms. */
	er1 = s3_line_get(&before);
	tk_dly_tsk(1000);
	er2 = s3_line_get(&after);
	(void)tk_get_tim(&now);
	TEST_CHECK("INT-03", (E_OK == er1) && (E_OK == er2)
		   && ((after.seq - before.seq) >= 80u)
		   && ((now.lo - after.captured_ms) <= 50u),
		   "line sensor publishes IF-06 every 10 ms");

	/* S3-U02: the published snapshot is internally consistent: no
	   steering unless valid, steering within +/-1000. */
	TEST_CHECK("S3-U02",
		   ((0u != (after.flags & S3_ST_VALID))
		    || (0 == after.steer_permille))
		   && (after.steer_permille <= 1000)
		   && (after.steer_permille >= -1000),
		   "IF-06 snapshot fields consistent");

	TEST_SUMMARY("subsystem3");

#if S3_IR_DIAGNOSTIC
	s3_ir_diagnostic();
#endif
}

#if S3_IR_DIAGNOSTIC
/** Print live ADC readings without changing the production snapshot. */
static void s3_ir_diagnostic(void)
{
    s3_ir_adaptive_config_t const left_config =
    {
        S3_LEFT_INITIAL_WHITE, S3_LEFT_MINIMUM_WHITE,
        S3_LEFT_MAXIMUM_WHITE, S3_LEFT_BLACK_MARGIN,
        S3_LEFT_WHITE_MARGIN
    };
    s3_ir_adaptive_config_t const right_config =
    {
        S3_RIGHT_INITIAL_WHITE, S3_RIGHT_MINIMUM_WHITE,
        S3_RIGHT_MAXIMUM_WHITE, S3_RIGHT_BLACK_MARGIN,
        S3_RIGHT_WHITE_MARGIN
    };
    s3_ir_adaptive_t left = {0};
    s3_ir_adaptive_t right = {0};
    bool const b_ready = s3_ir_adaptive_init(&left, &left_config)
                         && s3_ir_adaptive_init(&right, &right_config);

    tm_printf((UB *)"[S3 IR] diagnostic start: 60 seconds; "
              "move white and black under each sensor\n");

    for (uint16_t sample = 0U; sample < S3_IR_DIAG_SAMPLES; sample++)
    {
        s3_ir_adc_sample_t adc = {0U, 0U};
        ER const er = s3_ir_gpio_read_analog(&adc);
        bool const b_valid = b_ready && (E_OK == er);

        if (b_valid)
        {
            bool const b_left = s3_ir_adaptive_update(&left, adc.left);
            bool const b_right = s3_ir_adaptive_update(&right, adc.right);

            if ((0U == (sample % S3_IR_DIAG_HEARTBEAT))
                || b_left || b_right)
            {
                tm_printf((UB *)"[S3 IR] adc=%u/%u filtered=%u/%u "
                          "threshold=%u/%u black=%u/%u barcode=%u\n",
                          (UINT)adc.left, (UINT)adc.right,
                          (UINT)left.filtered, (UINT)right.filtered,
                          (UINT)left.threshold, (UINT)right.threshold,
                          (UINT)b_left, (UINT)b_right,
                          (UINT)s3_ir_gpio_read_barcode());
            }
        }
        else if (!b_valid)
        {
            tm_printf((UB *)"[S3 IR] ERROR ADC read/configuration %d\n", er);
        }
        else
        {
            /* The reading is unchanged between one-second heartbeats. */
        }

        (void)tk_dly_tsk((RELTIM)S3_IR_DIAG_PERIOD_MS);
    }

    tm_printf((UB *)"[S3 IR] diagnostic complete; reset to run again\n");
}
#endif

#endif /* APP_TEST */
