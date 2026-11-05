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
#include "../test_report.h"

#if APP_TEST

#ifndef S3_IR_DIAGNOSTIC
#define S3_IR_DIAGNOSTIC 0
#endif

#if S3_IR_DIAGNOSTIC
#define S3_IR_DIAG_PERIOD_MS        100U
#define S3_IR_DIAG_HEARTBEAT        10U
#define S3_IR_DIAG_SAMPLES          600U
#define S3_IR_DIAG_FRONT_LEFT_BIT   0x01U
#define S3_IR_DIAG_FRONT_RIGHT_BIT  0x02U
#define S3_IR_DIAG_BARCODE_BIT      0x04U

static void s3_ir_diagnostic(void);
static uint8_t s3_ir_read_levels(void);
static char const * s3_ir_surface(uint8_t black_mask, uint8_t sensor_bit);
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
/** Print live, normalised readings without changing the production snapshot. */
static void s3_ir_diagnostic(void)
{
    s3_calibration_t calibration = {0};
    uint8_t previous_black = S3_IR_PATTERNS;
    bool const b_ready = s3_ir_calibrate(
        &calibration,
        (uint8_t)(~S3_IR_BLACK_LEVELS & (S3_IR_PATTERNS - 1U)),
        S3_IR_BLACK_LEVELS);

    tm_printf((UB *)"[S3 IR] diagnostic start: 60 seconds; "
              "move white and black under each sensor\n");

    for (uint16_t sample = 0U; sample < S3_IR_DIAG_SAMPLES; sample++)
    {
        uint8_t black_mask = 0U;
        uint8_t const raw_levels = s3_ir_read_levels();
        bool const b_valid = b_ready
                             && s3_ir_normalize(&calibration, raw_levels,
                                                &black_mask);

        if (b_valid
            && ((black_mask != previous_black)
                || (0U == (sample % S3_IR_DIAG_HEARTBEAT))))
        {
            tm_printf((UB *)"[S3 IR] left=%s right=%s barcode=%s "
                      "mask=0x%02x\n",
                      s3_ir_surface(black_mask,
                                    S3_IR_DIAG_FRONT_LEFT_BIT),
                      s3_ir_surface(black_mask,
                                    S3_IR_DIAG_FRONT_RIGHT_BIT),
                      s3_ir_surface(black_mask, S3_IR_DIAG_BARCODE_BIT),
                      (UINT)black_mask);
            previous_black = black_mask;
        }
        else if (!b_valid)
        {
            tm_printf((UB *)"[S3 IR] ERROR invalid polarity configuration\n");
        }
        else
        {
            /* The reading is unchanged between one-second heartbeats. */
        }

        (void)tk_dly_tsk((RELTIM)S3_IR_DIAG_PERIOD_MS);
    }

    tm_printf((UB *)"[S3 IR] diagnostic complete; reset to run again\n");
}

/** Read the three comparator outputs in the subsystem's bit order. */
static uint8_t s3_ir_read_levels(void)
{
    UW const gpio_levels = in_w(GPIO_IN);
    uint8_t levels = 0U;

    if (0U != (gpio_levels & (1UL << S3_PIN_FRONT_LEFT)))
    {
        levels |= S3_IR_DIAG_FRONT_LEFT_BIT;
    }

    if (0U != (gpio_levels & (1UL << S3_PIN_FRONT_RIGHT)))
    {
        levels |= S3_IR_DIAG_FRONT_RIGHT_BIT;
    }

    if (0U != (gpio_levels & (1UL << S3_PIN_BARCODE)))
    {
        levels |= S3_IR_DIAG_BARCODE_BIT;
    }

    return levels;
}

/** Convert one bit of the normalised black mask to a console label. */
static char const * s3_ir_surface(uint8_t black_mask, uint8_t sensor_bit)
{
    char const * p_surface = "WHITE";

    if (0U != (black_mask & sensor_bit))
    {
        p_surface = "BLACK";
    }

    return p_surface;
}
#endif

#endif /* APP_TEST */
