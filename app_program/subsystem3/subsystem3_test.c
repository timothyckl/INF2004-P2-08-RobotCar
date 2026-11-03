/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	The logic cases run on the host (testing/host: test_main.c,
 *	test_track_scan.c). This checks the live Line Sensor task.
 *	Record results in testing/subsystem3/TEST_LOG.md and the INT- result
 *	in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

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
}

#endif /* APP_TEST */
