/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "comms_test.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	T_RTSK	rtsk;
	ER	er;

	TEST_INIT();

	/* S1-U01: The temporary IF-05 compatibility task exists. */
	er = tk_ref_tsk(tid_led, &rtsk);
	TEST_CHECK("S1-U01", er == E_OK, "compatibility task exists");

	/* S1-U02: The compatibility task remains asleep and does not busy-loop. */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++)
		{
			tk_dly_tsk(97);
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI))
			{
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9,
		           "compatibility task remains asleep");
	}

    TEST_CHECK("S1-U04", comms_test_commands(), "command parser logic");
    TEST_CHECK("S1-U05", comms_test_telemetry(), "telemetry codec logic");
    TEST_CHECK("S1-U06", comms_test_timers(), "heartbeat and retry logic");
    TEST_CHECK("S1-U07", comms_test_bridge(), "MQTT service bridge logic");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
