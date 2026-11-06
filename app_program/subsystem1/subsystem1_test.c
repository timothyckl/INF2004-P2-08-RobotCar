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
	TEST_INIT();

	/* S1-U01..S1-U03 tested the starter LED / IF-05 compatibility task,
	   retired with it (testing/subsystem1/TEST_LOG.md). */

    TEST_CHECK("S1-U04", comms_test_commands(), "command parser logic");
    TEST_CHECK("S1-U05", comms_test_telemetry(), "telemetry codec logic");
    TEST_CHECK("S1-U06", comms_test_timers(), "heartbeat and retry logic");
    TEST_CHECK("S1-U07", comms_test_bridge(), "MQTT service bridge logic");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
