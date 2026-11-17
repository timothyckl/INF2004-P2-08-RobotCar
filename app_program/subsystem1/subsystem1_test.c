/**
 * @file subsystem1_test.c
 * @brief On-target tests enabled by APP_TEST.
 */
#include "subsystem1.h"
#include "comms_test.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test (void)
{
    TEST_INIT();

    TEST_CHECK("S1-U04", comms_test_commands(), "command parser logic");
    TEST_CHECK("S1-U05", comms_test_telemetry(), "telemetry codec logic");
    TEST_CHECK("S1-U06", comms_test_timers(), "heartbeat/retry logic");
    TEST_CHECK("S1-U07", comms_test_bridge(), "MQTT service bridge logic");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/* End of file. */
