/**
 * @file subsystem5_test.c
 * @brief Pico W test entry called by usermain in TEST=1 firmware.
 *
 * S5-U01 and S5-U02 are retired. Cases start at S5-U03.
 * Source: testing/subsystem5/TEST_LOG.md.
 */
#include "../include/subsystem5_test.h"

#if APP_TEST
#include "../../../testing/subsystem5/unit_tests/include/test_ranging.h"
#include "../../../testing/subsystem5/unit_tests/include/test_runner.h"
#include "../../../testing/subsystem5/unit_tests/include/test_safety.h"
#include "../../../testing/subsystem5/unit_tests/include/test_scanning.h"
#include "../../../testing/subsystem5/unit_tests/include/test_subject.h"
#include <stddef.h>

void
subsystem5_test (void)
{
    test_interface_api_t const * p_api = test_subject_get();

    test_runner_begin();

    if ((NULL != p_api) && (!test_subject_valid(p_api)))
    {
        test_runner_check("BINDING", false, "incomplete subsystem adapter");
        p_api = NULL;
    }

    test_ranging_run(p_api);
    test_safety_run(p_api);
    test_scanning_run(p_api);
    test_runner_finish();
}
#endif

/* End of file. */
