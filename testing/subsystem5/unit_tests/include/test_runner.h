/**
 * @file test_runner.h
 * @brief Pico W console reporting for passed, failed and blocked cases.
 */
#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include "../../../../app_program/subsystem5/include/test_interface.h"

/** One independently reportable subsystem test case. */
typedef struct
{
    char const * p_id;
    char const * p_description;
    bool (*p_run)(test_interface_api_t const * p_api);
} test_runner_case_t;

/** Reset reporting totals before each on-device test run. */
void test_runner_begin (void);

/** Report an assertion using the existing serial PASS/FAIL format. */
void test_runner_check (char const * p_id, bool b_passed,
                        char const * p_description);

/** Run cases, or mark them blocked when actual S5 logic is unavailable. */
void test_runner_cases (test_runner_case_t const * p_cases, uint32_t count,
                        test_interface_api_t const * p_api);

/** Print the on-device totals; blocked cases are not counted as passed. */
void test_runner_finish (void);

#endif /* TEST_RUNNER_H */

/* End of file. */
