/**
 * @file test_safety.h
 * @brief Request matching, inhibition and cancellation specifications.
 */
#ifndef TEST_SAFETY_H
#define TEST_SAFETY_H

#include "../../../../app_program/subsystem5/include/test_interface.h"

/** Run this suite against actual logic, or report every case blocked. */
void test_safety_run (test_interface_api_t const * p_api);

#endif /* TEST_SAFETY_H */

/* End of file. */
