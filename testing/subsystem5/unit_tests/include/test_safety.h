/**
 * @file test_safety.h
 * @brief Request matching, inhibition and cancellation specifications.
 */
#ifndef TEST_SAFETY_H
#define TEST_SAFETY_H

#include "../../../../app_program/subsystem5/include/test_interface.h"

/** Run the safety-gating cases against the bound production logic. */
void test_safety_run (test_interface_api_t const * p_api);

#endif /* TEST_SAFETY_H */

/* End of file. */
