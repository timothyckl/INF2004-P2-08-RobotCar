/**
 * @file test_line_control.h
 * @brief Tests for the pure line-following recovery policy.
 */
#ifndef TEST_LINE_CONTROL_H
#define TEST_LINE_CONTROL_H

#include <stdbool.h>

#if APP_TEST

bool test_line_control_crossing(void);
bool test_line_control_search(void);

#endif

#endif /* TEST_LINE_CONTROL_H */

/* End of file. */
