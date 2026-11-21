/**
 * @file test_line_control.h
 * @brief Tests for the pure line-following recovery policy.
 */
#ifndef TEST_LINE_CONTROL_H
#define TEST_LINE_CONTROL_H

#include <stdbool.h>

#if APP_TEST

bool test_line_control_junction(void);
bool test_line_control_curve(void);
bool test_line_control_soften(void);    /* S2-U73 */

#endif

#endif /* TEST_LINE_CONTROL_H */

/* End of file. */
