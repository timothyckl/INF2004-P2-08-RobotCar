/**
 * @file test_bypass_control.h
 * @brief Tests for the pure bypass and line reacquisition.
 */
#ifndef TEST_BYPASS_CONTROL_H
#define TEST_BYPASS_CONTROL_H

#include <stdbool.h>

#if APP_TEST

bool test_bypass_control_segments(void);    /* S2-U64 */
bool test_bypass_control_motion(void);      /* S2-U65 */
bool test_bypass_control_hazard(void);      /* S2-U66 */
bool test_bypass_control_reacquire(void);   /* S2-U67 */

#endif

#endif /* TEST_BYPASS_CONTROL_H */

/* End of file. */
