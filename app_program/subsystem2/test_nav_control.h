/**
 * @file test_nav_control.h
 * @brief Tests for the pure barcode-command junction policy.
 */
#ifndef TEST_NAV_CONTROL_H
#define TEST_NAV_CONTROL_H

#include <stdbool.h>

#if APP_TEST

bool test_nav_control_hold(void);       /* S2-U74 */
bool test_nav_control_turn(void);       /* S2-U75 */
bool test_nav_control_uturn(void);      /* S2-U76 */
bool test_nav_control_fail(void);       /* S2-U77 */

#endif

#endif /* TEST_NAV_CONTROL_H */

/* End of file. */
