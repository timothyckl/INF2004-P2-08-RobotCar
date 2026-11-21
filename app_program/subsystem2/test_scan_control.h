/**
 * @file test_scan_control.h
 * @brief Tests for the pure Scan & Plan handshake.
 */
#ifndef TEST_SCAN_CONTROL_H
#define TEST_SCAN_CONTROL_H

#include <stdbool.h>

#if APP_TEST

bool test_scan_control_begin(void);     /* S2-U60 */
bool test_scan_control_grant(void);     /* S2-U61 */
bool test_scan_control_outcomes(void);  /* S2-U62 */
bool test_scan_control_end(void);       /* S2-U63 */

#endif

#endif /* TEST_SCAN_CONTROL_H */

/* End of file. */
