/**
 * @file test_controller.h
 * @brief Subsystem 2 Vehicle Controller unit cases S2-U41..S2-U53 and
 *        S2-U60..S2-U67 (TEST=1).
 *        S2-U51's kernel half is in subsystem2_test.c.
 */
#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <stdbool.h>

bool test_ctrl_boot (void);             /* S2-U41 */
bool test_ctrl_init_timeout (void);     /* S2-U42 */
bool test_ctrl_start (void);            /* S2-U43 */
bool test_ctrl_script (void);           /* S2-U44 */
bool test_ctrl_stop_running (void);     /* S2-U45 */
bool test_ctrl_stop_other (void);       /* S2-U46 */
bool test_ctrl_faults (void);           /* S2-U47 */
bool test_ctrl_reset_stopped (void);    /* S2-U48 */
bool test_ctrl_reset_fault (void);      /* S2-U49 */
bool test_ctrl_remote_rules (void);     /* S2-U50 */
bool test_ctrl_stop_queue_full (void);  /* S2-U51 (logic half) */
bool test_ctrl_only_start_clears (void);/* S2-U52 */
bool test_ctrl_robot_state (void);      /* S2-U53 */
bool test_ctrl_start_line (void);       /* S2-U60 */
bool test_ctrl_follow (void);           /* S2-U61 */
bool test_ctrl_navigate (void);         /* S2-U62 */
bool test_ctrl_bypass (void);           /* S2-U63 */
bool test_ctrl_scan_outcomes (void);    /* S2-U64 */
bool test_ctrl_reacquire (void);        /* S2-U65 */
bool test_ctrl_mission_stop (void);     /* S2-U66 */
bool test_ctrl_line_policy (void);      /* S2-U67 */

#endif /* TEST_CONTROLLER_H */

/* End of file. */
