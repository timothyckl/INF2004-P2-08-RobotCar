/**
 * @file test_motion.h
 * @brief Subsystem 2 Motion Control unit cases S2-U20..S2-U35, S2-U55,
 *        S2-U56 (TEST=1).
 */
#ifndef TEST_MOTION_H
#define TEST_MOTION_H

#include <stdbool.h>

bool test_motion_start (void);          /* S2-U20 */
bool test_motion_speed (void);          /* S2-U21 */
bool test_motion_move (void);           /* S2-U22 */
bool test_motion_turn (void);           /* S2-U23 */
bool test_motion_straight (void);       /* S2-U24 */
bool test_motion_stop_all (void);       /* S2-U25 */
bool test_motion_reject_stop (void);    /* S2-U26 */
bool test_motion_replace (void);        /* S2-U27 */
bool test_motion_stop_timing (void);    /* S2-U28 */
bool test_motion_stall (void);          /* S2-U29 */
bool test_motion_timeout (void);        /* S2-U30 */
bool test_motion_encoder_fault (void);  /* S2-U31 */
bool test_motion_recovery (void);       /* S2-U32 */
bool test_motion_params (void);         /* S2-U33 */
bool test_motion_odometry (void);       /* S2-U34 */
bool test_motion_api (void);            /* S2-U35 */
bool test_motion_ramp (void);           /* S2-U55 */
bool test_motion_ramp_stall (void);     /* S2-U56 */

#endif /* TEST_MOTION_H */

/* End of file. */
