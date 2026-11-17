/**
 * @file controller_config.h
 * @brief Vehicle Controller mission tuning: the ONE place for its numbers.
 *
 * UNMEASURED values are starting points so the mission runs on the bench;
 * replace them from testing/subsystem2/TEST_LOG.md. AGREED values come
 * from docs/decisions.md (subsystem 5 decision 9).
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "motion_config.h"

/* ZIP line-follow profile. These bench-derived values remain provisional
   until they are validated on the final course and sensor height. */
#define CTRL_FOLLOW_SPEED_MM_S       40
#define CTRL_LINE_CORRECT_MIN_MM_S   32
#define CTRL_LINE_CORRECT_MAX_MM_S   38
#define CTRL_LINE_CURVE_INNER_MM_S    5
#define CTRL_LINE_CURVE_OUTER_MM_S   35
#define CTRL_LINE_STRENGTH_MAX      100U
#define CTRL_LINE_CURVE_ENTRY_MS    200U
#define CTRL_LINE_LEFT_HOLD_MS      250U
#define CTRL_LINE_RIGHT_HOLD_MS     150U
#define CTRL_LINE_JUNCTION_MS       100U
#define CTRL_LINE_CROSSING_MS      1500U
#define CTRL_LINE_EXIT_MIN_MS       600U
#define CTRL_LINE_EXIT_CENTER_MS    100U
#define CTRL_LINE_EXIT_SIDE_MS      150U
#define CTRL_LINE_EXIT_LIMIT_MS    2500U

/* Search speed and the STRAIGHT barcode-command crossing distance remain
   independent of ordinary line following. */
#define CTRL_SEARCH_SPEED_MM_S  100
#define CTRL_JUNCTION_CROSS_MM  60u

/* IF-06 freshness. A snapshot older than CTRL_LINE_MAX_AGE_MS (5 line
   samples) gives no steering, so the wheels are held still; one stale for
   CTRL_LINE_FAIL_MS is a line fault. UNMEASURED. */
#define CTRL_LINE_MAX_AGE_MS    50u
#define CTRL_LINE_FAIL_MS       500u

/* Reacquisition. AGREED: stable line for 100 ms, search bounded to 5 s
   (and to the plan's distance after a bypass), heading within 15 degrees
   of the original direction. */
#define CTRL_FOUND_MS           100u
#define CTRL_SEARCH_LIMIT_MS    5000u
#define CTRL_HEADING_TOL_MDEG   15000

/* Scan & Plan: the 15 s scan limit (decision 4) plus the stop and
   confirmation before it (2 s, decision 6) and a margin. */
#define CTRL_SCAN_LIMIT_MS      20000u

#endif /* CONTROLLER_CONFIG_H */

/* End of file. */
