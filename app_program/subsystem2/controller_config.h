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

/* Line following. UNMEASURED: base speed, search speed, how far to drive
   to clear a junction for a STRAIGHT command. */
#define CTRL_FOLLOW_SPEED_MM_S  150
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
