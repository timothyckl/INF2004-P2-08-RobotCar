/**
 * @file subsystem4.h
 * @brief Public and task-local interfaces for IMU terrain sensing.
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "terrain_logic.h"

#ifndef S4_RAW_LOG
#define S4_RAW_LOG              0       /* build option, see the makefile */
#endif

#define S4_PRI_IMU              18
/* Data-ready poll period. The kernel tick is 10 ms (CNF_TIMER_PERIOD), so
 * each delay lasts 10 to 20 ms and the 100 Hz sensor is read at its
 * newest sample, skipping some (decision 6). The calibration line prints
 * the measured read rate. */
#define S4_POLL_PERIOD_MS       10u
#define S4_RETRY_PERIOD_MS      1000u

ER subsystem4_get_status(s4_terrain_result_t * p_result);

#if S4_RAW_LOG
/* Diagnostic only (S4_RAW_LOG builds): s4_watch_task reports this task's
 * kernel wait state each second that *p_loops has not advanced. Call from
 * one task only, before the loop starts; at most three tasks. */
void s4_watch_add(ID task_id, volatile uint32_t const * p_loops,
                  char const * p_name);
#endif

#endif /* SUBSYSTEM4_H */
