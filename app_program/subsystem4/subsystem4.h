/**
 * @file subsystem4.h
 * @brief Public and task-local interfaces for IMU terrain sensing.
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "terrain_logic.h"

#define S4_PRI_IMU              18
/* Data-ready poll period. The kernel tick is 10 ms (CNF_TIMER_PERIOD), so
 * each delay lasts 10 to 20 ms and the 100 Hz sensor is read at its
 * newest sample, skipping some (decision 6). The calibration line prints
 * the measured read rate. */
#define S4_POLL_PERIOD_MS       10u
#define S4_RETRY_PERIOD_MS      1000u

/* Legacy starter interface retained until subsystem integration begins. */
#define S4_SUMMARY_MAX          48
#define FLG_BATCH               0x0002u

IMPORT ID mbf_summary;
IMPORT ID flg_events;

ER subsystem4_get_status(s4_terrain_result_t * p_result);

#endif /* SUBSYSTEM4_H */
