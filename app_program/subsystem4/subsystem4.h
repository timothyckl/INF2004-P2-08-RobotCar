/**
 * @file subsystem4.h
 * @brief Public and task-local interfaces for IMU terrain sensing.
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "terrain_logic.h"

#define S4_PRI_IMU              18
#define S4_SAMPLE_PERIOD_MS     10u
#define S4_RETRY_PERIOD_MS      1000u

/* Legacy starter interface retained until subsystem integration begins. */
#define S4_SUMMARY_MAX          48
#define FLG_BATCH               0x0002u

IMPORT ID mbf_summary;
IMPORT ID flg_events;

ER subsystem4_get_status(s4_terrain_result_t * p_result);

#endif /* SUBSYSTEM4_H */
