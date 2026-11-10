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

ER subsystem4_get_status(s4_terrain_result_t * p_result);

#endif /* SUBSYSTEM4_H */
