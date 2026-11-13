/**
 * @file subsystem4_internal.h
 * @brief Private interfaces shared by Subsystem 4 implementation modules.
 */
#ifndef SUBSYSTEM4_INTERNAL_H
#define SUBSYSTEM4_INTERNAL_H

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

#ifndef S4_RAW_LOG
#define S4_RAW_LOG                  0
#endif

void s4_imu_task(INT start_code, void * p_exinf);
ER s4_status_publish(s4_terrain_result_t const * p_result);

#if S4_RAW_LOG
/* Diagnostic only (S4_RAW_LOG): reports where the IMU task waits if its
 * loop stops advancing. */
extern ID s4_watch_imu_id;
void s4_watch_task(INT start_code, void * p_exinf);
#endif

#endif /* SUBSYSTEM4_INTERNAL_H */
