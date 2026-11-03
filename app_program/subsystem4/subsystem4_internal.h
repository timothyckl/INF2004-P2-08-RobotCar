/**
 * @file subsystem4_internal.h
 * @brief Private interfaces shared by Subsystem 4 implementation modules.
 */
#ifndef SUBSYSTEM4_INTERNAL_H
#define SUBSYSTEM4_INTERNAL_H

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

void s4_imu_task(INT start_code, void * p_exinf);
ER s4_status_publish(s4_terrain_result_t const * p_result);

#endif /* SUBSYSTEM4_INTERNAL_H */
