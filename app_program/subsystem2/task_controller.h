/**
 * @file task_controller.h
 * @brief Subsystem 2 Vehicle Controller task. PRIVATE to subsystem 2.
 *
 * subsystem2_init() calls s2_controller_create() before
 * s2_motion_create(); subsystem2_start() calls s2_controller_start().
 */
#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* 1: Running performs the Week 10 demo script (forward 500 mm, left
   90 degrees, forward 500 mm) after START. 0: Running does nothing yet. */
#ifndef S2_DEMO_SCRIPT
#define S2_DEMO_SCRIPT  1
#endif

ER s2_controller_create (void);
ER s2_controller_start (void);

/* Bench and demo without MQTT. s2_local_stop() sets CTRL_FLG_LOCAL_STOP;
   s2_local_command() sends START or RESET through remote_q like S1, with
   request IDs 0x80000001 upwards so they never equal S1's. false if not
   created yet, not START/RESET, or remote_q is full. */
bool s2_local_stop (void);
bool s2_local_command (uint8_t command);

#endif /* TASK_CONTROLLER_H */

/* End of file. */
