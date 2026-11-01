/**
 * @file task_motion.h
 * @brief Subsystem 2 Motion Control task. PRIVATE to subsystem 2.
 *
 * subsystem2_init() calls s2_motion_create(); subsystem2_start() calls
 * s2_motion_start(). The Vehicle Controller calls s2_motion_wake() right
 * after setting the latched STOP, so the task reacts before its next
 * 10 ms period.
 */
#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "../app.h"

ER   s2_motion_create (void);
ER   s2_motion_start (void);
void s2_motion_wake (void);

#endif /* TASK_MOTION_H */

/* End of file. */
