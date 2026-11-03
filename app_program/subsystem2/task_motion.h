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

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* Bench mode, the one place to set it (see subsystem2_test.c). Non-zero:
   subsystem2_start() does not start the motion or controller task, so
   the bench sequence owns the motors. Never commit it non-zero:
     1  encoder counts every 100 ms, motors off (S2-H03)
     2  also spins the motors (S2-H01, S2-H02, S2-H04)
     3  only the S2-H04 full-speed diagnostic (each motor, then both) */
#ifndef S2_HW_TEST
#define S2_HW_TEST  0
#endif

ER   s2_motion_create (void);
ER   s2_motion_start (void);
void s2_motion_wake (void);

/* STOP latency probe (TIMERAWL microseconds). The controller calls
   s2_motion_note_latch() when it sets the latch; Motion Control records
   when it next writes zero duty with STOP seen. Neither waits. */
void s2_motion_note_latch (void);
bool s2_stop_timing (uint32_t * p_latch_us, uint32_t * p_motor_off_us);

#endif /* TASK_MOTION_H */

/* End of file. */
