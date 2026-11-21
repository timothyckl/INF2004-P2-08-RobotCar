/**
 * @file drv_servo.h
 * @brief SG90 servo on GP12, PWM slice 6 channel A (AGENTS.md, TEAM.md).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * 50 Hz frame with a 1 us tick (125 MHz clk_sys / 125, TOP 19999).
 * Pulse = 1400 us + 10 us per degree, positive = left. The Tower Pro SG90
 * sheet gives a 20 ms period and 500-2400 us for about 180 degrees; this
 * mapping is provisional until fully calibrated (S5-U47): on 2026-11-12
 * the sensor faced forward at 1400 to 1450 us and was last clear of the
 * chassis at 950 and 1850 us, which Dalton put at 45 degrees each way, so
 * the centre is 1400 us. Angles are limited to +/-45 degrees (950..1850
 * us, S2 decision 22).
 *
 * Only clears the PWM block reset, never asserts it: Motion Control uses
 * slices 4 and 5 of the same block (docs/decisions.md, subsystem 2,
 * decision 2). Call only from the scan task.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

#include "../../app.h"

#define DRV_SERVO_ANGLE_MAX     (45)

/** Configure slice 6 for 50 Hz, centre the servo, then hand GP12 to PWM. */
ER drv_servo_init (void);

/** Command an angle in degrees; E_PAR outside +/-DRV_SERVO_ANGLE_MAX. */
ER drv_servo_set_angle (int16_t angle_deg);

#endif /* DRV_SERVO_H */

/* End of file. */
