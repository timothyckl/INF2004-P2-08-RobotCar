/**
 * @file drv_motor.h
 * @brief Robo Pico dual DC-motor PWM driver.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

#include <stdint.h>

#define DRV_MOTOR_SPEED_MAX (1000U)

ER drv_motor_init(void);
ER drv_motor_set_forward(uint16_t left_speed, uint16_t right_speed);
ER drv_motor_stop(void);

#endif /* DRV_MOTOR_H */

/* End of file. */
