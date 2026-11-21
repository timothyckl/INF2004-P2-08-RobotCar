/**
 * @file drv_motor.h
 * @brief Subsystem 2 DC motor driver for the Robo Pico motor outputs.
 *
 * Only the Motion Control task may call drv_motor_init(), drv_motor_set(),
 * drv_motor_brake() and drv_motor_coast() (TEAM.md: Motion Control is the
 * only writer of motor PWM).
 *
 * Hardware (Robo-Pico-datasheet.pdf section 4 and table 1; design review
 * section 2): Motor 1 = GP8 (A) / GP9 (B), PWM slice 4. Motor 2 = GP10 (A)
 * / GP11 (B), PWM slice 5. Truth table: A high B low = forward, A low B
 * high = backward, both low = brake, both high = coast. Whether "forward"
 * moves the robot forward depends on the motor wiring; Motion Control owns
 * that mapping.
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* 125 MHz clk_sys (pico_rp2040 sysdef.h CLK_PLL_SYS_FREQ) / 20 kHz, the
   driver's PWM frequency in Robo-Pico-datasheet.pdf table 1 row 10. Duty
   is in counter steps: DRV_MOTOR_DUTY_MAX is 100%. */
#define DRV_MOTOR_DUTY_MAX  6250

typedef enum
{
    DRV_MOTOR_1 = 0,    /* M1A/M1B, GP8/GP9 */
    DRV_MOTOR_2,        /* M2A/M2B, GP10/GP11 */
    DRV_MOTOR_COUNT
} drv_motor_id_t;

/** Compare values for one motor's A and B outputs. */
typedef struct
{
    uint16_t cc_a;
    uint16_t cc_b;
} drv_motor_cc_t;

ER   drv_motor_init (void);
ER   drv_motor_set (drv_motor_id_t motor, int32_t duty);
ER   drv_motor_brake (drv_motor_id_t motor);
ER   drv_motor_coast (drv_motor_id_t motor);

/* Pure mapping used by drv_motor_set(); exposed for unit tests. */
bool drv_motor_map (int32_t duty, drv_motor_cc_t * p_cc);

#endif /* DRV_MOTOR_H */

/* End of file. */
