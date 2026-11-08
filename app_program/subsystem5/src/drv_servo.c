/**
 * @file drv_servo.c
 * @brief SG90 servo PWM (see drv_servo.h).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../include/drv_servo.h"

#define DRV_SERVO_PIN           (12u)
#define DRV_SERVO_SLICE         (6u)
#define DRV_SERVO_DIV           (125u << 4)     /* 8.4 fixed point: 125.0 */
#define DRV_SERVO_TOP           (19999u)        /* 20 ms frame of 1 us */
#define DRV_SERVO_CENTRE_US     (1500)
#define DRV_SERVO_US_PER_DEG    (10)

/** Pulse width for an angle already checked against the range. */
static uint32_t drv_servo_pulse (int16_t angle_deg);

ER
drv_servo_init (void)
{
    UW const slice_base = PWM_BASE + (DRV_SERVO_SLICE * 0x14u);

    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait a few clock cycles for the block to come out of reset. */
    }

    out_w(slice_base + PWM_CHx_DIV, DRV_SERVO_DIV);

    ER er = pwm_set_wrap(DRV_SERVO_PIN, DRV_SERVO_TOP);

    if (E_OK == er)
    {
        er = pwm_set_cc(DRV_SERVO_PIN, drv_servo_pulse(0));
    }

    if (E_OK == er)
    {
        er = pwm_set_enabled(DRV_SERVO_PIN, TRUE);
    }

    if (E_OK == er)
    {
        er = pwm_set_pin(DRV_SERVO_PIN);
    }

    return (er);
}

ER
drv_servo_set_angle (int16_t angle_deg)
{
    ER er = E_PAR;

    if ((angle_deg >= -DRV_SERVO_ANGLE_MAX)
        && (angle_deg <= DRV_SERVO_ANGLE_MAX))
    {
        er = pwm_set_cc(DRV_SERVO_PIN, drv_servo_pulse(angle_deg));
    }

    return (er);
}

static uint32_t
drv_servo_pulse (int16_t angle_deg)
{
    /* The cast is safe: |angle| <= 60 gives 900..2100 us. */
    return ((uint32_t) (DRV_SERVO_CENTRE_US
                        + ((int32_t) angle_deg * DRV_SERVO_US_PER_DEG)));
}

/* End of file. */
