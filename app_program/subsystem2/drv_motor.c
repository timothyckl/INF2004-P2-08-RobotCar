/**
 * @file drv_motor.c
 * @brief Robo Pico dual DC-motor PWM driver for the RP2040 BSP.
 *
 * The Robo Pico maps motor 1 A/B to GP8/GP9 and motor 2 A/B to
 * GP10/GP11. The chassis motors are mirrored: forward motion drives M1A
 * and M2B with PWM while holding M1B and M2A low. Both inputs low applies
 * the driver's brake state.
 */

#include "drv_motor.h"

#include <sys/sysdef.h>
#include <tk/syslib.h>

#include <stdbool.h>

#define DRV_MOTOR_LEFT_FORWARD_PIN  (8U)
#define DRV_MOTOR_LEFT_REVERSE_PIN  (9U)
#define DRV_MOTOR_RIGHT_FORWARD_PIN (11U)
#define DRV_MOTOR_RIGHT_REVERSE_PIN (10U)
#define DRV_MOTOR_PWM_WRAP          (9999U)
#define DRV_MOTOR_COMPARE_SCALE     (10U)
#define DRV_MOTOR_RESET_TIMEOUT     (100000U)

static bool g_b_motor_initialized = false;

static ER drv_motor_configure_pin(uint32_t pin);

ER
drv_motor_init (void)
{
    uint32_t reset_timeout = DRV_MOTOR_RESET_TIMEOUT;
    ER error = E_OK;

    /* USE_PTMR is disabled in this build, so kernel startup does not release
     * the PWM block. Motion Control owns that initialization here. */
    set_w(RESETS_RESET, RESETS_RESET_PWM);
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while ((0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) &&
           (0U < reset_timeout))
    {
        reset_timeout--;
    }

    if (0U == reset_timeout)
    {
        error = E_TMOUT;
    }

    if (E_OK == error)
    {
        error = drv_motor_configure_pin(DRV_MOTOR_LEFT_FORWARD_PIN);
    }
    if (E_OK == error)
    {
        error = drv_motor_configure_pin(DRV_MOTOR_LEFT_REVERSE_PIN);
    }
    if (E_OK == error)
    {
        error = drv_motor_configure_pin(DRV_MOTOR_RIGHT_FORWARD_PIN);
    }
    if (E_OK == error)
    {
        error = drv_motor_configure_pin(DRV_MOTOR_RIGHT_REVERSE_PIN);
    }

    if (E_OK == error)
    {
        error = pwm_set_wrap(DRV_MOTOR_LEFT_FORWARD_PIN,
                             DRV_MOTOR_PWM_WRAP);
    }
    if (E_OK == error)
    {
        error = pwm_set_wrap(DRV_MOTOR_RIGHT_FORWARD_PIN,
                             DRV_MOTOR_PWM_WRAP);
    }

    if (E_OK == error)
    {
        error = pwm_set_enabled(DRV_MOTOR_LEFT_FORWARD_PIN, TRUE);
    }
    if (E_OK == error)
    {
        error = pwm_set_enabled(DRV_MOTOR_RIGHT_FORWARD_PIN, TRUE);
    }

    if (E_OK == error)
    {
        g_b_motor_initialized = true;
        error = drv_motor_stop();
    }

    return (error);
}

ER
drv_motor_set_forward (uint16_t left_speed, uint16_t right_speed)
{
    ER error = E_OBJ;

    if ((!g_b_motor_initialized) ||
        (DRV_MOTOR_SPEED_MAX < left_speed) ||
        (DRV_MOTOR_SPEED_MAX < right_speed))
    {
        if (g_b_motor_initialized)
        {
            error = E_PAR;
        }
    }
    else
    {
        error = pwm_set_cc(DRV_MOTOR_LEFT_REVERSE_PIN, 0U);
        if (E_OK == error)
        {
            error = pwm_set_cc(DRV_MOTOR_RIGHT_REVERSE_PIN, 0U);
        }
        if (E_OK == error)
        {
            error = pwm_set_cc(
                DRV_MOTOR_LEFT_FORWARD_PIN,
                (uint32_t)left_speed * DRV_MOTOR_COMPARE_SCALE);
        }
        if (E_OK == error)
        {
            error = pwm_set_cc(
                DRV_MOTOR_RIGHT_FORWARD_PIN,
                (uint32_t)right_speed * DRV_MOTOR_COMPARE_SCALE);
        }
    }

    return (error);
}

ER
drv_motor_stop (void)
{
    ER error = E_OBJ;

    if (g_b_motor_initialized)
    {
        error = pwm_set_cc(DRV_MOTOR_LEFT_FORWARD_PIN, 0U);
        if (E_OK == error)
        {
            error = pwm_set_cc(DRV_MOTOR_LEFT_REVERSE_PIN, 0U);
        }
        if (E_OK == error)
        {
            error = pwm_set_cc(DRV_MOTOR_RIGHT_FORWARD_PIN, 0U);
        }
        if (E_OK == error)
        {
            error = pwm_set_cc(DRV_MOTOR_RIGHT_REVERSE_PIN, 0U);
        }
    }

    return (error);
}

static ER
drv_motor_configure_pin (uint32_t pin)
{
    ER error = pwm_set_pin(pin);

    if (E_OK == error)
    {
        error = pwm_set_cc(pin, 0U);
    }

    return (error);
}

/* End of file. */
