/**
 * @file drv_control_buttons.c
 * @brief Robo Pico active-low user buttons on GP20 and GP21.
 */

#include "drv_control_buttons.h"

#include <bsp/libbsp.h>
#include <sys/sysdef.h>
#include <tk/syslib.h>

#include <stdint.h>

#define DRV_CONTROL_BUTTON_START_PIN (20U)
#define DRV_CONTROL_BUTTON_STOP_PIN  (21U)

static ER drv_control_buttons_configure(uint32_t pin);

ER
drv_control_buttons_init (void)
{
    ER error = drv_control_buttons_configure(
        DRV_CONTROL_BUTTON_START_PIN);

    if (E_OK == error)
    {
        error = drv_control_buttons_configure(
            DRV_CONTROL_BUTTON_STOP_PIN);
    }

    return (error);
}

ER
drv_control_buttons_read (bool * p_start_pressed,
                          bool * p_stop_pressed)
{
    ER error = E_PAR;

    if ((NULL != p_start_pressed) && (NULL != p_stop_pressed))
    {
        *p_start_pressed =
            (0U == gpio_get_val(DRV_CONTROL_BUTTON_START_PIN));
        *p_stop_pressed =
            (0U == gpio_get_val(DRV_CONTROL_BUTTON_STOP_PIN));
        error = E_OK;
    }

    return (error);
}

static ER
drv_control_buttons_configure (uint32_t pin)
{
    ER error = gpio_set_pin(pin, GPIO_MODE_IN);

    if (E_OK == error)
    {
        clr_w(GPIO(pin), GPIO_PDE);
        set_w(GPIO(pin), GPIO_PUE);
    }

    return (error);
}

/* End of file. */
