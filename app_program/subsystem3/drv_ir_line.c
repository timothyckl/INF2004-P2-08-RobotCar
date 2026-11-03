/**
 * @file drv_ir_line.c
 * @brief GPIO owner for the two line-following IR digital outputs.
 *
 * Hardware sources: docs/datasheets/ST188-datasheet.pdf,
 * docs/datasheets/TCRT5000-datahsheet.pdf, and
 * docs/datasheets/pico_w_datasheet.pdf. The modules are powered at 3.3 V so
 * their LM393-based digital outputs remain compatible with Pico W GPIO.
 */

#include "drv_ir_line.h"

#include <bsp/libbsp.h>
#include <sys/sysdef.h>
#include <tk/syslib.h>

#include <stdint.h>

#define DRV_IR_LINE_LEFT_PIN  (6U)
#define DRV_IR_LINE_RIGHT_PIN (16U)

static ER drv_ir_line_configure_input(uint32_t pin);

ER
drv_ir_line_init (void)
{
    ER error = drv_ir_line_configure_input(DRV_IR_LINE_LEFT_PIN);

    if (E_OK == error)
    {
        error = drv_ir_line_configure_input(DRV_IR_LINE_RIGHT_PIN);
    }

    return (error);
}

ER
drv_ir_line_read (drv_ir_line_sample_t * p_sample)
{
    ER error = E_PAR;

    if (NULL != p_sample)
    {
        p_sample->b_left_raw = (0U != gpio_get_val(DRV_IR_LINE_LEFT_PIN));
        p_sample->b_right_raw = (0U != gpio_get_val(DRV_IR_LINE_RIGHT_PIN));
        error = E_OK;
    }

    return (error);
}

static ER
drv_ir_line_configure_input (uint32_t pin)
{
    ER error = gpio_set_pin(pin, GPIO_MODE_IN);

    if (E_OK == error)
    {
        /* LM393 outputs are open-collector. Disable the RP2040 pad's
         * default pull-down before enabling its internal pull-up. */
        clr_w(GPIO(pin), GPIO_PDE);
        set_w(GPIO(pin), GPIO_PUE);
    }

    return (error);
}

/* End of file. */
