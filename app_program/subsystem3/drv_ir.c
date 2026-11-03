/**
 * @file drv_ir.c
 * @brief Driver for the three ST188 IR line-sensor modules (digital DOUT).
 */
#include "drv_ir.h"

#include <bsp/libbsp.h>

#include "line_track.h"

#define DRV_IR_COUNT    (3u)

static uint8_t const g_pins[DRV_IR_COUNT] =
{
    DRV_IR_PIN_LEFT, DRV_IR_PIN_CENTRE, DRV_IR_PIN_RIGHT
};

static uint8_t const g_bits[DRV_IR_COUNT] =
{
    LINE_TRACK_BIT_LEFT, LINE_TRACK_BIT_CENTRE, LINE_TRACK_BIT_RIGHT
};

ER
drv_ir_init (void)
{
    ER err = E_OK;

    for (uint32_t index = 0u; (index < DRV_IR_COUNT) && (E_OK == err); index++)
    {
        err = gpio_set_pin(g_pins[index], GPIO_MODE_IN);

        if (E_OK == err)
        {
            /* NOTE: The open-collector LM393 output needs a pull-up. The
               internal pull-up is harmless beside the module's own and
               keeps the input defined if that resistor is absent. Both
               pulls are set explicitly, so the pad's reset state does not
               matter. */
            clr_w(GPIO(g_pins[index]), GPIO_PDE);
            set_w(GPIO(g_pins[index]), GPIO_PUE);
        }
    }

    return (err);
}

uint8_t
drv_ir_read (void)
{
    /* One register read samples all three pins at the same instant. */
    UW const levels = in_w(GPIO_IN);
    uint8_t  bits   = 0u;

    for (uint32_t index = 0u; index < DRV_IR_COUNT; index++)
    {
        if (DRV_IR_BLACK_LEVEL == ((levels >> g_pins[index]) & 1u))
        {
            bits |= g_bits[index];
        }
    }

    return (bits);
}

/* End of file. */

