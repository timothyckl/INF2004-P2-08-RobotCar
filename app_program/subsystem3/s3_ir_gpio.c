/**
 * @file s3_ir_gpio.c
 * @brief Digital GPIO access for the two front IR line sensors.
 */
#include "s3_ir_gpio.h"

#include "s3_ir.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#define S3_IR_GPIO_LEFT_PIN   6u
#define S3_IR_GPIO_RIGHT_PIN  16u

static void s3_ir_gpio_input_init(uint32_t pin);

void
s3_ir_gpio_init (void)
{
    s3_ir_gpio_input_init(S3_IR_GPIO_LEFT_PIN);
    s3_ir_gpio_input_init(S3_IR_GPIO_RIGHT_PIN);
}

uint8_t
s3_ir_gpio_read (void)
{
    uint32_t const gpio_levels = in_w(GPIO_IN);
    uint8_t sensor_levels = 0u;

    if (0u != (gpio_levels & (UINT32_C(1) << S3_IR_GPIO_LEFT_PIN)))
    {
        sensor_levels |= S3_IR_LEFT_MASK;
    }
    if (0u != (gpio_levels & (UINT32_C(1) << S3_IR_GPIO_RIGHT_PIN)))
    {
        sensor_levels |= S3_IR_RIGHT_MASK;
    }

    return (sensor_levels);
}

static void
s3_ir_gpio_input_init (uint32_t pin)
{
    uint32_t const pin_mask = UINT32_C(1) << pin;

    out_w(GPIO_OE_CLR, pin_mask);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT);
}

/* End of file. */
