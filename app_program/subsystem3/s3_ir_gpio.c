/**
 * @file s3_ir_gpio.c
 * @brief Hardware access for the subsystem 3 infrared sensors.
 */
#include "s3_ir_gpio.h"

#include "s3_config.h"
#include "s3_ir.h"

#include <sys/sysdef.h>
#include <tk/syslib.h>

static ID g_s3_ir_adc_device = 0;

static void s3_ir_gpio_input_init(uint32_t pin);
static ER s3_ir_gpio_read_channel(W channel, uint16_t * p_value);

ER
s3_ir_gpio_init (void)
{
    /* T-Kernel device names use UB rather than plain char. */
    ID const device = tk_opn_dev((UB const *)"adca", TD_READ);

    s3_ir_gpio_input_init(S3_PIN_BARCODE);

    if (device < E_OK)
    {
        /* A failed device open returns a negative kernel error in the ID. */
        return (ER)device;
    }

    g_s3_ir_adc_device = device;

    return E_OK;
}

ER
s3_ir_gpio_read_analog (s3_ir_adc_sample_t * p_sample)
{
    ER error;

    if (NULL == p_sample)
    {
        return E_PAR;
    }

    error = s3_ir_gpio_read_channel(S3_ADC_FRONT_LEFT, &p_sample->left);

    if (E_OK == error)
    {
        error = s3_ir_gpio_read_channel(S3_ADC_FRONT_RIGHT,
                                        &p_sample->right);
    }

    return error;
}

bool
s3_ir_gpio_read_barcode (void)
{
    uint32_t const mask = UINT32_C(1) << S3_PIN_BARCODE;

    return (0U != (in_w(GPIO_IN) & mask));
}

static void
s3_ir_gpio_input_init (uint32_t pin)
{
    uint32_t const mask = UINT32_C(1) << pin;

    out_w(GPIO_OE_CLR, mask);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT | GPIO_PUE);
}

static ER
s3_ir_gpio_read_channel (W channel, uint16_t * p_value)
{
    UW raw_value = 0U;
    SZ actual_size = 0;
    ER error;

    if ((g_s3_ir_adc_device <= 0) || (NULL == p_value))
    {
        return E_OBJ;
    }

    error = tk_srea_dev(g_s3_ir_adc_device, channel, &raw_value, 1,
                        &actual_size);

    if ((E_OK == error) && (1 == actual_size)
        && (raw_value <= S3_IR_ADC_MAX))
    {
        /* The range check above makes the narrowing conversion safe. */
        *p_value = (uint16_t)raw_value;
    }
    else if (E_OK == error)
    {
        error = E_IO;
    }
    else
    {
        /* Preserve the device error returned by the synchronous read. */
    }

    return error;
}

/** End of file. */
