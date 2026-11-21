/**
 * @file s3_ir_gpio.c
 * @brief Hardware access for the subsystem 3 infrared sensors.
 */
#include "s3_ir_gpio.h"

#include "s3_ir.h"

/* AGENTS.md, Agreed pin mapping: all three sensors are analogue only. */
#define S3_IR_ADC_LEFT_CH      0    /* ADC0, GP26, front-left AO */
#define S3_IR_ADC_RIGHT_CH     1    /* ADC1, GP27, front-right AO */
#define S3_IR_ADC_BARCODE_CH   2    /* ADC2, GP28, side barcode AO */

static ID g_s3_ir_adc_device = 0;

static ER s3_ir_gpio_read_channel(W channel, uint16_t * p_value);

ER
s3_ir_gpio_init (void)
{
    ID const device = tk_opn_dev((UB const *)"adca", TD_READ);

    if (device < E_OK)
    {
        return ((ER)device);
    }
    g_s3_ir_adc_device = device;

    return (E_OK);
}

ER
s3_ir_gpio_read_analog (s3_ir_adc_sample_t * p_sample)
{
    ER error;

    if (NULL == p_sample)
    {
        return (E_PAR);
    }

    error = s3_ir_gpio_read_channel(S3_IR_ADC_LEFT_CH, &p_sample->left);
    if (E_OK == error)
    {
        error = s3_ir_gpio_read_channel(S3_IR_ADC_RIGHT_CH,
                                        &p_sample->right);
    }

    return (error);
}

ER
s3_ir_gpio_read_barcode (uint16_t * p_value)
{
    return (s3_ir_gpio_read_channel(S3_IR_ADC_BARCODE_CH, p_value));
}

static ER
s3_ir_gpio_read_channel (W channel, uint16_t * p_value)
{
    UW raw_value = 0u;
    SZ actual_size = 0;
    ER error;

    if ((g_s3_ir_adc_device <= 0) || (NULL == p_value))
    {
        return (E_OBJ);
    }

    error = tk_srea_dev(g_s3_ir_adc_device, channel, &raw_value, 1,
                        &actual_size);
    if ((E_OK == error) && (1 == actual_size)
        && (raw_value <= S3_IR_ADC_MAX))
    {
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

    return (error);
}

/* End of file. */
