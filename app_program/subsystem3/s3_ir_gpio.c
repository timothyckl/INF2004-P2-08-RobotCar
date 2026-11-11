/**
 * @file s3_ir_gpio.c
 * @brief Hardware access for the subsystem 3 infrared sensors.
 */
#include "s3_ir_gpio.h"

#include "s3_ir.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef S3_HARDWARE_TEST
#define S3_HARDWARE_TEST 0
#endif

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

#define S3_IR_ANALOG_PROFILE  (S3_HARDWARE_TEST || S3_LINE_FOLLOW_TEST)

#define S3_IR_GPIO_LEFT_PIN   6u
#define S3_IR_GPIO_RIGHT_PIN  16u
#define S3_IR_BARCODE_PIN     16u
#define S3_IR_ADC_LEFT_CH      0
#define S3_IR_ADC_RIGHT_CH     1

#if S3_IR_ANALOG_PROFILE
static ID g_s3_ir_adc_device = 0;
#endif

static void s3_ir_gpio_input_init(uint32_t pin);
#if S3_IR_ANALOG_PROFILE
static ER s3_ir_gpio_read_channel(W channel, uint16_t * p_value);
#endif

ER
s3_ir_gpio_init (void)
{
#if S3_IR_ANALOG_PROFILE
    ID const device = tk_opn_dev((UB const *)"adca", TD_READ);

    s3_ir_gpio_input_init(S3_IR_BARCODE_PIN);
    if (device < E_OK)
    {
        return ((ER)device);
    }
    g_s3_ir_adc_device = device;
#else
    s3_ir_gpio_input_init(S3_IR_GPIO_LEFT_PIN);
    s3_ir_gpio_input_init(S3_IR_GPIO_RIGHT_PIN);
#endif

    return (E_OK);
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

ER
s3_ir_gpio_read_analog (s3_ir_adc_sample_t * p_sample)
{
#if S3_IR_ANALOG_PROFILE
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
#else
    (void)p_sample;
    return (E_NOSPT);
#endif
}

bool
s3_ir_gpio_read_barcode (void)
{
    uint32_t const barcode_mask = UINT32_C(1) << S3_IR_BARCODE_PIN;

    return (0u != (in_w(GPIO_IN) & barcode_mask));
}

static void
s3_ir_gpio_input_init (uint32_t pin)
{
    uint32_t const pin_mask = UINT32_C(1) << pin;

    out_w(GPIO_OE_CLR, pin_mask);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT);
}

#if S3_IR_ANALOG_PROFILE
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
#endif

/* End of file. */
