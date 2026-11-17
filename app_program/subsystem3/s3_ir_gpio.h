/**
 * @file s3_ir_gpio.h
 * @brief Hardware access for the subsystem 3 infrared sensors.
 */
#ifndef S3_IR_GPIO_H
#define S3_IR_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    uint16_t left;
    uint16_t right;
    uint16_t barcode;
} s3_ir_adc_sample_t;

/** Open the ADC device for all three infrared AOUT signals. */
ER s3_ir_gpio_init(void);

/** Read front-left ADC0, front-right ADC1 and barcode ADC2. */
ER s3_ir_gpio_read_analog(s3_ir_adc_sample_t * p_sample);

#endif /* S3_IR_GPIO_H */

/** End of file. */
