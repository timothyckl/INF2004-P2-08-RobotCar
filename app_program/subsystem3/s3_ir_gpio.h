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
} s3_ir_adc_sample_t;

/** Open the ADC for the two front sensors and the side barcode sensor. */
ER s3_ir_gpio_init(void);

/** Read front-left ADC0/GP26 and front-right ADC1/GP27. */
ER s3_ir_gpio_read_analog(s3_ir_adc_sample_t * p_sample);

/** Read the side barcode sensor's analogue output, ADC2/GP28. */
ER s3_ir_gpio_read_barcode(uint16_t * p_value);

#endif /* S3_IR_GPIO_H */

/* End of file. */
