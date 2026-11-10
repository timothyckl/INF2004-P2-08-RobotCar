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

/** Configure the infrared inputs required by the selected build profile. */
ER s3_ir_gpio_init(void);

/**
 * Read the two front sensor DOUT pins as a compact raw-level mask.
 * Bit 0 is robot-left GP6 and bit 1 is robot-right GP16, viewed from behind.
 */
uint8_t s3_ir_gpio_read(void);

/** Read front-left ADC0/GP26 and front-right ADC1/GP27. */
ER s3_ir_gpio_read_analog(s3_ir_adc_sample_t * p_sample);

/** Read the raw digital level of the side barcode sensor on GP16. */
bool s3_ir_gpio_read_barcode(void);

#endif /* S3_IR_GPIO_H */

/* End of file. */
