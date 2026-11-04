/**
 * @file s3_ir_gpio.h
 * @brief Digital GPIO access for the two front IR line sensors.
 */
#ifndef S3_IR_GPIO_H
#define S3_IR_GPIO_H

#include <stdint.h>

/** Configure GP6 and GP16 as digital inputs without internal pulls. */
void s3_ir_gpio_init(void);

/**
 * Read the two front sensor DOUT pins as a compact raw-level mask.
 * Bit 0 is robot-left GP6 and bit 1 is robot-right GP16, viewed from behind.
 */
uint8_t s3_ir_gpio_read(void);

#endif /* S3_IR_GPIO_H */

/* End of file. */
