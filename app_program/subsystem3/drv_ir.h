/**
 * @file drv_ir.h
 * @brief Driver for the three ST188 IR line-sensor modules (digital DOUT).
 *
 * Only the line sensor task calls this driver (TEAM.md section 3). Each
 * module's LM393 comparator threshold is set with its trimmer at ride
 * height, not in software. See S3-U10 in testing/subsystem3/TEST_LOG.md.
 *
 * WARNING: Power the modules from 3V3. The LM393 output is open collector
 * (LM393 datasheet: output sink and leakage figures), so DOUT high is set
 * by the module's pull-up to its own supply. The Pico W GPIO is fixed at
 * 3.3 V (Pico W datasheet section 3.2), so a 5 V module supply would put
 * 5 V on the pin.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Pins from the agreed mapping in AGENTS.md (IR1, IR2, IR3).
   TODO: Confirm which module is physically left, centre and right. */
#define DRV_IR_PIN_LEFT     (16u)
#define DRV_IR_PIN_CENTRE   (6u)
#define DRV_IR_PIN_RIGHT    (26u)

/* GPIO level read when a sensor is over black.
   TODO: Measure it during calibration (S3-U10). Black reflects little IR,
   so the comparator output is expected to be off and pulled high (1).
   Change this value if the readings are inverted. */
#define DRV_IR_BLACK_LEVEL  (1u)

/**
 * @brief Configure the three sensor pins as inputs with pull-ups.
 * @return E_OK, or the error from gpio_set_pin().
 */
ER drv_ir_init (void);

/**
 * @brief Sample all three sensors at the same instant.
 * @return LINE_TRACK_BIT_* values; a set bit means black.
 */
uint8_t drv_ir_read (void);

#endif /* DRV_IR_H */

/* End of file. */

