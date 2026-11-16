/** @file s3_config.h Subsystem 3 pins, polarity and provisional timing.
 *
 * The ONE place for subsystem 3's hardware assumptions. Pins follow the
 * agreed mapping in AGENTS.md. Every value marked UNMEASURED is a starting
 * point, not a calibration; replace it from testing/subsystem3/TEST_LOG.md.
 */
#ifndef S3_CONFIG_H
#define S3_CONFIG_H

#include "s3_code39.h"

/* Active wiring selected on 2026-11-11: all three IR sensors use AOUT.
 * The front sensors use ADC0/ADC1 and the side barcode uses ADC2. */
#define S3_ADC_FRONT_LEFT       0
#define S3_ADC_FRONT_RIGHT      1
#define S3_ADC_BARCODE          2
#define S3_PIN_BARCODE_DRIVE_BUTTON 20U

/* Bench-derived starting values from the ZIP. They are provisional until
 * the assembled robot is re-measured at its final sensor height. */
#define S3_LEFT_INITIAL_WHITE       136U
#define S3_LEFT_MINIMUM_WHITE        96U
#define S3_LEFT_MAXIMUM_WHITE       168U
#define S3_LEFT_BLACK_MARGIN         32U
#define S3_LEFT_WHITE_MARGIN         20U
#define S3_LEFT_BLACK_REFERENCE    1500U
#define S3_RIGHT_INITIAL_WHITE      180U
#define S3_RIGHT_MINIMUM_WHITE      100U
#define S3_RIGHT_MAXIMUM_WHITE      300U
#define S3_RIGHT_BLACK_MARGIN       500U
#define S3_RIGHT_WHITE_MARGIN       350U
#define S3_RIGHT_BLACK_REFERENCE   3300U

/* Barcode AOUT classification remains PROVISIONAL. At the original mounted
 * height on 2026-11-11, 50 samples gave white 154..184, solid black
 * 1679..1709 and faded black 1637..1672. The 554..584 black threshold and
 * 454..484 release level repeatedly decoded the biggest and three-quarter
 * Z prints in both directions. A lower threshold trial for the smallest print
 * was not reliable and is deliberately not retained. */
#define S3_BARCODE_INITIAL_WHITE     169U
#define S3_BARCODE_MINIMUM_WHITE     154U
#define S3_BARCODE_MAXIMUM_WHITE     184U
#define S3_BARCODE_BLACK_MARGIN      400U
#define S3_BARCODE_WHITE_MARGIN      300U

/* Sampling period: the kernel tick (10 ms), within the design's 20 ms. */
#define S3_PERIOD_MS            10U

/* Barcode capture, PROVISIONAL: a controlled 40 mm/s run on 2026-11-11
 * observed 120..220 ms narrow runs and 420..540 ms wide runs. These bounds
 * admit that evidence with margin. S3-H04 must still confirm the printed
 * quiet zone, every symbol, lighting variation and repeated passes. */
#define S3_BAR_RUN_MAX_US       800000U
#define S3_BAR_QUIET_US         700000U

/* Course barcodes use the standard Code 39 framing and A-to-Z table. */
#define S3_BAR_SENTINEL         S3_CODE39_SENTINEL

/* A decoded command waits this long for its junction, then expires.
 * UNMEASURED: depends on barcode-to-junction distance and speed. */
#define S3_NAV_MAX_AGE_MS       5000U

#endif /* S3_CONFIG_H */
