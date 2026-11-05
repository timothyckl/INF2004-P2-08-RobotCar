/** @file s3_config.h Subsystem 3 pins, polarity and provisional timing.
 *
 * The ONE place for subsystem 3's hardware assumptions. Pins follow the
 * agreed mapping in AGENTS.md. Every value marked UNMEASURED is a starting
 * point, not a calibration; replace it from testing/subsystem3/TEST_LOG.md.
 */
#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* Active ZIP wiring selected on 2026-11-06: front AOUT signals use ADC0
 * and ADC1; the side barcode DOUT is the only digital S3 input. */
#define S3_ADC_FRONT_LEFT       0
#define S3_ADC_FRONT_RIGHT      1
#define S3_PIN_BARCODE          16U
#define S3_BARCODE_BLACK_HIGH   1U

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

/* Sampling period: the kernel tick (10 ms), within the design's 20 ms. */
#define S3_PERIOD_MS            10U

/* Barcode capture, UNMEASURED: longest single bar or space accepted, and
 * the white time that ends a code. Must exceed the widest in-code space
 * at the slowest speed and be shorter than the printed quiet zone. */
#define S3_BAR_RUN_MAX_US       400000U
#define S3_BAR_QUIET_US         250000U

/* Barcode symbology: Code 39 patterns (bit 8 = first element, 1 = wide)
 * for the start/stop '*' and the brief's commands A..D. Assumed; confirm
 * against the printed course barcodes (S3-H04). */
#define S3_BAR_SENTINEL         0x094U
#define S3_BAR_PATTERN_A        0x109U
#define S3_BAR_PATTERN_B        0x049U
#define S3_BAR_PATTERN_C        0x148U
#define S3_BAR_PATTERN_D        0x019U

/* A decoded command waits this long for its junction, then expires.
 * UNMEASURED: depends on barcode-to-junction distance and speed. */
#define S3_NAV_MAX_AGE_MS       5000U

/* Barcode edges buffered between samples (a code is 30 edges). */
#define S3_EDGE_RING_SIZE       64U

#endif /* S3_CONFIG_H */
