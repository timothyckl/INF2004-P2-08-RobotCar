/** @file s3_config.h Subsystem 3 pins, polarity and provisional timing.
 *
 * The ONE place for subsystem 3's hardware assumptions. Pins follow the
 * agreed mapping in AGENTS.md. Every value marked UNMEASURED is a starting
 * point, not a calibration; replace it from testing/subsystem3/TEST_LOG.md.
 */
#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* Agreed pins (AGENTS.md): two front line sensors and the side barcode
 * sensor. The barcode sensor is never a line-following input. GP17 and
 * GP27 (analogue outputs) are not configured. Front-right AO must stay
 * unconnected: Robo Pico Grove 5 shares GP6 with GP26. */
#define S3_PIN_FRONT_LEFT       16U
#define S3_PIN_FRONT_RIGHT      6U
#define S3_PIN_BARCODE          26U

/* GPIO level over black, per sensor, as s3_ir black_levels (bit 0 front
 * left, bit 1 front right, bit 2 barcode). Documented as 1 = black for
 * all three; NOT yet verified on the robot (S3-H01). */
#define S3_IR_BLACK_LEVELS      0x07U

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
