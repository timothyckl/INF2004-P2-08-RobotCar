/** @file s3_ir.h Digital IR calibration and configurable line decisions. */
#ifndef S3_IR_H
#define S3_IR_H

#include <stdbool.h>
#include <stdint.h>

#define S3_IR_COUNT 3U
#define S3_IR_PATTERNS 8U

typedef enum
{
    S3_LINE_UNKNOWN = 0,
    S3_LINE_STRAIGHT,
    S3_LINE_LEFT,
    S3_LINE_RIGHT,
    S3_LINE_LOST,
    S3_LINE_JUNCTION
} s3_line_t;

typedef struct
{
    uint8_t black_levels;
    bool    valid;
} s3_calibration_t;

typedef struct
{
    uint8_t candidate;
    uint8_t repetitions;
    uint8_t stable;
    bool    ready;
} s3_filter_t;

/** Bits 0..2 identify sensors, not physical left/centre/right positions.
 * Calibration observations must be stable readings on known surfaces.
 * Failed calibration invalidates the previous result.
 */
bool s3_ir_calibrate(s3_calibration_t * p_cal, uint8_t white_levels,
                     uint8_t black_levels);
bool s3_ir_normalize(s3_calibration_t const * p_cal, uint8_t levels,
                    uint8_t * p_black_mask);
/** Call at a fixed rate. Required is 1..255 consecutive equal samples.
 * Reset the filter to {0} when changing required or recalibrating.
 * Returns true only when the current candidate has settled.
 */
bool s3_ir_filter(s3_filter_t * p_filter, uint8_t mask, uint8_t required);
/** Caller supplies all eight decisions after checking sensor geometry. */
s3_line_t s3_ir_line(uint8_t mask,
                     s3_line_t const decisions[S3_IR_PATTERNS]);

#endif /* S3_IR_H */

