/** @file s3_ir.h Two-sensor analogue IR thresholds and line decisions. */
#ifndef S3_IR_H
#define S3_IR_H

#include <stdbool.h>
#include <stdint.h>

#define S3_IR_COUNT       2u
#define S3_IR_PATTERNS    4u
#define S3_IR_LEFT_MASK   UINT8_C(1)
#define S3_IR_RIGHT_MASK  UINT8_C(2)
#define S3_IR_ADC_MAX     UINT16_C(4095)

typedef enum
{
    S3_LINE_UNKNOWN = 0,
    S3_LINE_STRAIGHT,
    S3_LINE_LEFT,
    S3_LINE_RIGHT,
    S3_LINE_JUNCTION
} s3_line_t;

typedef struct
{
    uint8_t candidate;
    uint8_t repetitions;
    uint8_t stable;
    bool    ready;
} s3_filter_t;

typedef struct
{
    uint16_t initial_white;
    uint16_t minimum_white;
    uint16_t maximum_white;
    uint16_t black_margin;
    uint16_t white_margin;
} s3_ir_adaptive_config_t;

typedef struct
{
    s3_ir_adaptive_config_t config;
    uint16_t white_baseline;
    uint16_t filtered;
    uint16_t threshold;
    uint8_t adaptation_count;
    bool b_black;
    bool b_valid;
} s3_ir_adaptive_t;

/* Black masks: bit 0 is the front-left sensor and bit 1 the front-right
 * sensor. The side barcode sensor is deliberately excluded.
 */

/** Call at a fixed rate. Required is 1..255 consecutive equal samples.
 * Reset the filter to {0} when changing required.
 * Returns true only when the current candidate has settled.
 */
bool s3_ir_filter(s3_filter_t * p_filter, uint8_t mask, uint8_t required);
/** Configure a filtered, hysteretic threshold that follows the white level. */
bool s3_ir_adaptive_init(s3_ir_adaptive_t * p_state,
                         s3_ir_adaptive_config_t const * p_config);
/** Update the filtered ADC value and adaptive threshold; true means black. */
bool s3_ir_adaptive_update(s3_ir_adaptive_t * p_state, uint16_t sample);
/** Return the filtered contrast as 0..100 relative to a measured black level. */
uint8_t s3_ir_adaptive_strength(s3_ir_adaptive_t const * p_state,
                                uint16_t black_reference);
/** Caller supplies all four decisions after checking sensor geometry. */
s3_line_t s3_ir_line(uint8_t mask,
                     s3_line_t const decisions[S3_IR_PATTERNS]);
/** Two-sensor on-line geometry for the supervised line-follow trial:
 * both sensors on white is centred, left black needs a left correction,
 * right black needs a right correction, and both black marks a crossing.
 */
s3_line_t s3_ir_follow_decision(uint8_t black_mask);

#endif /* S3_IR_H */

