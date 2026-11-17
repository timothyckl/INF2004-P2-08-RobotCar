/** @file s3_track.h Line tracking from the two front sensors (pure logic).
 *
 * Builds on s3_ir: the debounce filter and the eight-entry decision table.
 * Only the front-left (bit 0) and front-right (bit 1) sensors take part;
 * the side barcode sensor is never a line input (AGENTS.md), so bit 2 of
 * the mask must be clear and table entries 4..7 stay UNKNOWN.
 *
 * Output per sample: the settled line decision, a steering request for the
 * Vehicle Controller (permille, positive turns left) and events.
 * - LOST: no front sensor has seen black for lost_ms, only for a geometry
 *   whose table does not define both-white as STRAIGHT.  The active straddle
 *   geometry defines both-white as centred, so it cannot infer loss from
 *   that observation alone and does not raise a false LOST on long straights.
 * - FAULT: both sensors black for fault_ms (robot lifted, or a module
 *   unpowered: its pulled-up output reads black). Latched until init.
 * - JUNCTION: the settled decision became JUNCTION.
 * All thresholds are provisional until measured on the track.
 */
#ifndef S3_TRACK_H
#define S3_TRACK_H

#include <stdbool.h>
#include <stdint.h>

#include "s3_ir.h"

#define S3_TRACK_LEFT_BIT       0x01U   /* front-left, ADC0/GP26 */
#define S3_TRACK_RIGHT_BIT      0x02U   /* front-right, ADC1/GP27 */
#define S3_TRACK_FRONT_BITS     0x03U

#define S3_TRACK_EV_LOST        0x01U
#define S3_TRACK_EV_FOUND       0x02U
#define S3_TRACK_EV_JUNCTION    0x04U
#define S3_TRACK_EV_FAULT       0x08U

#define S3_TRACK_SIDE_NONE      0U
#define S3_TRACK_SIDE_LEFT      1U
#define S3_TRACK_SIDE_RIGHT     2U

#define S3_TRACK_STEER_MAX      1000

typedef struct
{
    s3_line_t decisions[S3_IR_PATTERNS];  /* indexed by black mask */
    uint8_t   filter_samples;             /* 1..255 equal samples */
    uint32_t  lost_ms;                    /* > 0 */
    uint32_t  fault_ms;                   /* > 0 */
    int16_t   steer_side_permille;        /* 1..S3_TRACK_STEER_MAX */
} s3_track_cfg_t;

typedef struct
{
    s3_filter_t filter;
    s3_line_t   line;
    uint32_t    last_black_ms;
    uint32_t    both_since_ms;
    bool        both_black;
    uint8_t     last_side;
    bool        lost;
    bool        fault;
} s3_track_t;

typedef struct
{
    s3_line_t line;             /* UNKNOWN until the filter settles */
    int16_t   steer_permille;   /* 0 unless valid */
    uint8_t   events;           /* S3_TRACK_EV_* raised by this sample */
    uint8_t   last_side;        /* side the line was last seen alone */
    bool      valid;            /* settled, not lost, not faulted */
    bool      lost;
    bool      fault;
} s3_track_out_t;

/** Fill the default straddle table: both white = straight, one sensor
 * black = line on that side, both black = junction. Geometry must be
 * confirmed on the robot before the table is relied on.
 */
void s3_track_default_cfg(s3_track_cfg_t * p_cfg);
/** Validate the configuration and start with no history at now_ms. */
bool s3_track_init(s3_track_t * p_track, s3_track_cfg_t const * p_cfg,
                   uint32_t now_ms);
/** Process one normalised sample (1 = black). Call at a fixed rate. */
void s3_track_update(s3_track_t * p_track, s3_track_cfg_t const * p_cfg,
                     uint8_t black_mask, uint32_t now_ms,
                     s3_track_out_t * p_out);

#endif /* S3_TRACK_H */
