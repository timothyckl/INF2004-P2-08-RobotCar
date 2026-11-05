/** @file s3_track.c Pure logic; no GPIO, kernel or motor access. */
#include "s3_track.h"
#include <stddef.h>

/* Provisional defaults (UNMEASURED): two 10 ms samples to settle, 1.5 s
 * without black before LOST, 2 s of both black before FAULT, half-scale
 * steering for a one-sided reading. Tune on the track (S3-H02, S3-H03).
 */
#define S3_TRACK_DEF_FILTER     2U
#define S3_TRACK_DEF_LOST_MS    1500U
#define S3_TRACK_DEF_FAULT_MS   2000U
#define S3_TRACK_DEF_STEER      500

static bool s3_track_cfg_valid(s3_track_cfg_t const * p_cfg);
static int16_t s3_track_steer(s3_line_t line, int16_t side_permille);

void s3_track_default_cfg(s3_track_cfg_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        for (uint8_t idx = 0U; idx < S3_IR_PATTERNS; ++idx)
        {
            p_cfg->decisions[idx] = S3_LINE_UNKNOWN;
        }

        p_cfg->decisions[0U] = S3_LINE_STRAIGHT;
        p_cfg->decisions[S3_TRACK_LEFT_BIT] = S3_LINE_LEFT;
        p_cfg->decisions[S3_TRACK_RIGHT_BIT] = S3_LINE_RIGHT;
        p_cfg->decisions[S3_TRACK_FRONT_BITS] = S3_LINE_JUNCTION;
        p_cfg->filter_samples = S3_TRACK_DEF_FILTER;
        p_cfg->lost_ms = S3_TRACK_DEF_LOST_MS;
        p_cfg->fault_ms = S3_TRACK_DEF_FAULT_MS;
        p_cfg->steer_side_permille = S3_TRACK_DEF_STEER;
    }
}

bool s3_track_init(s3_track_t * p_track, s3_track_cfg_t const * p_cfg,
                   uint32_t now_ms)
{
    bool valid = (NULL != p_track) && s3_track_cfg_valid(p_cfg);

    if (valid)
    {
        s3_track_t const zero = {0};

        *p_track = zero;
        p_track->line = S3_LINE_UNKNOWN;
        p_track->last_black_ms = now_ms;
        p_track->last_side = S3_TRACK_SIDE_NONE;
    }

    return valid;
}

void s3_track_update(s3_track_t * p_track, s3_track_cfg_t const * p_cfg,
                     uint8_t black_mask, uint32_t now_ms,
                     s3_track_out_t * p_out)
{
    if ((NULL == p_track) || (NULL == p_cfg) || (NULL == p_out))
    {
        return;
    }

    uint8_t const front = (uint8_t)(black_mask & S3_TRACK_FRONT_BITS);
    s3_line_t const previous = p_track->line;
    uint8_t events = 0U;

    /* Raw history first: loss and fault timing must not wait for the
     * debounce filter.  With straddle geometry, both-white is the expected
     * centred observation and therefore counts as seeing the line. */
    if ((0U != front) || (S3_LINE_STRAIGHT == p_cfg->decisions[0U]))
    {
        p_track->last_black_ms = now_ms;

        if (p_track->lost)
        {
            p_track->lost = false;
            events |= S3_TRACK_EV_FOUND;
        }
    }
    else if ((!p_track->lost)
             && ((now_ms - p_track->last_black_ms) >= p_cfg->lost_ms))
    {
        p_track->lost = true;
        events |= S3_TRACK_EV_LOST;
    }
    else
    {
        /* Still white, either within lost_ms or already lost. */
    }

    if (S3_TRACK_FRONT_BITS == front)
    {
        if (!p_track->both_black)
        {
            p_track->both_black = true;
            p_track->both_since_ms = now_ms;
        }
        else if ((!p_track->fault)
                 && ((now_ms - p_track->both_since_ms) >= p_cfg->fault_ms))
        {
            p_track->fault = true;
            events |= S3_TRACK_EV_FAULT;
        }
        else
        {
            /* Crossing a junction, or fault already latched. */
        }
    }
    else
    {
        p_track->both_black = false;
    }

    if (S3_TRACK_LEFT_BIT == front)
    {
        p_track->last_side = S3_TRACK_SIDE_LEFT;
    }
    else if (S3_TRACK_RIGHT_BIT == front)
    {
        p_track->last_side = S3_TRACK_SIDE_RIGHT;
    }
    else
    {
        /* Neither or both: the side is unchanged. */
    }

    if (s3_ir_filter(&p_track->filter, front, p_cfg->filter_samples))
    {
        p_track->line = s3_ir_line(p_track->filter.stable, p_cfg->decisions);
    }
    else
    {
        p_track->line = S3_LINE_UNKNOWN;
    }

    if ((S3_LINE_JUNCTION == p_track->line)
        && (S3_LINE_JUNCTION != previous))
    {
        events |= S3_TRACK_EV_JUNCTION;
    }

    p_out->line = p_track->line;
    p_out->events = events;
    p_out->last_side = p_track->last_side;
    p_out->lost = p_track->lost;
    p_out->fault = p_track->fault;
    p_out->valid = (S3_LINE_UNKNOWN != p_track->line) && (!p_track->lost)
                   && (!p_track->fault);
    p_out->steer_permille = p_out->valid
                            ? s3_track_steer(p_track->line,
                                             p_cfg->steer_side_permille)
                            : 0;
}

static bool s3_track_cfg_valid(s3_track_cfg_t const * p_cfg)
{
    bool valid = (NULL != p_cfg) && (p_cfg->filter_samples > 0U)
                 && (p_cfg->lost_ms > 0U) && (p_cfg->fault_ms > 0U)
                 && (p_cfg->steer_side_permille > 0)
                 && (p_cfg->steer_side_permille <= S3_TRACK_STEER_MAX);

    /* The barcode sensor (bit 2) must never select a line decision. */
    for (uint8_t idx = 4U; valid && (idx < S3_IR_PATTERNS); ++idx)
    {
        valid = (S3_LINE_UNKNOWN == p_cfg->decisions[idx]);
    }

    return valid;
}

static int16_t s3_track_steer(s3_line_t line, int16_t side_permille)
{
    int16_t steer = 0;

    if (S3_LINE_LEFT == line)
    {
        steer = side_permille;
    }
    else if (S3_LINE_RIGHT == line)
    {
        steer = (int16_t)(-side_permille);
    }
    else
    {
        /* Straight, junction, lost: no correction. */
    }

    return steer;
}

/** End of file. */
