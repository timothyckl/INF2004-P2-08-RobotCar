/**
 * @file line_track.c
 * @brief Line position, line loss and sensor-fault logic.
 *
 * Losing the line is not a sensor fault (Design Review section 8.1). An
 * all-white sample counts only towards LOST. Only the physically
 * impossible pattern, left and right black with the centre white, counts
 * towards FAULT.
 *
 * NOTE: A sensor stuck at one level cannot be detected from the pattern
 * alone.
 */
#include "line_track.h"

#include <stddef.h>

typedef enum
{
    LINE_TRACK_PATTERN_LINE,
    LINE_TRACK_PATTERN_NONE,
    LINE_TRACK_PATTERN_INVALID
} line_track_pattern_t;

/** Classify a sample and, for a line pattern, give its position. */
static line_track_pattern_t line_track_classify (uint8_t bits,
                                                 int8_t * p_position);

/** Handle a sample that shows a line. */
static uint8_t line_track_on_line (line_track_t * p_track, int8_t position);

/** Handle a sample with no line under any sensor. */
static uint8_t line_track_on_empty (line_track_t * p_track);

/** Handle a physically impossible sample. */
static uint8_t line_track_on_invalid (line_track_t * p_track);

/** Add one to a counter without wrapping past UINT8_MAX. */
static uint8_t line_track_increment (uint8_t count);

void
line_track_init (line_track_t * p_track)
{
    if (NULL != p_track)
    {
        p_track->mode          = LINE_TRACK_FOLLOWING;
        p_track->position      = 0;
        p_track->b_valid       = false;
        p_track->lost_count    = 0u;
        p_track->found_count   = 0u;
        p_track->invalid_count = 0u;
    }
}

uint8_t
line_track_update (line_track_t * p_track, uint8_t bits)
{
    uint8_t events = 0u;

    if (NULL == p_track)
    {
        /* There is nothing to update. */
    }
    else if (LINE_TRACK_FAULT == p_track->mode)
    {
        p_track->b_valid = false;
    }
    else
    {
        int8_t                     position = 0;
        line_track_pattern_t const pattern  =
            line_track_classify(bits, &position);

        switch (pattern)
        {
            case LINE_TRACK_PATTERN_LINE:
                events = line_track_on_line(p_track, position);
            break;

            case LINE_TRACK_PATTERN_NONE:
                events = line_track_on_empty(p_track);
            break;

            default:
                events = line_track_on_invalid(p_track);
            break;
        }
    }

    return (events);
}

static line_track_pattern_t
line_track_classify (uint8_t bits, int8_t * p_position)
{
    line_track_pattern_t pattern = LINE_TRACK_PATTERN_LINE;

    switch (bits & LINE_TRACK_BITS_ALL)
    {
        case LINE_TRACK_BIT_CENTRE:
            *p_position = 0;
        break;

        case (LINE_TRACK_BIT_LEFT | LINE_TRACK_BIT_CENTRE):
            *p_position = -1;
        break;

        case LINE_TRACK_BIT_LEFT:
            *p_position = LINE_TRACK_POS_MIN;
        break;

        case (LINE_TRACK_BIT_RIGHT | LINE_TRACK_BIT_CENTRE):
            *p_position = 1;
        break;

        case LINE_TRACK_BIT_RIGHT:
            *p_position = LINE_TRACK_POS_MAX;
        break;

        case LINE_TRACK_BITS_ALL:
            /* Full-width black is a crossing line or a barcode bar. */
            *p_position = 0;
        break;

        case 0u:
            pattern = LINE_TRACK_PATTERN_NONE;
        break;

        default:
            /* Left and right are black while the centre is white. */
            pattern = LINE_TRACK_PATTERN_INVALID;
        break;
    }

    return (pattern);
}

static uint8_t
line_track_on_line (line_track_t * p_track, int8_t position)
{
    uint8_t events = 0u;

    p_track->invalid_count = 0u;
    p_track->lost_count    = 0u;
    p_track->position      = position;
    p_track->b_valid       = true;

    if (LINE_TRACK_LOST == p_track->mode)
    {
        p_track->found_count = line_track_increment(p_track->found_count);

        if (p_track->found_count >= LINE_TRACK_FOUND_SAMPLES)
        {
            p_track->mode        = LINE_TRACK_FOLLOWING;
            p_track->found_count = 0u;
            events               = LINE_TRACK_EVENT_FOUND;
        }
    }

    return (events);
}

static uint8_t
line_track_on_empty (line_track_t * p_track)
{
    uint8_t events = 0u;

    /* Keep the last position so a search knows which side the line left
       from. */
    p_track->invalid_count = 0u;
    p_track->found_count   = 0u;
    p_track->b_valid       = false;

    if (LINE_TRACK_FOLLOWING == p_track->mode)
    {
        p_track->lost_count = line_track_increment(p_track->lost_count);

        if (p_track->lost_count >= LINE_TRACK_LOST_SAMPLES)
        {
            p_track->mode       = LINE_TRACK_LOST;
            p_track->lost_count = 0u;
            events              = LINE_TRACK_EVENT_LOST;
        }
    }

    return (events);
}

static uint8_t
line_track_on_invalid (line_track_t * p_track)
{
    uint8_t events = 0u;

    p_track->b_valid       = false;
    p_track->invalid_count = line_track_increment(p_track->invalid_count);

    if (p_track->invalid_count >= LINE_TRACK_FAULT_SAMPLES)
    {
        p_track->mode = LINE_TRACK_FAULT;
        events        = LINE_TRACK_EVENT_FAULT;
    }

    return (events);
}

static uint8_t
line_track_increment (uint8_t count)
{
    uint8_t result = count;

    if (count < UINT8_MAX)
    {
        /* The cast is safe: count is below UINT8_MAX in this branch. */
        result = (uint8_t) (count + 1u);
    }

    return (result);
}

/* End of file. */

