/**
 * @file barcode.c
 * @brief Barcode capture (run lengths) and Code 39 decoding.
 *
 * Code 39 is specified in ISO/IEC 16388. Each character is 9 elements
 * (bar, space, ..., bar), and exactly 3 of them are wide. The patterns
 * below are 9 bits wide, with the first element in bit 8 and 1 = wide.
 *
 * TODO: Check the patterns against the printed course barcodes.
 */
#include "barcode.h"

#include <stddef.h>

#define BARCODE_CHAR_ELEMENTS   (9u)
#define BARCODE_CHAR_STRIDE     (10u)       /* A character and its gap. */
#define BARCODE_CHARS           (3u)
#define BARCODE_WIDE_PER_CHAR   (3u)
#define BARCODE_GAPS            (BARCODE_CHARS - 1u)

#define BARCODE_PATTERN_STAR    (0x094u)    /* 010 010 100 */
#define BARCODE_PATTERN_A       (0x109u)    /* 100 001 001 */
#define BARCODE_PATTERN_B       (0x049u)    /* 001 001 001 */
#define BARCODE_PATTERN_C       (0x148u)    /* 101 001 000 */
#define BARCODE_PATTERN_D       (0x019u)    /* 000 011 001 */

/** Store one finished run, or note an overflow. */
static void barcode_store (barcode_t * p_capture, uint16_t width);

/** End the capture and classify what was stored. */
static barcode_result_t barcode_finish (barcode_t * p_capture,
                                        barcode_cmd_t * p_cmd);

/** Decode "*X*" reading the widths forwards or in reverse. */
static bool barcode_decode_direction (uint16_t const p_widths[],
                                      bool b_reverse,
                                      barcode_cmd_t * p_cmd);

/** Find the 3 wide elements of one character and check the contrast. */
static bool barcode_decode_char (uint16_t const p_elements[],
                                 uint32_t * p_pattern,
                                 uint16_t * p_min_wide);

/** Map a character pattern to a navigation command. */
static barcode_cmd_t barcode_lookup (uint32_t pattern);

/** Add two widths, saturating at UINT16_MAX. */
static uint16_t barcode_add_width (uint16_t augend, uint16_t addend);

void
barcode_init (barcode_t * p_capture)
{
    if (NULL != p_capture)
    {
        p_capture->count       = 0u;
        p_capture->run         = 0u;
        p_capture->narrowest   = UINT16_MAX;
        p_capture->b_active    = false;
        p_capture->b_run_black = false;
        p_capture->b_overflow  = false;
    }
}

barcode_result_t
barcode_feed (barcode_t * p_capture, bool b_black, uint16_t width,
              barcode_cmd_t * p_cmd)
{
    barcode_result_t result = BARCODE_NONE;

    if ((NULL == p_capture) || (NULL == p_cmd) || (0u == width))
    {
        /* There is nothing to feed. */
    }
    else if (!p_capture->b_active)
    {
        if (b_black)
        {
            barcode_init(p_capture);
            p_capture->b_active    = true;
            p_capture->b_run_black = true;
            p_capture->run         = width;
        }
    }
    else
    {
        if (b_black == p_capture->b_run_black)
        {
            p_capture->run = barcode_add_width(p_capture->run, width);
        }
        else
        {
            barcode_store(p_capture, p_capture->run);
            p_capture->b_run_black = b_black;
            p_capture->run         = width;
        }

        /* A long enough white run is the quiet zone after the code. The
           casts widen to 32 bits so the product cannot overflow. */
        if ((!p_capture->b_run_black)
            && ((uint32_t) p_capture->run
                >= ((uint32_t) p_capture->narrowest * BARCODE_QUIET_FACTOR)))
        {
            result = barcode_finish(p_capture, p_cmd);
        }
    }

    return (result);
}

bool
barcode_decode (uint16_t const p_widths[], uint8_t count,
                barcode_cmd_t * p_cmd)
{
    bool b_decoded = false;

    if ((NULL != p_widths) && (NULL != p_cmd)
        && (BARCODE_MAX_ELEMENTS == count))
    {
        /* A barcode read backwards is the same elements in reverse. */
        b_decoded = (barcode_decode_direction(p_widths, false, p_cmd))
                    || (barcode_decode_direction(p_widths, true, p_cmd));
    }

    return (b_decoded);
}

static void
barcode_store (barcode_t * p_capture, uint16_t width)
{
    if (p_capture->count < BARCODE_MAX_ELEMENTS)
    {
        p_capture->widths[p_capture->count] = width;
        p_capture->count++;

        if (width < p_capture->narrowest)
        {
            p_capture->narrowest = width;
        }
    }
    else
    {
        /* Keep consuming until the quiet zone and then report one error,
           rather than starting new captures in the middle of the code. */
        p_capture->b_overflow = true;
    }
}

static barcode_result_t
barcode_finish (barcode_t * p_capture, barcode_cmd_t * p_cmd)
{
    barcode_result_t result = BARCODE_ERROR;

    if (p_capture->b_overflow)
    {
        result = BARCODE_ERROR;
    }
    else if (1u == p_capture->count)
    {
        result = BARCODE_SINGLE_BAR;
    }
    else if (barcode_decode(p_capture->widths, p_capture->count, p_cmd))
    {
        result = BARCODE_DECODED;
    }
    else
    {
        result = BARCODE_ERROR;
    }

    p_capture->b_active = false;

    return (result);
}

static bool
barcode_decode_direction (uint16_t const p_widths[], bool b_reverse,
                          barcode_cmd_t * p_cmd)
{
    uint16_t elements[BARCODE_MAX_ELEMENTS];
    uint32_t patterns[BARCODE_CHARS];
    uint16_t min_wide[BARCODE_CHARS];
    bool     b_valid = true;

    for (uint32_t index = 0u; index < BARCODE_MAX_ELEMENTS; index++)
    {
        elements[index] = b_reverse
                          ? p_widths[BARCODE_MAX_ELEMENTS - 1u - index]
                          : p_widths[index];
    }

    for (uint32_t index = 0u; (index < BARCODE_CHARS) && b_valid; index++)
    {
        b_valid = barcode_decode_char(&elements[index * BARCODE_CHAR_STRIDE],
                                      &patterns[index], &min_wide[index]);
    }

    /* Each inter-character gap must be narrow. */
    for (uint32_t index = 0u; (index < BARCODE_GAPS) && b_valid; index++)
    {
        uint16_t const gap = elements[(index * BARCODE_CHAR_STRIDE)
                                      + BARCODE_CHAR_ELEMENTS];

        b_valid = (gap < min_wide[index]);
    }

    if (b_valid)
    {
        barcode_cmd_t const cmd = barcode_lookup(patterns[1u]);

        b_valid = (BARCODE_PATTERN_STAR == patterns[0u])
                  && (BARCODE_PATTERN_STAR == patterns[2u])
                  && (BARCODE_CMD_NONE != cmd);

        if (b_valid)
        {
            *p_cmd = cmd;
        }
    }

    return (b_valid);
}

static bool
barcode_decode_char (uint16_t const p_elements[], uint32_t * p_pattern,
                     uint16_t * p_min_wide)
{
    bool     b_wide[BARCODE_CHAR_ELEMENTS] = {false};
    uint16_t min_wide   = UINT16_MAX;
    uint16_t max_narrow = 0u;
    uint32_t pattern    = 0u;

    /* The three widest elements are the wide ones. */
    for (uint32_t pick = 0u; pick < BARCODE_WIDE_PER_CHAR; pick++)
    {
        uint32_t widest = 0u;
        bool     b_have = false;

        for (uint32_t index = 0u; index < BARCODE_CHAR_ELEMENTS; index++)
        {
            if ((!b_wide[index])
                && ((!b_have) || (p_elements[index] > p_elements[widest])))
            {
                widest = index;
                b_have = true;
            }
        }

        b_wide[widest] = true;
    }

    for (uint32_t index = 0u; index < BARCODE_CHAR_ELEMENTS; index++)
    {
        if (b_wide[index])
        {
            pattern |= (1u << (BARCODE_CHAR_ELEMENTS - 1u - index));

            if (p_elements[index] < min_wide)
            {
                min_wide = p_elements[index];
            }
        }
        else if (p_elements[index] > max_narrow)
        {
            max_narrow = p_elements[index];
        }
        else
        {
            /* This narrow element is not the widest narrow one so far. */
        }
    }

    *p_pattern  = pattern;
    *p_min_wide = min_wide;

    /* Reject a character whose wide and narrow elements cannot be told
       apart. The casts widen to 32 bits so the products cannot overflow. */
    return ((max_narrow > 0u)
            && (((uint32_t) min_wide * BARCODE_WIDE_RATIO_DEN)
                >= ((uint32_t) max_narrow * BARCODE_WIDE_RATIO_NUM)));
}

static barcode_cmd_t
barcode_lookup (uint32_t pattern)
{
    barcode_cmd_t cmd = BARCODE_CMD_NONE;

    switch (pattern)
    {
        case BARCODE_PATTERN_A:
            cmd = BARCODE_CMD_LEFT;
        break;

        case BARCODE_PATTERN_B:
            cmd = BARCODE_CMD_RIGHT;
        break;

        case BARCODE_PATTERN_C:
            cmd = BARCODE_CMD_STRAIGHT;
        break;

        case BARCODE_PATTERN_D:
            cmd = BARCODE_CMD_UTURN;
        break;

        default:
            cmd = BARCODE_CMD_NONE;
        break;
    }

    return (cmd);
}

static uint16_t
barcode_add_width (uint16_t augend, uint16_t addend)
{
    /* The casts widen to 32 bits, so the sum of two 16-bit values cannot
       overflow. */
    uint32_t const sum    = (uint32_t) augend + (uint32_t) addend;
    uint16_t       result = UINT16_MAX;

    if (sum < UINT16_MAX)
    {
        /* The cast is safe: sum is below UINT16_MAX in this branch. */
        result = (uint16_t) sum;
    }

    return (result);
}

/* End of file. */

