/** @file s3_nav.c Navigation intentions are subject to controller authority. */
#include "s3_nav.h"
#include <stddef.h>

#define S3_NAV_LETTERS 26U

/* Letter A-Z to s3_command_t, one byte each. Written only by
 * s3_nav_set_letter() (WiFi service task), read by s3_nav_command(). */
static volatile uint8_t g_s3_nav_map[S3_NAV_LETTERS] =
{
    [0] = (uint8_t)S3_NAV_LEFT,      /* A */
    [1] = (uint8_t)S3_NAV_RIGHT,     /* B */
    [2] = (uint8_t)S3_NAV_STRAIGHT,  /* C */
    [3] = (uint8_t)S3_NAV_UTURN,     /* D */
    [5] = (uint8_t)S3_NAV_F          /* F: test letter, 0..4 at build time */
};

s3_command_t s3_nav_command(char symbol)
{
    s3_command_t command = S3_NAV_NONE;

    if ((symbol >= 'A') && (symbol <= 'Z'))
    {
        command = (s3_command_t)g_s3_nav_map[(uint8_t)(symbol - 'A')];
    }

    return command;
}

bool s3_nav_set_letter(s3_command_t command, char letter)
{
    bool const valid = (command >= S3_NAV_LEFT) &&
                       (command <= S3_NAV_UTURN) &&
                       (letter >= 'A') && (letter <= 'Z');

    if (valid)
    {
        for (uint8_t index = 0U; index < S3_NAV_LETTERS; index++)
        {
            if ((uint8_t)command == g_s3_nav_map[index])
            {
                g_s3_nav_map[index] = (uint8_t)S3_NAV_NONE;
            }
        }
        g_s3_nav_map[(uint8_t)(letter - 'A')] = (uint8_t)command;
    }

    return valid;
}

bool s3_nav_stage(s3_navigation_t * p_nav, char symbol,
                   uint32_t scan_id, uint32_t now_ms)
{
    s3_command_t command = s3_nav_command(symbol);
    bool accepted = false;

    if ((NULL != p_nav) && (S3_NAV_NONE != command))
    {
        accepted = (S3_NAV_NONE == p_nav->pending) &&
                   ((!p_nav->have_scan) || (scan_id != p_nav->scan_id));
        if (accepted)
        {
            p_nav->pending = command;
            p_nav->scan_id = scan_id;
            p_nav->staged_ms = now_ms;
            p_nav->have_scan = true;
        }
    }

    return accepted;
}

s3_command_t s3_nav_take(s3_navigation_t * p_nav, bool stopped,
                         bool authorised, bool junction, uint32_t now_ms,
                         uint32_t max_age_ms)
{
    s3_command_t command = S3_NAV_NONE;

    if (NULL != p_nav)
    {
        if (stopped || (0U == max_age_ms) ||
            ((now_ms - p_nav->staged_ms) > max_age_ms))
        {
            p_nav->pending = S3_NAV_NONE;
        }
        else if (authorised && junction)
        {
            command = p_nav->pending;
            p_nav->pending = S3_NAV_NONE;
        }
        else
        {
            /* Hold the request until the controller permits its junction. */
        }
    }

    return command;
}

/** End of file. */

