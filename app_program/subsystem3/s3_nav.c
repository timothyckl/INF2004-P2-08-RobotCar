/** @file s3_nav.c Navigation intentions are subject to controller authority. */
#include "s3_nav.h"
#include <stddef.h>

s3_command_t s3_nav_command(char symbol)
{
    s3_command_t command = S3_NAV_NONE;

    switch (symbol)
    {
        case 'A':
            command = S3_NAV_LEFT;
        break;
        case 'B':
            command = S3_NAV_RIGHT;
        break;
        case 'C':
            command = S3_NAV_STRAIGHT;
        break;
        case 'D':
            command = S3_NAV_UTURN;
        break;
        default:
            command = S3_NAV_NONE;
        break;
    }

    return command;
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

