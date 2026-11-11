/**
 * @file scan_look.c
 * @brief One stationary sideways look during a bypass (see scan_look.h).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../include/scan_look.h"

#include <stddef.h>

/** Clear one call's results. */
static void scan_look_clear (scan_look_output_t * p_out);

/** End the look with a fault and turn the servo forward. */
static void scan_look_fault (scan_look_t * p_look,
                             scan_look_output_t * p_out);

/** Median of the three samples. */
static uint16_t scan_look_median (uint16_t const * p_samples);

bool
scan_look_init (scan_look_t * p_look, scan_look_cfg_t const * p_cfg)
{
    bool const b_valid = (NULL != p_look) && (NULL != p_cfg)
                         && (0u < p_cfg->settling_us)
                         && (0u < p_cfg->still_limit_us)
                         && (0u < p_cfg->limit_us);

    if (b_valid)
    {
        scan_look_t const idle = {0};

        *p_look = idle;
        p_look->cfg = *p_cfg;
        p_look->state = SCAN_LOOK_IDLE;
    }

    return (b_valid);
}

bool
scan_look_active (scan_look_t const * p_look)
{
    return ((NULL != p_look) && (SCAN_LOOK_IDLE != p_look->state));
}

bool
scan_look_capturing (scan_look_t const * p_look)
{
    return ((NULL != p_look)
            && ((SCAN_LOOK_SAMPLE == p_look->state)
                || (SCAN_LOOK_FRESH == p_look->state)));
}

void
scan_look_start (scan_look_t * p_look, uint32_t id, int16_t angle_deg,
                 uint32_t now_us, scan_look_output_t * p_out)
{
    if ((NULL == p_look) || (NULL == p_out))
    {
        return;
    }

    scan_look_clear(p_out);

    if (SCAN_LOOK_IDLE == p_look->state)
    {
        p_look->id = id;
        p_look->angle_deg = angle_deg;
        p_look->started_us = now_us;
        p_look->since_us = now_us;
        p_look->count = 0u;
        p_look->invalid = 0u;
        p_look->state = SCAN_LOOK_STILL;

        if ((angle_deg > SCAN_LOOK_ANGLE_MAX)
            || (angle_deg < -SCAN_LOOK_ANGLE_MAX))
        {
            scan_look_fault(p_look, p_out);
        }
    }
}

void
scan_look_tick (scan_look_t * p_look, uint32_t now_us, bool b_still,
                bool b_authority, scan_look_output_t * p_out)
{
    if ((NULL == p_look) || (NULL == p_out))
    {
        return;
    }

    scan_look_clear(p_out);

    if (SCAN_LOOK_IDLE == p_look->state)
    {
        /* Nothing to do. */
    }
    else if ((!b_authority)
             || ((now_us - p_look->started_us) > p_look->cfg.limit_us))
    {
        scan_look_fault(p_look, p_out);
    }
    else if (SCAN_LOOK_STILL == p_look->state)
    {
        if (b_still)
        {
            p_look->state = SCAN_LOOK_OUT;
            p_look->since_us = now_us;
            p_out->b_servo = true;
            p_out->servo_deg = p_look->angle_deg;
        }
        else if ((now_us - p_look->started_us) > p_look->cfg.still_limit_us)
        {
            scan_look_fault(p_look, p_out);
        }
        else
        {
            /* Still stopping. */
        }
    }
    else if (!b_still)
    {
        scan_look_fault(p_look, p_out);     /* the vehicle moved */
    }
    else if ((SCAN_LOOK_OUT == p_look->state)
             && ((now_us - p_look->since_us) >= p_look->cfg.settling_us))
    {
        p_look->state = SCAN_LOOK_SAMPLE;
    }
    else if ((SCAN_LOOK_BACK == p_look->state)
             && ((now_us - p_look->since_us) >= p_look->cfg.settling_us))
    {
        p_look->state = SCAN_LOOK_FRESH;
        p_look->invalid = 0u;
        p_out->b_resume = true;
    }
    else
    {
        /* Settling, or waiting for a reading. */
    }
}

void
scan_look_sample (scan_look_t * p_look, bool b_valid, uint16_t distance_mm,
                  uint32_t now_us, scan_look_output_t * p_out)
{
    if ((NULL == p_look) || (NULL == p_out))
    {
        return;
    }

    scan_look_clear(p_out);

    if (!scan_look_capturing(p_look))
    {
        /* Readings while the servo moves or waits do not count. */
    }
    else if (!b_valid)
    {
        p_look->invalid++;

        if (p_look->invalid >= SCAN_LOOK_SAMPLES)
        {
            scan_look_fault(p_look, p_out);
        }
    }
    else if (SCAN_LOOK_SAMPLE == p_look->state)
    {
        p_look->invalid = 0u;
        p_look->samples[p_look->count] = distance_mm;
        p_look->count++;

        if (p_look->count >= SCAN_LOOK_SAMPLES)
        {
            p_look->median_mm = scan_look_median(p_look->samples);
            p_look->state = SCAN_LOOK_BACK;
            p_look->since_us = now_us;
            p_out->b_servo = true;
            p_out->servo_deg = 0;
        }
    }
    else
    {
        /* The fresh forward reading: the look is complete. */
        p_look->state = SCAN_LOOK_IDLE;
        p_out->result = SCAN_LOOK_DONE;
        p_out->distance_mm = p_look->median_mm;
    }
}

static void
scan_look_clear (scan_look_output_t * p_out)
{
    p_out->b_servo = false;
    p_out->servo_deg = 0;
    p_out->b_resume = false;
    p_out->result = SCAN_LOOK_NONE;
    p_out->distance_mm = 0u;
}

static void
scan_look_fault (scan_look_t * p_look, scan_look_output_t * p_out)
{
    p_look->state = SCAN_LOOK_IDLE;
    p_out->b_servo = true;
    p_out->servo_deg = 0;
    p_out->b_resume = true;
    p_out->result = SCAN_LOOK_FAULT;
    p_out->distance_mm = 0u;
}

static uint16_t
scan_look_median (uint16_t const * p_samples)
{
    uint16_t const a = p_samples[0];
    uint16_t const b = p_samples[1];
    uint16_t const c = p_samples[2];
    uint16_t median = c;

    if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
    {
        median = b;
    }
    else if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
    {
        median = a;
    }
    else
    {
        /* c lies between a and b. */
    }

    return (median);
}

/* End of file. */
