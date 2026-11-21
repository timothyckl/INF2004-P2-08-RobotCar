/**
 * @file nav_control.c
 * @brief Pure barcode-command policy: act at the next junction.
 */
#include "nav_control.h"

/* No <stddef.h>: tk/syslib.h (through motion_api.h) defines size_t and
   NULL itself, and the two definitions conflict on the target. */
#include "motion_api.h"

#define NAV_QUARTER_TURN_MDEG   90000
#define NAV_HALF_TURN_MDEG      180000
#define NAV_FULL_TURN_MDEG      360000

static void    nav_fail(nav_control_t * p_nav, nav_control_output_t * p_out);
static void    nav_spin_start(nav_control_t * p_nav,
                              nav_control_input_t const * p_in,
                              nav_control_output_t * p_out);
static void    nav_spin_step(nav_control_t * p_nav,
                             nav_control_input_t const * p_in,
                             nav_control_output_t * p_out);
static int32_t nav_wrap_mdeg(int32_t mdeg);

bool
nav_control_init (nav_control_t * p_nav, nav_control_cfg_t const * p_cfg)
{
    if ((NULL == p_nav) || (NULL == p_cfg))
    {
        return (false);
    }

    p_nav->cfg = *p_cfg;
    p_nav->b_valid = (p_cfg->sensor_ahead_mm > 0u)
                     && (p_cfg->pending_max_ms > 0u)
                     && (p_cfg->phase_limit_ms > 0u)
                     && (p_cfg->accept_mdeg > 0)
                     && (p_cfg->accept_mdeg < NAV_QUARTER_TURN_MDEG)
                     && (p_cfg->over_mdeg > 0)
                     && (p_cfg->over_mdeg < NAV_QUARTER_TURN_MDEG);
    nav_control_abort(p_nav);

    return (p_nav->b_valid);
}

bool
nav_control_stage (nav_control_t * p_nav, uint8_t command, uint32_t now_ms)
{
    bool const b_ok = (NULL != p_nav) && p_nav->b_valid
                      && (NAV_CONTROL_CMD_NONE != command)
                      && (command <= NAV_CONTROL_CMD_UTURN)
                      && (!nav_control_manoeuvring(p_nav));

    if (b_ok)
    {
        p_nav->command = command;
        p_nav->staged_ms = now_ms;
        p_nav->state = NAV_CONTROL_PENDING;
    }

    return (b_ok);
}

void
nav_control_step (nav_control_t * p_nav, nav_control_input_t const * p_in,
                  nav_control_output_t * p_out)
{
    if ((NULL == p_out) || (NULL == p_nav) || (NULL == p_in))
    {
        return;
    }

    p_out->action = NAV_CONTROL_ACT_NONE;
    p_out->value = 0;
    p_out->outcome = NAV_CONTROL_OUT_NONE;

    if (!p_nav->b_valid)
    {
        return;
    }

    switch (p_nav->state)
    {
        case NAV_CONTROL_PENDING:
            if ((p_in->now_ms - p_nav->staged_ms) > p_nav->cfg.pending_max_ms)
            {
                nav_control_abort(p_nav);
                p_out->outcome = NAV_CONTROL_OUT_EXPIRED;
            }
            else if (p_in->b_junction
                     && (NAV_CONTROL_CMD_STRAIGHT == p_nav->command))
            {
                nav_control_abort(p_nav);
                p_out->outcome = NAV_CONTROL_OUT_STRAIGHT;
            }
            else if (p_in->b_junction)
            {
                p_nav->state = NAV_CONTROL_HALT;
                p_nav->phase_ms = p_in->now_ms;
                p_out->action = NAV_CONTROL_ACT_STOP;
                p_out->outcome = NAV_CONTROL_OUT_STARTED;
            }
            else
            {
                /* Keep following until the junction. */
            }
        break;

        case NAV_CONTROL_HALT:
            if (p_in->b_still)
            {
                /* The axle is sensor_ahead_mm behind where both sensors
                   went black; subtract what was rolled since then. */
                int32_t const step = (int32_t)p_nav->cfg.sensor_ahead_mm
                                     - (p_in->travel_mm
                                        - p_in->junction_travel_mm);

                if (step > 0)
                {
                    p_nav->state = NAV_CONTROL_ADVANCE;
                    p_nav->phase_ms = p_in->now_ms;
                    p_nav->b_waiting = false;
                    p_out->action = NAV_CONTROL_ACT_MOVE;
                    p_out->value = step;
                }
                else
                {
                    nav_spin_start(p_nav, p_in, p_out);
                }
            }
            else if ((p_in->now_ms - p_nav->phase_ms)
                     > p_nav->cfg.phase_limit_ms)
            {
                nav_fail(p_nav, p_out);
            }
            else
            {
                /* Braking. */
            }
        break;

        case NAV_CONTROL_ADVANCE:
        {
            bool const b_ours = p_nav->b_waiting && p_in->b_status_ok
                                && (p_in->last_id == p_nav->wait_id);

            if (b_ours && (MOTION_EVT_DONE == p_in->last_result))
            {
                nav_spin_start(p_nav, p_in, p_out);
            }
            else if ((b_ours && ((MOTION_EVT_REJECTED == p_in->last_result)
                                 || (MOTION_EVT_FAULT == p_in->last_result)
                                 || (MOTION_EVT_STOPPED
                                     == p_in->last_result)))
                     || ((p_in->now_ms - p_nav->phase_ms)
                         > p_nav->cfg.phase_limit_ms))
            {
                nav_fail(p_nav, p_out);
            }
            else
            {
                /* Stepping. */
            }
        }
        break;

        case NAV_CONTROL_SPIN:
            nav_spin_step(p_nav, p_in, p_out);
        break;

        case NAV_CONTROL_SETTLE:
            if (p_in->b_still)
            {
                nav_control_abort(p_nav);
                p_out->outcome = NAV_CONTROL_OUT_DONE;
            }
            else if ((p_in->now_ms - p_nav->phase_ms)
                     > p_nav->cfg.phase_limit_ms)
            {
                nav_fail(p_nav, p_out);
            }
            else
            {
                /* Braking on the new line. */
            }
        break;

        default:
            /* Idle: nothing held. */
        break;
    }
}

void
nav_control_sent (nav_control_t * p_nav, uint16_t cmd_id)
{
    if ((NULL != p_nav) && (NAV_CONTROL_ADVANCE == p_nav->state))
    {
        p_nav->wait_id = cmd_id;
        p_nav->b_waiting = true;
    }
}

void
nav_control_abort (nav_control_t * p_nav)
{
    if (NULL != p_nav)
    {
        p_nav->state = NAV_CONTROL_IDLE;
        p_nav->command = NAV_CONTROL_CMD_NONE;
        p_nav->b_waiting = false;
        p_nav->turned_mdeg = 0;
    }
}

bool
nav_control_manoeuvring (nav_control_t const * p_nav)
{
    return ((NULL != p_nav)
            && (NAV_CONTROL_IDLE != p_nav->state)
            && (NAV_CONTROL_PENDING != p_nav->state));
}

static void
nav_fail (nav_control_t * p_nav, nav_control_output_t * p_out)
{
    nav_control_abort(p_nav);
    p_out->action = NAV_CONTROL_ACT_NONE;
    p_out->outcome = NAV_CONTROL_OUT_FAILED;
}

static void
nav_spin_start (nav_control_t * p_nav, nav_control_input_t const * p_in,
                nav_control_output_t * p_out)
{
    p_nav->state = NAV_CONTROL_SPIN;
    p_nav->phase_ms = p_in->now_ms;
    p_nav->spin_last_mdeg = p_in->heading_mdeg;
    p_nav->turned_mdeg = 0;
    p_out->action = NAV_CONTROL_ACT_SPIN;
    p_out->value = (NAV_CONTROL_CMD_RIGHT == p_nav->command) ? -1 : 1;
}

/* Accumulate the turn each period, so a U-turn can pass 180 degrees. */
static void
nav_spin_step (nav_control_t * p_nav, nav_control_input_t const * p_in,
               nav_control_output_t * p_out)
{
    int32_t const want = (NAV_CONTROL_CMD_UTURN == p_nav->command)
                         ? NAV_HALF_TURN_MDEG : NAV_QUARTER_TURN_MDEG;
    int32_t const delta = nav_wrap_mdeg(p_in->heading_mdeg
                                        - p_nav->spin_last_mdeg);

    p_nav->spin_last_mdeg = p_in->heading_mdeg;
    p_nav->turned_mdeg += (NAV_CONTROL_CMD_RIGHT == p_nav->command)
                          ? -delta : delta;

    if (p_in->b_on_line
        && (p_nav->turned_mdeg >= (want - p_nav->cfg.accept_mdeg)))
    {
        p_nav->state = NAV_CONTROL_SETTLE;
        p_nav->phase_ms = p_in->now_ms;
        p_out->action = NAV_CONTROL_ACT_STOP;
    }
    else if ((p_nav->turned_mdeg > (want + p_nav->cfg.over_mdeg))
             || ((p_in->now_ms - p_nav->phase_ms)
                 > p_nav->cfg.phase_limit_ms))
    {
        nav_fail(p_nav, p_out);
    }
    else
    {
        /* Still turning. */
    }
}

/* Both headings lie in (-180000, 180000], so the difference fits int32_t;
   fold it back into that range. */
static int32_t
nav_wrap_mdeg (int32_t mdeg)
{
    int32_t wrapped = mdeg;

    if (wrapped > NAV_HALF_TURN_MDEG)
    {
        wrapped -= NAV_FULL_TURN_MDEG;
    }
    else if (wrapped <= -NAV_HALF_TURN_MDEG)
    {
        wrapped += NAV_FULL_TURN_MDEG;
    }
    else
    {
        /* Already in range. */
    }

    return (wrapped);
}

/* End of file. */
