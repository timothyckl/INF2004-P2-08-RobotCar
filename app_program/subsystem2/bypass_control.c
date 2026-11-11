/**
 * @file bypass_control.c
 * @brief Implement the pure bypass and reacquisition (see bypass_control.h).
 */
#include "bypass_control.h"

#include "motion_api.h"
#include "subsystem2.h"

#define BYPASS_HALF_TURN_MDEG   180000
#define BYPASS_FULL_TURN_MDEG   360000

/* The align spin gives up this far past the align angle: 135 degrees in
   all for the 45 degree align (agreed with Dalton, S2 decision 21). The
   total must stay below 180 degrees, the most a wrapped heading change
   can show. */
#define BYPASS_SPIN_OVER_MDEG   90000

static void    bypass_clear(bypass_control_output_t * p_out);
static bool    bypass_plan_valid(bypass_control_plan_t const * p_plan);
static void    bypass_next(bypass_control_t * p_bypass,
                           bypass_control_input_t const * p_in,
                           bypass_control_output_t * p_out);
static void    bypass_look(bypass_control_t * p_bypass,
                           bypass_control_input_t const * p_in,
                           bypass_control_output_t * p_out);
static void    bypass_pass(bypass_control_t * p_bypass,
                           bypass_control_input_t const * p_in, bool b_done,
                           bypass_control_output_t * p_out);
static void    bypass_search(bypass_control_t * p_bypass,
                             bypass_control_input_t const * p_in,
                             bypass_control_output_t * p_out);
static void    bypass_halt(bypass_control_t * p_bypass,
                           bypass_control_input_t const * p_in,
                           bypass_control_output_t * p_out);
static void    bypass_align(bypass_control_t * p_bypass,
                            bypass_control_input_t const * p_in,
                            bypass_control_output_t * p_out);
static void    bypass_spin(bypass_control_t * p_bypass,
                           bypass_control_input_t const * p_in,
                           bypass_control_output_t * p_out);
static void    bypass_check_heading(bypass_control_t * p_bypass,
                                    bypass_control_input_t const * p_in,
                                    bypass_control_output_t * p_out);
static void    bypass_request(bypass_control_t * p_bypass,
                              bypass_control_output_t * p_out,
                              bypass_control_action_t action, int32_t value);
static void    bypass_finish(bypass_control_t * p_bypass,
                             bypass_control_output_t * p_out,
                             bypass_control_outcome_t outcome,
                             bypass_control_fail_t fail);
static int32_t bypass_wrap_mdeg(int32_t mdeg);

bool
bypass_control_init (bypass_control_t * p_bypass,
                     bypass_control_cfg_t const * p_cfg)
{
    bool const b_valid = (NULL != p_bypass) && (NULL != p_cfg)
                         && (0u != p_cfg->search_limit_ms)
                         && (0u != p_cfg->found_ms)
                         && (0u != p_cfg->step_mm)
                         && (0u != p_cfg->wait_limit_ms)
                         && (p_cfg->heading_tol_mdeg > 0)
                         && (p_cfg->heading_tol_mdeg < BYPASS_HALF_TURN_MDEG);

    if (b_valid)
    {
        bypass_control_t const idle = {0};

        *p_bypass = idle;
        p_bypass->cfg = *p_cfg;
        p_bypass->state = BYPASS_CONTROL_IDLE;
    }

    return (b_valid);
}

bool
bypass_control_active (bypass_control_t const * p_bypass)
{
    return ((NULL != p_bypass) && (BYPASS_CONTROL_IDLE != p_bypass->state));
}

uint8_t
bypass_control_mode (bypass_control_t const * p_bypass)
{
    uint8_t mode = S2_MODE_NONE;

    if (bypass_control_active(p_bypass))
    {
        mode = ((BYPASS_CONTROL_SEGMENT == p_bypass->state)
                || (BYPASS_CONTROL_PASS == p_bypass->state))
               ? S2_MODE_BYPASS : S2_MODE_REACQUIRE;
    }

    return (mode);
}

void
bypass_control_begin (bypass_control_t * p_bypass,
                      bypass_control_plan_t const * p_plan,
                      bypass_control_input_t const * p_in,
                      bypass_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    bypass_clear(p_out);

    if ((NULL == p_bypass) || (NULL == p_plan) || (NULL == p_in)
        || bypass_control_active(p_bypass))
    {
        return;
    }

    if (!bypass_plan_valid(p_plan))
    {
        p_out->outcome = BYPASS_CONTROL_FAILED;
        p_out->fail = BYPASS_CONTROL_FAIL_PLAN;
        return;
    }

    p_bypass->plan = *p_plan;
    p_bypass->heading_ref_mdeg = p_in->heading_mdeg;
    p_bypass->next = 0u;
    p_bypass->b_have_id = false;
    p_bypass->b_have_look = false;
    p_bypass->b_looking = false;
    p_bypass->b_extra = false;
    p_bypass->b_aligned = false;
    p_bypass->b_passed = (0 == p_plan->look_deg);
    p_bypass->stepped_mm = 0;
    p_bypass->b_seen = false;
    p_bypass->state = BYPASS_CONTROL_SEGMENT;
    bypass_next(p_bypass, p_in, p_out);
}

void
bypass_control_sent (bypass_control_t * p_bypass, bool b_sent,
                     uint16_t cmd_id, bypass_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    bypass_clear(p_out);

    if (!bypass_control_active(p_bypass))
    {
        return;
    }

    if (!b_sent)
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      p_bypass->b_looking ? BYPASS_CONTROL_FAIL_LOOK
                                          : BYPASS_CONTROL_FAIL_MOTION);
    }
    else if (p_bypass->b_looking)
    {
        p_bypass->look_cmd_id = cmd_id;
        p_bypass->b_have_look = true;
    }
    else
    {
        p_bypass->cmd_id = cmd_id;
        p_bypass->b_have_id = true;
    }
}

void
bypass_control_step (bypass_control_t * p_bypass,
                     bypass_control_input_t const * p_in,
                     bypass_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    bypass_clear(p_out);

    if ((!bypass_control_active(p_bypass)) || (NULL == p_in))
    {
        return;
    }

    bool const b_ours = p_in->b_status_ok && p_bypass->b_have_id
                        && (p_in->last_id == p_bypass->cmd_id);
    bool const b_done = b_ours && (MOTION_EVT_DONE == p_in->last_result);

    /* The halt's own command ends STOPPED: that is its success. */
    if (b_ours && ((MOTION_EVT_REJECTED == p_in->last_result)
                   || (MOTION_EVT_FAULT == p_in->last_result)
                   || ((MOTION_EVT_STOPPED == p_in->last_result)
                       && (BYPASS_CONTROL_HALT != p_bypass->state))))
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_MOTION);
    }
    else if (p_bypass->b_forward && p_in->b_hazard && (!b_done))
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_HAZARD);
    }
    else if (BYPASS_CONTROL_SEARCH == p_bypass->state)
    {
        bypass_search(p_bypass, p_in, p_out);
    }
    else if (BYPASS_CONTROL_PASS == p_bypass->state)
    {
        bypass_pass(p_bypass, p_in, b_done, p_out);
    }
    else if (BYPASS_CONTROL_HALT == p_bypass->state)
    {
        bypass_halt(p_bypass, p_in, p_out);
    }
    else if (BYPASS_CONTROL_ALIGN == p_bypass->state)
    {
        bypass_spin(p_bypass, p_in, p_out);
    }
    else if (!b_done)
    {
        /* Waiting for the MOVE or TURN in progress. */
    }
    else if (BYPASS_CONTROL_SEGMENT == p_bypass->state)
    {
        bypass_next(p_bypass, p_in, p_out);
    }
    else
    {
        bypass_align(p_bypass, p_in, p_out);    /* the step is done */
    }
}

void
bypass_control_abort (bypass_control_t * p_bypass)
{
    if (bypass_control_active(p_bypass))
    {
        p_bypass->state = BYPASS_CONTROL_IDLE;
        p_bypass->b_have_id = false;
        p_bypass->b_forward = false;
    }
}

static void
bypass_clear (bypass_control_output_t * p_out)
{
    p_out->action = BYPASS_CONTROL_ACT_NONE;
    p_out->value = 0;
    p_out->outcome = BYPASS_CONTROL_NONE;
    p_out->fail = BYPASS_CONTROL_FAIL_NONE;
}

/* Every value must fit one motion command (motion_api.h limits). */
static bool
bypass_plan_valid (bypass_control_plan_t const * p_plan)
{
    int32_t const turn_max = (int32_t)MOTION_ANGLE_MAX_MDEG;
    int32_t const move_max = (int32_t)MOTION_DISTANCE_MAX_MM;
    bool const    b_pass   = (0 != p_plan->look_deg);
    bool          b_valid  = (p_plan->count <= BYPASS_CONTROL_SEGMENTS)
                             && (0u != p_plan->search_mm)
                             && ((!b_pass)
                                 || ((p_plan->look_deg >= -90)
                                     && (p_plan->look_deg <= 90)
                                     && (0u != p_plan->clear_mm)
                                     && ((int32_t)p_plan->extra_mm
                                         <= move_max)
                                     && ((int32_t)p_plan->pass_max_mm
                                         <= move_max)))
                             && (p_plan->approach_mdeg >= -turn_max)
                             && (p_plan->approach_mdeg <= turn_max)
                             && (p_plan->align_mdeg >= -turn_max)
                             && (p_plan->align_mdeg <= turn_max);

    for (uint8_t i = 0u; b_valid && (i < p_plan->count); i++)
    {
        bypass_control_segment_t const * p_seg = &p_plan->seg[i];
        int32_t const max = (BYPASS_CONTROL_TURN == p_seg->kind) ? turn_max
                                                                 : move_max;

        b_valid = ((BYPASS_CONTROL_TURN == p_seg->kind)
                   || (BYPASS_CONTROL_MOVE == p_seg->kind))
                  && (p_seg->value >= -max) && (p_seg->value <= max);
    }

    return (b_valid);
}

/* Request the next nonzero segment, then the approach turn, then the
   search. */
static void
bypass_next (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
             bypass_control_output_t * p_out)
{
    while (p_bypass->next < p_bypass->plan.count)
    {
        bypass_control_segment_t const * p_seg =
            &p_bypass->plan.seg[p_bypass->next];

        p_bypass->next++;

        if (0 != p_seg->value)
        {
            bypass_request(p_bypass, p_out,
                           (BYPASS_CONTROL_TURN == p_seg->kind)
                               ? BYPASS_CONTROL_ACT_TURN
                               : BYPASS_CONTROL_ACT_MOVE,
                           p_seg->value);
            return;
        }
    }

    if ((p_bypass->next == p_bypass->plan.count) && (!p_bypass->b_passed))
    {
        /* Pass the obstacle by measurement before turning back. */
        p_bypass->state = BYPASS_CONTROL_PASS;
        bypass_look(p_bypass, p_in, p_out);
        return;
    }

    if (p_bypass->next == p_bypass->plan.count)
    {
        p_bypass->next++;                   /* the approach turn, once */

        if (0 != p_bypass->plan.approach_mdeg)
        {
            bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_TURN,
                           p_bypass->plan.approach_mdeg);
            return;
        }
    }

    p_bypass->state = BYPASS_CONTROL_SEARCH;
    p_bypass->since_ms = p_in->now_ms;
    p_bypass->start_mm = p_in->travel_mm;
    p_bypass->b_seen = false;
    bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_SEARCH, 0);
}

static void
bypass_look (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
             bypass_control_output_t * p_out)
{
    p_bypass->b_looking = true;
    p_bypass->b_have_look = false;
    p_bypass->wait_since_ms = p_in->now_ms;
    bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_LOOK,
                   (int32_t)p_bypass->plan.look_deg);
}

/* Look; step until a look has passed the obstacle; then drive extra_mm
   and continue with the approach turn (S2 decision 19). */
static void
bypass_pass (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
             bool b_done, bypass_control_output_t * p_out)
{
    bool const b_answer = p_bypass->b_looking && p_bypass->b_have_look
                          && (p_in->look_id == p_bypass->look_cmd_id);

    if (p_bypass->b_looking)
    {
        int32_t const step_mm = (int32_t)p_bypass->cfg.step_mm;

        if (b_answer && p_in->b_look_fault)
        {
            bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                          BYPASS_CONTROL_FAIL_LOOK);
        }
        else if (b_answer && p_in->b_look_done)
        {
            p_bypass->b_looking = false;

            if ((p_in->look_mm > p_bypass->plan.clear_mm)
                && (0u == p_bypass->plan.extra_mm))
            {
                p_bypass->b_passed = true;
                p_bypass->state = BYPASS_CONTROL_SEGMENT;
                bypass_next(p_bypass, p_in, p_out);
            }
            else if (p_in->look_mm > p_bypass->plan.clear_mm)
            {
                p_bypass->b_extra = true;
                bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_MOVE,
                               (int32_t)p_bypass->plan.extra_mm);
            }
            else if ((p_bypass->stepped_mm + step_mm)
                     > (int32_t)p_bypass->plan.pass_max_mm)
            {
                bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                              BYPASS_CONTROL_FAIL_DEPTH);
            }
            else
            {
                p_bypass->stepped_mm += step_mm;
                bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_MOVE,
                               step_mm);
            }
        }
        else if ((p_in->now_ms - p_bypass->wait_since_ms)
                 > p_bypass->cfg.wait_limit_ms)
        {
            bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                          BYPASS_CONTROL_FAIL_LOOK);
        }
        else
        {
            /* Waiting for the look. */
        }
    }
    else if (!b_done)
    {
        /* Waiting for the step or the extra move. */
    }
    else if (p_bypass->b_extra)
    {
        p_bypass->b_passed = true;
        p_bypass->state = BYPASS_CONTROL_SEGMENT;
        bypass_next(p_bypass, p_in, p_out);
    }
    else
    {
        bypass_look(p_bypass, p_in, p_out);
    }
}

static void
bypass_search (bypass_control_t * p_bypass,
               bypass_control_input_t const * p_in,
               bypass_control_output_t * p_out)
{
    int32_t const travel_mm = p_in->travel_mm - p_bypass->start_mm;

    if (((p_in->now_ms - p_bypass->since_ms) > p_bypass->cfg.search_limit_ms)
        || (travel_mm > (int32_t)p_bypass->plan.search_mm))
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_SEARCH);
    }
    else if (!p_in->b_on_line)
    {
        p_bypass->b_seen = false;
    }
    else if (!p_bypass->b_seen)
    {
        p_bypass->b_seen = true;
        p_bypass->seen_ms = p_in->now_ms;
        p_bypass->seen_mm = p_in->travel_mm;
    }
    else if ((p_in->now_ms - p_bypass->seen_ms) >= p_bypass->cfg.found_ms)
    {
        /* Found: stop first, so the step is measured from where the car
           actually comes to rest, not from where it was still rolling. */
        p_bypass->state = BYPASS_CONTROL_HALT;
        p_bypass->b_aligned = false;
        p_bypass->wait_since_ms = p_in->now_ms;
        bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_HALT, 0);
    }
    else
    {
        /* On the line, not yet for found_ms. */
    }
}

/* Once still after the spin, check the heading; once still after the
   search, bring the axle to where the sensors first saw the line. */
static void
bypass_halt (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
             bypass_control_output_t * p_out)
{
    if (p_in->b_still && p_bypass->b_aligned)
    {
        bypass_check_heading(p_bypass, p_in, p_out);
    }
    else if (p_in->b_still)
    {
        int32_t const step_mm = (int32_t)p_bypass->cfg.sensor_ahead_mm
                                - (p_in->travel_mm - p_bypass->seen_mm);

        if (0 != step_mm)
        {
            p_bypass->state = BYPASS_CONTROL_STEP;
            bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_MOVE,
                           step_mm);
        }
        else
        {
            bypass_align(p_bypass, p_in, p_out);
        }
    }
    else if ((p_in->now_ms - p_bypass->wait_since_ms)
             > p_bypass->cfg.wait_limit_ms)
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_MOTION);
    }
    else
    {
        /* Braking. */
    }
}

static void
bypass_align (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
              bypass_control_output_t * p_out)
{
    if (0 != p_bypass->plan.align_mdeg)
    {
        p_bypass->state = BYPASS_CONTROL_ALIGN;
        p_bypass->align_start_mdeg = p_in->heading_mdeg;
        bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_SPIN,
                       (p_bypass->plan.align_mdeg > 0) ? 1 : -1);
    }
    else
    {
        bypass_check_heading(p_bypass, p_in, p_out);
    }
}

/* Spin until a sensor sees the line (S2 decision 21): the dead-reckoned
   45 degrees alone can leave the line outside both sensors, which read
   the same white as when they straddle it. */
static void
bypass_spin (bypass_control_t * p_bypass, bypass_control_input_t const * p_in,
             bypass_control_output_t * p_out)
{
    int32_t const align = p_bypass->plan.align_mdeg;
    int32_t const tol = p_bypass->cfg.heading_tol_mdeg;
    int32_t const turned_raw = bypass_wrap_mdeg(p_in->heading_mdeg
                                                - p_bypass->align_start_mdeg);
    int32_t const turned = (align > 0) ? turned_raw : -turned_raw;
    int32_t const want = (align > 0) ? align : -align;

    if (p_in->b_on_line && (turned >= (want - (2 * tol))))
    {
        p_bypass->state = BYPASS_CONTROL_HALT;
        p_bypass->b_aligned = true;
        p_bypass->wait_since_ms = p_in->now_ms;
        bypass_request(p_bypass, p_out, BYPASS_CONTROL_ACT_HALT, 0);
    }
    else if (turned > (want + BYPASS_SPIN_OVER_MDEG))
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_SEARCH);
    }
    else
    {
        /* Spinning. */
    }
}

static void
bypass_check_heading (bypass_control_t * p_bypass,
                      bypass_control_input_t const * p_in,
                      bypass_control_output_t * p_out)
{
    int32_t const err = bypass_wrap_mdeg(p_in->heading_mdeg
                                         - p_bypass->heading_ref_mdeg);
    int32_t const tol = p_bypass->cfg.heading_tol_mdeg;

    if ((err > tol) || (err < -tol))
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_HEADING);
    }
    else if (p_in->b_hazard)
    {
        /* Subsystem 5 raises a hazard only on its edge, so following must
           not resume into one already present. */
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_FAILED,
                      BYPASS_CONTROL_FAIL_HAZARD);
    }
    else
    {
        bypass_finish(p_bypass, p_out, BYPASS_CONTROL_DONE,
                      BYPASS_CONTROL_FAIL_NONE);
    }
}

static void
bypass_request (bypass_control_t * p_bypass, bypass_control_output_t * p_out,
                bypass_control_action_t action, int32_t value)
{
    p_bypass->b_forward = (BYPASS_CONTROL_ACT_SEARCH == action)
                          || ((BYPASS_CONTROL_ACT_MOVE == action)
                              && (value > 0));
    p_out->action = action;
    p_out->value = value;
}

static void
bypass_finish (bypass_control_t * p_bypass, bypass_control_output_t * p_out,
               bypass_control_outcome_t outcome, bypass_control_fail_t fail)
{
    p_bypass->state = BYPASS_CONTROL_IDLE;
    p_bypass->b_have_id = false;
    p_bypass->b_forward = false;
    p_out->action = BYPASS_CONTROL_ACT_NONE;
    p_out->value = 0;
    p_out->outcome = outcome;
    p_out->fail = fail;
}

/* Both headings lie in (-180000, 180000], so the difference fits int32_t
   and one correction brings it into the same range. */
static int32_t
bypass_wrap_mdeg (int32_t mdeg)
{
    int32_t wrapped = mdeg;

    if (wrapped > BYPASS_HALF_TURN_MDEG)
    {
        wrapped -= BYPASS_FULL_TURN_MDEG;
    }
    else if (wrapped <= -BYPASS_HALF_TURN_MDEG)
    {
        wrapped += BYPASS_FULL_TURN_MDEG;
    }
    else
    {
        /* Already in range. */
    }

    return (wrapped);
}

/* End of file. */
