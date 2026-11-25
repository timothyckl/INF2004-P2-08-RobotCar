/**
 * @file robot_state.c
 * @brief robot_state under state_mutex (see robot_state.h, subsystem2.h).
 *
 * state_mutex uses priority inheritance and is held only for one struct
 * copy, so a reader (including the higher-priority WiFi task) can never
 * hold up the controller for long. The WiFi task's reader never waits.
 */
#include "robot_state.h"
#include "../subsystem4/subsystem4.h"   /* subsystem4_get_status (IF-18) */

#define S2_STATE_LOCK_TMO_MS    10      /* task readers, not the WiFi task */
#define S2_DISTANCE_MAX_MM      65535   /* uint16_t telemetry field */
#define S2_HUMP_MAX_MM          255u    /* uint8_t telemetry field */

static ID               g_state_mtx = 0;
static s2_robot_state_t g_state;        /* guarded by g_state_mtx */

static uint8_t s2_hump_mm8 (uint16_t height_mm);

ER
s2_robot_state_create (void)
{
    T_CMTX         cmtx = {0};
    s2_robot_state_t const boot = {0u, 0, 0u, S2_SYS_INITIALISING,
                                   S2_FAULT_NONE, S2_RS_STOP_LATCHED,
                                   0u, S2_RESULT_NONE, 0u, S2_MODE_NONE,
                                   S2_LINE_STOP, 0, 0, 0, '\0', S2_NAV_NONE,
                                   0u, S2_STOP_BOOT};

    g_state      = boot;
    cmtx.mtxatr  = TA_INHERIT;

    ID const mtx = tk_cre_mtx(&cmtx);

    if (mtx < E_OK)
    {
        return (mtx);
    }

    g_state_mtx = mtx;      /* published only once it exists */

    return (E_OK);
}

void
s2_robot_state_publish (s2_robot_state_t const * p_state)
{
    if ((NULL != p_state) && (g_state_mtx > 0)
        && (tk_loc_mtx(g_state_mtx, S2_STATE_LOCK_TMO_MS) >= E_OK))
    {
        g_state = *p_state;
        (void)tk_unl_mtx(g_state_mtx);
    }
}

ER
s2_get_robot_state (s2_robot_state_t * p_state)
{
    ID const mtx = g_state_mtx;

    if (NULL == p_state)
    {
        return (E_PAR);
    }

    if (mtx <= 0)
    {
        return (E_OBJ);
    }

    ER const ercd = tk_loc_mtx(mtx, S2_STATE_LOCK_TMO_MS);

    if (ercd < E_OK)
    {
        return (ercd);
    }

    *p_state = g_state;
    (void)tk_unl_mtx(mtx);

    return (E_OK);
}

/* WiFi task: never waits; a busy state_mutex skips one publication. */
bool
robot_read_telemetry (void * p_context,
                      telemetry_encoder_snapshot_t * p_snapshot)
{
    ID const               mtx = g_state_mtx;
    s2_robot_state_t       state;
    s4_terrain_result_t    hump;
    scan_telemetry_t       scan;

    (void)p_context;

    if ((NULL == p_snapshot) || (mtx <= 0)
        || (tk_loc_mtx(mtx, TMO_POL) < E_OK))
    {
        return (false);
    }

    state = g_state;
    (void)tk_unl_mtx(mtx);

    /* The S4 and S5 locks are taken after state_mutex is released, so no
     * two locks ever nest. A busy provider reports nothing this time. */
    bool const b_hump = (E_OK == subsystem4_get_status(&hump));
    bool const b_scan = scan_telemetry_get(&scan);

    s2_telemetry_from_state(&state, p_snapshot);
    s2_telemetry_add_hump(p_snapshot, b_hump ? hump.maximum_height_mm : 0u,
                          b_hump && hump.b_maximum_height_valid,
                          b_hump ? hump.current_height_mm : 0u,
                          b_hump && hump.b_current_height_valid,
                          b_hump ? hump.hump_count : 0u);
    s2_telemetry_add_scan(p_snapshot, b_scan ? &scan : NULL);

    return (true);
}

void
s2_telemetry_add_hump (telemetry_encoder_snapshot_t * p_out,
                       uint16_t max_mm, bool b_max_valid,
                       uint16_t last_mm, bool b_last_valid,
                       uint8_t count)
{
    if (NULL == p_out)
    {
        return;
    }

    p_out->b_hump_valid      = b_max_valid;
    p_out->hump_max_mm     = b_max_valid ? s2_hump_mm8(max_mm) : 0u;
    p_out->b_hump_last_valid = b_last_valid;
    p_out->hump_last_mm    = b_last_valid ? s2_hump_mm8(last_mm) : 0u;
    p_out->hump_count      = count;
}

/* Saturate a height to the uint8_t telemetry field. */
static uint8_t
s2_hump_mm8 (uint16_t height_mm)
{
    return ((height_mm > S2_HUMP_MAX_MM) ? (uint8_t)S2_HUMP_MAX_MM
                                         : (uint8_t)height_mm);
}

void
s2_telemetry_add_scan (telemetry_encoder_snapshot_t * p_out,
                       scan_telemetry_t const * p_scan)
{
    if (NULL == p_out)
    {
        return;
    }

    p_out->b_forward_valid    = false;
    p_out->b_profile_valid    = false;
    p_out->b_shape_valid      = false;
    p_out->b_width_raw        = false;
    p_out->forward_mm       = 0u;
    p_out->nearest_mm       = 0u;
    p_out->width_mm         = 0u;
    p_out->centre_offset_mm = 0;
    p_out->scan_result      = SCAN_TELEMETRY_NONE;
    p_out->bypass_side      = SCAN_TELEMETRY_SIDE_NONE;

    if (NULL == p_scan)
    {
        return;
    }

    p_out->scan_result = p_scan->scan_result;
    p_out->bypass_side = p_scan->bypass_side;

    if (0u != (p_scan->flags & SCAN_TELEMETRY_FORWARD_VALID))
    {
        p_out->b_forward_valid = true;
        p_out->forward_mm    = p_scan->forward_mm;
    }

    if (0u != (p_scan->flags & SCAN_TELEMETRY_PROFILE_SEEN))
    {
        p_out->b_profile_valid = true;
        p_out->nearest_mm    = p_scan->nearest_observed_mm;
    }

    if (0u != (p_scan->flags & SCAN_TELEMETRY_SHAPE_ESTIMATE))
    {
        p_out->b_shape_valid      = true;
        p_out->width_mm         = p_scan->observed_width_mm;
        p_out->centre_offset_mm = p_scan->centre_offset_mm;
    }
    else if (0u != (p_scan->flags & SCAN_TELEMETRY_WIDTH_RAW))
    {
        /* Display only: an edge is missing (S1 decision 7). */
        p_out->b_width_raw = true;
        p_out->width_mm  = p_scan->observed_width_mm;
    }
    else
    {
        /* No width at all. */
    }
}

void
s2_telemetry_from_state (s2_robot_state_t const * p_state,
                         telemetry_encoder_snapshot_t * p_out)
{
    if ((NULL == p_state) || (NULL == p_out))
    {
        return;
    }

    int32_t const distance = p_state->run_distance_mm;
    uint8_t const flag_bits = p_state->flags;
    bool const    b_known = (0u != (flag_bits & S2_RS_DISTANCE_VALID));
    bool const    b_motion = (0u != (flag_bits & S2_RS_MOTION_VALID));
    bool const    b_line = (0u != (flag_bits & S2_RS_LINE_FRESH));
    bool const    b_barcode = (0u != (flag_bits & S2_RS_BARCODE_VALID));

    p_out->captured_ms    = p_state->captured_ms;
    p_out->b_distance_valid = b_known && (distance >= 0)
                            && (distance <= S2_DISTANCE_MAX_MM);
    /* distance is saturated at S2_DISTANCE_MAX_MM (65535) before the cast. */
    p_out->distance_mm    = (!b_known || (distance < 0)) ? 0u
                            : ((distance > S2_DISTANCE_MAX_MM)
                               ? (uint16_t)S2_DISTANCE_MAX_MM
                               : (uint16_t)distance);
    p_out->b_stopped        = (0u != (flag_bits & S2_RS_STOPPED));
    p_out->b_stop_latched   = (0u != (flag_bits & S2_RS_STOP_LATCHED));
    p_out->state          = p_state->state;
    p_out->mode           = p_state->mode;
    p_out->fault          = p_state->fault;
    p_out->b_motion_valid   = b_motion;
    p_out->left_mm_s      = b_motion ? p_state->left_mm_s : 0;
    p_out->right_mm_s     = b_motion ? p_state->right_mm_s : 0;
    p_out->heading_cdeg   = b_motion ? p_state->heading_cdeg : 0;
    p_out->b_line_fresh     = b_line;
    p_out->line           = b_line ? p_state->line : 0u;
    p_out->b_barcode_valid  = b_barcode;
    p_out->barcode        = b_barcode ? p_state->barcode : '\0';
    p_out->barcode_command = b_barcode ? p_state->barcode_command
                                       : S2_NAV_NONE;
    p_out->barcode_count  = b_barcode ? p_state->barcode_count : 0u;
    p_out->stop_cause     = (0u != (flag_bits & S2_RS_STOP_LATCHED))
                            ? p_state->stop_cause : S2_STOP_NONE;
}

/* End of file. */
