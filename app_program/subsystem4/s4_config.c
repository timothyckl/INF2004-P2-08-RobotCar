/**
 * @file s4_config.c
 * @brief Subsystem 4 terrain settings (s4_config.h).
 */
#include "s4_config.h"

s4_terrain_config_t
s4_config_mission (void)
{
    s4_terrain_config_t const config =
    {
        .calibration_samples = S4_CALIBRATION_SAMPLES,
        .calibration_span_mg = S4_CALIBRATION_SPAN_MG,
        .impact_threshold_mg = S4_IMPACT_THRESHOLD_MG,
        .max_sample_gap_ms = S4_MAX_SAMPLE_GAP_MS,
        .max_hump_ms = S4_MAX_HUMP_MS,
        .track_mm = S4_TRACK_MM,
        .wheelbase_mm = S4_WHEELBASE_MM,
        .rise_um = S4_RISE_UM,
        .level_um = S4_LEVEL_UM,
        .confirmation_samples = S4_CONFIRMATION_SAMPLES,
        .invalid_limit = S4_INVALID_SAMPLE_LIMIT,
        .average_samples = S4_AVERAGE_SAMPLES,
        .height_scale_permille = S4_HEIGHT_SCALE_PERMILLE,
        .height_offset_um = S4_HEIGHT_OFFSET_UM,
        .lift_up_mg = S4_LIFT_UP_MG,
        .lift_tilt_mg = S4_LIFT_TILT_MG,
        .lift_samples = S4_LIFT_SAMPLES,
        .lift_raw_tilt_mg = S4_LIFT_RAW_TILT_MG,
        .lift_raw_samples = S4_LIFT_RAW_SAMPLES
    };

    return (config);
}

/* End of file. */
