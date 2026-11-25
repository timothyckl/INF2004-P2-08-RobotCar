/**
 * @file replay_raw.c
 * @brief Replay an S4_RAW_LOG console log through terrain_logic.c.
 *
 *   make -C testing/host s4-replay LOG=<log> [ARGS="..."]
 *   or build by hand (see testing/subsystem4/README.md) and run
 *   replay_raw <log> [scale_permille] [offset_um] [rise_um] [follow]
 *   ("-" for any number keeps the mission value)
 *
 * Every IMU sample the car processed is in the log as
 *   s4 raw <ms>: fwd=<mg> left=<mg> up=<mg> mg | ... | pitch used|ignored
 *   | state <n> | humps|detect on|off
 * (vehicle axes, after s4_sample_to_vehicle()). This feeds each one to the
 * real s4_terrain_process() with the detection gate and pitch flag the
 * car used, starts a new run where the car did, and prints every hump the
 * replay completes with its unrounded peak, beside the humps the car
 * itself reported. With the settings the car ran (the defaults are the
 * mission's, s4_config_mission()) the two lists must agree; then the same
 * samples can be replayed with other settings, for calibration (decision
 * 17). Host-only: not part of the firmware.
 *
 * Each "=== capture attached" line (the capture script reconnecting, so
 * possibly a reboot) starts the replay afresh, as the car calibrates
 * again. Fidelity: every sample's replayed tilt is compared with the
 * tilt the car logged on that line; the count of samples that differ is
 * printed. When a capture began part-way through the car's calibration
 * (so the replay calibrated on a shifted window), the first full average
 * window after calibration is used to move the replay's offsets onto the
 * car's: the shift is printed, once per capture.
 *
 * With "follow" as the fifth argument, detection is also kept off from
 * "[S2] nav: junction; stopping to turn" to "[S2] nav: on the new line"
 * (or "failed") and for S4_SETTLE_SAMPLES samples after, as the gate of
 * decision 17 does on the car (FOLLOW only); without it the replay uses
 * the gate the car logged.
 *
 * Limits: the log has 1 ms time resolution and only the samples the car
 * processed; lines the USB console garbled are skipped; I2C errors and
 * sensor restarts are not replayed.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "terrain_logic.h"
#include "s4_config.h"

#define REPLAY_LINE_MAX 512

/* Logged hump report: "s4: hump N mm under the ...". */
static int replay_logged_hump(char const * p_line)
{
    char const * p_hump = strstr(p_line, "s4: hump ");
    int height_mm = -1;

    if ((NULL != p_hump) && (NULL != strstr(p_hump, " mm under")))
    {
        height_mm = atoi(p_hump + strlen("s4: hump "));
    }

    return (height_mm);
}

int
main (int argc, char * argv[])
{
    s4_terrain_config_t config = s4_config_mission();
    s4_terrain_context_t context;
    char line[REPLAY_LINE_MAX];
    FILE * p_file;
    unsigned int samples = 0u;
    unsigned int replayed = 0u;
    unsigned int logged = 0u;
    unsigned int tilt_mismatch = 0u;
    unsigned int monitoring = 0u;   /* samples monitored this capture */
    unsigned int faults = 0u;
    bool b_follow_only = false;
    bool b_in_nav = false;
    unsigned int settle = 0u;
    bool b_synced = false;
    int32_t highest_on_um = 0;      /* largest height while detecting */

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s <log> [scale_permille] [offset_um] "
                        "[rise_um]\n", argv[0]);
        return (2);
    }
    if ((argc > 2) && (0 != strcmp(argv[2], "-")))
    {
        config.height_scale_permille = (uint16_t)atoi(argv[2]);
    }
    if ((argc > 3) && (0 != strcmp(argv[3], "-")))
    {
        config.height_offset_um = (uint16_t)atoi(argv[3]);
    }
    if ((argc > 4) && (0 != strcmp(argv[4], "-")))
    {
        config.rise_um = (uint16_t)atoi(argv[4]);
    }
    if (argc > 5)
    {
        b_follow_only = (0 == strcmp(argv[5], "follow"));
    }
    /* Lines the USB console garbled leave gaps the car never had, and the
       stale-sample check would then fault the replay; it is not what the
       replay studies, so it is relaxed here only. */
    config.max_sample_gap_ms = UINT16_MAX;

    p_file = fopen(argv[1], "r");
    if (NULL == p_file)
    {
        perror(argv[1]);
        return (2);
    }

    printf("# replay of %s: scale %u permille, offset %u um, rise %u um, "
           "level %u um\n", argv[1],
           (unsigned)config.height_scale_permille,
           (unsigned)config.height_offset_um, (unsigned)config.rise_um,
           (unsigned)config.level_um);
    s4_terrain_init(&context, &config);

    while (NULL != fgets(line, sizeof(line), p_file))
    {
        unsigned long time_ms = 0ul;
        int fwd = 0;
        int left = 0;
        int up = 0;
        int const logged_mm = replay_logged_hump(line);
        char const * p_raw = strstr(line, "s4 raw ");

        if (logged_mm >= 0)
        {
            logged++;
            printf("car   hump %3d mm   (%s", logged_mm,
                   strstr(line, "under"));
            continue;
        }
        if (NULL != strstr(line, "=== capture attached"))
        {
            s4_terrain_init(&context, &config);
            monitoring = 0u;
            b_synced = false;
            printf("      capture attached: replay restarted\n");
            continue;
        }
        if (NULL != strstr(line, "[S2] nav: junction; stopping to turn"))
        {
            b_in_nav = true;
            continue;
        }
        if ((NULL != strstr(line, "[S2] nav: on the new line"))
            || (NULL != strstr(line, "[S2] nav: junction manoeuvre failed")))
        {
            b_in_nav = false;
            settle = S4_SETTLE_SAMPLES;
            continue;
        }
        if (NULL != strstr(line, "s4: new run"))
        {
            s4_terrain_start_new_run(&context);
            printf("      new run\n");
            continue;
        }
        if ((NULL != strstr(line, "s4: LSM303DLHC ready"))
            || (NULL != strstr(line, "calibration failed")))
        {
            s4_terrain_request_recovery(&context);
            continue;
        }
        if ((NULL == p_raw)
            || (4 != sscanf(p_raw, "s4 raw %lu: fwd=%d left=%d up=%d",
                            &time_ms, &fwd, &left, &up)))
        {
            continue;
        }

        {
            s4_terrain_sample_t sample;
            bool const b_pitch = (NULL != strstr(p_raw, "pitch used"));
            bool b_detect = (NULL != strstr(p_raw, "humps on"))
                             || (NULL != strstr(p_raw, "detect on"));
            int32_t const peak_before_um = context.peak_height_um;

            sample.acceleration_x_mg = (int16_t)fwd;
            sample.acceleration_y_mg = (int16_t)left;
            sample.acceleration_z_mg = (int16_t)up;
            sample.capture_time_us = (uint32_t)(time_ms * 1000ul);
            sample.b_valid = true;
            samples++;

            if (b_follow_only)
            {
                if (b_in_nav || (!b_detect))
                {
                    b_detect = false;
                }
                else if (settle > 0u)
                {
                    settle--;
                    b_detect = false;
                }
                else
                {
                    /* Following and settled: as logged. */
                }
            }

            s4_terrain_set_detect_enable(&context, b_detect);
            s4_terrain_process(&context, &sample, b_pitch);
            if (S4_TERRAIN_STATE_FAULT == context.result.state)
            {
                /* As s4_imu_task() does: a calibration fault calibrates
                   again in place; any other fault reopens the sensor,
                   which also requests recovery. */
                s4_terrain_request_recovery(&context);
                faults++;
            }

            {
                int tilt_fwd = 0;
                int tilt_left = 0;
                char const * p_tilt = strstr(p_raw, "tilt fwd=");

                bool const b_tilt = (NULL != p_tilt)
                    && (2 == sscanf(p_tilt, "tilt fwd=%d left=%d",
                                    &tilt_fwd, &tilt_left));

                monitoring = (S4_TERRAIN_STATE_CALIBRATING
                              == context.result.state) ? 0u
                                                       : (monitoring + 1u);
                if (b_tilt && (!b_synced)
                    && (monitoring == config.average_samples))
                {
                    int32_t const dx = context.result.forward_tilt_mg
                                       - tilt_fwd;
                    int32_t const dy = context.result.lateral_tilt_mg
                                       - tilt_left;

                    b_synced = true;
                    if ((abs(dx) > 1) || (abs(dy) > 1))
                    {
                        for (uint8_t idx = 0u; idx < config.average_samples;
                             idx++)
                        {
                            context.average_x_mg[idx] -= dx;
                            context.average_y_mg[idx] -= dy;
                        }
                        context.average_sum_x_mg -=
                            dx * (int32_t)config.average_samples;
                        context.average_sum_y_mg -=
                            dy * (int32_t)config.average_samples;
                        context.filtered_x_mg -= dx;
                        context.filtered_y_mg -= dy;
                        context.offset_x_mg = (int16_t)(context.offset_x_mg
                                                        + dx);
                        context.offset_y_mg = (int16_t)(context.offset_y_mg
                                                        + dy);
                        context.result.forward_tilt_mg = (int16_t)tilt_fwd;
                        context.result.lateral_tilt_mg = (int16_t)tilt_left;
                        printf("      capture began mid-calibration: "
                               "offsets moved %d/%d mg onto the car's\n",
                               (int)dx, (int)dy);
                    }
                }
                if (b_tilt
                    && ((tilt_fwd != context.result.forward_tilt_mg)
                        || (tilt_left != context.result.lateral_tilt_mg)))
                {
                    tilt_mismatch++;
                }
            }
            if (b_detect && (context.peak_height_um > highest_on_um))
            {
                highest_on_um = context.peak_height_um;
            }
            if (0u != (context.result.events
                       & S4_TERRAIN_EVENT_HUMP_COMPLETE))
            {
                replayed++;
                printf("replay hump %3u mm   peak %6d um at %lu ms\n",
                       (unsigned)context.result.current_height_mm,
                       (int)peak_before_um, time_ms);
            }
        }
    }
    (void)fclose(p_file);

    printf("# samples %u (tilt differs from the car on %u; replay faults "
           "%u), humps: car %u, replay %u; highest candidate while "
           "detecting %d um\n", samples, tilt_mismatch, faults, logged,
           replayed, (int)highest_on_um);

    return (0);
}

/* End of file. */
