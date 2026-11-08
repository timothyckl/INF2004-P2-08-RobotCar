/**
 * @file forward_monitor.h
 * @brief Forward clearance verdict while the vehicle moves (pure logic).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: obstacle-scanning decision 3. One forward reading every 65 ms;
 * an unsafe, invalid or stale reading requests a stop immediately, with no
 * filtering delay. Data older than fresh_us (agreed 150 ms) is unusable.
 * An attempt with no completed echo within 30 ms is invalid, not clear.
 */
#ifndef FORWARD_MONITOR_H
#define FORWARD_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#define FORWARD_MONITOR_INVALID_LIMIT (3u)

/** Monitoring verdict; anything except SAFE requests a stop. */
typedef enum
{
    FORWARD_MONITOR_SAFE = 0,
    FORWARD_MONITOR_UNSAFE,     /* valid reading below hazard_mm */
    FORWARD_MONITOR_INVALID,    /* latest attempt gave no measurement */
    FORWARD_MONITOR_STALE       /* no reading within fresh_us */
} forward_monitor_verdict_t;

/** Thresholds supplied by the caller. */
typedef struct
{
    uint16_t hazard_mm;
    uint32_t fresh_us;
} forward_monitor_config_t;

/** Latest reading; owned by one task. */
typedef struct
{
    uint32_t sample_us;
    uint16_t distance_mm;
    uint8_t  invalid_count;
    bool     b_has_sample;
    bool     b_is_valid;
} forward_monitor_t;

/** Forget every reading; the verdict is STALE until a new one arrives. */
void forward_monitor_reset (forward_monitor_t * p_monitor);

/** Record one attempt; distance_mm is ignored when b_valid is false. */
void forward_monitor_sample (forward_monitor_t * p_monitor, bool b_valid,
                             uint16_t distance_mm, uint32_t now_us);

/** True after three consecutive invalid forward attempts. */
bool forward_monitor_faulted (forward_monitor_t const * p_monitor);

/** Judge the latest reading at now_us. */
forward_monitor_verdict_t forward_monitor_check (
    forward_monitor_t const * p_monitor,
    forward_monitor_config_t const * p_config, uint32_t now_us);

#endif /* FORWARD_MONITOR_H */

/* End of file. */
