/**
 * @file scan_engine.h
 * @brief Safety-gated obstacle scan sequencing.
 */
#ifndef SCAN_ENGINE_H
#define SCAN_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

/** Inputs accepted by the scan state machine. */
typedef enum
{
    SCAN_ENGINE_TICK,
    SCAN_ENGINE_ECHO_EDGE,
    SCAN_ENGINE_REQUEST,
    SCAN_ENGINE_AUTHORISE,
    SCAN_ENGINE_STOP_CONFIRM,
    SCAN_ENGINE_CANCEL,
    SCAN_ENGINE_STOP,
    SCAN_ENGINE_MOVEMENT,
    SCAN_ENGINE_RESET
} scan_engine_event_kind_t;

/** One decoded scan-control or timing event. */
typedef struct
{
    scan_engine_event_kind_t kind;
    uint32_t                 request_id;
    bool                     b_asserted;
} scan_engine_event_t;

/** Observable actions requested by the state machine. */
typedef enum
{
    SCAN_ENGINE_TRIGGER,
    SCAN_ENGINE_SERVO,
    SCAN_ENGINE_DISTANCE,
    SCAN_ENGINE_HAZARD,
    SCAN_ENGINE_FAULT,
    SCAN_ENGINE_CANCEL_ACK,
    SCAN_ENGINE_READY,
    SCAN_ENGINE_PLAN
} scan_engine_output_kind_t;

/** One decoded state-machine output. */
typedef struct
{
    scan_engine_output_kind_t kind;
    uint32_t                  request_id;
    int16_t                   angle_deg;
    uint16_t                  distance_mm;
    bool                      b_asserted;
} scan_engine_output_t;

/**
 * @brief Clock and output dependencies owned by the calling environment.
 *
 * Both callbacks execute synchronously and shall be bounded and non-blocking.
 * They shall not re-enter the same scan engine. The microsecond clock shall
 * advance monotonically through its 32-bit wraparound. Calls shall occur often
 * enough that no measured interval spans a complete clock period.
 */
typedef struct
{
    uint32_t (*p_time_now_us)(void);
    bool     (*p_publish)(scan_engine_output_t const * p_output);
} scan_engine_port_t;

/** Calibrated values supplied by the calling environment. */
typedef struct
{
    uint32_t settling_us;
    uint16_t hazard_mm;
} scan_engine_config_t;

/** State owned by the task that operates the scanner. */
typedef struct
{
    scan_engine_port_t   port;
    scan_engine_config_t config;
    uint32_t             request_id;
    uint32_t             settling_started_us;
    bool                 b_has_request;
    bool                 b_is_authorised;
    bool                 b_is_stopped;
    bool                 b_is_scanning;
    bool                 b_trigger_started;
} scan_engine_t;

/**
 * @brief Reset a scanner using explicit dependencies and configuration.
 *
 * Null dependency or configuration pointers leave the corresponding fields
 * cleared. A null engine pointer has no effect.
 *
 * @param p_engine Scanner state to reset.
 * @param p_port Clock and publication dependencies, or NULL.
 * @param p_config Calibrated scanner configuration, or NULL.
 */
void scan_engine_reset (scan_engine_t * p_engine,
                        scan_engine_port_t const * p_port,
                        scan_engine_config_t const * p_config);

/**
 * @brief Process one decoded event without blocking.
 *
 * Null engine or event pointers have no effect. Processing remains
 * non-blocking when the port callbacks satisfy scan_engine_port_t's contract.
 *
 * @param p_engine Scanner state that owns the event.
 * @param p_event Decoded event to process.
 */
void scan_engine_process (scan_engine_t * p_engine,
                          scan_engine_event_t const * p_event);

#endif /* SCAN_ENGINE_H */

/* End of file. */
