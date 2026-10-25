/**
 * @file test_interface.h
 * @brief Subsystem 5 Pico W unit-test interfaces, not a wire format.
 *
 * Inputs describe decoded events. Native integer containers here do not set
 * packed field widths. No sensor, servo or controller behaviour is implemented.
 * Sources: docs/decisions.md, subsystem 5 sections 3, 4, 6 and 8.
 */
#ifndef TEST_INTERFACE_H
#define TEST_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

/** Events supplied by an on-device fixture or platform adapter. */
typedef enum
{
    TEST_INTERFACE_TICK,
    TEST_INTERFACE_ECHO,
    TEST_INTERFACE_SCAN_REQUEST,
    TEST_INTERFACE_SCAN_AUTHORISE,
    TEST_INTERFACE_STOP_CONFIRM,
    TEST_INTERFACE_CANCEL,
    TEST_INTERFACE_STOP,
    TEST_INTERFACE_MOVEMENT,
    TEST_INTERFACE_RESET
} test_interface_input_kind_t;

/** A decoded input; b_value carries edge level or authorisation state. */
typedef struct
{
    test_interface_input_kind_t kind;
    uint32_t                    request_id;
    bool                        b_value;
} test_interface_input_t;

/** Observable outputs; safety signals are distinct from queued results. */
typedef enum
{
    TEST_INTERFACE_TRIGGER,
    TEST_INTERFACE_SERVO,
    TEST_INTERFACE_DISTANCE,
    TEST_INTERFACE_HAZARD,
    TEST_INTERFACE_FAULT,
    TEST_INTERFACE_CANCEL_ACK,
    TEST_INTERFACE_READY,
    TEST_INTERFACE_PLAN
} test_interface_output_kind_t;

/** Decoded observations, not a packed shared-memory or MQTT payload. */
typedef struct
{
    test_interface_output_kind_t kind;
    uint32_t                     request_id;
    int16_t                      angle_deg;
    uint16_t                     distance_mm;
    bool                         b_value;
} test_interface_output_t;

/** Dependencies replaced by controlled inputs during on-device logic tests. */
typedef struct
{
    uint32_t (*p_now_us)(void);
    bool     (*p_emit)(test_interface_output_t const * p_output);
} test_interface_port_t;

/** Parameters; logic fixtures use synthetic, not calibrated, values. */
typedef struct
{
    uint32_t settling_us;
    uint16_t hazard_mm;
} test_interface_config_t;

/**
 * @brief Operations a future adapter must bind to actual subsystem 5 logic.
 *
 * reset_fixture establishes a clean, ready, forward-monitoring fixture.
 * pulse_mm converts an echo width at 340 m/s. Exact-value tests use integral
 * millimetre fixtures. Invalid conversion leaves the output untouched.
 * dispatch uses the injected clock. SCAN_AUTHORISE and STOP_CONFIRM are
 * separate, request-matched inputs; b_value=false withdraws their assertion.
 * STOP is a new stop event, not a persistent motor-inhibit level.
 */
typedef struct
{
    void (*p_reset_fixture)(test_interface_port_t const * p_port,
                           test_interface_config_t const * p_config);
    bool (*p_pulse_mm)(uint32_t width_us, uint16_t * p_distance_mm);
    void (*p_dispatch)(test_interface_input_t const * p_input);
} test_interface_api_t;

#endif /* TEST_INTERFACE_H */

/* End of file. */
