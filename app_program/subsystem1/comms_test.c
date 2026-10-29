/** @file comms_test.c
 * @brief Hardware-independent regression tests using controlled inputs.
 */
#include "comms_test.h"

#if (defined(APP_TEST) && APP_TEST) || defined(S1_HOST_TEST)

#include "command_parser.h"
#include "comms_timer.h"
#include "service_bridge.h"
#include "telemetry_encoder.h"

#include <string.h>

static void check (bool condition, bool * p_passed);

typedef struct
{
    uint32_t                   stop_calls;
    uint32_t                   command_calls;
    uint32_t                   last_received_ms;
    command_parser_command_t   last_command;
    telemetry_encoder_snapshot_t snapshot;
    bool                       deliver;
    bool                       snapshot_ready;
} comms_test_context_t;

static bool fake_stop (void * p_context, uint32_t received_ms);
static bool fake_command (void * p_context,
    command_parser_command_t command, uint32_t received_ms);
static bool fake_snapshot (void * p_context,
    telemetry_encoder_snapshot_t * p_snapshot);

bool comms_test_commands (void)
{
    static const char * const p_invalid[] =
    {
        "", "start", "Stop", "RESET\n", " STOP", "STOP ",
        "STAR", "STARTSTOP", "GO", "STOP\0junk"
    };
    static const size_t lengths[] = {0u, 5u, 4u, 6u, 5u, 5u,
                                    4u, 9u, 2u, 9u};
    static const uint8_t raw_stop[] = {'S', 'T', 'O', 'P'};
    static const uint8_t embedded_null[] = {'S', 'T', 0u, 'P'};
    bool passed = true;

    check((COMMAND_PARSER_START == command_parser_parse("START", 5u)),
              &passed);
    check((COMMAND_PARSER_RESET == command_parser_parse("RESET", 5u)), &passed);
    check((COMMAND_PARSER_STOP ==
               command_parser_parse(raw_stop, sizeof(raw_stop))), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 4u)), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 0u)), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("S", SIZE_MAX)),
              &passed);
    check((COMMAND_PARSER_INVALID ==
               command_parser_parse(embedded_null, sizeof(embedded_null))),
              &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("STOP", 5u)),
              &passed);

    for (size_t idx = 0u; idx < (sizeof(lengths) / sizeof(lengths[0])); ++idx)
    {
        check((COMMAND_PARSER_INVALID ==
                   command_parser_parse(p_invalid[idx], lengths[idx])),
              &passed);
    }

    return passed;
}

bool comms_test_telemetry (void)
{
    telemetry_encoder_snapshot_t snapshot =
    {
        UINT32_C(0x12345678), UINT16_C(0xabcd), true, true, true
    };
    const uint8_t golden[] = {1u, 7u, 0x78u, 0x56u, 0x34u, 0x12u,
                              0xcdu, 0xabu};
    uint8_t output[TELEMETRY_ENCODER_SIZE + 1u] = {0u};
    bool    passed = true;

    output[TELEMETRY_ENCODER_SIZE] = 0xa5u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((0 == memcmp(output, golden, sizeof(golden))), &passed);
    check((0xa5u == output[TELEMETRY_ENCODER_SIZE]), &passed);

    for (size_t capacity = 0u; capacity < TELEMETRY_ENCODER_SIZE; ++capacity)
    {
        memset(output, 0xa5, sizeof(output));
        check((0u ==
                   telemetry_encoder_encode(&snapshot, output, capacity)),
              &passed);

        for (size_t idx = 0u; idx < sizeof(output); ++idx)
        {
            check((0xa5u == output[idx]), &passed);
        }
    }

    check((0u == telemetry_encoder_encode(NULL, output, sizeof(output))),
              &passed);
    check((0xa5u == output[0]), &passed);
    check((0u == telemetry_encoder_encode(&snapshot, NULL,
                                               TELEMETRY_ENCODER_SIZE)),
              &passed);
    snapshot.captured_ms = UINT32_MAX;
    snapshot.distance_mm = UINT16_MAX;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);

    for (size_t idx = 2u; idx < TELEMETRY_ENCODER_SIZE; ++idx)
    {
        check((0xffu == output[idx]), &passed);
    }

    snapshot.distance_valid = false;
    snapshot.stopped        = false;
    snapshot.link_up        = false;
    snapshot.captured_ms    = 0u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);

    for (size_t idx = 1u; idx < TELEMETRY_ENCODER_SIZE; ++idx)
    {
        check((0u == output[idx]), &passed);
    }

    snapshot.distance_valid = true;
    snapshot.distance_mm    = 0u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_DISTANCE_VALID == output[1]), &passed);
    check((0u == output[6]), &passed);
    check((0u == output[7]), &passed);
    snapshot.distance_valid = false;
    snapshot.stopped        = true;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_STOPPED == output[1]), &passed);
    snapshot.stopped = false;
    snapshot.link_up = true;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_LINK_UP == output[1]), &passed);

    return passed;
}

bool comms_test_timers (void)
{
    comms_timer_t timer = {0};
    bool          passed = true;

    check(!comms_timer_init(NULL, 100u, 50u), &passed);
    check(!comms_timer_init(&timer, 0u, 50u), &passed);
    check(!comms_timer_init(&timer, 100u, 0u), &passed);
    check(!comms_timer_init(&timer, UINT32_MAX, 50u), &passed);
    check(!comms_timer_init(&timer, 100u, UINT32_MAX), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(NULL, 0u, true, true, false)), &passed);
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, true)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, false, false, true)), &passed);
    check((COMMS_TIMER_MQTT ==
               comms_timer_poll(&timer, 50u, true, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, true, false, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 51u, true, true, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 150u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 151u, true, true, false)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 152u, false, false, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 153u, true, true, false)), &passed);

    /* Start near clock wrap, then cross it for both independent schedules. */
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_WIFI ==
        comms_timer_poll(&timer, UINT32_MAX - 24u, false, false, false)),
              &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 24u, false, false, false)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 25u, false, false, false)), &passed);
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
        comms_timer_poll(&timer, UINT32_MAX - 49u, true, true, false)),
              &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 50u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &passed);
    check(comms_timer_init(&timer, UINT32_C(0x7fffffff), 1u), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, true, false)), &passed);

    return passed;
}

bool comms_test_bridge (void)
{
    comms_test_context_t context = {0};
    service_bridge_port_t port =
    {
        fake_stop, fake_command, fake_snapshot, &context
    };
    service_bridge_t bridge = {0};
    service_bridge_stats_t stats = {0};
    uint8_t output[TELEMETRY_ENCODER_SIZE] = {0u};
    bool    passed = true;

    check(!service_bridge_init(NULL, "robot/cmd", 9u, &port), &passed);
    check(!service_bridge_init(&bridge, NULL, 9u, &port), &passed);
    check(!service_bridge_init(&bridge, "", 0u, &port), &passed);
    check(!service_bridge_init(&bridge, "robot/cmd", 9u, NULL), &passed);
    port.p_raise_remote_stop = NULL;
    check(!service_bridge_init(&bridge, "robot/cmd", 9u, &port), &passed);
    port.p_raise_remote_stop = fake_stop;
    check(service_bridge_init(&bridge, "robot/cmd", 9u, &port), &passed);

    check(!service_bridge_begin(&bridge, "other/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_IGNORED == service_bridge_data(
        &bridge, "STOP", 4u, true, 1u)), &passed);
    check(!service_bridge_begin(&bridge, "robot/cmd", 9u, 0u), &passed);
    check(!service_bridge_begin(&bridge, "robot/cmd", 9u, 6u), &passed);
    check(!service_bridge_begin(&bridge, NULL, 9u, 4u), &passed);

    context.deliver = true;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_MORE == service_bridge_data(
        &bridge, "ST", 2u, false, 10u)), &passed);
    check((SERVICE_BRIDGE_FORWARDED == service_bridge_data(
        &bridge, "OP", 2u, true, 11u)), &passed);
    check((1u == context.stop_calls), &passed);
    check((0u == context.command_calls), &passed);
    check((11u == context.last_received_ms), &passed);

    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 5u), &passed);
    check((SERVICE_BRIDGE_FORWARDED == service_bridge_data(
        &bridge, "START", 5u, true, 20u)), &passed);
    check((1u == context.command_calls), &passed);
    check((COMMAND_PARSER_START == context.last_command), &passed);
    check((20u == context.last_received_ms), &passed);

    context.deliver = false;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 5u), &passed);
    check((SERVICE_BRIDGE_UNDELIVERED == service_bridge_data(
        &bridge, "RESET", 5u, true, 30u)), &passed);
    check((2u == context.command_calls), &passed);
    check((COMMAND_PARSER_RESET == context.last_command), &passed);

    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "NOPE", 4u, true, 40u)), &passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, NULL, 1u, true, 41u)), &passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "STOPX", 5u, true, 42u)), &passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "STOP", 4u, false, 43u)), &passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "ST", 2u, true, 44u)), &passed);

    context.snapshot = (telemetry_encoder_snapshot_t)
    {
        100u, 250u, true, true, true
    };
    context.snapshot_ready = false;
    check((0u == service_bridge_telemetry(&bridge, output,
                                          sizeof(output))), &passed);
    context.snapshot_ready = true;
    check((TELEMETRY_ENCODER_SIZE == service_bridge_telemetry(
        &bridge, output, sizeof(output))), &passed);
    check((TELEMETRY_ENCODER_VERSION == output[0]), &passed);
    check((7u == output[1]), &passed);
    check((100u == output[2]), &passed);
    check((250u == output[6]), &passed);

    check(service_bridge_get_stats(&bridge, &stats), &passed);
    check((2u == stats.valid_commands), &passed);
    check((7u == stats.invalid_messages), &passed);
    check((1u == stats.delivery_failures), &passed);
    check((2u == stats.ignored_topics), &passed);
    check(!service_bridge_get_stats(NULL, &stats), &passed);
    check(!service_bridge_get_stats(&bridge, NULL), &passed);

    return passed;
}

static bool fake_stop (void * p_context, uint32_t received_ms)
{
    comms_test_context_t * const p_test = p_context;

    ++p_test->stop_calls;
    p_test->last_received_ms = received_ms;

    return p_test->deliver;
}

static bool fake_command (void * p_context,
    command_parser_command_t command, uint32_t received_ms)
{
    comms_test_context_t * const p_test = p_context;

    ++p_test->command_calls;
    p_test->last_command = command;
    p_test->last_received_ms = received_ms;

    return p_test->deliver;
}

static bool fake_snapshot (void * p_context,
    telemetry_encoder_snapshot_t * p_snapshot)
{
    comms_test_context_t * const p_test = p_context;
    bool copied = false;

    if (p_test->snapshot_ready)
    {
        *p_snapshot = p_test->snapshot;
        copied = true;
    }

    return copied;
}

/** @brief Accumulate a failed check without skipping subsequent cases. */
static void check (bool condition, bool * p_passed)
{
    if (!condition)
    {
        *p_passed = false;
    }
}

#endif

/*** End of file. ***/

