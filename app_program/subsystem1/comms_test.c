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

static void check (bool b_condition, bool * p_passed);

typedef struct
{
    uint32_t                   stop_calls;
    uint32_t                   command_calls;
    uint32_t                   assign_calls;
    uint32_t                   last_received_ms;
    command_parser_command_t   last_command;
    uint8_t                    last_route;
    char                       last_letter;
    telemetry_encoder_snapshot_t snapshot;
    bool                       b_deliver;
    bool                       b_snapshot_ready;
} comms_test_context_t;

static bool fake_stop (void * p_context, uint32_t received_ms);
static bool fake_command (void * p_context,
    command_parser_command_t command, uint32_t received_ms);
static bool fake_snapshot (void * p_context,
    telemetry_encoder_snapshot_t * p_snapshot);
static bool fake_assign (void * p_context, uint8_t route, char letter);
static bool assign_is (const char * p_payload, size_t length,
                       uint8_t route, char letter);
static bool assign_rejects (const char * p_payload, size_t length);

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
    bool b_passed = true;

    check((COMMAND_PARSER_START == command_parser_parse("START", 5u)),
              &b_passed);
    check((COMMAND_PARSER_RESET == command_parser_parse("RESET", 5u)),
          &b_passed);
    check((COMMAND_PARSER_STOP ==
               command_parser_parse(raw_stop, sizeof(raw_stop))), &b_passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 4u)),
          &b_passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 0u)),
          &b_passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("S", SIZE_MAX)),
              &b_passed);
    check((COMMAND_PARSER_INVALID ==
               command_parser_parse(embedded_null, sizeof(embedded_null))),
              &b_passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("STOP", 5u)),
              &b_passed);

    for (size_t idx = 0u; idx < (sizeof(lengths) / sizeof(lengths[0])); ++idx)
    {
        check((COMMAND_PARSER_INVALID ==
                   command_parser_parse(p_invalid[idx], lengths[idx])),
              &b_passed);
    }

    /* Barcode letter assignments (S1 decision 6). */
    check(assign_is("L=A", 3u, COMMAND_PARSER_ROUTE_LEFT, 'A'), &b_passed);
    check(assign_is("R=Z", 3u, COMMAND_PARSER_ROUTE_RIGHT, 'Z'), &b_passed);
    check(assign_is("S=F", 3u, COMMAND_PARSER_ROUTE_STRAIGHT, 'F'),
          &b_passed);
    check(assign_is("U=D", 3u, COMMAND_PARSER_ROUTE_UTURN, 'D'), &b_passed);
    check(assign_rejects("L=a", 3u), &b_passed);
    check(assign_rejects("L=@", 3u), &b_passed);
    check(assign_rejects("L=[", 3u), &b_passed);
    check(assign_rejects("X=A", 3u), &b_passed);
    check(assign_rejects("l=A", 3u), &b_passed);
    check(assign_rejects("L:A", 3u), &b_passed);
    check(assign_rejects("L=AB", 4u), &b_passed);
    check(assign_rejects("L=", 2u), &b_passed);
    check(assign_rejects(NULL, 3u), &b_passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("L=A", 3u)),
          &b_passed);

    return b_passed;
}

bool comms_test_telemetry (void)
{
    /* Every field set; the golden frame is the version 5 layout. Letter
       'Z' (25) with command 4 is 0x99; stop cause 4 with count 0x1f
       (low bits 0xf) is 0xf4. Width raw is set but shape valid wins, so
       flags1 bit 4 stays clear. Version 5 byte 31: motion fault 7 (task),
       held command 2 (RIGHT) and 13 junctions (low bits 1) is
       7 | 2 << 3 | 1 << 6 = 0x57. */
    static const telemetry_encoder_snapshot_t full =
    {
        UINT32_C(0x12345678), UINT16_C(0xabcd),
        INT16_C(-150), INT16_C(320), INT16_C(-18000),
        UINT16_C(4000), UINT16_C(0x0102), UINT16_C(0x0304), INT16_C(-2),
        3u, 4u, 2u, 1u, 1u, 2u, 255u, 254u, 0x21u,
        'Z', 4u, 0x1fu, 4u, 7u, 2u, 13u,
        true, true, true, true, true, true, true, true, true, true, true,
        true, true
    };
    static const uint8_t golden[TELEMETRY_ENCODER_SIZE] =
    {
        5u, 0xffu, 0x78u, 0x56u, 0x34u, 0x12u, 0xcdu, 0xabu,
        0x0fu, 0x43u, 0x12u, 0x21u,
        0x6au, 0xffu, 0x40u, 0x01u, 0xb0u, 0xb9u,
        0xa0u, 0x0fu, 0x02u, 0x01u, 0x04u, 0x03u, 0xfeu, 0xffu,
        0xffu, 0x99u, 0xf4u, 0xfeu, 0x21u, 0x57u
    };
    telemetry_encoder_snapshot_t snapshot = full;
    uint8_t output[TELEMETRY_ENCODER_SIZE + 1u] = {0u};
    bool    b_passed = true;

    output[TELEMETRY_ENCODER_SIZE] = 0xa5u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
          &b_passed);
    check((0 == memcmp(output, golden, sizeof(golden))), &b_passed);
    check((0xa5u == output[TELEMETRY_ENCODER_SIZE]), &b_passed);

    /* Too little space, or a NULL argument, leaves the output alone. */
    for (size_t capacity = 0u; capacity < TELEMETRY_ENCODER_SIZE; ++capacity)
    {
        memset(output, 0xa5, sizeof(output));
        check((0u ==
                   telemetry_encoder_encode(&snapshot, output, capacity)),
              &b_passed);

        for (size_t idx = 0u; idx < sizeof(output); ++idx)
        {
            check((0xa5u == output[idx]), &b_passed);
        }
    }

    check((0u == telemetry_encoder_encode(NULL, output, sizeof(output))),
          &b_passed);
    check((0xa5u == output[0]), &b_passed);
    check((0u == telemetry_encoder_encode(&snapshot, NULL,
                                          TELEMETRY_ENCODER_SIZE)),
          &b_passed);

    /* Every validity flag clear: each guarded field is sent as zero. */
    snapshot.b_distance_valid = false;
    snapshot.b_stopped        = false;
    snapshot.b_link_up        = false;
    snapshot.b_stop_latched   = false;
    snapshot.b_motion_valid   = false;
    snapshot.b_line_fresh     = false;
    snapshot.b_hump_valid     = false;
    snapshot.b_forward_valid  = false;
    snapshot.b_profile_valid  = false;
    snapshot.b_shape_valid    = false;
    snapshot.b_barcode_valid  = false;
    snapshot.b_hump_last_valid = false;
    snapshot.b_width_raw      = false;
    snapshot.hump_count     = 0u;
    snapshot.captured_ms    = 0u;
    snapshot.state          = 0u;
    snapshot.mode           = 0u;
    snapshot.fault          = 0u;
    snapshot.scan_result    = 0u;
    snapshot.bypass_side    = 0u;
    snapshot.motion_fault   = 0u;
    snapshot.nav_held       = 0u;
    snapshot.junction_count = 0u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
          &b_passed);
    check((TELEMETRY_ENCODER_VERSION == output[0]), &b_passed);

    for (size_t idx = 1u; idx < TELEMETRY_ENCODER_SIZE; ++idx)
    {
        check((0u == output[idx]), &b_passed);
    }

    /* Version 5 byte 31: out-of-range detail and held command become 7,
       the junction count keeps its low 2 bits (4 is 0, 7 is 3). */
    snapshot.motion_fault = 9u;
    snapshot.nav_held = 5u;
    snapshot.junction_count = 4u;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0x3fu == output[31]), &b_passed);
    snapshot.motion_fault = 3u;
    snapshot.nav_held = 4u;
    snapshot.junction_count = 7u;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0xe3u == output[31]), &b_passed);
    snapshot.motion_fault = 0u;
    snapshot.nav_held = 0u;
    snapshot.junction_count = 0u;

    /* Single flags land on their own bits. */
    snapshot.b_stopped = true;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((TELEMETRY_ENCODER_STOPPED == output[1]), &b_passed);
    snapshot.b_stopped = false;
    snapshot.b_link_up = true;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((TELEMETRY_ENCODER_LINK_UP == output[1]), &b_passed);
    snapshot.b_link_up     = false;
    snapshot.b_shape_valid = true;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0u == output[1]), &b_passed);
    check((TELEMETRY_ENCODER_SHAPE_VALID == output[8]), &b_passed);
    check((0x04u == output[22]) && (0x03u == output[23]), &b_passed);

    /* Version 4: a raw width is sent, with its flag, only without a
       shape; the centre stays zero. */
    snapshot.b_shape_valid = false;
    snapshot.b_width_raw   = true;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((TELEMETRY_ENCODER_WIDTH_RAW == output[8]), &b_passed);
    check((0x04u == output[22]) && (0x03u == output[23]), &b_passed);
    check((0u == output[24]) && (0u == output[25]), &b_passed);
    snapshot.b_width_raw   = false;
    snapshot.b_hump_last_valid = true;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((TELEMETRY_ENCODER_HUMP_LAST_VALID == output[1]), &b_passed);
    check((0xfeu == output[29]) && (0u == output[30]), &b_passed);
    snapshot.b_hump_last_valid = false;
    snapshot.b_shape_valid = true;

    /* A code above 14 is sent as 15 (invalid), never masked. */
    snapshot.state       = 16u;
    snapshot.mode        = 14u;
    snapshot.b_line_fresh  = true;
    snapshot.line        = 200u;
    snapshot.scan_result = 15u;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0xefu == output[9]), &b_passed);
    check((0xf0u == output[10]), &b_passed);
    check((0x0fu == output[11]), &b_passed);

    /* Extremes of the signed and unsigned ranges. */
    snapshot = full;
    snapshot.left_mm_s        = INT16_MIN;
    snapshot.right_mm_s       = INT16_MAX;
    snapshot.distance_mm      = UINT16_MAX;
    snapshot.centre_offset_mm = INT16_MIN;
    snapshot.captured_ms      = UINT32_MAX;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0x00u == output[12]) && (0x80u == output[13]), &b_passed);
    check((0xffu == output[14]) && (0x7fu == output[15]), &b_passed);
    check((0xffu == output[6]) && (0xffu == output[7]), &b_passed);
    check((0x00u == output[24]) && (0x80u == output[25]), &b_passed);
    check((0xffu == output[2]) && (0xffu == output[5]), &b_passed);

    /* Version 3: an invalid letter is 31 and a command above 6 is 7; the
       count keeps its low bits; the cause is sent only while latched. */
    snapshot = full;
    snapshot.barcode         = 'a';
    snapshot.barcode_command = 9u;
    snapshot.barcode_count   = 0x10u;
    snapshot.stop_cause      = 20u;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0xffu == output[27]), &b_passed);
    check((0x0fu == output[28]), &b_passed);
    snapshot.barcode         = 'A';
    snapshot.barcode_command = 6u;
    snapshot.b_stop_latched    = false;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0xc0u == output[27]), &b_passed);
    check((0x00u == output[28]), &b_passed);
    snapshot.b_barcode_valid   = false;
    snapshot.b_stop_latched    = true;
    snapshot.stop_cause      = 4u;
    (void) telemetry_encoder_encode(&snapshot, output, sizeof(output));
    check((0x07u == output[8]), &b_passed);
    check((0x00u == output[27]), &b_passed);
    check((0x04u == output[28]), &b_passed);

    return b_passed;
}

bool comms_test_timers (void)
{
    comms_timer_t timer = {0};
    bool          b_passed = true;

    check(!comms_timer_init(NULL, 100u, 50u), &b_passed);
    check(!comms_timer_init(&timer, 0u, 50u), &b_passed);
    check(!comms_timer_init(&timer, 100u, 0u), &b_passed);
    check(!comms_timer_init(&timer, UINT32_MAX, 50u), &b_passed);
    check(!comms_timer_init(&timer, 100u, UINT32_MAX), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(NULL, 0u, true, true, false)), &b_passed);
    check(comms_timer_init(&timer, 100u, 50u), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, true)), &b_passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, false, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, false, false, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, false, false, true)), &b_passed);
    check((COMMS_TIMER_MQTT ==
               comms_timer_poll(&timer, 50u, true, false, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, true, false, false)), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 51u, true, true, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 150u, true, true, false)), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 151u, true, true, false)), &b_passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 152u, false, false, false)), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 153u, true, true, false)), &b_passed);

    /* Start near clock wrap, then cross it for both independent schedules. */
    check(comms_timer_init(&timer, 100u, 50u), &b_passed);
    check((COMMS_TIMER_WIFI ==
        comms_timer_poll(&timer, UINT32_MAX - 24u, false, false, false)),
              &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 24u, false, false, false)), &b_passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 25u, false, false, false)), &b_passed);
    check(comms_timer_init(&timer, 100u, 50u), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
        comms_timer_poll(&timer, UINT32_MAX - 49u, true, true, false)),
              &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, true, true, false)), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 50u, true, true, false)), &b_passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &b_passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &b_passed);
    check(comms_timer_init(&timer, UINT32_C(0x7fffffff), 1u), &b_passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, true, false)), &b_passed);

    return b_passed;
}

bool comms_test_bridge (void)
{
    comms_test_context_t context = {0};
    service_bridge_port_t port =
    {
        fake_stop, fake_command, fake_snapshot, fake_assign, &context
    };
    service_bridge_t bridge = {0};
    service_bridge_stats_t stats = {0};
    uint8_t output[TELEMETRY_ENCODER_SIZE] = {0u};
    bool    b_passed = true;

    check(!service_bridge_init(NULL, "robot/cmd", 9u, &port), &b_passed);
    check(!service_bridge_init(&bridge, NULL, 9u, &port), &b_passed);
    check(!service_bridge_init(&bridge, "", 0u, &port), &b_passed);
    check(!service_bridge_init(&bridge, "robot/cmd", 9u, NULL), &b_passed);
    port.p_raise_remote_stop = NULL;
    check(!service_bridge_init(&bridge, "robot/cmd", 9u, &port), &b_passed);
    port.p_raise_remote_stop = fake_stop;
    port.p_assign_letter = NULL;
    check(!service_bridge_init(&bridge, "robot/cmd", 9u, &port), &b_passed);
    port.p_assign_letter = fake_assign;
    check(service_bridge_init(&bridge, "robot/cmd", 9u, &port), &b_passed);

    check(!service_bridge_begin(&bridge, "other/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_IGNORED == service_bridge_data(
        &bridge, "STOP", 4u, true, 1u)), &b_passed);
    check(!service_bridge_begin(&bridge, "robot/cmd", 9u, 0u), &b_passed);
    check(!service_bridge_begin(&bridge, "robot/cmd", 9u, 6u), &b_passed);
    check(!service_bridge_begin(&bridge, NULL, 9u, 4u), &b_passed);

    context.b_deliver = true;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_MORE == service_bridge_data(
        &bridge, "ST", 2u, false, 10u)), &b_passed);
    check((SERVICE_BRIDGE_FORWARDED == service_bridge_data(
        &bridge, "OP", 2u, true, 11u)), &b_passed);
    check((1u == context.stop_calls), &b_passed);
    check((0u == context.command_calls), &b_passed);
    check((11u == context.last_received_ms), &b_passed);

    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 5u), &b_passed);
    check((SERVICE_BRIDGE_FORWARDED == service_bridge_data(
        &bridge, "START", 5u, true, 20u)), &b_passed);
    check((1u == context.command_calls), &b_passed);
    check((COMMAND_PARSER_START == context.last_command), &b_passed);
    check((20u == context.last_received_ms), &b_passed);

    context.b_deliver = false;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 5u), &b_passed);
    check((SERVICE_BRIDGE_UNDELIVERED == service_bridge_data(
        &bridge, "RESET", 5u, true, 30u)), &b_passed);
    check((2u == context.command_calls), &b_passed);
    check((COMMAND_PARSER_RESET == context.last_command), &b_passed);

    context.b_deliver = true;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 3u), &b_passed);
    check((SERVICE_BRIDGE_FORWARDED == service_bridge_data(
        &bridge, "U=K", 3u, true, 35u)), &b_passed);
    check((1u == context.assign_calls), &b_passed);
    check((COMMAND_PARSER_ROUTE_UTURN == context.last_route), &b_passed);
    check(('K' == context.last_letter), &b_passed);
    check((2u == context.command_calls), &b_passed);
    context.b_deliver = false;
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 3u), &b_passed);
    check((SERVICE_BRIDGE_UNDELIVERED == service_bridge_data(
        &bridge, "L=Q", 3u, true, 36u)), &b_passed);
    check((2u == context.assign_calls), &b_passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 3u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "L=q", 3u, true, 37u)), &b_passed);
    check((2u == context.assign_calls), &b_passed);

    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "NOPE", 4u, true, 40u)), &b_passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, NULL, 1u, true, 41u)), &b_passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "STOPX", 5u, true, 42u)), &b_passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "STOP", 4u, false, 43u)), &b_passed);
    check(service_bridge_begin(&bridge, "robot/cmd", 9u, 4u), &b_passed);
    check((SERVICE_BRIDGE_INVALID == service_bridge_data(
        &bridge, "ST", 2u, true, 44u)), &b_passed);

    context.snapshot = (telemetry_encoder_snapshot_t) {0};
    context.snapshot.captured_ms    = 100u;
    context.snapshot.distance_mm    = 250u;
    context.snapshot.b_distance_valid = true;
    context.snapshot.b_stopped        = true;
    context.b_snapshot_ready = false;
    check((0u == service_bridge_telemetry(&bridge, output,
                                          sizeof(output))), &b_passed);
    context.b_snapshot_ready = true;
    check((TELEMETRY_ENCODER_SIZE == service_bridge_telemetry(
        &bridge, output, sizeof(output))), &b_passed);
    check((TELEMETRY_ENCODER_VERSION == output[0]), &b_passed);
    /* The bridge sets link up: distance valid, stopped and link up. */
    check((7u == output[1]), &b_passed);
    check((100u == output[2]), &b_passed);
    check((250u == output[6]), &b_passed);

    check(service_bridge_get_stats(&bridge, &stats), &b_passed);
    check((3u == stats.valid_commands), &b_passed);
    check((8u == stats.invalid_messages), &b_passed);
    check((2u == stats.delivery_failures), &b_passed);
    check((2u == stats.ignored_topics), &b_passed);
    check(!service_bridge_get_stats(NULL, &stats), &b_passed);
    check(!service_bridge_get_stats(&bridge, NULL), &b_passed);

    return b_passed;
}

static bool fake_stop (void * p_context, uint32_t received_ms)
{
    comms_test_context_t * const p_test = p_context;

    ++p_test->stop_calls;
    p_test->last_received_ms = received_ms;

    return p_test->b_deliver;
}

static bool fake_command (void * p_context,
    command_parser_command_t command, uint32_t received_ms)
{
    comms_test_context_t * const p_test = p_context;

    ++p_test->command_calls;
    p_test->last_command = command;
    p_test->last_received_ms = received_ms;

    return p_test->b_deliver;
}

static bool fake_snapshot (void * p_context,
    telemetry_encoder_snapshot_t * p_snapshot)
{
    comms_test_context_t * const p_test = p_context;
    bool b_copied = false;

    if (p_test->b_snapshot_ready)
    {
        *p_snapshot = p_test->snapshot;
        b_copied = true;
    }

    return b_copied;
}

static bool fake_assign (void * p_context, uint8_t route, char letter)
{
    comms_test_context_t * const p_test = p_context;

    ++p_test->assign_calls;
    p_test->last_route = route;
    p_test->last_letter = letter;

    return p_test->b_deliver;
}

/** @brief True if the payload parses to exactly this route and letter. */
static bool assign_is (const char * p_payload, size_t length,
                       uint8_t route, char letter)
{
    uint8_t parsed_route = 0u;
    char    parsed_letter = '\0';

    return command_parser_parse_assign(p_payload, length, &parsed_route,
                                       &parsed_letter) &&
           (route == parsed_route) && (letter == parsed_letter);
}

/** @brief True if the payload is rejected and the outputs are unchanged. */
static bool assign_rejects (const char * p_payload, size_t length)
{
    uint8_t route = 9u;
    char    letter = '?';

    return !command_parser_parse_assign(p_payload, length, &route,
                                        &letter) &&
           (9u == route) && ('?' == letter);
}

/** @brief Accumulate a failed check without skipping subsequent cases. */
static void check (bool b_condition, bool * p_passed)
{
    if (!b_condition)
    {
        *p_passed = false;
    }
}

#endif

/*** End of file. ***/


