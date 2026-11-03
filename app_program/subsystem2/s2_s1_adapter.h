/**
 * @file s2_s1_adapter.h
 * @brief Subsystem 2 functions for subsystem 1's service_bridge.
 *
 * PROVISIONAL. Kernel-free on purpose: only <stdbool.h> and <stdint.h>.
 * S1's headers include <stddef.h>, whose size_t (unsigned int) conflicts
 * with the kernel's (long int, include/tk/syslib.h), so S1 code that
 * includes service_bridge.h cannot also include subsystem2.h.
 *
 * All functions run in the WiFi service task, never wait, and return
 * false until subsystem2_init() has created the objects (the WiFi task
 * starts before usermain()). Matching S1 typedefs on hy_wifi_telemetry:
 *   controller_raise_remote_stop   -> service_bridge_stop_fn_t
 *   controller_send_remote_command -> service_bridge_command_fn_t
 *                                     (uint8_t = command_parser_command_t)
 *   robot_read_telemetry           -> service_bridge_snapshot_fn_t
 */
#ifndef S2_S1_ADAPTER_H
#define S2_S1_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

/* Command codes, equal to COMMAND_PARSER_START/STOP/RESET. */
#define S2_REMOTE_START         1u
#define S2_REMOTE_STOP          2u
#define S2_REMOTE_RESET         3u

/* Records received_ms, then sets CTRL_FLG_REMOTE_STOP in
   controller_flags. true means the urgent flag is set, not that the
   wheels have stopped. Never uses remote_q. */
bool controller_raise_remote_stop (void * p_context, uint32_t received_ms);

/* START or RESET only, sent to remote_q with TMO_POL. false if the
   command is anything else, or remote_q is full or not created; the
   command is then dropped, never retried. Until S1 forwards its request
   ID, the adapter numbers accepted commands 1, 2, 3, ... itself. */
bool controller_send_remote_command (void * p_context, uint8_t command,
                                     uint32_t received_ms);

/* Latest remote-STOP receipt time and how many were received, for the
   receipt-to-flag latency test. The pair is not read atomically. */
bool s2_remote_stop_info (uint32_t * p_received_ms, uint32_t * p_count);

/* The telemetry_encoder_snapshot_t fields S2 provides (link_up is S1's). */
typedef struct
{
    uint32_t captured_ms;
    uint16_t distance_mm;       /* run distance, clamped to 0..65535 */
    bool     b_distance_valid;  /* false if negative, over-range or unknown */
    bool     b_stopped;         /* both wheels confirmed still */
} s2_telemetry_t;

/* One consistent copy from robot_state; false (skip this publication)
   if not created yet or state_mutex is busy. Never waits. */
bool s2_read_telemetry (s2_telemetry_t * p_out);

/* The controller's clock (tk_get_tim, low 32 bits, ms) for the
   received_ms of remote commands, so command age is judged on the same
   clock the controller uses. lwIP's sys_now() is operating time, which
   need not match. */
uint32_t s2_clock_ms (void);

#ifdef TELEMETRY_ENCODER_H
/* Available when S1's telemetry_encoder.h is included first. */
bool robot_read_telemetry (void * p_context,
                           telemetry_encoder_snapshot_t * p_snapshot);
#endif

#endif /* S2_S1_ADAPTER_H */

/* End of file. */
