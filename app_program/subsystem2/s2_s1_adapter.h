/**
 * @file s2_s1_adapter.h
 * @brief Subsystem 2 functions for subsystem 1's service_bridge.
 *
 * Kernel-free on purpose: only <stdbool.h>, <stdint.h> and S1's
 * kernel-free telemetry_snapshot.h. S1's other headers include <stddef.h>,
 * whose size_t (unsigned int) conflicts with the kernel's (long int,
 * include/tk/syslib.h), so S1 code that includes service_bridge.h cannot
 * also include subsystem2.h.
 *
 * All functions run in the WiFi service task, never wait, and return
 * false until subsystem2_init() has created the objects (the WiFi task
 * starts before usermain()). Matching S1 typedefs (service_bridge.h):
 *   controller_raise_remote_stop   -> service_bridge_stop_fn_t
 *   controller_send_remote_command -> service_bridge_command_fn_t
 *                                     (uint8_t = command_parser_command_t)
 *   robot_read_telemetry           -> service_bridge_snapshot_fn_t
 */
#ifndef S2_S1_ADAPTER_H
#define S2_S1_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem1/telemetry_snapshot.h"

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

/* Fill S1's telemetry snapshot (S1 decision 1, telemetry v2) from
   robot_state, S4's run maximum (IF-18) and S5's decoded status. Every
   lock is polled (TMO_POL), so this never waits. false (skip this
   publication) if robot_state is not created yet or state_mutex is busy;
   a busy S4 or S5 lock clears only that provider's validity flags.
   link_up is left false: S1 owns its link state. */
bool robot_read_telemetry (void * p_context,
                           telemetry_encoder_snapshot_t * p_snapshot);

#endif /* S2_S1_ADAPTER_H */

/* End of file. */
