/**
 * @file s2_bridge.h
 * @brief The S1 adapters' bodies, taking object IDs as arguments so tests
 *        can run them against their own queue and flag. PRIVATE.
 */
#ifndef S2_BRIDGE_H
#define S2_BRIDGE_H

#include "subsystem2.h"

bool s2_bridge_raise_stop (ID flg, uint32_t received_ms);
bool s2_bridge_send_command (ID q, uint8_t command, uint32_t request_id,
                             uint32_t received_ms);

#endif /* S2_BRIDGE_H */

/* End of file. */
