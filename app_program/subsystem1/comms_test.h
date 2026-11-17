/** @file comms_test.h
 * @brief Portable logic test groups for host and on-target execution.
 */
#ifndef COMMS_TEST_H
#define COMMS_TEST_H

#include <stdbool.h>

/** @brief Return true if bounded command parsing passes all cases. */
bool comms_test_commands (void);

/** @brief Return true if telemetry bounds and golden bytes match. */
bool comms_test_telemetry (void);

/** @brief Return true if retry and heartbeat scheduling pass all cases. */
bool comms_test_timers (void);

/** @brief Return true if fragmented MQTT bridge routing passes all cases. */
bool comms_test_bridge (void);

#endif /* COMMS_TEST_H */

/*** End of file. ***/

