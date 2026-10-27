/**
 * @file scan_lifecycle.h
 * @brief Shared application declarations for obstacle scanning.
 *
 * Cross-subsystem objects and messages remain intentionally absent until
 * their contracts are agreed with the controller, motion and line owners.
 */

#ifndef SCAN_LIFECYCLE_H
#define SCAN_LIFECYCLE_H

#include "../../app.h"

#define OBSTACLE_SCAN_PRIORITY 20

/** Initialize agreed subsystem objects without starting tasks. */
IMPORT ER scan_lifecycle_init (void);

/** Start agreed subsystem tasks after initialization. */
IMPORT ER scan_lifecycle_start (void);

#if APP_TEST
/** Run the isolated obstacle-scanning cases in test firmware. */
IMPORT void scan_lifecycle_test (void);
#endif

#endif /* SCAN_LIFECYCLE_H */

/* End of file. */
