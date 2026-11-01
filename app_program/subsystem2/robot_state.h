/**
 * @file robot_state.h
 * @brief robot_state store. PRIVATE to subsystem 2: only the Vehicle
 *        Controller publishes; readers use subsystem2.h.
 */
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "subsystem2.h"

/* Create state_mutex (priority inheritance) and set the boot state:
   Initialising, STOP latched. */
ER   s2_robot_state_create (void);

/* Replace the published copy (Vehicle Controller task only). */
void s2_robot_state_publish (s2_robot_state_t const * p_state);

#endif /* ROBOT_STATE_H */

/* End of file. */
