/**
 * @file subsystem3.c
 * @brief Initialise the subsystem 3 sensor inputs.
 *
 * Subsystem 3 has no task of its own: the mission runs s3_follow_run()
 * in the initial task (s3_follow.h).
 */

#include "subsystem3.h"
#include "s3_ir_gpio.h"

EXPORT ER
subsystem3_init (void)
{
    /* Inputs only: subsystem 3 never commands either motor. */
    return (s3_ir_gpio_init());
}

EXPORT ER
subsystem3_start (void)
{
    return (E_OK);
}

/* End of file. */
