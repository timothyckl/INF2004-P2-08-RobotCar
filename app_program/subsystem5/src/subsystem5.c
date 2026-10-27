/*
 *  subsystem5.c
 *  Application lifecycle compatibility hooks for obstacle scanning.
 *
 *  Production scan logic is implemented in focused modules and exercised
 *  through injected test ports. No task or hardware is started here until
 *  the controller interfaces and servo calibration have been agreed.
 */

#include "../include/subsystem5.h"

EXPORT ER
subsystem5_init (void)
{
    return E_OK;
}

EXPORT ER
subsystem5_start (void)
{
    return E_OK;
}

/* End of file. */
