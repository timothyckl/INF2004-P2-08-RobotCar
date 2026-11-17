/**
 * @file tmonitor.h
 * @brief Host stand-in for <tm/tmonitor.h>: tm_printf() prints to stdout.
 */
#ifndef HOST_TMONITOR_H
#define HOST_TMONITOR_H

#include <tk/tkernel.h>

INT tm_printf (UB const * p_format, ...);

#endif /* HOST_TMONITOR_H */

/* End of file. */
