/**
 * @file kernel_model.h
 * @brief Test controls for the host kernel model (kernel_model.c).
 */
#ifndef HOST_KERNEL_MODEL_H
#define HOST_KERNEL_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

/** Forget every object, the clock and the console text. */
void model_reset (void);
/** Forget the console text kept so far. */
void model_console_clear (void);
/** True if the console text since the last clear contains p_text. */
bool model_console_has (char const * p_text);
/** Messages waiting in a message buffer. */
INT  model_mbf_count (ID mbfid);
/** Current pattern of an event flag. */
UINT model_flag_pattern (ID flgid);
/** Inputs that read low on GPIO_IN (a pressed button), as a GPIO mask. */
void model_gpio_low (UW mask);

#endif /* HOST_KERNEL_MODEL_H */

/* End of file. */
