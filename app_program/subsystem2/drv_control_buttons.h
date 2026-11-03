/**
 * @file drv_control_buttons.h
 * @brief Robo Pico user-button GPIO driver.
 */

#ifndef DRV_CONTROL_BUTTONS_H
#define DRV_CONTROL_BUTTONS_H

#include "../app.h"

#include <stdbool.h>

ER drv_control_buttons_init(void);
ER drv_control_buttons_read(bool * p_start_pressed,
                            bool * p_stop_pressed);

#endif /* DRV_CONTROL_BUTTONS_H */

/* End of file. */
