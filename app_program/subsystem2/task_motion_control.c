/**
 * @file task_motion_control.c
 * @brief Temporary demo motion task for IR line-following validation.
 */

#include "subsystem2.h"

#include "drv_control_buttons.h"
#include "drv_motor.h"
#include "../subsystem3/line_sensor.h"

#include <stdbool.h>
#include <stdint.h>

#define S2_MOTION_PERIOD_MSEC      (10U)
#define S2_MOTION_STALE_LIMIT      (5U)
#define S2_IR_REPORT_DIVIDER       (25U)
#define S2_BUTTON_DEBOUNCE_SAMPLES (5U)

EXPORT void
motion_control_task (INT start_code, void * p_extension)
{
    line_sensor_snapshot_t snapshot = {0};
    uint32_t previous_sample_count = 0U;
    uint32_t calibration_sample_count = 0U;
    uint16_t ir_report_count = 0U;
    uint8_t stale_count = 0U;
    uint8_t start_pressed_count = 0U;
    uint8_t stop_pressed_count = 0U;
    bool b_start_pressed = false;
    bool b_stop_pressed = false;
    bool b_calibrated = false;
    bool b_start_latched = false;
    bool b_stop_latched = false;
    bool b_running = false;
    ER error = E_OK;

    (void)start_code;
    (void)p_extension;

    error = drv_motor_init();
    if (E_OK == error)
    {
        error = drv_control_buttons_init();
    }
    if (E_OK != error)
    {
        CHECK(error, "motion_control_init");
        tk_ext_tsk();
        return;
    }

    for (;;)
    {
        error = line_sensor_get_snapshot(&snapshot);
        if (E_OK == error)
        {
            error = drv_control_buttons_read(&b_start_pressed,
                                             &b_stop_pressed);
        }

        if (b_start_pressed && (!b_stop_latched))
        {
            if (S2_BUTTON_DEBOUNCE_SAMPLES > start_pressed_count)
            {
                start_pressed_count++;
            }
            if ((S2_BUTTON_DEBOUNCE_SAMPLES <= start_pressed_count) &&
                (!b_start_latched) && (0U != snapshot.sample_count))
            {
                error = line_sensor_set_calibration(
                    !snapshot.b_left_raw,
                    !snapshot.b_right_raw);
                if (E_OK == error)
                {
                    calibration_sample_count = snapshot.sample_count;
                    b_calibrated = true;
                    b_start_latched = true;
                }
            }
        }
        else
        {
            start_pressed_count = 0U;
        }

        if (b_stop_pressed)
        {
            if (S2_BUTTON_DEBOUNCE_SAMPLES > stop_pressed_count)
            {
                stop_pressed_count++;
            }
            if (S2_BUTTON_DEBOUNCE_SAMPLES <= stop_pressed_count)
            {
                b_stop_latched = true;
                b_start_latched = false;
            }
        }
        else
        {
            stop_pressed_count = 0U;
        }

        if ((E_OK == error) &&
            (snapshot.sample_count != previous_sample_count))
        {
            previous_sample_count = snapshot.sample_count;
            stale_count = 0U;
        }
        else if (UINT8_MAX > stale_count)
        {
            stale_count++;
        }
        else
        {
            /* Saturated stale counter. */
        }

        if ((E_OK != error) || snapshot.b_sensor_fault ||
            (0U == snapshot.sample_count) ||
            (S2_MOTION_STALE_LIMIT <= stale_count) ||
            (!b_calibrated) ||
            (calibration_sample_count == snapshot.sample_count) ||
            (!b_start_latched) || b_stop_latched)
        {
            if (b_running)
            {
                CHECK(drv_motor_stop(), "drv_motor_stop");
                b_running = false;
            }
        }
        else
        {
            error = drv_motor_set_forward(snapshot.control.left_target,
                                          snapshot.control.right_target);
            if (E_OK != error)
            {
                CHECK(error, "drv_motor_set_forward");
                CHECK(drv_motor_stop(), "drv_motor_stop");
                b_running = false;
            }
            else
            {
                b_running = true;
            }
        }

        if (b_calibrated &&
            (calibration_sample_count != snapshot.sample_count))
        {
            ir_report_count++;
            if (S2_IR_REPORT_DIVIDER <= ir_report_count)
            {
                ir_report_count = 0U;
                tm_printf((UB*)"[IR] LEFT=%s RIGHT=%s\n",
                          snapshot.b_left_black ? "BLACK" : "WHITE",
                          snapshot.b_right_black ? "BLACK" : "WHITE");
            }
        }
        else
        {
            ir_report_count = 0U;
        }

        tk_dly_tsk(S2_MOTION_PERIOD_MSEC);
    }
}

/* End of file. */
