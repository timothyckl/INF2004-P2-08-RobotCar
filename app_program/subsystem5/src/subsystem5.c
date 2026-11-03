#include "../include/subsystem5.h"
#include <stdint.h>

/*
 * ================================================================
 * Sensor configuration
 *
 * GP6  : Middle sensor follows the black line
 * GP16 : Left sensor normally reads the white area beside the line
 * GP26 : Side sensor reads the barcode
 *
 * Sensor calibration:
 *     Black = 1
 *     White = 0
 * ================================================================
 */

#define LINE_MIDDLE_PIN          6u
#define LINE_LEFT_PIN           16u
#define BARCODE_PIN             26u

/*
 * ================================================================
 * Motor pins
 * ================================================================
 */

#define LEFT_FORWARD_PIN        11u
#define LEFT_BACKWARD_PIN       10u
#define RIGHT_FORWARD_PIN        8u
#define RIGHT_BACKWARD_PIN       9u

/*
 * ================================================================
 * Controller tuning
 * ================================================================
 */

#define PWM_WRAP_VALUE         999u
#define PWM_MAX_SPEED          450

#define BASE_SPEED             240
#define SEARCH_SPEED           160

/*
 * Increase KP if the robot does not correct strongly enough.
 * Decrease KP if it turns too aggressively.
 *
 * Increase KD if the robot oscillates or overshoots.
 */
#define KP                      80
#define KD                      25

/*
 * Maximum time spent searching after both sensors see white.
 */
#define LOST_SEARCH_TIME_US     350000u

/*
 * Forget an old turning direction after remaining centred for
 * this number of samples.
 *
 * Task period = 10 ms
 * 10 samples = approximately 100 ms
 */
#define CENTRED_RESET_COUNT     10u

#define LAST_NONE                0
#define LAST_LEFT               -1
#define LAST_RIGHT               1

/*
 * ================================================================
 * RP2040 registers
 * ================================================================
 */

#define RESET_REG               0x4000C000u
#define RESET_DONE_REG          0x4000C008u
#define PWM_RESET_BIT           0x00004000u

#define PWM_BASE_ADDR           0x40050000u
#define PWM_SLICE_SIZE          0x14u

#define PWM_CSR_OFFSET          0x00u
#define PWM_DIV_OFFSET          0x04u
#define PWM_CTR_OFFSET          0x08u
#define PWM_CC_OFFSET           0x0Cu
#define PWM_TOP_OFFSET          0x10u

#define TIMER_RAW_LOW_REG       0x40054028u

/*
 * ================================================================
 * Register and timer functions
 * ================================================================
 */

static uint32_t register_read(uint32_t address)
{
    return *(volatile uint32_t *)(uintptr_t)address;
}

static void register_write(uint32_t address, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)address = value;
}

static uint32_t timer_get_us(void)
{
    return register_read(TIMER_RAW_LOW_REG);
}

static uint32_t timer_elapsed_us(
    uint32_t start_time,
    uint32_t current_time
)
{
    /*
     * Unsigned subtraction also handles the timer wrapping around.
     */
    return current_time - start_time;
}

/*
 * ================================================================
 * Motor functions
 * ================================================================
 */

static int clamp_signed_speed(int speed)
{
    if (speed > PWM_MAX_SPEED) {
        speed = PWM_MAX_SPEED;
    }

    if (speed < -PWM_MAX_SPEED) {
        speed = -PWM_MAX_SPEED;
    }

    return speed;
}

static int clamp_forward_speed(int speed)
{
    if (speed > PWM_MAX_SPEED) {
        speed = PWM_MAX_SPEED;
    }

    if (speed < 0) {
        speed = 0;
    }

    return speed;
}

/*
 * Positive value: forward
 * Negative value: backward
 * Zero: stop
 */
static void set_left_motor(int speed)
{
    speed = clamp_signed_speed(speed);

    if (speed > 0) {
        pwm_set_cc(LEFT_BACKWARD_PIN, 0u);
        pwm_set_cc(LEFT_FORWARD_PIN, (UW)speed);
    } else if (speed < 0) {
        pwm_set_cc(LEFT_FORWARD_PIN, 0u);
        pwm_set_cc(LEFT_BACKWARD_PIN, (UW)(-speed));
    } else {
        pwm_set_cc(LEFT_FORWARD_PIN, 0u);
        pwm_set_cc(LEFT_BACKWARD_PIN, 0u);
    }
}

static void set_right_motor(int speed)
{
    speed = clamp_signed_speed(speed);

    if (speed > 0) {
        pwm_set_cc(RIGHT_BACKWARD_PIN, 0u);
        pwm_set_cc(RIGHT_FORWARD_PIN, (UW)speed);
    } else if (speed < 0) {
        pwm_set_cc(RIGHT_FORWARD_PIN, 0u);
        pwm_set_cc(RIGHT_BACKWARD_PIN, (UW)(-speed));
    } else {
        pwm_set_cc(RIGHT_FORWARD_PIN, 0u);
        pwm_set_cc(RIGHT_BACKWARD_PIN, 0u);
    }
}

static void set_motor_speeds(
    int left_speed,
    int right_speed
)
{
    set_left_motor(left_speed);
    set_right_motor(right_speed);
}

static void stop_motors(void)
{
    set_motor_speeds(0, 0);
}

/*
 * Calculate and apply the digital PD correction.
 */
static int apply_pd_control(
    int error,
    int previous_error,
    int *left_speed,
    int *right_speed
)
{
    int derivative;
    int correction;

    derivative = error - previous_error;

    correction = (KP * error) +
                 (KD * derivative);

    /*
     * Positive correction:
     *     Slow left wheel
     *     Speed up right wheel
     *     Robot turns left
     */
    *left_speed = BASE_SPEED - correction;
    *right_speed = BASE_SPEED + correction;

    /*
     * Normal line following only drives forward.
     * Reversing is used only for lost-line recovery.
     */
    *left_speed = clamp_forward_speed(*left_speed);
    *right_speed = clamp_forward_speed(*right_speed);

    set_motor_speeds(
        *left_speed,
        *right_speed
    );

    return correction;
}

/*
 * ================================================================
 * PWM initialisation
 * ================================================================
 */

static ER initialise_pwm(void)
{
    uint32_t slice_address;
    uint32_t start_time;
    uint32_t current_time;
    UINT slice;
    ER error;

    /*
     * Release PWM hardware from reset.
     */
    register_write(
        RESET_REG,
        register_read(RESET_REG) & ~PWM_RESET_BIT
    );

    start_time = timer_get_us();

    while ((register_read(RESET_DONE_REG) &
            PWM_RESET_BIT) == 0u) {

        current_time = timer_get_us();

        if (timer_elapsed_us(
                start_time,
                current_time
            ) > 100000u) {
            return E_SYS;
        }

        tk_dly_tsk(1);
    }

    /*
     * GP8 and GP9 use PWM slice 4.
     * GP10 and GP11 use PWM slice 5.
     *
     * Divider = 125
     * TOP = 999
     * PWM frequency is approximately 1 kHz.
     */
    for (slice = 4u; slice <= 5u; ++slice) {
        slice_address =
            PWM_BASE_ADDR +
            (slice * PWM_SLICE_SIZE);

        register_write(
            slice_address + PWM_CSR_OFFSET,
            0u
        );

        register_write(
            slice_address + PWM_DIV_OFFSET,
            125u << 4
        );

        register_write(
            slice_address + PWM_CTR_OFFSET,
            0u
        );

        register_write(
            slice_address + PWM_CC_OFFSET,
            0u
        );

        register_write(
            slice_address + PWM_TOP_OFFSET,
            PWM_WRAP_VALUE
        );
    }

    error = pwm_set_pin(RIGHT_FORWARD_PIN);

    if (error < E_OK) {
        return error;
    }

    error = pwm_set_pin(RIGHT_BACKWARD_PIN);

    if (error < E_OK) {
        return error;
    }

    error = pwm_set_pin(LEFT_BACKWARD_PIN);

    if (error < E_OK) {
        return error;
    }

    error = pwm_set_pin(LEFT_FORWARD_PIN);

    if (error < E_OK) {
        return error;
    }

    stop_motors();

    /*
     * Enabling one pin enables the entire PWM slice.
     */
    error = pwm_set_enabled(
        RIGHT_FORWARD_PIN,
        1
    );

    if (error < E_OK) {
        return error;
    }

    error = pwm_set_enabled(
        LEFT_BACKWARD_PIN,
        1
    );

    if (error < E_OK) {
        return error;
    }

    return E_OK;
}

/*
 * ================================================================
 * Line-following and barcode task
 * ================================================================
 */

static void line_follow_task(
    INT stacd,
    void *exinf
)
{
    UINT left_sensor;
    UINT middle_sensor;
    UINT barcode_sensor;
    UINT previous_barcode;

    unsigned int print_counter;
    unsigned int centred_count;

    int error;
    int previous_error;
    int correction;

    int left_speed;
    int right_speed;
    int last_direction;

    uint32_t current_time;
    uint32_t lost_start_time;
    uint32_t barcode_change_time;
    uint32_t barcode_duration;

    BOOL lost_active;

    const char *action;

    (void)stacd;
    (void)exinf;

    error = 0;
    previous_error = 0;
    correction = 0;

    left_speed = 0;
    right_speed = 0;

    last_direction = LAST_NONE;
    centred_count = 0u;
    print_counter = 0u;

    lost_start_time = 0u;
    lost_active = 0;

    previous_barcode =
        gpio_get_val(BARCODE_PIN);

    barcode_change_time =
        timer_get_us();

    action = "WAITING";

    stop_motors();

    tm_printf(
        (UB *)"LINE: digital PD test starts in 3 seconds\n"
    );

    tm_printf(
        (UB *)"LINE: middle GP6, left GP16, barcode GP26\n"
    );

    tm_printf(
        (UB *)"LINE: raise the wheels for the first test\n"
    );

    tk_dly_tsk(3000);

    for (;;) {
        left_sensor =
            gpio_get_val(LINE_LEFT_PIN);

        middle_sensor =
            gpio_get_val(LINE_MIDDLE_PIN);

        barcode_sensor =
            gpio_get_val(BARCODE_PIN);

        current_time = timer_get_us();

        /*
         * --------------------------------------------------------
         * Barcode monitoring
         *
         * The barcode sensor never changes the motor direction.
         * It only records transitions and their durations.
         * --------------------------------------------------------
         */

        if (barcode_sensor != previous_barcode) {
            barcode_duration =
                timer_elapsed_us(
                    barcode_change_time,
                    current_time
                );

            tm_printf(
                (UB *)"BARCODE: %u -> %u, "
                      "previous duration=%u us\n",
                previous_barcode,
                barcode_sensor,
                (unsigned int)barcode_duration
            );

            previous_barcode = barcode_sensor;
            barcode_change_time = current_time;
        }

        /*
         * --------------------------------------------------------
         * State 1: Correct position
         *
         * Left sensor sees white.
         * Middle sensor sees black.
         * --------------------------------------------------------
         */

        if ((left_sensor == 0u) &&
            (middle_sensor == 1u)) {

            error = 0;
            lost_active = 0;

            if (centred_count <
                CENTRED_RESET_COUNT) {
                ++centred_count;
            } else {
                /*
                 * The robot has remained centred long enough
                 * that an old turning direction is no longer
                 * useful.
                 */
                last_direction = LAST_NONE;
            }

            correction = apply_pd_control(
                error,
                previous_error,
                &left_speed,
                &right_speed
            );

            previous_error = error;
            action = "FORWARD";
        }

        /*
         * --------------------------------------------------------
         * State 2: Line has moved towards the left sensor
         *
         * Turn left by slowing the left wheel and speeding up
         * the right wheel.
         * --------------------------------------------------------
         */

        else if ((left_sensor == 1u) &&
                 (middle_sensor == 0u)) {

            error = 2;
            last_direction = LAST_LEFT;
            centred_count = 0u;
            lost_active = 0;

            correction = apply_pd_control(
                error,
                previous_error,
                &left_speed,
                &right_speed
            );

            previous_error = error;
            action = "STEER LEFT";
        }

        /*
         * --------------------------------------------------------
         * State 3: Both sensors see black
         *
         * The line may be between both sensors, or the robot
         * may be crossing a wide line.
         *
         * Apply a smaller left correction.
         * --------------------------------------------------------
         */

        else if ((left_sensor == 1u) &&
                 (middle_sensor == 1u)) {

            error = 1;
            last_direction = LAST_LEFT;
            centred_count = 0u;
            lost_active = 0;

            correction = apply_pd_control(
                error,
                previous_error,
                &left_speed,
                &right_speed
            );

            previous_error = error;
            action = "BOTH BLACK";
        }

        /*
         * --------------------------------------------------------
         * State 4: Both sensors see white
         *
         * The line has moved away or has been completely lost.
         * Search using the previously detected direction.
         * --------------------------------------------------------
         */

        else {
            error = 0;
            correction = 0;
            centred_count = 0u;
            previous_error = 0;

            if (lost_active == 0) {
                lost_active = 1;
                lost_start_time = current_time;

                /*
                 * If the middle sensor lost the line without the
                 * left sensor detecting black, assume that the
                 * line moved towards the right.
                 */
                if (last_direction == LAST_NONE) {
                    last_direction = LAST_RIGHT;
                }
            }

            if (timer_elapsed_us(
                    lost_start_time,
                    current_time
                ) < LOST_SEARCH_TIME_US) {

                if (last_direction == LAST_LEFT) {
                    left_speed = -SEARCH_SPEED;
                    right_speed = SEARCH_SPEED;
                    action = "SEARCH LEFT";
                } else {
                    left_speed = SEARCH_SPEED;
                    right_speed = -SEARCH_SPEED;
                    action = "SEARCH RIGHT";
                }

                set_motor_speeds(
                    left_speed,
                    right_speed
                );
            } else {
                left_speed = 0;
                right_speed = 0;

                stop_motors();

                action = "LOST - STOP";
            }
        }

        /*
         * Print the line-following state approximately every
         * 200 ms.
         */
        ++print_counter;

        if (print_counter >= 20u) {
            tm_printf(
                (UB *)"LINE: LEFT=%u MIDDLE=%u "
                      "BARCODE=%u error=%d "
                      "correction=%d "
                      "motorL=%d motorR=%d -> %s\n",
                left_sensor,
                middle_sensor,
                barcode_sensor,
                error,
                correction,
                left_speed,
                right_speed,
                action
            );

            print_counter = 0u;
        }

        tk_dly_tsk(10);
    }
}

/*
 * ================================================================
 * Subsystem functions
 * ================================================================
 */

EXPORT ER subsystem5_init(void)
{
    static const UINT motor_pins[] = {
        RIGHT_FORWARD_PIN,
        RIGHT_BACKWARD_PIN,
        LEFT_BACKWARD_PIN,
        LEFT_FORWARD_PIN
    };

    static const UINT sensor_pins[] = {
        LINE_MIDDLE_PIN,
        LINE_LEFT_PIN,
        BARCODE_PIN
    };

    unsigned int index;
    ER error;

    /*
     * Set the motor output latches low before enabling PWM.
     */
    for (index = 0u; index < 4u; ++index) {
        error = gpio_set_val(
            motor_pins[index],
            0u
        );

        if (error < E_OK) {
            return error;
        }

        error = gpio_set_pin(
            motor_pins[index],
            GPIO_MODE_OUT
        );

        if (error < E_OK) {
            return error;
        }
    }

    /*
     * Configure the three IR digital outputs as GPIO inputs.
     */
    for (index = 0u; index < 3u; ++index) {
        error = gpio_set_pin(
            sensor_pins[index],
            GPIO_MODE_IN
        );

        if (error < E_OK) {
            return error;
        }
    }

    return initialise_pwm();
}

EXPORT ER subsystem5_start(void)
{
    ID task_id;

    task_id = app_start_task(
        (FP)line_follow_task,
        S5_PRI_MAIN,
        0,
        "line_follow"
    );

    if (task_id < E_OK) {
        return task_id;
    }

    return E_OK;
}