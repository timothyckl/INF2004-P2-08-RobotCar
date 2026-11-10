/**
 * @file s2_host_main.c
 * @brief Runs subsystem 2's pure-logic unit cases (test_motion.c,
 *        test_controller.c, test_line_control.c) on the host. The same functions run on the
 *        Pico W in the TEST=1 image; this runner adds no cases of its own.
 */
#include <stdbool.h>
#include <stdio.h>

#include "../../app_program/subsystem2/test_controller.h"
#include "../../app_program/subsystem2/test_line_control.h"
#include "../../app_program/subsystem2/test_motion.h"
#include "../../app_program/subsystem2/motion_cmd.h"

/* Created by task_motion.c on the Pico W; 0 here means "not created", so
   the motion API takes its E_OBJ path. */
ID motion_q = 0;

typedef struct
{
    char const * p_id;
    bool         (*p_case)(void);
} host_case_t;

static host_case_t const g_cases[] =
{
    {"S2-U20", test_motion_start},
    {"S2-U21", test_motion_speed},
    {"S2-U22", test_motion_move},
    {"S2-U23", test_motion_turn},
    {"S2-U24", test_motion_straight},
    {"S2-U25", test_motion_stop_all},
    {"S2-U26", test_motion_reject_stop},
    {"S2-U27", test_motion_replace},
    {"S2-U28", test_motion_stop_timing},
    {"S2-U29", test_motion_stall},
    {"S2-U30", test_motion_timeout},
    {"S2-U31", test_motion_encoder_fault},
    {"S2-U32", test_motion_recovery},
    {"S2-U33", test_motion_params},
    {"S2-U34", test_motion_odometry},
    {"S2-U35", test_motion_api},
    {"S2-U55", test_motion_ramp},
    {"S2-U56", test_motion_ramp_stall},
    {"S2-U41", test_ctrl_boot},
    {"S2-U42", test_ctrl_init_timeout},
    {"S2-U43", test_ctrl_start},
    {"S2-U44", test_ctrl_script},
    {"S2-U45", test_ctrl_stop_running},
    {"S2-U46", test_ctrl_stop_other},
    {"S2-U47", test_ctrl_faults},
    {"S2-U48", test_ctrl_reset_stopped},
    {"S2-U49", test_ctrl_reset_fault},
    {"S2-U50", test_ctrl_remote_rules},
    {"S2-U51", test_ctrl_stop_queue_full},
    {"S2-U52", test_ctrl_only_start_clears},
    {"S2-U53", test_ctrl_robot_state},
    {"S2-U58", test_line_control_junction},
    {"S2-U59", test_line_control_curve},
};

int
main (void)
{
    unsigned passed = 0u;
    unsigned failed = 0u;

    for (size_t i = 0u; i < (sizeof(g_cases) / sizeof(g_cases[0])); i++)
    {
        bool const b_ok = g_cases[i].p_case();

        (void)printf("[HOST] %s %s\n", g_cases[i].p_id, b_ok ? "PASS" : "FAIL");

        if (b_ok)
        {
            passed++;
        }
        else
        {
            failed++;
        }
    }

    (void)printf("[HOST] subsystem2 SUMMARY %u passed, %u failed\n", passed,
                 failed);

    return ((0u == failed) ? 0 : 1);
}

/* End of file. */
