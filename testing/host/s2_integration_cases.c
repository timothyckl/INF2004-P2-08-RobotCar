/**
 * @file s2_integration_cases.c
 * @brief INT-08 onwards: the cases run by s2_integration_main.c, which
 *        includes this file (it uses that file's simulation directly).
 *        Each case is defined in testing/integration/TEST_LOG.md.
 */

/* Junction 1.5 m from the start, every branch present. */
static sim_track_t const g_cross = {1500, true, true, true, 0, 0u};

static void int_08_remote_stop (void);
static void int_09_line_reports (void);
static void int_10_junction_commands (void);
static void int_11_hold_through_scan (void);
static void int_12_pickup_drops_command (void);
static void int_13_barcode_records (void);
static void int_14_scan_failure (void);
static void int_15_task_stall (void);
static void int_16_telemetry_v5 (void);
static void int_17_printed_code (void);
static void int_18_corrupted_code (void);
static void int_19_repeated_codes (void);
static void int_20_stop_drops_code (void);
static void int_21_obstacle_bypass (void);
static void int_22_stop_in_bypass (void);
static void int_23_bypass_no_line (void);
static void int_24_terrain_gate (void);
static void int_25_stop_in_turn (void);
static void int_26_fault_in_turn (void);
static void int_27_long_straight (void);
static void int_28_forward_fault (uint8_t fault, char const * p_name);
static void int_29_scan_silent (void);
static void int_30_look_silent (void);
static void int_31_motion_class (void);
static bool int_class_seen (uint8_t mode, uint8_t state);
static bool int_start_turn (uint8_t command);
static bool int_bypass_mission (int32_t width_mm, int32_t depth_mm,
                                char const * p_name);
static bool int_in_bypass (void);
static void int_drive_past (int32_t x_mm, uint32_t limit_ms);
static bool int_crossed_straight (void);
static bool int_frame (uint8_t * p_frame);
static bool int_turned_right (void);
static bool int_turn_to (uint8_t command, int32_t * p_heading_mdeg,
                         uint32_t * p_navigate_periods);
static void int_send_remote (uint8_t command);
static bool int_stopped_latched (uint8_t cause);
static bool int_wheels_still (void);

static void
s2_integration_cases (void)
{
    int_08_remote_stop();
    int_09_line_reports();
    int_10_junction_commands();
    int_11_hold_through_scan();
    int_12_pickup_drops_command();
    int_13_barcode_records();
    int_14_scan_failure();
    int_15_task_stall();
    int_16_telemetry_v5();
    int_17_printed_code();
    int_18_corrupted_code();
    int_19_repeated_codes();
    int_20_stop_drops_code();
    int_21_obstacle_bypass();
    int_22_stop_in_bypass();
    int_23_bypass_no_line();
    int_24_terrain_gate();
    int_25_stop_in_turn();
    int_26_fault_in_turn();
    int_27_long_straight();
    int_28_forward_fault(SIM_FWD_INVALID, "invalid echoes");
    int_28_forward_fault(SIM_FWD_SILENT, "stale (no readings)");
    int_29_scan_silent();
    int_30_look_silent();
    int_31_motion_class();
}

/* INT-08 (S1 -> S2): MQTT STOP through the bridge latches STOP at once;
   the wheels stop and get no duty while it is latched, however long line
   reports keep coming; START alone does not clear it; RESET then START
   does. */
static void
int_08_remote_stop (void)
{
    bool b_ok;

    sim_init(&g_cross);
    sim_start();
    sim_run_ms(2000u);
    b_ok = sim_running() && (sim_x_mm() > 40);

    controller_raise_remote_stop(NULL, g_sim.now_ms);
    sim_step();
    b_ok = b_ok && int_stopped_latched(S2_STOP_REMOTE);
    sim_check(b_ok, "INT-08", "remote STOP latches in one period");

    sim_run_ms(3000u);
    sim_check(int_wheels_still() && (0 == g_sim.duty[0])
              && (0 == g_sim.duty[1])
              && (g_sim.max_speed_after_stop < 60000),
              "INT-08", "no duty while latched; wheels still within 3 s");

    int_send_remote(S2_REMOTE_START);
    sim_run_ms(500u);
    sim_check(int_stopped_latched(S2_STOP_REMOTE), "INT-08",
              "START alone does not clear a latched STOP");

    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    b_ok = sim_run_until(sim_running, 1000u);
    sim_run_ms(1000u);
    sim_check(b_ok && sim_following(), "INT-08",
              "RESET then START runs again");
}

/* INT-09 (S3 -> S2): centred reports drive both wheels at the line speed;
   reports that stop arriving latch STOP with cause line within the 500 ms
   time-out; line_q holds six reports (S3 decision 10). */
static void
int_09_line_reports (void)
{
    s2_robot_state_t state;
    s2_line_report_t report;
    uint8_t accepted = 0u;

    sim_init(&g_cross);
    sim_start();
    sim_run_ms(3000u);
    sim_state(&state);
    sim_check((S2_LINE_CENTER == state.line)
              && (abs(state.left_mm_s - CTRL_LINE_BASE_MM_S) <= 2)
              && (abs(state.right_mm_s - CTRL_LINE_BASE_MM_S) <= 2),
              "INT-09", "centred reports drive both wheels at 40 mm/s");

    g_sim.b_reporting = false;      /* the S3 loop stops */
    sim_run_ms(400u);
    sim_check(sim_running(), "INT-09", "a 400 ms gap is tolerated");
    sim_run_ms(300u);
    sim_check(int_stopped_latched(S2_STOP_LINE), "INT-09",
              "no report for over 500 ms latches STOP (cause line)");

    /* Queue capacity: drain, then fill without the controller running. */
    while (tk_rcv_mbf(line_q, &report, TMO_POL) > 0)
    {
        /* Empty it. */
    }
    for (uint8_t idx = 0u; idx < 8u; idx++)
    {
        accepted = (uint8_t)(accepted
                   + (s2_line_submit_scaled(S2_LINE_CENTER, 0u, 0u,
                                            g_sim.now_ms) ? 1u : 0u));
    }
    sim_check(6u == accepted, "INT-09",
              "line_q holds six reports, then refuses (180 ms at 30 ms)");
}

/* INT-10 (S3 -> S2 -> Motion): each barcode command at a + junction 1.5 m
   on: RIGHT and LEFT stop, step onto the junction and spin onto the
   branch, then follow it; STRAIGHT crosses without stopping; U-TURN spins
   back onto the line behind. The command was decoded about 35 s before. */
static void
int_10_junction_commands (void)
{
    int32_t heading = 0;
    uint32_t navigate = 0u;
    bool b_ok;

    b_ok = int_turn_to(S2_NAV_RIGHT, &heading, &navigate);
    sim_check(b_ok && (navigate > 0u) && (abs(heading + 90000) < 20000),
              "INT-10", "RIGHT: NAVIGATE, then following at -90 degrees");

    b_ok = int_turn_to(S2_NAV_LEFT, &heading, &navigate);
    sim_check(b_ok && (navigate > 0u) && (abs(heading - 90000) < 20000),
              "INT-10", "LEFT: NAVIGATE, then following at +90 degrees");

    b_ok = int_turn_to(S2_NAV_UTURN, &heading, &navigate);
    sim_check(b_ok && (navigate > 0u) && (abs(heading) > 160000),
              "INT-10", "U-TURN: NAVIGATE, then following back");

    b_ok = int_turn_to(S2_NAV_STRAIGHT, &heading, &navigate);
    sim_check(b_ok && (0u == navigate) && (abs(heading) < 5000)
              && (sim_x_mm() > (g_cross.junction_mm + 200)),
              "INT-10", "STRAIGHT: no stop, crosses the junction");
}

/* INT-11 (S3 <-> S5 through S2): RIGHT decoded 0.6 m in; an obstacle at
   1.0 m is scanned (CLEAR after 5 s); the junction is 3.0 m in, so it is
   reached about 70 s after the decode (on demo day the obstacle came 58 s
   after it and the 60 s hold expired during the scan). The scan must wait
   for the wheels to stop, and the held command must survive it. */
static void
int_11_hold_through_scan (void)
{
    sim_track_t const track = {3000, false, true, true, 1000,
                               SCAN_TASK_RESULT_CLEAR};
    bool b_ok;

    sim_init(&track);
    sim_start();
    while ((sim_x_mm() < 600) && sim_running())
    {
        sim_step();
    }
    (void)s2_nav_submit(S2_NAV_RIGHT, 'O', 1u, g_sim.now_ms);
    b_ok = sim_run_until(int_turned_right, 150000u);

    sim_check((1u == g_sim.authorise_count) && g_sim.b_still_at_authorise,
              "INT-11", "scan authorised once, only with the wheels still");
    sim_check(g_sim.scan_periods >= (SIM_SCAN_MS / SIM_PERIOD_MS),
              "INT-11", "SCAN mode for the whole scan");
    sim_check(b_ok && (abs(sim_heading_mdeg() + 90000) < 20000)
              && ((g_sim.now_ms - 1000u) > 60000u),
              "INT-11", "RIGHT held through the scan, turned at 3.0 m");
}

/* INT-12 (S4 -> S2): a pick-up latches STOP with cause pick-up and drops
   the held command, so after RESET and START the junction is crossed. */
static void
int_12_pickup_drops_command (void)
{
    bool b_ok;

    sim_init(&g_cross);
    sim_start();
    sim_run_ms(2000u);
    (void)s2_nav_submit(S2_NAV_RIGHT, 'O', 1u, g_sim.now_ms);
    sim_run_ms(1000u);
    b_ok = (NAV_CONTROL_PENDING == g_nav.state);
    (void)tk_set_flg(controller_flags, CTRL_FLG_PICKUP);
    sim_step();
    sim_check(b_ok && int_stopped_latched(S2_STOP_PICKUP)
              && (NAV_CONTROL_IDLE == g_nav.state),
              "INT-12", "pick-up latches STOP and drops the held command");

    sim_run_ms(2000u);
    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    (void)sim_run_until(sim_running, 1000u);
    g_sim.navigate_periods = 0u;
    while ((sim_x_mm() < (g_cross.junction_mm + 300)) && sim_running())
    {
        sim_step();
    }
    sim_check(sim_following() && (0u == g_sim.navigate_periods),
              "INT-12", "after restart the junction is crossed, no turn");
}

/* INT-13 (S3 -> S2 -> S1): every letter reaches the published state
   (telemetry); an invalid record is ignored; a letter with no command is
   reported but not held; a letter during a manoeuvre is dropped. */
static void
int_13_barcode_records (void)
{
    s2_robot_state_t state;
    s2_nav_request_t bad = {7u, 0u, S2_NAV_LEFT, 'a', {0u, 0u}};
    bool b_ok;

    sim_init(&g_cross);
    sim_start();
    (void)s2_nav_submit(S2_NAV_NONE, 'Q', 1u, g_sim.now_ms);
    sim_step();
    sim_state(&state);
    b_ok = (0u != (state.flags & S2_RS_BARCODE_VALID))
           && ('Q' == state.barcode) && (S2_NAV_NONE == state.barcode_command)
           && (1u == state.barcode_count)
           && (NAV_CONTROL_IDLE == g_nav.state);
    sim_check(b_ok, "INT-13", "unmapped letter reported, not held");

    (void)tk_snd_mbf(nav_q, &bad, (INT)sizeof(bad), TMO_POL);
    sim_step();
    sim_state(&state);
    sim_check(('Q' == state.barcode) && (1u == state.barcode_count)
              && (NAV_CONTROL_IDLE == g_nav.state)
              && model_console_has("nav: invalid request ignored"),
              "INT-13", "invalid record ignored, telemetry unchanged");

    (void)s2_nav_submit(S2_NAV_RIGHT, 'O', 2u, g_sim.now_ms);
    (void)s2_nav_submit(S2_NAV_LEFT, 'M', 3u, g_sim.now_ms);
    sim_step();
    sim_state(&state);
    sim_check(('M' == state.barcode) && (S2_NAV_LEFT == state.barcode_command)
              && (3u == state.barcode_count)
              && (NAV_CONTROL_CMD_LEFT == g_nav.command),
              "INT-13", "a newer letter replaces the held command");

    b_ok = sim_run_until(sim_navigating, 60000u);
    (void)s2_nav_submit(S2_NAV_RIGHT, 'O', 4u, g_sim.now_ms);
    sim_step();
    sim_state(&state);
    sim_check(b_ok && ('O' == state.barcode)
              && (NAV_CONTROL_CMD_LEFT == g_nav.command)
              && model_console_has("RIGHT dropped (manoeuvre in progress)"),
              "INT-13", "a letter during a manoeuvre is reported, dropped");
}

/* INT-14 (S5 -> S2): a scan with no route latches STOP with cause scan
   and the car does not move again. */
static void
int_14_scan_failure (void)
{
    sim_track_t const track = {0, false, false, true, 800,
                               SCAN_TASK_RESULT_NO_ROUTE};

    sim_init(&track);
    sim_start();
    (void)sim_run_until(int_wheels_still, 60000u);
    sim_run_ms(SIM_SCAN_MS + 1000u);
    sim_check(int_stopped_latched(S2_STOP_SCAN)
              && g_sim.b_still_at_authorise, "INT-14",
              "no route: STOP latched with cause scan");
    sim_run_ms(3000u);
    sim_check(int_wheels_still() && (0 == g_sim.duty[0])
              && (0 == g_sim.duty[1]), "INT-14",
              "the car stays still after a failed scan");
}

/* INT-15 (S2 decision 37, all tasks -> Motion Control): the controller
   blocks for 600 ms while the car follows the line. Motion Control's
   liveness guard stops the motors within the 500 ms limit plus one
   period, without the controller, and holds them at 0; when the
   controller runs again it enters FAULT with STOP latched (cause fault,
   class motion); RESET and START run again. A stalled S3 loop instead
   stops the car through the controller's own 500 ms line time-out first
   (cause line, INT-09), because the S3 loop's liveness limit is longer. Tasks that never check in (here S4 and S5) are not
   monitored, so a long healthy run never trips. */
static void
int_15_task_stall (void)
{
    uint32_t blocked_at;
    uint32_t stopped_after = 0u;
    bool b_ok;

    sim_init(&g_cross);
    sim_start();
    sim_run_ms(10000u);
    sim_check(sim_following() && (MOTION_FAULT != motion_state(&g_sim.motion)),
              "INT-15", "10 s healthy run: no stall reported");

    g_sim.b_controller_blocked = true;
    blocked_at = g_sim.now_ms;
    while ((g_sim.now_ms - blocked_at) < 600u)
    {
        sim_step();
        if ((0u == stopped_after) && (0 == g_sim.duty[0])
            && (0 == g_sim.duty[1]))
        {
            stopped_after = g_sim.now_ms - blocked_at;
        }
    }
    sim_check((MOTION_FAULT_TASK == motion_fault_code(&g_sim.motion))
              && (stopped_after > S2_HEALTH_LIMIT_MS)
              && (stopped_after <= (S2_HEALTH_LIMIT_MS + 20u))
              && (0 == g_sim.duty[0]) && (0 == g_sim.duty[1]),
              "INT-15", "blocked controller: motors off at 500 ms, task fault");

    g_sim.b_controller_blocked = false;
    sim_run_ms(500u);
    {
        s2_robot_state_t state;

        sim_state(&state);
        sim_check((S2_SYS_FAULT == state.state)
                  && (0u != (state.flags & S2_RS_STOP_LATCHED))
                  && (S2_STOP_FAULT == state.stop_cause)
                  && (S2_FAULT_MOTION == state.fault) && int_wheels_still(),
                  "INT-15", "controller back: FAULT, STOP latched (fault)");
    }

    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    b_ok = sim_run_until(sim_running, 1000u);
    sim_run_ms(1000u);
    sim_check(b_ok && sim_following(), "INT-15", "RESET then START runs again");
}

/* INT-16 (S2 -> S1, telemetry version 6, S1 decisions 8 and 9): the frame S1
   would publish, from the real controller's published state through
   robot_read_telemetry() and the real encoder. Byte 31 shows the command
   held for the junction (bits 3-5), the junction count once it is crossed
   (bits 6-7) and, after a stalled controller, the motion fault detail
   (bits 0-2 = 7, task) with fault class motion in byte 10. */
static void
int_16_telemetry_v5 (void)
{
    uint8_t frame[TELEMETRY_ENCODER_SIZE];
    uint32_t blocked_at;
    bool b_ok;

    sim_init(&g_cross);
    sim_start();
    while ((sim_x_mm() < 200) && sim_running())
    {
        sim_step();
    }
    (void)s2_nav_submit(S2_NAV_STRAIGHT, 'C', 1u, g_sim.now_ms);
    sim_run_ms(100u);
    b_ok = int_frame(frame) && (TELEMETRY_ENCODER_VERSION == frame[0])
           && (S2_NAV_STRAIGHT == ((frame[31] >> 3) & 7u))
           && (0u == (frame[31] >> 6)) && (0u == (frame[31] & 7u));
    sim_check(b_ok, "INT-16", "v6 frame: STRAIGHT held, no junction yet");

    while ((sim_x_mm() < (g_cross.junction_mm + 300)) && sim_running())
    {
        sim_step();
    }
    b_ok = int_frame(frame) && (0u == ((frame[31] >> 3) & 7u))
           && (1u == (frame[31] >> 6));
    sim_check(b_ok, "INT-16", "after the junction: nothing held, count 1");

    g_sim.b_controller_blocked = true;
    blocked_at = g_sim.now_ms;
    while ((g_sim.now_ms - blocked_at) < 600u)
    {
        sim_step();
    }
    g_sim.b_controller_blocked = false;
    sim_run_ms(200u);
    b_ok = int_frame(frame) && (S2_FAULT_MOTION == (frame[10] & 15u))
           && (MOTION_FAULT_TASK == (frame[31] & 7u));
    sim_check(b_ok, "INT-16", "stalled task: fault motion, detail 7 (task)");
}

/* The frame S1 would publish now (service_bridge path). */
static bool
int_frame (uint8_t * p_frame)
{
    telemetry_encoder_snapshot_t snapshot;

    return robot_read_telemetry(NULL, &snapshot)
           && (TELEMETRY_ENCODER_SIZE
               == telemetry_encoder_encode(&snapshot, p_frame,
                                           TELEMETRY_ENCODER_SIZE));
}

/* Run the + junction with one command; report the final heading. */
static bool
int_turn_to (uint8_t command, int32_t * p_heading_mdeg,
             uint32_t * p_navigate_periods)
{
    bool b_done = false;

    sim_init(&g_cross);
    sim_start();
    while ((sim_x_mm() < 200) && sim_running())
    {
        sim_step();
    }
    (void)s2_nav_submit(command, 'X', 1u, g_sim.now_ms);

    /* Until the manoeuvre ends (or the junction is far behind). */
    for (uint32_t elapsed = 0u; (elapsed < 90000u) && sim_running();
         elapsed += SIM_PERIOD_MS)
    {
        sim_step();
        if ((g_sim.navigate_periods > 0u) && sim_following())
        {
            sim_run_ms(2000u);      /* then follow the new line */
            b_done = sim_following();
            break;
        }
        if ((0u == g_sim.navigate_periods)
            && (sim_x_mm() > (g_cross.junction_mm + 300)))
        {
            b_done = sim_following();
            break;
        }
    }

    *p_heading_mdeg = sim_heading_mdeg();
    *p_navigate_periods = g_sim.navigate_periods;

    return (b_done);
}

/* Following again after a manoeuvre that ended facing right. */
static bool
int_turned_right (void)
{
    return (sim_following() && (sim_heading_mdeg() < -60000)
            && (g_sim.navigate_periods > 0u));
}

/* S1's START and RESET path: the bridge puts them on remote_q. */
static void
int_send_remote (uint8_t command)
{
    static uint32_t request_id = 500u;

    request_id++;
    (void)s2_bridge_send_command(remote_q, command, request_id,
                                 g_sim.now_ms);
}

static bool
int_stopped_latched (uint8_t cause)
{
    s2_robot_state_t state;

    sim_state(&state);
    return ((S2_SYS_STOPPED == state.state)
            && (0u != (state.flags & S2_RS_STOP_LATCHED))
            && (cause == state.stop_cause));
}

static bool
int_wheels_still (void)
{
    return (motion_stop_confirmed());
}

/* INT-17 (S3 sensor -> decoder -> S2 -> Motion): B (RIGHT in the boot
   map) printed beside the line 300 mm in is read by the real side-sensor
   path while the car follows the line; the controller is told while the
   code is read, the letter and RIGHT reach the published state, the
   line is held, and the car turns right at the junction 1.5 m in. */
static void
int_17_printed_code (void)
{
    s2_robot_state_t state;
    bool b_held;
    bool b_turned;

    sim_init(&g_cross);
    sim_print_code('B', 300);
    sim_start();
    int_drive_past(600, 30000u);
    sim_state(&state);
    b_held = sim_following() && (1u == g_sim.bar_decoded)
             && (0u == g_sim.bar_rejected)
             && (0u != (state.flags & S2_RS_BARCODE_VALID))
             && ('B' == state.barcode)
             && (S2_NAV_RIGHT == state.barcode_command)
             && (S2_NAV_RIGHT == state.nav_held);
    sim_check(b_held, "INT-17",
              "printed B decoded once; RIGHT held; still following");
    sim_check(g_sim.b_flag_seen, "INT-17",
              "barcode-active flag reported while the code is read");

    b_turned = sim_run_until(int_turned_right, 120000u);
    sim_check(b_turned && (1u == g_sim.bar_decoded), "INT-17",
              "RIGHT carried out at the junction");
}

/* INT-18 (S3 sensor -> decoder -> S2): a damaged C (STRAIGHT): its last
   two runs are missing and a 0.8 mm speck (20 ms at 40 mm/s) splits run
   7. Before S3 decision 11 the decoder read this as P (F-25). P is mapped
   to LEFT for this case, so a wrong letter would turn the car. The
   capture must be rejected, nothing held, and the junction crossed. */
static void
int_18_corrupted_code (void)
{
    s2_robot_state_t state;
    bool b_ok;

    sim_init(&g_cross);
    sim_print_code('C', 300);
    g_sim.print.runs = (uint8_t)(g_sim.print.runs - 2u);
    sim_print_speck(7u, 800u);
    (void)s3_nav_set_letter(S3_NAV_LEFT, 'P');
    sim_start();
    int_drive_past(600, 30000u);
    sim_state(&state);
    b_ok = sim_following() && (0u == g_sim.bar_decoded)
           && (g_sim.bar_rejected >= 1u)
           && (0u == (state.flags & S2_RS_BARCODE_VALID))
           && (S2_NAV_NONE == state.nav_held);
    sim_check(b_ok, "INT-18",
              "damaged code rejected; nothing held; still following");

    b_ok = int_crossed_straight();
    (void)s3_nav_set_letter(S3_NAV_LEFT, 'A');  /* boot map again */
    sim_check(b_ok, "INT-18", "junction crossed, no turn");
}

/* INT-19 (S3 sensor -> S2): the same code read twice is counted twice
   and held once (one turn); a different code read later replaces the
   held command (S2 decision 27). */
static void
int_19_repeated_codes (void)
{
    s2_robot_state_t state;
    bool b_ok;

    sim_init(&g_cross);
    sim_print_code('B', 300);
    sim_start();
    int_drive_past(550, 30000u);
    /* The side sensor is 133 mm ahead: the next code starts ahead of it. */
    sim_print_code('B', 750);
    int_drive_past(1050, 30000u);
    sim_state(&state);
    b_ok = (2u == g_sim.bar_decoded) && (2u == state.barcode_count)
           && (S2_NAV_RIGHT == state.nav_held);
    b_ok = b_ok && sim_run_until(int_turned_right, 120000u);
    g_sim.navigate_periods = 0u;
    sim_run_ms(3000u);
    sim_check(b_ok && sim_following() && (0u == g_sim.navigate_periods),
              "INT-19", "a repeated code is held once: one right turn");

    sim_init(&g_cross);
    sim_print_code('A', 300);
    sim_start();
    int_drive_past(550, 30000u);
    sim_state(&state);
    b_ok = (S2_NAV_LEFT == state.nav_held);
    sim_print_code('B', 750);
    int_drive_past(1050, 30000u);
    sim_state(&state);
    b_ok = b_ok && ('B' == state.barcode)
           && (S2_NAV_RIGHT == state.nav_held);
    sim_check(b_ok && sim_run_until(int_turned_right, 120000u), "INT-19",
              "a later code replaces the held one: LEFT, then RIGHT turn");
}

/* INT-20 (S1 + S3 sensor -> S2): a remote STOP while a printed RIGHT is
   held drops it; after RESET and START the junction is crossed. */
static void
int_20_stop_drops_code (void)
{
    s2_robot_state_t state;
    bool b_ok;

    sim_init(&g_cross);
    sim_print_code('B', 300);
    sim_start();
    int_drive_past(600, 30000u);
    sim_state(&state);
    b_ok = (S2_NAV_RIGHT == state.nav_held);
    controller_raise_remote_stop(NULL, g_sim.now_ms);
    sim_step();
    sim_state(&state);
    sim_check(b_ok && int_stopped_latched(S2_STOP_REMOTE)
              && (S2_NAV_NONE == state.nav_held),
              "INT-20", "remote STOP latches and drops the held RIGHT");

    sim_run_ms(2000u);
    /* A new run restarts the pose, and so this track, at 0; the code was
       passed before the stop, so it is not read again. */
    g_sim.print.runs = 0u;
    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    (void)sim_run_until(sim_running, 1000u);
    sim_check(int_crossed_straight(), "INT-20",
              "after restart the junction is crossed, no turn");
}

/* A straight line with no junction; the obstacle is placed by the case. */
static sim_track_t const g_straight = {0, false, false, true, 0, 0u};

/* INT-21 (S5 -> S2 -> Motion -> S3, a whole obstacle mission): the demo
   obstacle (150 x 200 mm, S2 decision 22), in both orientations, sits on
   the line 900 mm in. The real forward monitor raises the hazard; the
   car stops before the scan; the real planner plans from the scan; the
   controller carries out the plan with side looks, searches for the line
   and follows it again. The body must never come within the agreed 50 mm
   of the box (the plan aims for 50 + 30 mm margin). Ideal rays and
   wheels: not evidence for the car (SYS-05 to SYS-07). */
static void
int_21_obstacle_bypass (void)
{
    sim_check(int_bypass_mission(150, 200, "150 wide, 200 deep"), "INT-21",
              "150 x 200 mm box: scan, bypass, reacquire, follow; >= 50 mm");
    sim_check(int_bypass_mission(200, 150, "200 wide, 150 deep"), "INT-21",
              "200 x 150 mm box: scan, bypass, reacquire, follow; >= 50 mm");
}

static bool
int_bypass_mission (int32_t width_mm, int32_t depth_mm, char const * p_name)
{
    uint8_t const want = (uint8_t)((1u << S2_MODE_FOLLOW)
                                   | (1u << S2_MODE_SCAN)
                                   | (1u << S2_MODE_BYPASS)
                                   | (1u << S2_MODE_REACQUIRE));
    int32_t const past_mm = 900 + depth_mm + 400;
    bool b_back = false;
    bool b_ok;

    sim_init(&g_straight);
    sim_place_box(900, width_mm, depth_mm);
    sim_start();
    for (uint32_t elapsed = 0u; (elapsed < 240000u) && sim_running();
         elapsed += SIM_PERIOD_MS)
    {
        sim_step();
        if (((g_sim.box.modes_seen & want) == want) && sim_following()
            && (sim_x_mm() > past_mm))
        {
            b_back = true;
            break;
        }
    }
    sim_run_ms(2000u);
    {
        motion_status_t status;

        (void)motion_get_status(&status);
        printf("[HOST] INT-21 %s: side %s, closest %.0f mm, then at x %ld "
               "y %ld mm heading %ld mdeg\n", p_name,
               (BYPASS_PLAN_LEFT == g_sim.box.plan.side) ? "left" : "right",
               g_sim.box.min_clear_mm, (long)status.x_mm, (long)status.y_mm,
               (long)status.heading_mdeg);
        b_ok = b_back && sim_following()
               && (1u == g_sim.authorise_count) && g_sim.b_still_at_authorise
               && (g_sim.box.min_clear_mm >= 50.0)
               && (abs(status.y_mm) <= 20)
               && (abs(status.heading_mdeg) <= 10000);
    }

    return (b_ok);
}

/* INT-22 (S1 -> S2 during a bypass): a remote STOP while the plan is
   being carried out latches STOP in that period, gives zero duty from the
   next Motion Control period, ends the bypass and stops the wheels;
   nothing moves while it is latched. */
static void
int_22_stop_in_bypass (void)
{
    bool b_ok;

    sim_init(&g_straight);
    sim_place_box(900, 150, 200);
    sim_start();
    b_ok = sim_run_until(int_in_bypass, 120000u);
    sim_run_ms(1500u);
    controller_raise_remote_stop(NULL, g_sim.now_ms);
    sim_step();
    b_ok = b_ok && int_stopped_latched(S2_STOP_REMOTE);
    /* Each model step runs Motion Control before the controller, so the
       latch reaches the motors in the next Motion Control period. */
    sim_step();
    b_ok = b_ok && (0 == g_sim.duty[0]) && (0 == g_sim.duty[1]);
    sim_check(b_ok, "INT-22", "remote STOP in BYPASS latches; duty 0 by the next motion period");
    sim_run_ms(3000u);
    sim_check(int_wheels_still() && int_stopped_latched(S2_STOP_REMOTE)
              && (!bypass_control_active(&g_bypass))
              && (g_sim.box.min_clear_mm >= 50.0), "INT-22",
              "bypass ended; wheels still; box never reached");
}

/* INT-23 (failed recovery): the line ends 60 mm before the box, so after
   the bypass the bounded search finds nothing. STOP latches with cause
   bypass and the car stays still. */
static void
int_23_bypass_no_line (void)
{
    sim_track_t const track = {840, false, false, false, 0, 0u};
    uint8_t const want = (uint8_t)((1u << S2_MODE_BYPASS)
                                   | (1u << S2_MODE_REACQUIRE));

    sim_init(&track);
    sim_place_box(900, 150, 200);
    sim_start();
    (void)sim_run_until(int_in_bypass, 120000u);
    for (uint32_t elapsed = 0u; (elapsed < 240000u) && sim_running();
         elapsed += SIM_PERIOD_MS)
    {
        sim_step();
    }
    sim_run_ms(3000u);
    sim_check(int_stopped_latched(S2_STOP_BYPASS)
              && ((g_sim.box.modes_seen & want) == want)
              && int_wheels_still() && (0 == g_sim.duty[0])
              && (0 == g_sim.duty[1]) && (g_sim.box.min_clear_mm >= 50.0),
              "INT-23", "no line after the bypass: STOP (bypass), still");
}

/* INT-24 (S2 -> S4): S4's real detection gate, fed the controller's
   published state every 20 ms through a whole obstacle bypass and a
   right turn at a junction, is never on in SCAN, BYPASS, REACQUIRE or
   NAVIGATE (S4 decisions 16 and 17), is on for most of the following,
   and is off for the settle count after each return to FOLLOW. */
static void
int_24_terrain_gate (void)
{
    uint8_t const follow_only = (uint8_t)(1u << S2_MODE_FOLLOW);
    int32_t heading = 0;
    uint32_t navigate = 0u;
    bool b_ok;

    b_ok = int_bypass_mission(150, 200, "gate run");
    b_ok = b_ok && (follow_only == g_sim.gate.on_modes)
           && (g_sim.gate.follow_on > (g_sim.gate.follow_samples / 2u))
           && (g_sim.gate.follow_on
               <= (g_sim.gate.follow_samples - (2u * S4_SETTLE_SAMPLES)));
    sim_check(b_ok, "INT-24",
              "bypass: detection only in FOLLOW, off while settling");

    b_ok = int_turn_to(S2_NAV_RIGHT, &heading, &navigate)
           && (navigate > 0u) && (follow_only == g_sim.gate.on_modes)
           && (g_sim.gate.follow_on
               <= (g_sim.gate.follow_samples - (2u * S4_SETTLE_SAMPLES)));
    sim_check(b_ok, "INT-24",
              "junction turn: detection only in FOLLOW, off while settling");
}

static bool
int_in_bypass (void)
{
    s2_robot_state_t state;

    sim_state(&state);
    return ((S2_SYS_RUNNING == state.state)
            && (S2_MODE_BYPASS == state.mode));
}

/* Follow until the axle is past x_mm (or limit_ms, or not running). */
static void
int_drive_past (int32_t x_mm, uint32_t limit_ms)
{
    for (uint32_t elapsed = 0u;
         (elapsed < limit_ms) && sim_running() && (sim_x_mm() < x_mm);
         elapsed += SIM_PERIOD_MS)
    {
        sim_step();
    }
}

/* Follow on until 300 mm past the junction: no NAVIGATE on the way and
   still following straight ahead. */
static bool
int_crossed_straight (void)
{
    g_sim.navigate_periods = 0u;
    int_drive_past(g_cross.junction_mm + 300, 90000u);

    return (sim_following() && (0u == g_sim.navigate_periods)
            && (abs(sim_heading_mdeg()) < 5000)
            && (sim_x_mm() >= (g_cross.junction_mm + 300)));
}

/* INT-25 (S1 -> S2 during NAVIGATE, post-demo 2026-11-26): a remote STOP
   1 s into a RIGHT manoeuvre at the + junction. STOP latches (cause
   remote) in that period and the motors get no duty from the next Motion
   Control period, though Motion Control is mid-move; the held command is
   gone, the wheels stay still, and START alone does not move the car;
   RESET then START follows the line again without resuming the turn. */
static void
int_25_stop_in_turn (void)
{
    s2_robot_state_t state;
    bool b_ok = int_start_turn(S2_NAV_RIGHT);

    controller_raise_remote_stop(NULL, g_sim.now_ms);
    sim_step();
    b_ok = b_ok && int_stopped_latched(S2_STOP_REMOTE);
    sim_step();
    b_ok = b_ok && (0 == g_sim.duty[0]) && (0 == g_sim.duty[1]);
    sim_check(b_ok, "INT-25",
              "remote STOP in NAVIGATE latches; duty 0 by the next period");

    sim_run_ms(3000u);
    sim_state(&state);
    sim_check(int_wheels_still() && int_stopped_latched(S2_STOP_REMOTE)
              && (S2_NAV_NONE == state.nav_held)
              && (INT32_MAX != g_sim.max_speed_after_stop), "INT-25",
              "no duty while latched; wheels still; held command dropped");

    int_send_remote(S2_REMOTE_START);
    sim_run_ms(500u);
    b_ok = int_stopped_latched(S2_STOP_REMOTE) && int_wheels_still();
    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    b_ok = b_ok && sim_run_until(sim_running, 1000u);
    sim_run_ms(300u);
    sim_check(b_ok && sim_following() && (!sim_navigating()), "INT-25",
              "START alone stays stopped; RESET, START follows, no turn");
}

/* INT-26 (all tasks -> Motion Control during NAVIGATE, post-demo
   2026-11-26): the controller blocks for 600 ms 1 s into a RIGHT
   manoeuvre. Motion Control's guard faults (MOTION_FAULT_TASK) at the
   500 ms limit without the controller, as in INT-15 while following, and
   holds the duty at 0. (The duty may reach 0 earlier: Motion Control
   ends the current turn segment and waits.) When the controller runs
   again it enters FAULT with STOP latched (cause fault) and the turn is
   not resumed; RESET then START follow again without the turn. */
static void
int_26_fault_in_turn (void)
{
    s2_robot_state_t state;
    uint32_t blocked_at;
    uint32_t fault_after = 0u;
    bool b_ok = int_start_turn(S2_NAV_RIGHT);

    g_sim.b_controller_blocked = true;
    blocked_at = g_sim.now_ms;
    while ((g_sim.now_ms - blocked_at) < 600u)
    {
        sim_step();
        if ((0u == fault_after)
            && (MOTION_FAULT == motion_state(&g_sim.motion)))
        {
            fault_after = g_sim.now_ms - blocked_at;
        }
    }
    sim_check(b_ok && (MOTION_FAULT_TASK == motion_fault_code(&g_sim.motion))
              && (fault_after > S2_HEALTH_LIMIT_MS)
              && (fault_after <= (S2_HEALTH_LIMIT_MS + 20u))
              && (0 == g_sim.duty[0]) && (0 == g_sim.duty[1]),
              "INT-26", "blocked controller mid-turn: task fault at 500 ms");

    g_sim.b_controller_blocked = false;
    sim_run_ms(2000u);
    sim_state(&state);
    sim_check((S2_SYS_FAULT == state.state)
              && (0u != (state.flags & S2_RS_STOP_LATCHED))
              && (S2_STOP_FAULT == state.stop_cause)
              && (S2_MODE_NONE == state.mode) && int_wheels_still(),
              "INT-26", "controller back: FAULT latched, turn not resumed");

    int_send_remote(S2_REMOTE_RESET);
    sim_run_ms(500u);
    int_send_remote(S2_REMOTE_START);
    b_ok = sim_run_until(sim_running, 1000u);
    sim_run_ms(300u);
    sim_check(b_ok && sim_following() && (!sim_navigating()), "INT-26",
              "RESET then START follows again, no turn resumed");
}

/* INT-27 (S3 -> S2, review F-17, post-demo 2026-11-26): 60 s of a long
   straight with the car centred. Both front sensors then read white
   (each sits about 6 mm outside the tape), which is also what a car fully
   off the line would read. Ordinary following must never stop on it: no
   STOP, no other mode, and both wheels near the line speed throughout. */
static void
int_27_long_straight (void)
{
    s2_robot_state_t state;
    bool b_always = true;

    sim_init(&g_straight);
    sim_start();
    sim_run_ms(2000u);
    for (uint32_t elapsed = 0u; elapsed < 60000u; elapsed += SIM_PERIOD_MS)
    {
        sim_step();
        b_always = b_always && sim_following();
    }
    sim_state(&state);
    sim_check(b_always && (S2_STOP_NONE == state.stop_cause)
              && (S2_LINE_CENTER == state.line)
              && (abs(state.left_mm_s - CTRL_LINE_BASE_MM_S) <= 2)
              && (abs(state.right_mm_s - CTRL_LINE_BASE_MM_S) <= 2),
              "INT-27", "60 s centred on a straight: following throughout");
    sim_check(sim_x_mm() > 2300, "INT-27",
              "about 2.4 m covered at 40 mm/s; no false line-loss stop");
}

/* Hold command 200 mm in, then follow until 1 s into its manoeuvre at the
   + junction. True if the car is then still navigating. */
static bool
int_start_turn (uint8_t command)
{
    sim_init(&g_cross);
    sim_start();
    int_drive_past(200, 30000u);
    (void)s2_nav_submit(command, 'X', 1u, g_sim.now_ms);

    bool b_navigating = sim_run_until(sim_navigating, 90000u);

    sim_run_ms(1000u);

    return (b_navigating && sim_navigating());
}

/* INT-28 (S5 sensor -> S2, 2026-11-26): forward readings that are invalid
   (no echo) or stop arriving (stale) while following, with no obstacle
   within 4 m. The real forward monitor reports them as not safe, which
   the controller treats as a hazard: it stops, scans with the wheels
   still, and resumes only on S5's CLEAR, so the car never drives on
   uncertain range for longer than one reading. Over 15 s of fault it
   moves less than 100 mm; when readings return it follows normally. */
static void
int_28_forward_fault (uint8_t fault, char const * p_name)
{
    int32_t x_before;
    int32_t x_after;
    bool b_ok;

    sim_init(&g_straight);
    sim_place_box(4000, 150, 200);
    sim_start();
    sim_run_ms(2000u);
    b_ok = sim_following() && (0u == g_sim.authorise_count);

    g_sim.fwd_fault = fault;
    x_before = sim_x_mm();
    sim_run_ms(15000u);
    x_after = sim_x_mm();
    sim_check(b_ok && sim_running() && (g_sim.authorise_count >= 1u)
              && (0u == g_sim.authorise_moving)
              && ((x_after - x_before) < 100), "INT-28", p_name);

    g_sim.fwd_fault = SIM_FWD_OK;
    (void)sim_run_until(sim_following, SIM_SCAN_MS + 2000u);
    x_before = sim_x_mm();
    sim_run_ms(5000u);
    sim_check(sim_following() && ((sim_x_mm() - x_before) > 150), "INT-28",
              "readings back: following resumes");
}

/* INT-29 (S2 <-> S5, 2026-11-26): a hazard at 0.8 m and a scan that S5
   never answers. The wheels stay still from the stop before the scan,
   and after the controller's 20 s scan limit STOP latches with cause
   scan; the car never moves on without a result. */
static void
int_29_scan_silent (void)
{
    sim_track_t const track = {0, false, false, true, 800,
                               SCAN_TASK_RESULT_CLEAR};
    bool b_still = true;
    bool b_stopped;

    sim_init(&track);
    g_sim.b_scan_mute = true;
    sim_start();
    (void)sim_run_until(int_wheels_still, 60000u);
    for (uint32_t elapsed = 0u; elapsed < 21000u; elapsed += SIM_PERIOD_MS)
    {
        sim_step();
        b_still = b_still && int_wheels_still();
    }
    b_stopped = int_stopped_latched(S2_STOP_SCAN);
    sim_check(g_sim.b_still_at_authorise && b_still && b_stopped
              && (0u == g_sim.authorise_moving), "INT-29",
              "no scan answer: wheels still, STOP (scan) after 20 s");
}

/* INT-30 (S2 <-> S5 during BYPASS, 2026-11-26): the demo obstacle; once
   the bypass has started, S5 stops answering side looks. The bypass must
   not drive on without them: the car ends stopped with STOP latched,
   never reaches the box and is still. */
static void
int_30_look_silent (void)
{
    s2_robot_state_t state;
    bool b_ok;

    sim_init(&g_straight);
    sim_place_box(900, 150, 200);
    sim_start();
    b_ok = sim_run_until(int_in_bypass, 120000u);
    g_sim.b_scan_mute = true;
    sim_run_ms(60000u);
    sim_state(&state);
    sim_check(b_ok && int_wheels_still()
              && (0u != (state.flags & S2_RS_STOP_LATCHED))
              && (g_sim.box.min_clear_mm >= 50.0), "INT-30",
              "no look answers in BYPASS: STOP latched, still, box clear");
}

/* INT-31 (S2 -> S4 -> S1, S4 decision 20, telemetry version 6): the real
   S4 motion classifier on the real Motion Control status through a whole
   run, and its result in the real telemetry frame. Straight: stationary
   before START, then cruising, never turning (the ramp to 40 mm/s stays
   under the 15 mm/s-per-sample gate). A RIGHT turn at the + junction:
   turning with a negative (right) encoder yaw rate near the 30 mm/s
   spin's 29 deg/s, and accelerating and braking on the 200 mm/s step. An
   obstacle: stationary for the scan; accelerating, braking and turning in
   BYPASS. Observational only: every other case passes with it running. */
static void
int_31_motion_class (void)
{
    uint8_t frame[TELEMETRY_ENCODER_SIZE];
    int32_t heading = 0;
    uint32_t navigate = 0u;
    bool b_ok;

    sim_init(&g_straight);
    sim_run_ms(500u);
    sim_start();
    sim_run_ms(10000u);
    b_ok = int_class_seen(S2_MODE_NONE, S4_MOTION_STATIONARY)
           && int_class_seen(S2_MODE_FOLLOW, S4_MOTION_CRUISING)
           && (!int_class_seen(S2_MODE_FOLLOW, S4_MOTION_TURNING))
           && (S4_MOTION_CRUISING == g_sim.s4.cls.state);
    sim_check(b_ok, "INT-31",
              "straight: stationary, then cruising; never turning");
    b_ok = int_frame(frame) && (S4_MOTION_CRUISING == frame[32])
           && (abs((int8_t)frame[33] * 20 - g_sim.s4.cls.yaw_ddeg_s) <= 10);
    sim_check(b_ok, "INT-31", "v6 frame carries the class and yaw rate");

    b_ok = int_turn_to(S2_NAV_RIGHT, &heading, &navigate);
    sim_check(b_ok && int_class_seen(S2_MODE_NAVIGATE, S4_MOTION_TURNING)
              && int_class_seen(S2_MODE_NAVIGATE, S4_MOTION_ACCELERATING)
              && int_class_seen(S2_MODE_NAVIGATE, S4_MOTION_BRAKING)
              && (g_sim.s4.yaw_min <= -250) && (g_sim.s4.yaw_min >= -400),
              "INT-31", "RIGHT turn: turning at about -29 deg/s; step "
                        "accelerating and braking");

    b_ok = int_bypass_mission(150, 200, "motion class run");
    sim_check(b_ok && int_class_seen(S2_MODE_SCAN, S4_MOTION_STATIONARY)
              && int_class_seen(S2_MODE_BYPASS, S4_MOTION_ACCELERATING)
              && int_class_seen(S2_MODE_BYPASS, S4_MOTION_BRAKING)
              && int_class_seen(S2_MODE_BYPASS, S4_MOTION_TURNING),
              "INT-31", "obstacle: stationary for the scan; bypass "
                        "accelerating, braking and turning");
}

/* Whether the classifier reported state while the controller was in mode. */
static bool
int_class_seen (uint8_t mode, uint8_t state)
{
    return ((mode < 8u) && (state < 16u)
            && (0u != (g_sim.s4.seen[mode] & (1u << state))));
}

/* End of file. */
