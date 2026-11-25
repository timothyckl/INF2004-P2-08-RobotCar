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

/* End of file. */
