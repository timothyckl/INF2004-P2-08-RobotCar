/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	S2-U03..S2-U14 test the pure PID and odometry logic with synthetic
 *	gains and geometry; they need no motors or encoders. S2-U20..S2-U35
 *	(test_motion.c) drive the Motion Control logic with a wheel model.
 *
 *	Bench mode, off by default (S2_HW_TEST 0). Set it locally for a
 *	bench session and never commit it non-zero:
 *	  1  encoder counts every 100 ms for 60 s; motors stay OFF (S2-H03)
 *	  2  also SPINS THE MOTORS (S2-H01, S2-H02, S2-H04): wheels off the
 *	     ground first
 */

#include "subsystem2.h"
#include "pid.h"
#include "odometry.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_gpio_irq.h"
#include "test_motion.h"
#include "../test_report.h"

#ifndef S2_HW_TEST
#define S2_HW_TEST	0
#endif

#if APP_TEST

#define SENTINEL	12345	/* output value a rejected call must keep */

/* Synthetic geometry: 20 counts per 200 mm revolution, 150 mm track */
static const odom_cfg_t odom_cfg = { 20u, 200000u, 150u };

/* Configure a controller with gains given as PID_GAIN_SCALE multiples */
static bool pid_setup(pid_ctrl_t *p_pid, int32_t kp, int32_t ki, int32_t kd,
		      int32_t out_min, int32_t out_max)
{
	pid_cfg_t cfg = { kp, ki, kd, out_min, out_max };
	return pid_init(p_pid, &cfg);
}

/* S2-U03: proportional term alone gives kp * error */
static bool test_pid_proportional(void)
{
	pid_ctrl_t pid;
	int32_t    out = 0;

	return pid_setup(&pid, 2 * PID_GAIN_SCALE, 0, 0, -1000, 1000) &&
	       pid_update(&pid, 100, 40, &out) && (out == 120) &&
	       pid_update(&pid, 40, 100, &out) && (out == -120);
}

/* S2-U04: output never leaves out_min..out_max */
static bool test_pid_clamp(void)
{
	pid_ctrl_t pid;
	int32_t    hi = 0, lo = 0;

	return pid_setup(&pid, 100 * PID_GAIN_SCALE, 0, 0, -255, 255) &&
	       pid_update(&pid, 1000, 0, &hi) && (hi == 255) &&
	       pid_update(&pid, -1000, 0, &lo) && (lo == -255);
}

/* S2-U05: integral term adds ki * error every sample */
static bool test_pid_integral(void)
{
	pid_ctrl_t pid;
	int32_t    a = 0, b = 0, c = 0;

	return pid_setup(&pid, 0, PID_GAIN_SCALE / 2, 0, -1000, 1000) &&
	       pid_update(&pid, 10, 0, &a) &&
	       pid_update(&pid, 10, 0, &b) &&
	       pid_update(&pid, 10, 0, &c) &&
	       (a == 5) && (b == 10) && (c == 15);
}

/* S2-U06: a long saturation (stalled wheel) does not wind up the integral,
   so the output reverses on the first sample after the error reverses */
static bool test_pid_anti_windup(void)
{
	pid_ctrl_t pid;
	int32_t    out = 0;
	bool       ok;
	int	   i;

	ok = pid_setup(&pid, 0, PID_GAIN_SCALE, 0, -100, 100);
	for (i = 0; i < 50; i++) {
		ok = pid_update(&pid, 1000, 0, &out) && (out == 100) && ok;
	}
	return ok && pid_update(&pid, 0, 10, &out) && (out == -10);
}

/* S2-U07: derivative acts on measurement change, not setpoint change */
static bool test_pid_derivative(void)
{
	pid_ctrl_t pid;
	int32_t    first = SENTINEL, step = SENTINEL, rise = SENTINEL;

	return pid_setup(&pid, 0, 0, PID_GAIN_SCALE, -1000, 1000) &&
	       pid_update(&pid, 0, 0, &first) &&	/* no history yet */
	       pid_update(&pid, 500, 0, &step) &&	/* setpoint step */
	       pid_update(&pid, 500, 5, &rise) &&	/* measurement +5 */
	       (first == 0) && (step == 0) && (rise == -5);
}

/* S2-U08: invalid tuning and inputs are rejected without side effects */
static bool test_pid_invalid(void)
{
	pid_ctrl_t pid;
	int32_t    out = SENTINEL;

	return !pid_setup(&pid, 0, 0, 0, 10, 10) &&	/* empty range */
	       !pid_setup(&pid, -1, 0, 0, -10, 10) &&	/* negative gain */
	       !pid_setup(&pid, PID_GAIN_MAX + 1, 0, 0, -10, 10) &&
	       !pid_init(NULL, NULL) &&
	       pid_setup(&pid, PID_GAIN_SCALE, 0, 0, -10, 10) &&
	       !pid_update(&pid, PID_INPUT_MAX + 1, 0, &out) &&
	       !pid_update(&pid, 0, -PID_INPUT_MAX - 1, &out) &&
	       !pid_update(&pid, 0, 0, NULL) &&
	       (out == SENTINEL);
}

/* S2-U09: reset clears the integral and derivative history */
static bool test_pid_reset(void)
{
	pid_ctrl_t pid;
	int32_t    out = SENTINEL;

	return pid_setup(&pid, 0, PID_GAIN_SCALE, PID_GAIN_SCALE, -1000, 1000) &&
	       pid_update(&pid, 50, 0, &out) &&
	       pid_update(&pid, 50, 0, &out) && (out == 100) &&
	       (pid_reset(&pid), pid_update(&pid, 7, 7, &out)) && (out == 0);
}

/* S2-U10: counts convert to millimetres, truncating toward zero */
static bool test_odom_distance(void)
{
	static const odom_cfg_t thirds = { 3u, 100000u, 150u };
	int32_t a = 0, b = 0, c = 0, d = 0;

	return odom_counts_to_mm(&odom_cfg, 20, &a) && (a == 200) &&
	       odom_counts_to_mm(&odom_cfg, -10, &b) && (b == -100) &&
	       odom_counts_to_mm(&thirds, 1, &c) && (c == 33) &&
	       odom_counts_to_mm(&thirds, -1, &d) && (d == -33);
}

/* S2-U11: counter deltas are correct across uint32_t wraparound */
static bool test_odom_wrap(void)
{
	return (odom_count_delta(5u, 0xFFFFFFFBu) == 10) &&
	       (odom_count_delta(0xFFFFFFFBu, 5u) == -10) &&
	       (odom_count_delta(0x7FFFFFFFu, 0u) == INT32_MAX) &&
	       (odom_count_delta(0x80000000u, 0u) == INT32_MIN) &&
	       (odom_count_delta(42u, 42u) == 0);
}

/* S2-U12: speed from counts over an interval; zero interval rejected */
static bool test_odom_speed(void)
{
	int32_t fwd = 0, rev = 0, slow = 0, out = SENTINEL;

	return odom_speed_mm_s(&odom_cfg, 10, 100000u, &fwd) && (fwd == 1000) &&
	       odom_speed_mm_s(&odom_cfg, -10, 100000u, &rev) && (rev == -1000) &&
	       odom_speed_mm_s(&odom_cfg, 1, 20000u, &slow) && (slow == 500) &&
	       !odom_speed_mm_s(&odom_cfg, 10, 0u, &out) && (out == SENTINEL);
}

/* S2-U13: turn angle sign (left positive) and magnitude (1 rad) */
static bool test_odom_turn(void)
{
	int32_t left = 0, right = 0, straight = SENTINEL;

	return odom_turn_mdeg(&odom_cfg, 0, 150, &left) && (left == 57295) &&
	       odom_turn_mdeg(&odom_cfg, 150, 0, &right) && (right == -57295) &&
	       odom_turn_mdeg(&odom_cfg, 80, 80, &straight) && (straight == 0);
}

/* S2-U14: invalid geometry and overflowing results are rejected */
static bool test_odom_invalid(void)
{
	static const odom_cfg_t no_cpr = { 0u, 200000u, 150u };
	static const odom_cfg_t no_track = { 20u, 200000u, 0u };
	static const odom_cfg_t coarse = { 1u, ODOM_CIRC_UM_MAX, 150u };
	int32_t out = SENTINEL;

	return !odom_cfg_valid(&no_cpr) && !odom_cfg_valid(&no_track) &&
	       !odom_cfg_valid(NULL) && odom_cfg_valid(&coarse) &&
	       !odom_counts_to_mm(&no_cpr, 1, &out) &&
	       !odom_counts_to_mm(&coarse, INT32_MAX, &out) &&
	       !odom_speed_mm_s(&coarse, INT32_MAX, 1u, &out) &&
	       !odom_turn_mdeg(&no_track, 0, 1, &out) &&
	       !odom_counts_to_mm(&odom_cfg, 1, NULL) &&
	       (out == SENTINEL);
}

/* S2-U15: duty sign selects the A or B output; zero brakes */
static bool test_motor_map(void)
{
	drv_motor_cc_t fwd, rev, stop, full;

	return drv_motor_map(1000, &fwd) &&
	       (fwd.cc_a == 1000u) && (fwd.cc_b == 0u) &&
	       drv_motor_map(-1000, &rev) &&
	       (rev.cc_a == 0u) && (rev.cc_b == 1000u) &&
	       drv_motor_map(0, &stop) &&
	       (stop.cc_a == 0u) && (stop.cc_b == 0u) &&
	       drv_motor_map(-DRV_MOTOR_DUTY_MAX, &full) &&
	       (full.cc_b == (uint16_t)DRV_MOTOR_DUTY_MAX);
}

/* S2-U16: out-of-range duty and NULL output are rejected, output kept */
static bool test_motor_reject(void)
{
	drv_motor_cc_t cc = { 7u, 9u };

	return !drv_motor_map(DRV_MOTOR_DUTY_MAX + 1, &cc) &&
	       !drv_motor_map(-DRV_MOTOR_DUTY_MAX - 1, &cc) &&
	       !drv_motor_map(INT32_MIN, &cc) &&
	       !drv_motor_map(0, NULL) &&
	       (cc.cc_a == 7u) && (cc.cc_b == 9u);
}

static void test_gpio_cb(uint32_t gpio, uint32_t events, uint32_t now_us)
{
	(void)gpio; (void)events; (void)now_us;
}

/* S2-U17: invalid registrations are refused before touching hardware */
static bool test_gpio_irq_reject(void)
{
	uint32_t const both = DRV_GPIO_IRQ_EDGE_RISE | DRV_GPIO_IRQ_EDGE_FALL;

	return (drv_gpio_irq_register(30u, both, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(23u, both, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(24u, both, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(25u, both, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(29u, both, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(5u, 0u, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(5u, 0x1u, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(5u, 0x2u, test_gpio_cb) == E_PAR) &&
	       (drv_gpio_irq_register(5u, both, NULL) == E_PAR) &&
	       (drv_gpio_irq_unregister(30u) == E_PAR);
}

/* S2-U18: x2 quadrature decode */
static bool test_encoder_step(void)
{
	uint32_t const rise = DRV_GPIO_IRQ_EDGE_RISE;
	uint32_t const fall = DRV_GPIO_IRQ_EDGE_FALL;

	return (drv_encoder_step(rise, false) == 1) &&
	       (drv_encoder_step(rise, true) == -1) &&
	       (drv_encoder_step(fall, true) == 1) &&
	       (drv_encoder_step(fall, false) == -1) &&
	       (drv_encoder_step(rise | fall, false) == 0) &&
	       (drv_encoder_step(rise | fall, true) == 0) &&
	       (drv_encoder_step(0u, true) == 0);
}

/* S2-U19: snapshot rejects NULL; idle counts do not change */
static bool test_encoder_snapshot(void)
{
	drv_encoder_snapshot_t a, b;
	UINT e;
	bool same = true;

	if ((drv_encoder_snapshot(NULL) != E_PAR) ||
	    (drv_encoder_snapshot(&a) != E_OK) ||
	    (drv_encoder_snapshot(&b) != E_OK)) {
		return false;
	}
	for (e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++) {
		same = same && (a.count[e] == b.count[e]) &&
		       (a.errors[e] == b.errors[e]);
	}
	return same;
}

#if S2_HW_TEST
/* Print both encoders every 100 ms for the given number of seconds */
static void hw_print_counts(const char *step, INT seconds)
{
	drv_encoder_snapshot_t s;
	INT i;

	for (i = 0; i < seconds * 10; i++) {
		if (drv_encoder_snapshot(&s) == E_OK) {
			tm_printf((UB*)"[HW] %s enc1 %d (err %d)  enc2 %d (err %d)\n",
				  step, (INT)s.count[0], (INT)s.errors[0],
				  (INT)s.count[1], (INT)s.errors[1]);
		}
		tk_dly_tsk(100);
	}
}

#if S2_HW_TEST >= 2
/* S2-H01, S2-H02, S2-H04.  Wheels OFF THE GROUND. */
static void hw_motor_sequence(void)
{
	const int32_t duty30 = (DRV_MOTOR_DUTY_MAX * 3) / 10;
	drv_encoder_snapshot_t s0, s1;
	UINT m;

	for (m = 0u; m < (UINT)DRV_MOTOR_COUNT; m++) {
		tm_printf((UB*)"[HW] S2-H01 motor %d forward 30%%\n", m + 1u);
		(void)drv_motor_set((drv_motor_id_t)m, duty30);
		hw_print_counts("S2-H01 fwd", 2);
		tm_printf((UB*)"[HW] S2-H02 motor %d BRAKE: turn by hand\n", m + 1u);
		(void)drv_motor_brake((drv_motor_id_t)m);
		hw_print_counts("S2-H02 brake", 3);
		tm_printf((UB*)"[HW] S2-H01 motor %d backward 30%%\n", m + 1u);
		(void)drv_motor_set((drv_motor_id_t)m, -duty30);
		hw_print_counts("S2-H01 back", 2);
		tm_printf((UB*)"[HW] S2-H02 motor %d COAST: turn by hand\n", m + 1u);
		(void)drv_motor_coast((drv_motor_id_t)m);
		hw_print_counts("S2-H02 coast", 3);
	}

	tm_printf((UB*)"[HW] S2-H04 both motors 100%% for 10 s\n");
	(void)drv_encoder_snapshot(&s0);
	(void)drv_motor_set(DRV_MOTOR_1, DRV_MOTOR_DUTY_MAX);
	(void)drv_motor_set(DRV_MOTOR_2, DRV_MOTOR_DUTY_MAX);
	tk_dly_tsk(10000);
	(void)drv_encoder_snapshot(&s1);
	(void)drv_motor_brake(DRV_MOTOR_1);
	(void)drv_motor_brake(DRV_MOTOR_2);
	tm_printf((UB*)"[HW] S2-H04 counts/s enc1 %d enc2 %d, new errors %d %d\n",
		  (INT)(odom_count_delta(s1.count[0], s0.count[0]) / 10),
		  (INT)(odom_count_delta(s1.count[1], s0.count[1]) / 10),
		  (INT)(s1.errors[0] - s0.errors[0]),
		  (INT)(s1.errors[1] - s0.errors[1]));
}
#endif /* S2_HW_TEST >= 2 */

static void hw_test(void)
{
	CHECK(drv_gpio_irq_init(), "drv_gpio_irq_init");
	CHECK(drv_motor_init(), "drv_motor_init");	/* starts braked */
	CHECK(drv_encoder_init(), "drv_encoder_init");

	/* S2-H03: motors coast so the wheels turn freely by hand */
	(void)drv_motor_coast(DRV_MOTOR_1);
	(void)drv_motor_coast(DRV_MOTOR_2);
	tm_printf((UB*)"[HW] S2-H03: turn each wheel by hand (60 s)\n");
	hw_print_counts("S2-H03", 60);
	(void)drv_motor_brake(DRV_MOTOR_1);
	(void)drv_motor_brake(DRV_MOTOR_2);

#if S2_HW_TEST >= 2
	hw_motor_sequence();
#endif
}
#endif /* S2_HW_TEST */

EXPORT void subsystem2_test(void)
{
	T_RSEM	rsem;
	T_RMPF	rmpf;
	UW	seq;
	INT	v, bad = 0;

	TEST_INIT();

	/* S2-U01: every sample value is inside the documented 0..100 range */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");

	/* S2-U02: flow control never over- or under-counts */
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

	TEST_CHECK("S2-U03", test_pid_proportional(),
		   "PID proportional term is kp * error");
	TEST_CHECK("S2-U04", test_pid_clamp(),
		   "PID output stays within its limits");
	TEST_CHECK("S2-U05", test_pid_integral(),
		   "PID integral adds ki * error per sample");
	TEST_CHECK("S2-U06", test_pid_anti_windup(),
		   "PID integral does not wind up while saturated");
	TEST_CHECK("S2-U07", test_pid_derivative(),
		   "PID derivative ignores setpoint steps");
	TEST_CHECK("S2-U08", test_pid_invalid(),
		   "PID rejects invalid tuning and inputs");
	TEST_CHECK("S2-U09", test_pid_reset(),
		   "PID reset clears history");
	TEST_CHECK("S2-U10", test_odom_distance(),
		   "counts convert to millimetres");
	TEST_CHECK("S2-U11", test_odom_wrap(),
		   "count delta survives counter wraparound");
	TEST_CHECK("S2-U12", test_odom_speed(),
		   "speed from counts over an interval");
	TEST_CHECK("S2-U13", test_odom_turn(),
		   "turn angle sign and magnitude");
	TEST_CHECK("S2-U14", test_odom_invalid(),
		   "odometry rejects invalid geometry and overflow");
	TEST_CHECK("S2-U15", test_motor_map(),
		   "motor duty maps to A/B outputs");
	TEST_CHECK("S2-U16", test_motor_reject(),
		   "motor rejects out-of-range duty");
	TEST_CHECK("S2-U17", test_gpio_irq_reject(),
		   "GPIO IRQ registration rejects invalid requests");
	TEST_CHECK("S2-U18", test_encoder_step(),
		   "quadrature x2 decode");
	TEST_CHECK("S2-U19", test_encoder_snapshot(),
		   "encoder snapshot rejects NULL, stable when idle");
	TEST_CHECK("S2-U20", test_motion_start(),
		   "motion start: Idle + READY, or Fault(INIT)");
	TEST_CHECK("S2-U21", test_motion_speed(),
		   "Speed state reaches the wheel targets");
	TEST_CHECK("S2-U22", test_motion_move(),
		   "MOVE stops at target, DONE after stop confirmed");
	TEST_CHECK("S2-U23", test_motion_turn(),
		   "TURN sign and angle (left positive)");
	TEST_CHECK("S2-U24", test_motion_straight(),
		   "straight PID recovers lost wheel distance");
	TEST_CHECK("S2-U25", test_motion_stop_all(),
		   "latched STOP in every state");
	TEST_CHECK("S2-U26", test_motion_reject_stop(),
		   "motion commands rejected while STOP latched");
	TEST_CHECK("S2-U27", test_motion_replace(),
		   "replace only while STOP clear; stale IDs rejected");
	TEST_CHECK("S2-U28", test_motion_stop_timing(),
		   "stop confirmation timing, query and 2 s limit");
	TEST_CHECK("S2-U29", test_motion_stall(),
		   "stalled wheel gives Fault(STALL)");
	TEST_CHECK("S2-U30", test_motion_timeout(),
		   "slow MOVE gives Fault(TIMEOUT)");
	TEST_CHECK("S2-U31", test_motion_encoder_fault(),
		   "bad snapshots or decode errors give Fault(ENCODER)");
	TEST_CHECK("S2-U32", test_motion_recovery(),
		   "Fault cleared only by RESET with wheels still");
	TEST_CHECK("S2-U33", test_motion_params(),
		   "invalid motion parameters rejected");
	TEST_CHECK("S2-U34", test_motion_odometry(),
		   "odometry travel, pose and NEW_RUN");
	TEST_CHECK("S2-U35", test_motion_api(),
		   "motion API builds and range-checks commands");

#if S2_HW_TEST
	hw_test();
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
