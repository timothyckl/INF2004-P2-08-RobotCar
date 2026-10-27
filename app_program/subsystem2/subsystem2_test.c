/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	S2-U03..S2-U14 test the pure PID and odometry logic with synthetic
 *	gains and geometry; they need no motors or encoders.
 */

#include "subsystem2.h"
#include "pid.h"
#include "odometry.h"
#include "../test_report.h"

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

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
