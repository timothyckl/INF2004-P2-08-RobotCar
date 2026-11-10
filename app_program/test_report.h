/*
 *	test_report.h
 *	Minimal on-target test helpers.
 *
 *	Every check prints ONE line in a fixed format, so a saved console log
 *	can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *	    [TEST] S1-U01 PASS  ADC reads in range
 *	    [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *	    [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *	Usage, inside a test function:
 *	    TEST_INIT();
 *	    TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in range");
 *	    TEST_SUMMARY("subsystem1");
 *
 *	Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 *
 *	USB console flow control: the console ring (4 KB) drops output when it
 *	is full, and the tests run in the priority-1 initial task, so a burst
 *	of checks can outrun the USB task (2026-11-04: S2-U64..U66 and the S2
 *	summary were lost).  TEST_DRAIN() therefore sleeps, 10 ms at a time,
 *	until at most half the ring is queued.  It gives up after 1 s so that
 *	a run without a host reading the port still finishes.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
IMPORT UW		tm_usb_pending(void);	/* usb_console.c */
IMPORT volatile UW	tm_usb_dropped;		/* usb_console.c */

#define TEST_DRAIN_LIMIT	2048U	/* bytes queued: half the ring */
#define TEST_DRAIN_WAIT_MS	10	/* per sleep */
#define TEST_DRAIN_TRIES	100	/* 100 x 10 ms = 1 s at most */

#define TEST_DRAIN()							\
	do {								\
		INT test_try_;						\
		for (test_try_ = 0; (test_try_ < TEST_DRAIN_TRIES) &&	\
		     (tm_usb_pending() > TEST_DRAIN_LIMIT); test_try_++) { \
			(void)tk_dly_tsk(TEST_DRAIN_WAIT_MS);		\
		}							\
	} while (0)

#define TEST_DROPPED()	((UW)tm_usb_dropped)	/* bytes lost since boot */
#else
#define TEST_DRAIN()	do { } while (0)
#define TEST_DROPPED()	(0U)
#endif

#define TEST_INIT()	INT test_pass_ = 0, test_fail_ = 0

#define TEST_CHECK(id, cond, desc)					\
	do {								\
		TEST_DRAIN();						\
		if (cond) {						\
			test_pass_++;					\
			tm_printf((UB*)"[TEST] %s PASS  %s\n", (id), (desc)); \
		} else {						\
			test_fail_++;					\
			tm_printf((UB*)"[TEST] %s FAIL  %s   (%s:%d)\n",	\
				  (id), (desc), __FILE__, __LINE__);	\
		}							\
	} while (0)

#define TEST_SUMMARY(name)						\
	do {								\
		TEST_DRAIN();						\
		tm_printf((UB*)"[TEST] %s SUMMARY %d passed, %d failed\n", \
			  (name), test_pass_, test_fail_);		\
	} while (0)

#endif /* TEST_REPORT_H */
