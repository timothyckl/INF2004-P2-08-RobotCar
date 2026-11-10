/**
 * @file scan_control.h
 * @brief Pure Scan & Plan handshake for the Vehicle Controller (IF-11).
 *
 * A hazard while following starts a scan: stop the wheels (motion STOP,
 * not the latched safety STOP) and send REQUEST. Once both wheels are
 * still, send STOP_CONFIRM and AUTHORISE. Then wait for subsystem 5's
 * result for this request ID (IF-13), up to limit_ms; on timeout send
 * CANCEL. A stop or fault from elsewhere ends the scan with a STOP
 * message. The caller sends the messages on scan_q and acts on the
 * outcome; this module touches no kernel object.
 */
#ifndef SCAN_CONTROL_H
#define SCAN_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define SCAN_CONTROL_MSGS_MAX       3u

/* Subsystem 5 result codes (scan_task.h SCAN_TASK_RESULT_*). */
#define SCAN_CONTROL_RESULT_NONE    0u
#define SCAN_CONTROL_RESULT_PLAN    1u
#define SCAN_CONTROL_RESULT_CLEAR   2u

typedef enum
{
    SCAN_CONTROL_IDLE = 0,
    SCAN_CONTROL_STOPPING,      /* REQUEST sent, waiting for stillness */
    SCAN_CONTROL_SCANNING       /* authorised, waiting for the result */
} scan_control_state_t;

typedef enum
{
    SCAN_CONTROL_NONE = 0,
    SCAN_CONTROL_PLAN,          /* a usable bypass plan */
    SCAN_CONTROL_CLEAR,         /* nothing in range ahead any more */
    SCAN_CONTROL_FAILED,        /* no route, fault or cancelled */
    SCAN_CONTROL_TIMEOUT        /* no result within limit_ms */
} scan_control_outcome_t;

typedef struct
{
    uint8_t kind;               /* S2_SCAN_* */
    bool    b_asserted;
} scan_control_msg_t;

/** Results of one call; cleared by every call. */
typedef struct
{
    scan_control_msg_t     msg[SCAN_CONTROL_MSGS_MAX];
    uint8_t                n_msg;
    bool                   b_stop_motion;   /* send a motion STOP */
    scan_control_outcome_t outcome;
} scan_control_output_t;

typedef struct
{
    uint32_t             limit_ms;
    uint32_t             request_id;    /* never 0 once a scan began */
    uint32_t             since_ms;
    scan_control_state_t state;
} scan_control_t;

/** Initialise idle. limit_ms must be nonzero. */
bool scan_control_init(scan_control_t * p_scan, uint32_t limit_ms);

/** True while a scan is in progress (STOPPING or SCANNING). */
bool scan_control_active(scan_control_t const * p_scan);

/** Start a scan with a new request ID; ignored while one is active. */
void scan_control_begin(scan_control_t * p_scan, uint32_t now_ms,
                        scan_control_output_t * p_out);

/**
 * Advance an active scan. b_still: both wheels confirmed still.
 * b_result_ok, result_id, result: subsystem 5's latest snapshot.
 */
void scan_control_step(scan_control_t * p_scan, bool b_still,
                       bool b_result_ok, uint32_t result_id, uint8_t result,
                       uint32_t now_ms, scan_control_output_t * p_out);

/** End an active scan because of a stop or fault elsewhere. */
void scan_control_abort(scan_control_t * p_scan,
                        scan_control_output_t * p_out);

#endif /* SCAN_CONTROL_H */

/* End of file. */
