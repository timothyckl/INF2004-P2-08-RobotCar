/**
 * @file drv_ir_line.h
 * @brief GPIO owner for the two line-following IR digital outputs.
 */

#ifndef DRV_IR_LINE_H
#define DRV_IR_LINE_H

#include <stdbool.h>
#include <tk/tkernel.h>

typedef struct
{
    bool b_left_raw;
    bool b_right_raw;
} drv_ir_line_sample_t;

/** Configure Grove 5 GP6 as left and Grove 4 GP16 as right inputs. */
ER drv_ir_line_init(void);

/** Read both digital comparator outputs from their owning task. */
ER drv_ir_line_read(drv_ir_line_sample_t * p_sample);

#endif /* DRV_IR_LINE_H */

/* End of file. */
