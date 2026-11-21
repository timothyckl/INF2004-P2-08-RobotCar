/**
 * @file device.h
 * @brief Host stand-in for <tk/device.h>, for pure-logic tests only.
 *
 * Declares the device-management calls and the I2C attribute types that
 * subsystem 4's LSM303DLHC driver uses (include/tk/syscall.h and
 * device/include/dev_i2c.h). The calls in device_stub.c fail with
 * E_NOSPT: host cases may test the driver's pure decode helper only.
 * Firmware never includes this file (testing/host only).
 */
#ifndef HOST_DEVICE_H
#define HOST_DEVICE_H

#include <tk/tkernel.h>

typedef W SZ;

#define TD_UPDATE   (0x0003u)

typedef enum
{
    TDN_I2C_EXEC = -100
} T_DN_I2C_ATR;

typedef struct
{
    UW   sadr;
    SZ   snd_size;
    UB * snd_data;
    SZ   rcv_size;
    UB * rcv_data;
} T_I2C_EXEC;

ID tk_opn_dev (UB const * p_devnm, UINT omode);
ER tk_cls_dev (ID dd, UINT option);
ER tk_swri_dev (ID dd, W start, void const * p_buf, SZ size, SZ * p_asize);
ER i2c_write_reg (ID dd, UW sadr, UW radr, UB data);

#endif /* HOST_DEVICE_H */

/* End of file. */
