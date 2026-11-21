/**
 * @file device_stub.c
 * @brief Host stand-ins for the device-management and I2C calls.
 *
 * Every call fails with E_NOSPT: bus access is tested on the Pico W
 * (S4-H01 in a TEST=1 S4_HW_TEST=1 image), not here.
 */
#include <tk/device.h>

ID
tk_opn_dev (UB const * p_devnm, UINT omode)
{
    (void)p_devnm;
    (void)omode;
    return (E_NOSPT);
}

ER
tk_cls_dev (ID dd, UINT option)
{
    (void)dd;
    (void)option;
    return (E_NOSPT);
}

ER
tk_swri_dev (ID dd, W start, void const * p_buf, SZ size, SZ * p_asize)
{
    (void)dd;
    (void)start;
    (void)p_buf;
    (void)size;
    (void)p_asize;
    return (E_NOSPT);
}

ER
i2c_write_reg (ID dd, UW sadr, UW radr, UB data)
{
    (void)dd;
    (void)sadr;
    (void)radr;
    (void)data;
    return (E_NOSPT);
}

/* End of file. */
