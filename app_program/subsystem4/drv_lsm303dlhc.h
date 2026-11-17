/**
 * @file drv_lsm303dlhc.h
 * @brief LSM303DLHC accelerometer access through the kernel I2C driver.
 */
#ifndef S4_DRV_LSM303DLHC_H
#define S4_DRV_LSM303DLHC_H

#include <tk/tkernel.h>
#include <tk/device.h>

#include "terrain_logic.h"

#define S4_LSM303_ACCEL_ADDRESS    0x19u
#define S4_LSM303_MAG_ADDRESS      0x1eu
#define S4_LSM303_ID_A_VALUE       0x48u
#define S4_LSM303_ID_B_VALUE       0x34u
#define S4_LSM303_ID_C_VALUE       0x33u

typedef struct
{
    uint8_t id_a;
    uint8_t id_b;
    uint8_t id_c;
} s4_lsm303_identity_t;

typedef struct
{
    ID      device_id;
    bool    b_open;
} s4_lsm303_driver_t;

void s4_lsm303_init(s4_lsm303_driver_t * p_driver);
ER s4_lsm303_open(s4_lsm303_driver_t * p_driver);
ER s4_lsm303_close(s4_lsm303_driver_t * p_driver);
ER s4_lsm303_read_identity(s4_lsm303_driver_t const * p_driver,
                           s4_lsm303_identity_t * p_identity);
ER s4_lsm303_read_acceleration(s4_lsm303_driver_t const * p_driver,
                               uint32_t capture_time_us,
                               s4_terrain_sample_t * p_sample);
bool s4_lsm303_decode_acceleration(uint8_t const raw_data[6],
                                   uint32_t capture_time_us,
                                   s4_terrain_sample_t * p_sample);

#endif /* S4_DRV_LSM303DLHC_H */
