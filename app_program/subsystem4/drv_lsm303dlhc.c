/**
 * @file drv_lsm303dlhc.c
 * @brief LSM303DLHC accelerometer access through the kernel I2C driver.
 */
#include "drv_lsm303dlhc.h"

#define S4_LSM303_STATUS_A_REG        0x27u
#define S4_LSM303_OUT_X_L_A_REG       0x28u
#define S4_LSM303_CTRL_REG1_A         0x20u
#define S4_LSM303_CTRL_REG4_A         0x23u
#define S4_LSM303_IRA_REG_M           0x0au
#define S4_LSM303_IRB_REG_M           0x0bu
#define S4_LSM303_IRC_REG_M           0x0cu

#define S4_LSM303_CTRL1_100HZ_XYZ     0x57u
#define S4_LSM303_CTRL4_BDU_HR_2G     0x88u
#define S4_LSM303_STATUS_DATA_READY    0x08u
#define S4_LSM303_STATUS_OVERRUN       0x80u
#define S4_LSM303_AUTO_INCREMENT       0x80u

static ER s4_lsm303_configure(s4_lsm303_driver_t const * p_driver);
static ER s4_lsm303_read_registers(ID device_id,
                                   uint8_t address,
                                   uint8_t register_address,
                                   uint8_t * p_data,
                                   uint16_t data_size);
static ER s4_lsm303_write_register(ID device_id,
                                   uint8_t address,
                                   uint8_t register_address,
                                   uint8_t data);
static int16_t s4_lsm303_decode_axis(uint8_t low_byte,
                                     uint8_t high_byte);

void
s4_lsm303_init (s4_lsm303_driver_t * p_driver)
{
    if (NULL != p_driver)
    {
        p_driver->device_id = 0;
        p_driver->b_open = false;
    }
}

ER
s4_lsm303_open (s4_lsm303_driver_t * p_driver)
{
    static UB device_name[] = "iica";
    ER error = E_PAR;

    if (NULL != p_driver)
    {
        if (p_driver->b_open)
        {
            error = E_OBJ;
        }
        else
        {
            p_driver->device_id = tk_opn_dev(device_name, TD_UPDATE);
            if (p_driver->device_id < E_OK)
            {
                error = p_driver->device_id;
            }
            else
            {
                p_driver->b_open = true;
                error = s4_lsm303_configure(p_driver);
                if (error < E_OK)
                {
                    (void) s4_lsm303_close(p_driver);
                }
            }
        }
    }

    return (error);
}

ER
s4_lsm303_close (s4_lsm303_driver_t * p_driver)
{
    ER error = E_PAR;

    if (NULL != p_driver)
    {
        if (p_driver->b_open)
        {
            error = tk_cls_dev(p_driver->device_id, 0u);
            p_driver->device_id = 0;
            p_driver->b_open = false;
        }
        else
        {
            error = E_OK;
        }
    }

    return (error);
}

ER
s4_lsm303_read_identity (s4_lsm303_driver_t const * p_driver,
                         s4_lsm303_identity_t * p_identity)
{
    ER error = E_PAR;

    if ((NULL != p_driver) && p_driver->b_open &&
        (NULL != p_identity))
    {
        error = s4_lsm303_read_registers(p_driver->device_id,
                                         S4_LSM303_MAG_ADDRESS,
                                         S4_LSM303_IRA_REG_M,
                                         &p_identity->id_a, 1u);
        if (E_OK == error)
        {
            error = s4_lsm303_read_registers(p_driver->device_id,
                                             S4_LSM303_MAG_ADDRESS,
                                             S4_LSM303_IRB_REG_M,
                                             &p_identity->id_b, 1u);
        }
        if (E_OK == error)
        {
            error = s4_lsm303_read_registers(p_driver->device_id,
                                             S4_LSM303_MAG_ADDRESS,
                                             S4_LSM303_IRC_REG_M,
                                             &p_identity->id_c, 1u);
        }
    }

    return (error);
}

ER
s4_lsm303_read_acceleration (s4_lsm303_driver_t const * p_driver,
                             uint32_t capture_time_us,
                             s4_terrain_sample_t * p_sample)
{
    uint8_t status = 0u;
    uint8_t raw_data[6] = {0u};
    ER error = E_PAR;

    if ((NULL != p_driver) && p_driver->b_open && (NULL != p_sample))
    {
        p_sample->b_valid = false;
        error = s4_lsm303_read_registers(p_driver->device_id,
                                         S4_LSM303_ACCEL_ADDRESS,
                                         S4_LSM303_STATUS_A_REG,
                                         &status, 1u);
        if ((E_OK == error) &&
            (0u != (status & S4_LSM303_STATUS_OVERRUN)))
        {
            error = E_IO;
        }
        else if ((E_OK == error) &&
                 (0u == (status & S4_LSM303_STATUS_DATA_READY)))
        {
            error = E_BUSY;
        }
        else if (E_OK == error)
        {
            error = s4_lsm303_read_registers(
                p_driver->device_id, S4_LSM303_ACCEL_ADDRESS,
                S4_LSM303_OUT_X_L_A_REG | S4_LSM303_AUTO_INCREMENT,
                raw_data, sizeof(raw_data));
        }
        else
        {
            /* The I2C error is returned unchanged. */
        }

        if ((E_OK == error) &&
            (!s4_lsm303_decode_acceleration(raw_data, capture_time_us,
                                            p_sample)))
        {
            error = E_IO;
        }
    }

    return (error);
}

bool
s4_lsm303_decode_acceleration (uint8_t const raw_data[6],
                               uint32_t capture_time_us,
                               s4_terrain_sample_t * p_sample)
{
    bool b_decoded = false;

    if ((NULL != raw_data) && (NULL != p_sample))
    {
        p_sample->acceleration_x_mg =
            s4_lsm303_decode_axis(raw_data[0], raw_data[1]);
        p_sample->acceleration_y_mg =
            s4_lsm303_decode_axis(raw_data[2], raw_data[3]);
        p_sample->acceleration_z_mg =
            s4_lsm303_decode_axis(raw_data[4], raw_data[5]);
        p_sample->capture_time_us = capture_time_us;
        p_sample->b_valid = true;
        b_decoded = true;
    }

    return (b_decoded);
}

static ER
s4_lsm303_configure (s4_lsm303_driver_t const * p_driver)
{
    s4_lsm303_identity_t identity = {0u, 0u, 0u};
    uint8_t register_value = 0u;
    ER error = s4_lsm303_read_identity(p_driver, &identity);

    if ((E_OK == error) &&
        ((S4_LSM303_ID_A_VALUE != identity.id_a) ||
         (S4_LSM303_ID_B_VALUE != identity.id_b) ||
         (S4_LSM303_ID_C_VALUE != identity.id_c)))
    {
        error = E_NOEXS;
    }
    if (E_OK == error)
    {
        error = s4_lsm303_write_register(p_driver->device_id,
                                         S4_LSM303_ACCEL_ADDRESS,
                                         S4_LSM303_CTRL_REG1_A,
                                         S4_LSM303_CTRL1_100HZ_XYZ);
    }
    if (E_OK == error)
    {
        error = s4_lsm303_write_register(p_driver->device_id,
                                         S4_LSM303_ACCEL_ADDRESS,
                                         S4_LSM303_CTRL_REG4_A,
                                         S4_LSM303_CTRL4_BDU_HR_2G);
    }
    if (E_OK == error)
    {
        error = s4_lsm303_read_registers(p_driver->device_id,
                                         S4_LSM303_ACCEL_ADDRESS,
                                         S4_LSM303_CTRL_REG1_A,
                                         &register_value, 1u);
    }
    if ((E_OK == error) &&
        (S4_LSM303_CTRL1_100HZ_XYZ != register_value))
    {
        error = E_IO;
    }
    if (E_OK == error)
    {
        error = s4_lsm303_read_registers(p_driver->device_id,
                                         S4_LSM303_ACCEL_ADDRESS,
                                         S4_LSM303_CTRL_REG4_A,
                                         &register_value, 1u);
    }
    if ((E_OK == error) &&
        (S4_LSM303_CTRL4_BDU_HR_2G != register_value))
    {
        error = E_IO;
    }

    return (error);
}

static ER
s4_lsm303_read_registers (ID device_id,
                          uint8_t address,
                          uint8_t register_address,
                          uint8_t * p_data,
                          uint16_t data_size)
{
    UB send_data = register_address;
    SZ actual_size = 0;
    T_I2C_EXEC transaction =
    {
        .sadr = address,
        .snd_size = 1,
        .snd_data = &send_data,
        .rcv_size = data_size,
        .rcv_data = p_data
    };
    ER error = E_PAR;

    if ((NULL != p_data) && (data_size > 0u))
    {
        error = tk_swri_dev(device_id, TDN_I2C_EXEC, &transaction,
                            sizeof(transaction), &actual_size);
        if ((E_OK == error) &&
            (actual_size != (SZ) sizeof(transaction)))
        {
            error = E_IO;
        }
    }

    return (error);
}

static ER
s4_lsm303_write_register (ID device_id,
                          uint8_t address,
                          uint8_t register_address,
                          uint8_t data)
{
    return (i2c_write_reg(device_id, address, register_address, data));
}

static int16_t
s4_lsm303_decode_axis (uint8_t low_byte,
                       uint8_t high_byte)
{
    uint16_t raw_word = ((uint16_t) high_byte << 8u) | low_byte;
    int32_t signed_word = raw_word;

    if (0u != (raw_word & 0x8000u))
    {
        signed_word -= 65536;
    }

    /* High-resolution two-g data is 12-bit, left justified, at 1 mg/LSB. */
    return ((int16_t) (signed_word / 16));
}

/* End of file. */
