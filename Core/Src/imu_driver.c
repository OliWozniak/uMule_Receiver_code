#include "imu_driver.h"

static HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr7 << 1),
                             reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

static HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7, uint8_t reg,
                                        uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr7 << 1),
                            reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

// ---------------------------------------------------------------------------
bool IMU_Init(I2C_HandleTypeDef *hi2c)
{
    /* --- LSM6DS3TR-C ---
     * CTRL3_C  0x44: BDU=1 (blok danych), IF_INC=1 (auto-increment adresu)
     * CTRL1_XL 0x4C: ODR=104 Hz (0100), FS=±8g (11)
     * CTRL2_G  0x4C: ODR=104 Hz (0100), FS=±2000 dps (11)
     */
    if (i2c_write_reg(hi2c, LSM6DS3_ADDR, LSM6_CTRL3_C,  0x44) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LSM6DS3_ADDR, LSM6_CTRL1_XL, 0x4C) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LSM6DS3_ADDR, LSM6_CTRL2_G,  0x4C) != HAL_OK) return false;

    /* --- LIS3MDL ---
     * CTRL_REG1 0x70: OM=ultra (11), DO=10 Hz (100)
     * CTRL_REG2 0x00: FS=±4 gauss
     * CTRL_REG3 0x00: MD=continuous
     * CTRL_REG4 0x0C: OMZ=ultra (11)
     * CTRL_REG5 0x40: BDU=1
     */
    if (i2c_write_reg(hi2c, LIS3MDL_ADDR, LIS3_CTRL_REG1, 0x70) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LIS3MDL_ADDR, LIS3_CTRL_REG2, 0x00) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LIS3MDL_ADDR, LIS3_CTRL_REG3, 0x00) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LIS3MDL_ADDR, LIS3_CTRL_REG4, 0x0C) != HAL_OK) return false;
    if (i2c_write_reg(hi2c, LIS3MDL_ADDR, LIS3_CTRL_REG5, 0x40) != HAL_OK) return false;

    return true;
}

// ---------------------------------------------------------------------------
bool IMU_Read(I2C_HandleTypeDef *hi2c, IMU_Data_t *data)
{
    uint8_t buf[6];
    bool ok = false;

    /* Gyro: 6 bajtow od OUTX_L_G.
     * IF_INC aktywny (ustawiony w CTRL3_C) — adres bez MSB. */
    if (i2c_read_regs(hi2c, LSM6DS3_ADDR, LSM6_OUTX_L_G, buf, 6) == HAL_OK) {
        data->gyro_x = (float)(int16_t)(buf[0] | (buf[1] << 8)) * LSM6_GYRO_SENS_RAD_S;
        data->gyro_y = (float)(int16_t)(buf[2] | (buf[3] << 8)) * LSM6_GYRO_SENS_RAD_S;
        data->gyro_z = (float)(int16_t)(buf[4] | (buf[5] << 8)) * LSM6_GYRO_SENS_RAD_S;
        ok = true;
    }

    /* Accel: 6 bajtow od OUTX_L_XL. */
    if (i2c_read_regs(hi2c, LSM6DS3_ADDR, LSM6_OUTX_L_XL, buf, 6) == HAL_OK) {
        data->accel_x = (float)(int16_t)(buf[0] | (buf[1] << 8)) * LSM6_ACCEL_SENS_MS2;
        data->accel_y = (float)(int16_t)(buf[2] | (buf[3] << 8)) * LSM6_ACCEL_SENS_MS2;
        data->accel_z = (float)(int16_t)(buf[4] | (buf[5] << 8)) * LSM6_ACCEL_SENS_MS2;
        ok = true;
    }

    /* Mag: LIS3MDL wymaga MSB=1 w adresie rejestru dla odczytu multi-bajtowego w I2C.
     * HAL_I2C_Mem_Read wysyla adres rejestru jako 1 bajt — ustawiamy bit 7. */
    if (i2c_read_regs(hi2c, LIS3MDL_ADDR, LIS3_OUT_X_L | 0x80, buf, 6) == HAL_OK) {
        data->mag_x = (float)(int16_t)(buf[0] | (buf[1] << 8)) * LIS3_MAG_SENS_T;
        data->mag_y = (float)(int16_t)(buf[2] | (buf[3] << 8)) * LIS3_MAG_SENS_T;
        data->mag_z = (float)(int16_t)(buf[4] | (buf[5] << 8)) * LIS3_MAG_SENS_T;
        ok = true;
    }

    return ok;
}
