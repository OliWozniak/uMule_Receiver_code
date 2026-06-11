/**
 * @file    imu_driver.c
 * @brief   IMU hardware abstraction using official ST MEMS C drivers.
 *
 * Platform adapter functions wrap HAL_I2C_Mem_Write/Read.
 * Each sensor has its own stmdev_ctx_t with the correct I2C address.
 *
 * Sensors:
 *   LSM6DS3TR-C  — accelerometer + gyroscope (I2C4, ADD_L: SDO=GND)
 *   LIS3MDL      — magnetometer              (I2C4, ADD_L: SA1=GND)
 */

#include "imu_driver.h"

/* --------------------------------------------------------------------------
 * Platform adapter functions
 * Signature must match stmdev_write_ptr / stmdev_read_ptr:
 *   int32_t fn(void *handle, uint8_t reg, [const] uint8_t *buf, uint16_t len)
 * Return 0 on success, non-zero on error.
 * -------------------------------------------------------------------------- */

static int32_t lsm6_write(void *handle, uint8_t reg,
                           const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)handle,
        LSM6DS3TR_C_I2C_ADD_L,          /* 8-bit address (HAL ignores LSB) */
        reg, I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)buf, len, 50);
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t lsm6_read(void *handle, uint8_t reg,
                          uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)handle,
        LSM6DS3TR_C_I2C_ADD_L,
        reg, I2C_MEMADD_SIZE_8BIT,
        buf, len, 50);
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t lis3_write(void *handle, uint8_t reg,
                           const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)handle,
        LIS3MDL_I2C_ADD_L,
        reg, I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)buf, len, 50);
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t lis3_read(void *handle, uint8_t reg,
                          uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)handle,
        LIS3MDL_I2C_ADD_L,
        reg, I2C_MEMADD_SIZE_8BIT,
        buf, len, 50);
    return (s == HAL_OK) ? 0 : -1;
}

/* --------------------------------------------------------------------------
 * Static contexts — initialised once in IMU_Init()
 * -------------------------------------------------------------------------- */
static stmdev_ctx_t lsm6_ctx;
static stmdev_ctx_t lis3_ctx;
static bool         ctx_ready = false;

static void ctx_init(I2C_HandleTypeDef *hi2c)
{
    lsm6_ctx.write_reg = lsm6_write;
    lsm6_ctx.read_reg  = lsm6_read;
    lsm6_ctx.mdelay    = NULL;
    lsm6_ctx.handle    = hi2c;

    lis3_ctx.write_reg = lis3_write;
    lis3_ctx.read_reg  = lis3_read;
    lis3_ctx.mdelay    = NULL;
    lis3_ctx.handle    = hi2c;

    ctx_ready = true;
}

/* --------------------------------------------------------------------------
 * IMU_Init
 * -------------------------------------------------------------------------- */
bool IMU_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    bool ok = true;

    ctx_init(hi2c);

    /* --- LSM6DS3TR-C ------------------------------------------------------- */
    /* Verify WHO_AM_I */
    lsm6ds3tr_c_device_id_get(&lsm6_ctx, &id);
    if (id != LSM6DS3TR_C_ID) ok = false;

    /* Block data update — output registers not updated until both MSB/LSB read */
    lsm6ds3tr_c_block_data_update_set(&lsm6_ctx, PROPERTY_ENABLE);

    /* Full scale: ±8 g, ±2000 dps */
    lsm6ds3tr_c_xl_full_scale_set(&lsm6_ctx, LSM6DS3TR_C_8g);
    lsm6ds3tr_c_gy_full_scale_set(&lsm6_ctx, LSM6DS3TR_C_2000dps);

    /* Output data rate: 104 Hz */
    lsm6ds3tr_c_xl_data_rate_set(&lsm6_ctx, LSM6DS3TR_C_XL_ODR_104Hz);
    lsm6ds3tr_c_gy_data_rate_set(&lsm6_ctx, LSM6DS3TR_C_GY_ODR_104Hz);

    /* --- LIS3MDL ----------------------------------------------------------- */
    lis3mdl_device_id_get(&lis3_ctx, &id);
    if (id != LIS3MDL_ID) ok = false;

    lis3mdl_block_data_update_set(&lis3_ctx, PROPERTY_ENABLE);

    /* Full scale ±4 gauss */
    lis3mdl_full_scale_set(&lis3_ctx, LIS3MDL_4_GAUSS);

    /* Data rate: HP_10Hz (high-performance 10 Hz) */
    lis3mdl_data_rate_set(&lis3_ctx, LIS3MDL_HP_10Hz);

    /* Continuous conversion mode */
    lis3mdl_operating_mode_set(&lis3_ctx, LIS3MDL_CONTINUOUS_MODE);

    return ok;
}

/* --------------------------------------------------------------------------
 * IMU_Read
 * -------------------------------------------------------------------------- */
bool IMU_Read(I2C_HandleTypeDef *hi2c, IMU_Data_t *data)
{
    int16_t raw[3];
    bool ok = false;

    /* Re-attach handle in case hi2c changed (defensive). */
    if (!ctx_ready) ctx_init(hi2c);
    lsm6_ctx.handle = hi2c;
    lis3_ctx.handle = hi2c;

    /* Gyroscope */
    if (lsm6ds3tr_c_angular_rate_raw_get(&lsm6_ctx, raw) == 0) {
        data->gyro_x = (float)raw[0] * LSM6_GYRO_SENS_RAD_S;
        data->gyro_y = (float)raw[1] * LSM6_GYRO_SENS_RAD_S;
        data->gyro_z = (float)raw[2] * LSM6_GYRO_SENS_RAD_S;
        ok = true;
    }

    /* Accelerometer */
    if (lsm6ds3tr_c_acceleration_raw_get(&lsm6_ctx, raw) == 0) {
        data->accel_x = (float)raw[0] * LSM6_ACCEL_SENS_MS2;
        data->accel_y = (float)raw[1] * LSM6_ACCEL_SENS_MS2;
        data->accel_z = (float)raw[2] * LSM6_ACCEL_SENS_MS2;
        ok = true;
    }

    /* Magnetometer */
    if (lis3mdl_magnetic_raw_get(&lis3_ctx, raw) == 0) {
        data->mag_x = (float)raw[0] * LIS3_MAG_SENS_T;
        data->mag_y = (float)raw[1] * LIS3_MAG_SENS_T;
        data->mag_z = (float)raw[2] * LIS3_MAG_SENS_T;
        ok = true;
    }

    return ok;
}
