/**
 * @file    ina219_driver.c
 * @brief   INA219 platform driver — HAL I2C adapter + high-level read/init.
 *
 * The low-level register access is handled by ina219_reg.c.
 * This file provides:
 *   - Platform adapter functions (HAL_I2C_Mem_Write/Read wrappers)
 *   - INA219_Init()  — configure measurement mode + calibration
 *   - INA219_Read()  — read voltage, current, power as physical floats
 */

#include "ina219_driver.h"

/* --------------------------------------------------------------------------
 * Platform adapter functions
 * -------------------------------------------------------------------------- */

static int32_t ina219_i2c_write(void *handle, uint8_t reg,
                                  const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)handle,
        INA219_I2C_ADD_GND_GND,         /* 0x80 — A0=GND, A1=GND */
        reg, I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)buf, len, 10);
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t ina219_i2c_read(void *handle, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)handle,
        INA219_I2C_ADD_GND_GND,
        reg, I2C_MEMADD_SIZE_8BIT,
        buf, len, 10);
    return (s == HAL_OK) ? 0 : -1;
}

/* --------------------------------------------------------------------------
 * Static context
 * -------------------------------------------------------------------------- */
static ina219_ctx_t ina219_ctx;
static bool         ctx_ready = false;

static void ctx_init(I2C_HandleTypeDef *hi2c)
{
    ina219_ctx.write_reg = ina219_i2c_write;
    ina219_ctx.read_reg  = ina219_i2c_read;
    ina219_ctx.mdelay    = NULL;
    ina219_ctx.handle    = hi2c;
    ctx_ready = true;
}

/* --------------------------------------------------------------------------
 * INA219_Init
 *
 * CONFIG: 32 V bus range, gain 1/8 (±320 mV shunt), 12-bit ADC,
 *         128 sample averaging on both channels, continuous shunt+bus.
 *         → register value 0x3FFF
 *
 * mindThomas setCalibration_32V_2A equivalent (adapted for 3.2 A):
 *   CAL = 4194  (R_shunt=0.1 Ω, I_max=3.2 A, CurrentLSB≈97.66 µA)
 * -------------------------------------------------------------------------- */
bool INA219_Init(I2C_HandleTypeDef *hi2c)
{
    ctx_init(hi2c);

    /* Configure: 32V range, ±320mV shunt, 12-bit 128-sample avg, continuous */
    if (ina219_config_set(
            &ina219_ctx,
            INA219_BUS_RANGE_32V,
            INA219_GAIN_8_320MV,
            INA219_BADC_12BIT,
            INA219_SADC_12BIT_128S,
            INA219_MODE_SHUNT_BUS_CONTINUOUS) != 0)
        return false;

    /* Calibration register */
    if (ina219_calibration_set(&ina219_ctx, INA219_CALIB_VAL) != 0)
        return false;

    return true;
}

/* --------------------------------------------------------------------------
 * INA219_Read
 * -------------------------------------------------------------------------- */
bool INA219_Read(I2C_HandleTypeDef *hi2c, INA219_Data_t *data)
{
    int16_t  raw_i;
    int16_t  raw_v;
    uint16_t raw_p;

    if (!ctx_ready) ctx_init(hi2c);
    ina219_ctx.handle = hi2c;  /* re-attach handle (defensive) */

    /* Bus voltage: raw count × 4 mV */
    if (ina219_bus_voltage_raw_get(&ina219_ctx, &raw_v) != 0) return false;
    data->voltage_V = (float)raw_v * 0.004f;

    /* Current: raw × CurrentLSB [A] */
    if (ina219_current_raw_get(&ina219_ctx, &raw_i) != 0) return false;
    data->current_A = (float)raw_i * INA219_CURRENT_LSB;

    /* Power: raw × 20 × CurrentLSB [W] */
    if (ina219_power_raw_get(&ina219_ctx, &raw_p) != 0) return false;
    data->power_W = (float)raw_p * (20.0f * INA219_CURRENT_LSB);

    return true;
}
