/**
 * @file    ina219_reg.c
 * @brief   INA219 current/power monitor — register-level driver.
 *
 * All registers are 16-bit, big-endian (MSB first on the wire).
 * The read/write function pointers in ina219_ctx_t operate on raw byte
 * arrays, so this file handles endianness conversion.
 */

#include "ina219_reg.h"

/* --------------------------------------------------------------------------
 * Low-level helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read a single 16-bit register (big-endian → host).
 */
int32_t ina219_read_reg(const ina219_ctx_t *ctx, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2] = {0, 0};
    int32_t ret = ctx->read_reg(ctx->handle, reg, buf, 2);
    if (ret == 0)
        *val = (uint16_t)((uint16_t)(buf[0] << 8) | buf[1]);
    return ret;
}

/**
 * @brief  Write a single 16-bit register (host → big-endian).
 */
int32_t ina219_write_reg(const ina219_ctx_t *ctx, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFFU) };
    return ctx->write_reg(ctx->handle, reg, buf, 2);
}

/* --------------------------------------------------------------------------
 * Configuration helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief  Issue a software reset.  The reset bit self-clears; we verify
 *         by reading back until it is cleared (or after a short delay).
 */
int32_t ina219_reset(const ina219_ctx_t *ctx)
{
    int32_t  ret;
    uint16_t cfg;

    ret = ina219_write_reg(ctx, INA219_REG_CONFIG, INA219_CONFIG_RESET);
    if (ret != 0) return ret;

    /* Poll until the reset bit clears (typically < 1 ms). */
    do {
        ret = ina219_read_reg(ctx, INA219_REG_CONFIG, &cfg);
        if (ret != 0) return ret;
    } while (cfg & INA219_CONFIG_RESET);

    return 0;
}

/**
 * @brief  Write the calibration register.
 *
 *   Cal = trunc(0.04096 / (CurrentLSB × R_shunt))
 *
 * Example: R_shunt = 0.1 Ω, I_max = 3.2 A →
 *   CurrentLSB = 3.2 / 32768 ≈ 97.66 µA  →  Cal = 4194
 */
int32_t ina219_calibration_set(const ina219_ctx_t *ctx, uint16_t cal)
{
    return ina219_write_reg(ctx, INA219_REG_CALIBRATION, cal);
}

/**
 * @brief  Write the CONFIG register.
 *
 * All enum values are pre-shifted to their correct bit positions so they
 * can be OR-ed directly.
 */
int32_t ina219_config_set(const ina219_ctx_t *ctx,
                           ina219_bus_range_t  brng,
                           ina219_gain_t       gain,
                           ina219_badc_t       badc,
                           ina219_sadc_t       sadc,
                           ina219_mode_t       mode)
{
    uint16_t cfg = (uint16_t)brng | (uint16_t)gain |
                   (uint16_t)badc | (uint16_t)sadc | (uint16_t)mode;
    return ina219_write_reg(ctx, INA219_REG_CONFIG, cfg);
}

/* --------------------------------------------------------------------------
 * Measurement read-back
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read bus voltage.
 *
 * The bus voltage register layout:
 *   bits [15:3] — VBUS measurement (LSB = 4 mV)
 *   bit  [1]    — CNVR (conversion ready)
 *   bit  [0]    — OVF (overflow)
 *
 * We shift right by 3 to get the raw count.  The caller converts:
 *   V_bus [V] = *val × 0.004f
 */
int32_t ina219_bus_voltage_raw_get(const ina219_ctx_t *ctx, int16_t *val)
{
    uint16_t raw;
    int32_t  ret = ina219_read_reg(ctx, INA219_REG_BUS_VOLTAGE, &raw);
    if (ret == 0)
        *val = (int16_t)(raw >> 3);
    return ret;
}

/**
 * @brief  Read shunt voltage (signed 16-bit, LSB = 10 µV).
 */
int32_t ina219_shunt_voltage_raw_get(const ina219_ctx_t *ctx, int16_t *val)
{
    uint16_t raw;
    int32_t  ret = ina219_read_reg(ctx, INA219_REG_SHUNT_VOLTAGE, &raw);
    if (ret == 0)
        *val = (int16_t)raw;
    return ret;
}

/**
 * @brief  Read current register (signed 16-bit).
 *   I [A] = *val × CurrentLSB
 */
int32_t ina219_current_raw_get(const ina219_ctx_t *ctx, int16_t *val)
{
    uint16_t raw;
    int32_t  ret = ina219_read_reg(ctx, INA219_REG_CURRENT, &raw);
    if (ret == 0)
        *val = (int16_t)raw;
    return ret;
}

/**
 * @brief  Read power register (unsigned 16-bit).
 *   P [W] = *val × 20 × CurrentLSB
 */
int32_t ina219_power_raw_get(const ina219_ctx_t *ctx, uint16_t *val)
{
    return ina219_read_reg(ctx, INA219_REG_POWER, val);
}
