#include "ina219_driver.h"

/* INA219 uzywa big-endian (MSB first). */
static HAL_StatusTypeDef write_reg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(INA219_ADDR << 1), buf, 3, 10);
}

static HAL_StatusTypeDef read_reg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(INA219_ADDR << 1), &reg, 1, 10) != HAL_OK)
        return HAL_ERROR;
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(INA219_ADDR << 1), buf, 2, 10) != HAL_OK)
        return HAL_ERROR;
    *val = (uint16_t)((buf[0] << 8) | buf[1]);
    return HAL_OK;
}

// ---------------------------------------------------------------------------
bool INA219_Init(I2C_HandleTypeDef *hi2c)
{
    /* Config: BRNG=1 (32V bus), PG=11 (320 mV shunt gain), BADC=1111 (128 avg),
     *         SADC=1111 (128 avg), MODE=111 (continuous shunt+bus) → 0x3FFF */
    if (write_reg16(hi2c, INA219_REG_CONFIG, 0x3FFF) != HAL_OK) return false;

    /* Kalibracja: ustawia LSB pradu i mocy zgodnie z R_SHUNT i MAX_CURRENT */
    if (write_reg16(hi2c, INA219_REG_CALIB, INA219_CALIB_VAL) != HAL_OK) return false;

    return true;
}

// ---------------------------------------------------------------------------
bool INA219_Read(I2C_HandleTypeDef *hi2c, INA219_Data_t *data)
{
    uint16_t raw;

    /* Napiecie szyny: bity [15:3], LSB = 4 mV */
    if (read_reg16(hi2c, INA219_REG_BUS_V, &raw) != HAL_OK) return false;
    data->voltage_V = (float)(raw >> 3) * 0.004f;

    /* Prad: signed 16-bit, LSB = INA219_CURRENT_LSB [A] */
    if (read_reg16(hi2c, INA219_REG_CURRENT, &raw) != HAL_OK) return false;
    data->current_A = (float)(int16_t)raw * INA219_CURRENT_LSB;

    /* Moc: unsigned, LSB = 20 × INA219_CURRENT_LSB [W] */
    if (read_reg16(hi2c, INA219_REG_POWER, &raw) != HAL_OK) return false;
    data->power_W = (float)raw * (20.0f * INA219_CURRENT_LSB);

    return true;
}
