/* ina219_driver.h
 *
 * Abstrakcja sprzetu czujnika pradu/napiecia INA219AxD.
 * Implementacja oparta na sterowniku ina219_reg (styl ST MEMS / stmdev_ctx_t).
 * Interfejs: I2C3, adres A0=A1=GND (domyslny).
 *
 * Na tej samej magistrali I2C3 znajduje sie rowniez:
 *   FRAM MB85RC  adres 0x50  — bez implementacji funkcjonalnej
 */

#ifndef INA219_DRIVER_H_
#define INA219_DRIVER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

#include "ina219_reg.h"

/* 7-bit I2C address alias — backward-compatible with ina219_task.c probes.
 * INA219_ADDR << 1 == INA219_I2C_ADD_GND_GND == 0x80 */
#define INA219_ADDR  (INA219_I2C_ADD_GND_GND >> 1)   /* 0x40 */

/* --------------------------------------------------------------------------
 * Parametry bocznika — dostosuj do hardware'u
 *   CurrentLSB = MAX_CURRENT_A / 32768
 *   Cal        = trunc(0.04096 / (CurrentLSB × R_SHUNT_OHM))
 *
 * Przy R=0.1 Ω, I_max=3.2 A:
 *   CurrentLSB ≈ 97.66 µA/bit
 *   Cal        = 4194
 * -------------------------------------------------------------------------- */
#define INA219_R_SHUNT_OHM      0.1f
#define INA219_MAX_CURRENT_A    3.2f
#define INA219_CURRENT_LSB      (INA219_MAX_CURRENT_A / 32768.0f)
#define INA219_CALIB_VAL        ((uint16_t)(0.04096f / (INA219_CURRENT_LSB * INA219_R_SHUNT_OHM)))

typedef struct {
    float voltage_V;
    float current_A;
    float power_W;
} INA219_Data_t;

/* Konfiguruje CONFIG i CALIBRATION przez ina219_reg API.
 * Wywolac raz przed pierwszym odczytem.
 * Zwraca false przy bledzie I2C lub niedopasowanym WHO_AM_I. */
bool INA219_Init(I2C_HandleTypeDef *hi2c);

/* Odczytuje napiecie szyny, prad i moc.
 * Zwraca false przy bledzie I2C (data pozostaje bez zmian). */
bool INA219_Read(I2C_HandleTypeDef *hi2c, INA219_Data_t *data);

#endif /* INA219_DRIVER_H_ */
