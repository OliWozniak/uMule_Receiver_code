/* ina219_driver.h
 *
 * Abstrakcja sprzetu czujnika pradu/napiecia INA219AxD
 * Interfejs: I2C3
 * Adres I2C (7-bit): 0x40 (A0=A1=GND, domyslny)
 *
 * Na tej samej magistrali I2C3 znajduje sie rowniez:
 *   FRAM MB85RC  adres 0x50  — bez implementacji funkcjonalnej
 */

#ifndef INA219_DRIVER_H_
#define INA219_DRIVER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* --- Adres I2C (7-bit) --- */
#define INA219_ADDR             0x40

/* --- Rejestry --- */
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_V      0x01
#define INA219_REG_BUS_V        0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIB        0x05

/* --- Parametry bocznika (dostosuj do swojego hardware'u) ---
 *
 * R_SHUNT   = rezystancja bocznika [Ω]
 * MAX_A     = maksymalny spodziewany prad [A] (wplywa na LSB pradu i kalibracjê)
 *
 * CurrentLSB = MAX_A / 32768
 * Cal        = 0.04096 / (CurrentLSB × R_SHUNT)   [wielkosc bezwymiarowa, uint16]
 *
 * Przy R=0.1 Ω, MAX=3.2 A:
 *   CurrentLSB = 97.66 µA/bit
 *   Cal        = 4194
 */
#define INA219_R_SHUNT_OHM      0.1f
#define INA219_MAX_CURRENT_A    3.2f
#define INA219_CURRENT_LSB      (INA219_MAX_CURRENT_A / 32768.0f)
#define INA219_CALIB_VAL        ((uint16_t)(0.04096f / (INA219_CURRENT_LSB * INA219_R_SHUNT_OHM)))

typedef struct {
    float voltage_V;
    float current_A;
    float power_W;
} INA219_Data_t;

/* Konfiguruje rejestr Config i Calibration.
 * Wywolac raz przed pierwszym odczytem.
 * Zwraca false przy bledzie I2C. */
bool INA219_Init(I2C_HandleTypeDef *hi2c);

/* Odczytuje napiecie szyny, prad i moc.
 * Zwraca false przy bledzie I2C (data pozostaje bez zmian). */
bool INA219_Read(I2C_HandleTypeDef *hi2c, INA219_Data_t *data);

#endif /* INA219_DRIVER_H_ */
