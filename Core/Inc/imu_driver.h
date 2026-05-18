/* imu_driver.h
 *
 * Abstrakcja sprzetu IMU: LSM6DS3TR-C (accel + gyro) + LIS3MDL (mag)
 * Implementacja oparta na oficjalnych sterownikach ST MEMS (stmdev_ctx_t).
 * Interfejs: I2C4, oba sensory na tym samym busie.
 *
 * Adresy I2C (8-bit, HAL format):
 *   LSM6DS3TR-C  LSM6DS3TR_C_I2C_ADD_L = 0xD5  (SDO/SA0 = GND, 7-bit 0x6A)
 *   LIS3MDL      LIS3MDL_I2C_ADD_L     = 0x39  (SA1 = GND, 7-bit 0x1C)
 */

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

#include "lsm6ds3tr_c_reg.h"
#include "lis3mdl_reg.h"

/* --------------------------------------------------------------------------
 * Convenience aliases — backward-compatible with imu_task.c probes.
 * These are the 7-bit addresses; use << 1 for HAL_I2C_IsDeviceReady.
 * -------------------------------------------------------------------------- */
#define LSM6DS3_ADDR   (LSM6DS3TR_C_I2C_ADD_L >> 1)   /* 0x6A */
#define LIS3MDL_ADDR   (LIS3MDL_I2C_ADD_L     >> 1)   /* 0x1C */

/* --------------------------------------------------------------------------
 * Physical unit conversion factors
 *   LSM6 gyro  ±2000 dps : 70 mdps/LSB  = 70e-3 × π/180 = 1.22173e-3 rad/s
 *   LSM6 accel ±8g        : 0.244 mg/LSB = 0.244e-3 × 9.80665 = 2.39281e-3 m/s²
 *   LIS3MDL    ±4 gauss   : 1/6842 gauss/LSB = 1.46131e-8 T/LSB
 * -------------------------------------------------------------------------- */
#define LSM6_GYRO_SENS_RAD_S    1.22173e-3f
#define LSM6_ACCEL_SENS_MS2     2.39281e-3f
#define LIS3_MAG_SENS_T         1.46131e-8f

typedef struct {
    float accel_x, accel_y, accel_z;  /* m/s²  */
    float gyro_x,  gyro_y,  gyro_z;   /* rad/s */
    float mag_x,   mag_y,   mag_z;    /* Tesla */
} IMU_Data_t;

/* Inicjalizuje oba sensory przez ST driver API (stmdev_ctx_t).
 * Ustawia ODR=104 Hz, FS=±8g / ±2000 dps (LSM6), FS=±4G / ODR=10 Hz (LIS3MDL).
 * Zwraca false jesli komunikacja z ktorymkolwiek sensorem zawiedzie. */
bool IMU_Init(I2C_HandleTypeDef *hi2c);

/* Odczytuje accel, gyro i mag w jednym wywolaniu przez ST driver API.
 * Blad jednego sensora nie blokuje pozostalych — dane blednego sensora
 * pozostaja bez zmian; zwraca false jesli oba sensory zawodza. */
bool IMU_Read(I2C_HandleTypeDef *hi2c, IMU_Data_t *data);

#endif /* IMU_DRIVER_H_ */
