/* imu_driver.h
 *
 * Abstrakcja sprzetu IMU: LSM6DS3TR-C (accel + gyro) + LIS3MDL (mag)
 * Interfejs: I2C4, oba sensory na tym samym busie.
 *
 * Adresy I2C (7-bit):
 *   LSM6DS3TR-C  0x6A  (SDO/SA0 = GND)
 *   LIS3MDL      0x1C  (SA1 = GND)
 */

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* --- Adresy I2C (7-bit) --- */
#define LSM6DS3_ADDR            0x6A
#define LIS3MDL_ADDR            0x1C

/* --- Rejestry LSM6DS3TR-C --- */
#define LSM6_WHO_AM_I           0x0F
#define LSM6_CTRL1_XL           0x10  /* Accelerometer control */
#define LSM6_CTRL2_G            0x11  /* Gyroscope control */
#define LSM6_CTRL3_C            0x12  /* BDU + IF_INC */
#define LSM6_STATUS_REG         0x1E
#define LSM6_OUTX_L_G           0x22  /* Gyro X/Y/Z: 6 bytes */
#define LSM6_OUTX_L_XL          0x28  /* Accel X/Y/Z: 6 bytes */

/* --- Rejestry LIS3MDL --- */
#define LIS3_WHO_AM_I           0x0F
#define LIS3_CTRL_REG1          0x20
#define LIS3_CTRL_REG2          0x21
#define LIS3_CTRL_REG3          0x22
#define LIS3_CTRL_REG4          0x23
#define LIS3_CTRL_REG5          0x24
#define LIS3_OUT_X_L            0x28  /* Mag X/Y/Z: 6 bytes (I2C: bit7=1 dla auto-increment) */

/* --- Przeliczniki ---
 * LSM6 gyro  ±2000 dps:  70 mdps/LSB = 0.070 × π/180  = 0.001221730 rad/s/LSB
 * LSM6 accel ±8g:       0.244 mg/LSB = 0.000244 × g    = 0.002393 m/s²/LSB
 * LIS3MDL    ±4 gauss:  1/6842 gauss/LSB               = 1.461×10⁻⁸ T/LSB
 */
#define LSM6_GYRO_SENS_RAD_S    0.001221730f
#define LSM6_ACCEL_SENS_MS2     0.002393219f
#define LIS3_MAG_SENS_T         1.46131e-8f

typedef struct {
    float accel_x, accel_y, accel_z;  /* m/s²  */
    float gyro_x,  gyro_y,  gyro_z;   /* rad/s */
    float mag_x,   mag_y,   mag_z;    /* Tesla */
} IMU_Data_t;

/* Inicjalizuje oba sensory (ustawia tryb pracy, ODR, FS, BDU).
 * Wywolac raz przed pierwszym odczytem.
 * Zwraca false jesli komunikacja z ktorymkolwiek sensorem zawiedzie. */
bool IMU_Init(I2C_HandleTypeDef *hi2c);

/* Odczytuje accel, gyro i mag w jednym wywolaniu.
 * Bledny odczyt jednego sensora nie blokuje pozostalych — dane z blednego
 * sensora pozostaja bez zmian (nalezy je ignorowac po stronie callera). */
bool IMU_Read(I2C_HandleTypeDef *hi2c, IMU_Data_t *data);

#endif /* IMU_DRIVER_H_ */
