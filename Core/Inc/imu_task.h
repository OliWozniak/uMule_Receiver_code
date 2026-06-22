/* imu_task.h */

#ifndef IMU_TASK_H_
#define IMU_TASK_H_

#include "cmsis_os.h"
#include "imu_driver.h"

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float mag_x,   mag_y,   mag_z;
    /* Wariancja (diagonala macierzy kowariancji) — liczy Welford online */
    float var_gx, var_gy, var_gz;
    float var_ax, var_ay, var_az;
    float var_mx, var_my, var_mz;
    bool  cov_valid; /* true po zebraniu IMU_COV_MIN_SAMPLES prob */
} IMU_QueueData_t;

#define IMU_COV_MIN_SAMPLES 200u  /* ~4 s przy 50 Hz — wymagane przed uznaniem kowariancji */

/* Struktura diagnostyczna — obserwuj w debuggerze (Live Expressions).
 *
 * Interpretacja:
 *   lsm6_ready  = HAL_OK (0) → LSM6 odpowiada na 0x6A
 *   lis3_ready  = HAL_OK (0) → LIS3 odpowiada na 0x1C
 *   Jesli ready = HAL_ERROR (1): NACK → sensor nie jest pod tym adresem lub nie jest zasilony
 *   Jesli ready = HAL_TIMEOUT (3): bus stuck / brak pull-upow / konflikT pinow
 *
 *   lsm6_who_am_i powinno byc 0x6A → jezeli 0x00 przy ready=HAL_OK: odczyt WHO_AM_I zly
 *   lis3_who_am_i powinno byc 0x3D
 *
 *   Alternatywne adresy:
 *     LSM6DS3TR-C: 0x6B jesli pin SDO/SA0 jest podciagniety do VCC
 *     LIS3MDL:     0x1E jesli pin SA1 jest podciagniety do VCC
 */
typedef struct {
    HAL_StatusTypeDef lsm6_ready;       /* HAL_OK = urządzenie odpowiada */
    HAL_StatusTypeDef lis3_ready;
    uint8_t           lsm6_who_am_i;   /* oczekiwane: 0x6A */
    uint8_t           lis3_who_am_i;   /* oczekiwane: 0x3D */
    bool              init_ok;
    uint32_t          read_ok_count;
    uint32_t          read_fail_count;
} IMU_Debug_t;

extern volatile IMU_Debug_t   imu_dbg;
extern osMessageQueueId_t     imu_data_queue;
extern const osThreadAttr_t   imu_task_attr;

void IMU_Manager_Init(void);
void IMU_Manager_Task(void *argument);

#endif /* IMU_TASK_H_ */
