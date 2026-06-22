#include "imu_task.h"

volatile IMU_Debug_t imu_dbg = {0};
osMessageQueueId_t   imu_data_queue = NULL;

const osThreadAttr_t imu_task_attr = {
    .name       = "IMU_Manager",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 768  /* Cortex-M4 + FPU: kontekst 196 B + 9 floatow lokalne + HAL I2C; poprzednie 512 B bylo za male */
};

// ---------------------------------------------------------------------------
void IMU_Manager_Init(void)
{
    /* HAL_I2C_IsDeviceReady: wyslij START + adres + odbierz ACK/NACK.
     * Trzy probki, timeout 100ms. Wynik w imu_dbg.lsm6_ready / lis3_ready:
     *   HAL_OK      (0) = sensor odpowiada pod tym adresem
     *   HAL_ERROR   (1) = NACK (zly adres, brak zasilania, konflikt pinow)
     *   HAL_TIMEOUT (3) = brak odpowiedzi (bus stuck, brak pull-upow)
     *
     * Uzywamy adresow z naglowkow ST:
     *   LSM6DS3TR_C_I2C_ADD_L = 0xD5  (SDO=GND, 7-bit 0x6A)
     *   LIS3MDL_I2C_ADD_L     = 0x39  (SA1=GND, 7-bit 0x1C)
     * HAL ignoruje bit LSB adresu (R/W), wiec mozna uzyc wprost.
     */
    imu_dbg.lsm6_ready = HAL_I2C_IsDeviceReady(&hi2c3, LSM6DS3TR_C_I2C_ADD_L, 3, 100);
    imu_dbg.lis3_ready  = HAL_I2C_IsDeviceReady(&hi2c3, LIS3MDL_I2C_ADD_L,     3, 100);

    /* WHO_AM_I — tylko jesli sensor odpowiada.
     * LSM6DS3TR_C_ID = 0x6A, LIS3MDL_ID = 0x3D */
    if (imu_dbg.lsm6_ready == HAL_OK)
        HAL_I2C_Mem_Read(&hi2c3, LSM6DS3TR_C_I2C_ADD_L, LSM6DS3TR_C_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)&imu_dbg.lsm6_who_am_i, 1, 100);

    if (imu_dbg.lis3_ready == HAL_OK)
        HAL_I2C_Mem_Read(&hi2c3, LIS3MDL_I2C_ADD_L, LIS3MDL_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)&imu_dbg.lis3_who_am_i, 1, 100);

    imu_dbg.init_ok = IMU_Init(&hi2c3);
}

/* Welford online variance — aktualizuje mean i M2 dla jednej osi */
static inline void welford_update(float x, uint32_t n, float *mean, float *M2)
{
    float delta  = x - *mean;
    *mean += delta / (float)n;
    float delta2 = x - *mean;
    *M2  += delta * delta2;
}

// ---------------------------------------------------------------------------
void IMU_Manager_Task(void *argument)
{
    IMU_Data_t      hw;
    IMU_QueueData_t q = {0};

    /* Stan Welford — 9 osi × (mean + M2) */
    float mean_gx = 0, mean_gy = 0, mean_gz = 0;
    float mean_ax = 0, mean_ay = 0, mean_az = 0;
    float mean_mx = 0, mean_my = 0, mean_mz = 0;
    float M2_gx   = 0, M2_gy  = 0, M2_gz  = 0;
    float M2_ax   = 0, M2_ay  = 0, M2_az  = 0;
    float M2_mx   = 0, M2_my  = 0, M2_mz  = 0;
    uint32_t n = 0;

    for (;;)
    {
        if (IMU_Read(&hi2c3, &hw))
        {
            q.accel_x = hw.accel_x;  q.accel_y = hw.accel_y;  q.accel_z = hw.accel_z;
            q.gyro_x  = hw.gyro_x;   q.gyro_y  = hw.gyro_y;   q.gyro_z  = hw.gyro_z;
            q.mag_x   = hw.mag_x;    q.mag_y   = hw.mag_y;    q.mag_z   = hw.mag_z;

            /* Welford update — n > 0 wymagane przed dzieleniem */
            n++;
            welford_update(hw.gyro_x,  n, &mean_gx, &M2_gx);
            welford_update(hw.gyro_y,  n, &mean_gy, &M2_gy);
            welford_update(hw.gyro_z,  n, &mean_gz, &M2_gz);
            welford_update(hw.accel_x, n, &mean_ax, &M2_ax);
            welford_update(hw.accel_y, n, &mean_ay, &M2_ay);
            welford_update(hw.accel_z, n, &mean_az, &M2_az);
            welford_update(hw.mag_x,   n, &mean_mx, &M2_mx);
            welford_update(hw.mag_y,   n, &mean_my, &M2_my);
            welford_update(hw.mag_z,   n, &mean_mz, &M2_mz);

            if (n >= IMU_COV_MIN_SAMPLES)
            {
                float inv = 1.0f / (float)(n - 1);
                q.var_gx = M2_gx * inv;
                q.var_gy = M2_gy * inv;
                q.var_gz = M2_gz * inv;
                q.var_ax = M2_ax * inv;
                q.var_ay = M2_ay * inv;
                q.var_az = M2_az * inv;
                q.var_mx = M2_mx * inv;
                q.var_my = M2_my * inv;
                q.var_mz = M2_mz * inv;
                q.cov_valid = true;

                /* Zapobiegaj przepełnieniu akumulatora przy bardzo długim biegu.
                 * Po 10× MIN_SAMPLES resetujemy do aktualnych wartości (soft reset). */
                if (n >= IMU_COV_MIN_SAMPLES * 10u)
                {
                    mean_gx = q.gyro_x;  mean_gy = q.gyro_y;  mean_gz = q.gyro_z;
                    mean_ax = q.accel_x; mean_ay = q.accel_y; mean_az = q.accel_z;
                    mean_mx = q.mag_x;   mean_my = q.mag_y;   mean_mz = q.mag_z;
                    M2_gx = M2_gy = M2_gz = M2_ax = M2_ay = M2_az = 0;
                    M2_mx = M2_my = M2_mz = 0;
                    n = 1;
                }
            }

            if (osMessageQueueGetCount(imu_data_queue) > 0)
                osMessageQueueReset(imu_data_queue);
            osMessageQueuePut(imu_data_queue, &q, 0, 0);

            imu_dbg.read_ok_count++;
        }
        else
        {
            imu_dbg.read_fail_count++;
        }

        osDelay(20);
    }
}
