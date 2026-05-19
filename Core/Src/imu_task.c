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
    imu_dbg.lsm6_ready = HAL_I2C_IsDeviceReady(&hi2c4, LSM6DS3TR_C_I2C_ADD_L, 3, 100);
    imu_dbg.lis3_ready  = HAL_I2C_IsDeviceReady(&hi2c4, LIS3MDL_I2C_ADD_L,     3, 100);

    /* WHO_AM_I — tylko jesli sensor odpowiada.
     * LSM6DS3TR_C_ID = 0x6A, LIS3MDL_ID = 0x3D */
    if (imu_dbg.lsm6_ready == HAL_OK)
        HAL_I2C_Mem_Read(&hi2c4, LSM6DS3TR_C_I2C_ADD_L, LSM6DS3TR_C_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)&imu_dbg.lsm6_who_am_i, 1, 100);

    if (imu_dbg.lis3_ready == HAL_OK)
        HAL_I2C_Mem_Read(&hi2c4, LIS3MDL_I2C_ADD_L, LIS3MDL_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)&imu_dbg.lis3_who_am_i, 1, 100);

    imu_dbg.init_ok = IMU_Init(&hi2c4);
}

// ---------------------------------------------------------------------------
void IMU_Manager_Task(void *argument)
{
    IMU_Data_t      hw;
    IMU_QueueData_t q = {0};

    for (;;)
    {
        if (IMU_Read(&hi2c4, &hw))
        {
            q.accel_x = hw.accel_x;  q.accel_y = hw.accel_y;  q.accel_z = hw.accel_z;
            q.gyro_x  = hw.gyro_x;   q.gyro_y  = hw.gyro_y;   q.gyro_z  = hw.gyro_z;
            q.mag_x   = hw.mag_x;    q.mag_y   = hw.mag_y;    q.mag_z   = hw.mag_z;

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
