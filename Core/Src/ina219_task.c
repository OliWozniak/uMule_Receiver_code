#include "ina219_task.h"

volatile INA219_Debug_t ina219_dbg = {0};
osMessageQueueId_t      ina219_data_queue = NULL;

const osThreadAttr_t ina219_task_attr = {
    .name       = "INA219_Manager",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 256
};

// ---------------------------------------------------------------------------
void INA219_Manager_Init(void)
{
    ina219_dbg.ready = HAL_I2C_IsDeviceReady(&hi2c3, INA219_ADDR << 1, 3, 100);

    if (ina219_dbg.ready == HAL_OK)
    {
        ina219_dbg.init_ok = INA219_Init(&hi2c3);

        /* Odczyt rejestru konfiguracji po inicjalizacji.
         * Oczekiwane: 0x3FFF (po naszym write). Domyslne (reset): 0x399F. */
        if (ina219_dbg.init_ok)
        {
            uint8_t buf[2] = {0};
            uint8_t reg = INA219_REG_CONFIG;
            if (HAL_I2C_Master_Transmit(&hi2c3, INA219_ADDR << 1, &reg, 1, 100) == HAL_OK)
                HAL_I2C_Master_Receive(&hi2c3, INA219_ADDR << 1, buf, 2, 100);
            ina219_dbg.config_reg = (uint16_t)((buf[0] << 8) | buf[1]);
        }
    }
}

// ---------------------------------------------------------------------------
void INA219_Manager_Task(void *argument)
{
    INA219_Data_t      hw;
    INA219_QueueData_t q = {0};

    for (;;)
    {
        if (INA219_Read(&hi2c3, &hw))
        {
            q.voltage_V = hw.voltage_V;
            q.current_A = hw.current_A;
            q.power_W   = hw.power_W;

            if (osMessageQueueGetCount(ina219_data_queue) > 0)
                osMessageQueueReset(ina219_data_queue);
            osMessageQueuePut(ina219_data_queue, &q, 0, 0);

            ina219_dbg.read_ok_count++;
        }
        else
        {
            ina219_dbg.read_fail_count++;
        }

        osDelay(200);
    }
}
