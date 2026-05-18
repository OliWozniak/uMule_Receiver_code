/* ina219_task.h */

#ifndef INA219_TASK_H_
#define INA219_TASK_H_

#include "cmsis_os.h"
#include "ina219_driver.h"

typedef struct {
    float voltage_V;
    float current_A;
    float power_W;
} INA219_QueueData_t;

/* Struktura diagnostyczna — obserwuj w debuggerze.
 *   ready = HAL_OK (0)  → INA219 odpowiada pod adresem INA219_ADDR (0x40)
 *   ready = HAL_ERROR   → NACK (zly adres lub brak zasilania)
 *   ready = HAL_TIMEOUT → bus stuck
 *   config_reg = 0x399F → wartosc domyslna po reset (przed inicjalizacja)
 *   config_reg = 0x3FFF → wartosc po naszej inicjalizacji (OK) */
typedef struct {
    HAL_StatusTypeDef ready;
    bool              init_ok;
    uint16_t          config_reg;
    uint32_t          read_ok_count;
    uint32_t          read_fail_count;
} INA219_Debug_t;

extern volatile INA219_Debug_t  ina219_dbg;
extern osMessageQueueId_t       ina219_data_queue;
extern const osThreadAttr_t     ina219_task_attr;

void INA219_Manager_Init(void);
void INA219_Manager_Task(void *argument);

#endif /* INA219_TASK_H_ */
