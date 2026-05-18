#ifndef DXL_CONTROL_H_
#define DXL_CONTROL_H_

#include "cmsis_os.h"
#include "dxl_driver.h"
#include "usart.h"

// Konfiguracja fizyczna robota
#define ID_RIGHT_WHEEL    3
#define ID_LEFT_WHEEL     1

typedef struct {
    uint8_t id;
    float   speed_pct; // -100.0 do 100.0
} DXL_RosCommand_t;

typedef struct {
    float data[6]; // [R_pos, R_vel, R_load, L_pos, L_vel, L_load]
} DXL_RosFeedback_t;

// Uchwyty RTOS
extern osMessageQueueId_t dxl_cmd_queue;
extern osMessageQueueId_t dxl_feedback_queue;
extern const osThreadAttr_t dxl_task_attr;

// Inicjalizacja całego systemu sterowania
void DXL_Manager_Init(void);
// Task FreeRTOS
void DXL_Manager_Task(void *argument);

#endif
