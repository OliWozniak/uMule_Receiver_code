#include "dxl_control.h"

static DXL_Port_t bus;
osMessageQueueId_t dxl_cmd_queue = NULL;
osMessageQueueId_t dxl_feedback_queue = NULL;

const osThreadAttr_t dxl_task_attr = {
    .name = "DXL_Manager",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 2048
};

void DXL_Manager_Init(void) {
    // 1. Inicjalizacja niskopoziomowa sterownika
    DXL_Init(&bus, &huart3, GPIOB, GPIO_PIN_2);

    // 2. Konfiguracja serwomechanizmów (Tryb koła, Torque ON)
    uint8_t wheels[] = {ID_RIGHT_WHEEL, ID_LEFT_WHEEL};
    for(int i=0; i<2; i++) {
        DXL_SetWheelMode(&bus, wheels[i]);
        HAL_Delay(10);
        DXL_SetTorque(&bus, wheels[i], true);
    }
}

void DXL_Manager_Task(void *argument) {
    DXL_RosCommand_t cmd;
    DXL_RosFeedback_t feedback;
    int tick = 0;

    for(;;) {
        // A. Wykonaj komendy z ROS
        if (osMessageQueueGet(dxl_cmd_queue, &cmd, NULL, 0) == osOK) {
            uint16_t raw_val = (uint16_t)((cmd.speed_pct < 0 ? -cmd.speed_pct : cmd.speed_pct) * 10.23f);
            if(raw_val > 1023) raw_val = 1023;

            // Logika kierunku z uwzględnieniem montażu silników
            if (cmd.id == ID_RIGHT_WHEEL) {
                if (cmd.speed_pct < 0) raw_val |= 1024; // CW
            } else if (cmd.id == ID_LEFT_WHEEL) {
                if (cmd.speed_pct >= 0) raw_val |= 1024; // CCW (Inwersja)
            }
            DXL_SetGoalSpeedRaw(&bus, cmd.id, raw_val);
        }

        // B. Pobierz Feedback (co ok. 100ms)
        if (++tick >= 5) {
            tick = 0;
            uint8_t ids[] = {ID_RIGHT_WHEEL, ID_LEFT_WHEEL};

            for(int i=0; i<2; i++) {
                uint16_t p = DXL_Read16(&bus, ids[i], DXL_REG_PRESENT_POSITION);
                uint16_t v = DXL_Read16(&bus, ids[i], DXL_REG_PRESENT_SPEED);
                uint16_t l = DXL_Read16(&bus, ids[i], DXL_REG_PRESENT_LOAD);

                int offset = (i == 0) ? 0 : 3;
                feedback.data[offset]     = (float)p * 0.29f; // Position
                feedback.data[offset + 1] = (v > 1023) ? -(float)(v-1024) : (float)v; // Speed
                feedback.data[offset + 2] = (l > 1023) ? (float)(l-1024)/10.23f : (float)l/10.23f; // Load
            }
            osMessageQueuePut(dxl_feedback_queue, &feedback, 0, 0);
        }
        osDelay(20);
    }
}
