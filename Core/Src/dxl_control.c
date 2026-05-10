/* dxl_control.c
 *
 * Task FreeRTOS sterujący serwomechanizmami Dynamixel AX przez RS-485
 *
 * Kolejki:
 *   dxl_cmd_queue      ← komendy od micro-ROS (speed_pct)
 *   dxl_feedback_queue → dane odczytu dla micro-ROS (pozycja/prędkość/load)
 *
 * Layout DXL_RosFeedback_t.data[6]:
 *   [0] = RIGHT position (stopnie, 0-300)
 *   [1] = RIGHT velocity (DXL raw signed, + = CCW, - = CW)
 *   [2] = RIGHT load     (%, + = CCW, - = CW)
 *   [3] = LEFT  position (stopnie)
 *   [4] = LEFT  velocity (DXL raw signed)
 *   [5] = LEFT  load     (%)
 */

#include "dxl_control.h"

static DXL_Port_t bus;

osMessageQueueId_t dxl_cmd_queue      = NULL;
osMessageQueueId_t dxl_feedback_queue = NULL;

const osThreadAttr_t dxl_task_attr = {
    .name       = "DXL_Manager",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 2048
};

// ---------------------------------------------------------------------------
// DXL_Manager_Init — konfiguracja serwomechanizmów
// Wywoływane PRZED startem tasków, z MX_FREERTOS_Init()
// ---------------------------------------------------------------------------
void DXL_Manager_Init(void)
{
    /* Inicjalizacja niskopoziomowa magistrali RS-485 */
    DXL_Init(&bus, &huart3, GPIOB, GPIO_PIN_2);

    /* Konfiguracja obu kół:
     * - Tryb koła (continuous rotation, bez limitu pozycji)
     * - Torque ON
     * - Chwilowe zatrzymanie (prędkość = 0)
     */
    uint8_t wheels[] = {ID_RIGHT_WHEEL, ID_LEFT_WHEEL};
    for (int i = 0; i < 2; i++)
    {
        DXL_SetWheelMode(&bus, wheels[i]);
        HAL_Delay(10);
        DXL_SetTorque(&bus, wheels[i], true);
        HAL_Delay(10);
        DXL_SetGoalSpeedRaw(&bus, wheels[i], 0);  // start z prędkością 0
        HAL_Delay(5);
    }
}

// ---------------------------------------------------------------------------
// DXL_Manager_Task — główna pętla sterowania
// Częstotliwość pętli: ~50Hz (osDelay(20))
// Feedback wysyłany co ~100ms (co 5 iteracji)
// ---------------------------------------------------------------------------
void DXL_Manager_Task(void *argument)
{
    DXL_RosCommand_t  cmd;
    DXL_RosFeedback_t feedback;
    int tick = 0;

    for (;;)
    {
        /* A. Wykonaj wszystkie oczekujące komendy prędkości */
        while (osMessageQueueGet(dxl_cmd_queue, &cmd, NULL, 0) == osOK)
        {
            /* speed_pct [-100, 100] → raw Dynamixel [0, 1023] + bit kierunku */
            float abs_pct = (cmd.speed_pct < 0) ? -cmd.speed_pct : cmd.speed_pct;
            uint16_t raw_val = (uint16_t)(abs_pct * 10.23f);
            if (raw_val > 1023) raw_val = 1023;

            /*
             * Kierunek obrotu (Dynamixel AX w trybie koła):
             *   Bit 10 = 0 → CCW (wg zegara od strony osi)
             *   Bit 10 = 1 → CW
             *
             * Prawe koło:  positive speed_pct → CCW → do przodu
             * Lewe koło:   positive speed_pct → CW  → do przodu (montaż lustrzany!)
             *              dlatego dla lewego inwertujemy kierunek
             */
            if (cmd.id == ID_RIGHT_WHEEL)
            {
                if (cmd.speed_pct < 0) raw_val |= 0x400;  // CW
                /* speed_pct >= 0 → CCW (bit 10 = 0, już OK) */
            }
            else if (cmd.id == ID_LEFT_WHEEL)
            {
                if (cmd.speed_pct >= 0) raw_val |= 0x400; // CCW → CW (inwersja)
                /* speed_pct < 0 → CCW (bit 10 = 0) */
            }

            DXL_SetGoalSpeedRaw(&bus, cmd.id, raw_val);
        }

        /* B. Odczyt feedbacku co ~100ms (5 * 20ms) */
        if (++tick >= 5)
        {
            tick = 0;

            /*
             * Kolejność odczytu: RIGHT najpierw (indeks 0), LEFT drugi (indeks 3)
             * Pasuje do DXL_RosFeedback_t.data[] layout opisanego w nagłówku
             */
            uint8_t ids[]    = {ID_RIGHT_WHEEL, ID_LEFT_WHEEL};
            int     offsets[] = {0, 3};

            for (int i = 0; i < 2; i++)
            {
                uint8_t  id     = ids[i];
                int      offset = offsets[i];

                uint16_t p = DXL_Read16(&bus, id, DXL_REG_PRESENT_POSITION);
                uint16_t v = DXL_Read16(&bus, id, DXL_REG_PRESENT_SPEED);
                uint16_t l = DXL_Read16(&bus, id, DXL_REG_PRESENT_LOAD);

                /* Pozycja: 0-1023 jednostki → 0-300 stopni (AX: 1 unit = 0.293°) */
                feedback.data[offset] = (float)p * 0.293f;

                /* Prędkość: bit 10 = kierunek (0=CCW, 1=CW)
                 * Zwracamy signed: + = CCW (do przodu dla prawego koła)
                 *                  - = CW
                 * app_freertos.c przelicza na rad/s przez DXL_UNIT_TO_RAD_S
                 */
                if (v > 1023)
                    feedback.data[offset + 1] = -(float)(v - 1024); // CW = ujemna
                else
                    feedback.data[offset + 1] =  (float)v;          // CCW = dodatnia

                /* Load: bit 10 = kierunek, 0-1023 = wartość
                 * Zwracamy signed % */
                if (l > 1023)
                    feedback.data[offset + 2] = -(float)(l - 1024) / 10.23f;
                else
                    feedback.data[offset + 2] =  (float)l / 10.23f;
            }

            /* Wstaw do kolejki (nadpisz stare dane jeśli pełna) */
            if (osMessageQueueGetCount(dxl_feedback_queue) > 0)
                osMessageQueueReset(dxl_feedback_queue);

            osMessageQueuePut(dxl_feedback_queue, &feedback, 0, 0);
        }

        osDelay(20);  // ~50 Hz pętla sterowania
    }
}
