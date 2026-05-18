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
    .priority   = (osPriority_t)osPriorityAboveNormal, /* wyzszy niz micro-ROS, by HAL_UART_Receive nie byl przerywany */
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
        /* Status Return Level = 1: silnik odpowiada TYLKO na READ, nie na WRITE.
         * Bez tego bajty statusu po kazdym WRITE zalegaja w buforze UART i
         * korumpuja kolejne odczyty (OVR error -> DXL_Read zwraca 0xFFFF). */
        DXL_WriteByte(&bus, wheels[i], DXL_REG_STATUS_RETURN_LEVEL, 1);
        HAL_Delay(55); /* zapis EEPROM wymaga min ~55 ms */

        /* Return Delay Time = 1 unit = 2 µs (domyslnie 250 = 500 µs).
         * Skraca czas oczekiwania na odpowiedz, zmniejsza latencje odczytu. */
        DXL_WriteByte(&bus, wheels[i], DXL_REG_RETURN_DELAY_TIME, 1);
        HAL_Delay(55);

        DXL_SetWheelMode(&bus, wheels[i]);
        HAL_Delay(55); /* CW/CCW_ANGLE_LIMIT rowniez w EEPROM */
        DXL_SetTorque(&bus, wheels[i], true);
        HAL_Delay(10);
        DXL_SetGoalSpeedRaw(&bus, wheels[i], 0);
        HAL_Delay(5);
    }
}

// ---------------------------------------------------------------------------
// DXL_Manager_Task — główna pętla sterowania
// Częstotliwość pętli: ~50Hz (osDelay(20))
// Feedback odczytywany i wysyłany co iterację (~50Hz)
// ---------------------------------------------------------------------------
void DXL_Manager_Task(void *argument)
{
    DXL_RosCommand_t  cmd;
    DXL_RosFeedback_t feedback = {0};

    for (;;)
    {
        /* A. Wykonaj wszystkie oczekujące komendy prędkości */
        while (osMessageQueueGet(dxl_cmd_queue, &cmd, NULL, 0) == osOK)
        {
            /* speed_pct [-100, 100] → raw Dynamixel [0, 1023] + bit kierunku */
            float abs_pct = (cmd.speed_pct < 0.0f) ? -cmd.speed_pct : cmd.speed_pct;
            uint16_t raw_val = (uint16_t)(abs_pct * 10.23f);
            if (raw_val > 1023) raw_val = 1023;

            /*
             * Kierunek obrotu (RX-64 w trybie koła):
             *   Bit 10 = 0 → CCW
             *   Bit 10 = 1 → CW
             *
             * Prawe koło:  positive speed_pct → CCW → do przodu
             * Lewe koło:   positive speed_pct → CW  → do przodu (montaż lustrzany)
             */
            if (cmd.id == ID_RIGHT_WHEEL)
            {
                if (cmd.speed_pct < 0.0f) raw_val |= 0x400;
            }
            else if (cmd.id == ID_LEFT_WHEEL)
            {
                if (cmd.speed_pct >= 0.0f) raw_val |= 0x400;
            }

            DXL_SetGoalSpeedRaw(&bus, cmd.id, raw_val);
        }

        /* B. Odczyt feedbacku — jeden blokowy pakiet na silnik (6 bajtow danych).
         *    Zastepuje 3 osobne DXL_Read16, skracajac czas RS-485 z ~11ms do ~4ms.
         *    Przy bledzie odczytu zachowujemy ostatnie poprawne wartosci. */
        {
            static const uint8_t ids[]     = {ID_RIGHT_WHEEL, ID_LEFT_WHEEL};
            static const int     offsets[] = {0, 3};

            for (int i = 0; i < 2; i++)
            {
                uint16_t p, v, l;
                if (!DXL_ReadPresentState(&bus, ids[i], &p, &v, &l))
                    continue; /* blad: nie nadpisuj starych wartosci */

                int off = offsets[i];

                /* Pozycja: 0-1023 jednostek → 0-300° (RX-64: 1 unit = 0.293°) */
                feedback.data[off] = (float)p * 0.293f;

                /* Predkosc: bit 10 = kierunek (0=CCW +, 1=CW -) */
                feedback.data[off + 1] = (v > 1023) ? -(float)(v - 1024)
                                                     :  (float)v;

                /* Obciazenie: bit 10 = kierunek, wartosc 0-1023 -> % */
                feedback.data[off + 2] = (l > 1023) ? -(float)(l - 1024) / 10.23f
                                                     :  (float)l           / 10.23f;
            }

            /* Nadpisz stare dane i wstaw nowe */
            if (osMessageQueueGetCount(dxl_feedback_queue) > 0)
                osMessageQueueReset(dxl_feedback_queue);

            osMessageQueuePut(dxl_feedback_queue, &feedback, 0, 0);
        }

        osDelay(20);  /* ~50 Hz */
    }
}
