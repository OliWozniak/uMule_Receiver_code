#include "sonar_task.h"
#include "tim.h"     /* htim2, htim6 */
#include <math.h>    /* NAN */

/* -------------------------------------------------------------------------- */
/* Zmienne globalne                                                            */
/* -------------------------------------------------------------------------- */
volatile Sonar_Debug_t sonar_dbg      = {0};
osMessageQueueId_t     sonar_data_queue = NULL;

const osThreadAttr_t sonar_task_attr = {
    .name       = "Sonar_Manager",
    .priority   = (osPriority_t)osPriorityBelowNormal, /* niżej niż micro-ROS */
    .stack_size = 1024, /* Cortex-M4 + FPU: sam kontekst = 196 B, poprzednie 384 B było za małe */
};

/* Deskryptory sprzętu — muszą być spójne z sonar_task.h */
static const Sonar_HW_t sonar_hw[SONAR_COUNT] = {
    { SONAR_TRIG0_PORT, SONAR_TRIG0_PIN, SONAR_ECHO0_CHAN },
    { SONAR_TRIG1_PORT, SONAR_TRIG1_PIN, SONAR_ECHO1_CHAN },
    { SONAR_TRIG2_PORT, SONAR_TRIG2_PIN, SONAR_ECHO2_CHAN },
    { SONAR_TRIG3_PORT, SONAR_TRIG3_PIN, SONAR_ECHO3_CHAN },
};

/* -------------------------------------------------------------------------- */
/* Init                                                                        */
/* -------------------------------------------------------------------------- */
void Sonar_Manager_Init(void)
{
    /* htim2 i htim6 są generowane przez CubeMX w tim.c */
    Sonar_Init(sonar_hw, &htim2, &htim6);
}

/* -------------------------------------------------------------------------- */
/* Task                                                                        */
/* -------------------------------------------------------------------------- */
void Sonar_Manager_Task(void *argument)
{
    (void)argument;
    Sonar_QueueData_t q;

    Sonar_Start();

    for (;;)
    {
        /* Czekaj na kompletny cykl (4 sensorów) — odpytywanie co 10 ms.
         * Cykl trwa ~140 ms, więc opóźnienie 10 ms daje mały narzut. */
        if (Sonar_CycleComplete())
        {
            bool any_valid = false;

            for (uint8_t i = 0; i < SONAR_COUNT; i++)
            {
                Sonar_Result_t r = Sonar_GetResult(i);

                sonar_dbg.last_raw_mm[i] = r.distance_mm;

                if (r.distance_mm == SONAR_NO_ECHO) {
                    sonar_dbg.no_echo_count[i]++;
                    q.distance_m[i]   = NAN;           /* brak echa = NaN w ROS */
                } else {
                    q.distance_m[i]   = r.distance_mm * 0.001f;
                    any_valid = true;
                }
                q.timestamp_ms[i] = r.timestamp_ms;
            }

            sonar_dbg.cycle_count++;

            /* Wstaw do kolejki (nadpisuje starą wartość jeśli niepobrana) */
            if (osMessageQueueGetCount(sonar_data_queue) > 0)
                osMessageQueueReset(sonar_data_queue);
            osMessageQueuePut(sonar_data_queue, &q, 0, 0);

            (void)any_valid;
        }

        osDelay(10);
    }
}
