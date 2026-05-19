/* sonar_task.h
 *
 * Zadanie FreeRTOS zarządzające czterema sensorami HC-SR04.
 * Odczytuje wyniki z sonar_driver i wkłada je do kolejki,
 * skąd app_freertos.c publikuje na topiki ROS /STM_sonar_N.
 *
 * Pinout zgodny z PCB i konfiguracją CubeMX (tim.c):
 *   TRIG0: PB6    ECHO0: PA0  (TIM2_CH1, AF1)
 *   TRIG1: PB7    ECHO1: PA1  (TIM2_CH2, AF1)
 *   TRIG2: PB8    ECHO2: PA9  (TIM2_CH3, AF10)
 *   TRIG3: PC14   ECHO3: PA10 (TIM2_CH4, AF10)
 *
 * Piny TRIG zdefiniowane makrami poniżej.
 * Piny ECHO są konfigurowane przez CubeMX w MX_TIM2_Init() (tim.c).
 */

#ifndef SONAR_TASK_H_
#define SONAR_TASK_H_

#include "cmsis_os.h"
#include "sonar_driver.h"
#include "main.h"

/* ---- Przypisanie pinów TRIG (GPIO Output) -------------------------------- */
#define SONAR_TRIG0_PORT   HC_SR_01_GPIO_Port
#define SONAR_TRIG0_PIN    HC_SR_01_Pin
#define SONAR_TRIG1_PORT   HC_SR_02_GPIO_Port
#define SONAR_TRIG1_PIN    HC_SR_02_Pin
#define SONAR_TRIG2_PORT   HC_SR_03_GPIO_Port
#define SONAR_TRIG2_PIN    HC_SR_03_Pin
#define SONAR_TRIG3_PORT   HC_SR_04_GPIO_Port
#define SONAR_TRIG3_PIN    HC_SR_04_Pin

/* ---- Przypisanie kanałów ECHO (TIM2 Input Capture) ----------------------- */
#define SONAR_ECHO0_CHAN   TIM_CHANNEL_1   /* PA0  = TIM2_CH1 (AF1)  */
#define SONAR_ECHO1_CHAN   TIM_CHANNEL_2   /* PA1  = TIM2_CH2 (AF1)  */
#define SONAR_ECHO2_CHAN   TIM_CHANNEL_3   /* PA9  = TIM2_CH3 (AF10) */
#define SONAR_ECHO3_CHAN   TIM_CHANNEL_4   /* PA10 = TIM2_CH4 (AF10) */

/* ---- Dane w kolejce ------------------------------------------------------- */
typedef struct {
    float    distance_m[SONAR_COUNT];  /**< Odległość [m]; NaN = brak echa    */
    uint32_t timestamp_ms[SONAR_COUNT];/**< Czas każdego pomiaru (ms)         */
} Sonar_QueueData_t;

/* ---- Diagnostyka ---------------------------------------------------------- */
/**
 * Obserwuj w debuggerze (Live Expressions):
 *   cycle_count        — liczba pełnych cykli (4 sensorów) od startu
 *   no_echo_count[i]   — liczba timeoutów sensora i
 *   last_raw_mm[i]     — ostatnia surowa wartość w mm (0xFFFF = brak echa)
 */
typedef struct {
    uint32_t cycle_count;
    uint32_t no_echo_count[SONAR_COUNT];
    uint16_t last_raw_mm[SONAR_COUNT];
} Sonar_Debug_t;

extern volatile Sonar_Debug_t sonar_dbg;
extern osMessageQueueId_t     sonar_data_queue;
extern const osThreadAttr_t   sonar_task_attr;

void Sonar_Manager_Init(void);
void Sonar_Manager_Task(void *argument);

#endif /* SONAR_TASK_H_ */
