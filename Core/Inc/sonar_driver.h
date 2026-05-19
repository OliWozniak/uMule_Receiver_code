/**
 * @file    sonar_driver.h
 * @brief   Nieblokujący sterownik dla 4× HC-SR04 oparty na przerwaniach.
 *
 * Wymagania sprzętowe
 * -------------------
 *  Konfiguruj w CubeMX:
 *  - TIM2 (32-bit): prescaler=169 → 1 MHz tick, ARR=0xFFFFFFFF,
 *    kanały CH1–CH4 jako Input Capture direct mode, przerwania IC włączone.
 *  - TIM6 (basic, one-pulse mode=ON): prescaler=169 → 1 MHz,
 *    przerwanie Update włączone.  ARR jest nadpisywany dynamicznie.
 *  - 4× GPIO Output PP dla pinów TRIG.
 *  - 4× GPIO AF (TIM2_CHx) dla pinów ECHO.
 *
 * Integracja w stm32g4xx_it.c lub w callbackach HAL:
 * ---------------------------------------------------
 * @code
 *   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
 *       Sonar_IC_Callback(htim, htim->Channel);
 *   }
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *       Sonar_PeriodElapsed_Callback(htim);
 *   }
 * @endcode
 *
 * Cykl pomiarowy (sekwencyjny, jeden sensor naraz):
 *   TRIG 10 µs → echo (max 30 ms = ~4,3 m) → przerwa 5 ms → następny sensor
 *   Łączny czas: 4 × 35 ms = ~140 ms → ~7 Hz na sensor.
 */

#ifndef SONAR_DRIVER_H_
#define SONAR_DRIVER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Konfiguracja — dostosuj do sprzętu
 * -------------------------------------------------------------------------- */

/** Liczba sensorów HC-SR04. */
#define SONAR_COUNT          4U

/** Czas trwania impulsu TRIG (µs). HC-SR04 wymaga min. 10 µs. */
#define SONAR_TRIG_US        10U

/** Maksymalny czas oczekiwania na echo (µs). 30 ms ≈ 5,1 m zasięgu. */
#define SONAR_TIMEOUT_US     30000U

/** Przerwa między kolejnymi sensorami (µs).
 *  Czas dla wytłumienia fali poprzedniego sensora. */
#define SONAR_GAP_US         5000U

/** Prędkość dźwięku (m/s) przy 20°C. */
#define SONAR_SOUND_SPEED_MS 343U

/** Maksymalna odległość zgłaszana jako poprawna (mm). */
#define SONAR_DIST_MAX_MM    4000U

/** Wartość sygnalizująca brak echa (brak przeszkody w zasięgu). */
#define SONAR_NO_ECHO        0xFFFFU

/* --------------------------------------------------------------------------
 * Typy
 * -------------------------------------------------------------------------- */

/** Deskryptor jednego sensora HC-SR04. */
typedef struct {
    GPIO_TypeDef *trig_port;   /**< Port GPIO pinu TRIG              */
    uint16_t      trig_pin;    /**< Numer pinu TRIG (GPIO_PIN_x)     */
    uint32_t      tim_channel; /**< Kanał timera echa: TIM_CHANNEL_x */
} Sonar_HW_t;

/** Wynik jednego pomiaru. */
typedef struct {
    uint16_t distance_mm;   /**< Zmierzona odległość [mm]; SONAR_NO_ECHO = brak echa */
    uint32_t timestamp_ms;  /**< HAL_GetTick() w chwili zakończenia pomiaru           */
    bool     valid;         /**< false dopóki nie wykonano pierwszego pomiaru         */
} Sonar_Result_t;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief  Inicjalizacja sterownika.
 * @param  hw        Tablica SONAR_COUNT deskryptorów sprzętowych.
 * @param  htim_ic   Uchwyt timera do przechwytywania echa (TIM2, 1 MHz).
 * @param  htim_seq  Uchwyt timera sekwencera (TIM6, 1 MHz, one-pulse).
 */
void Sonar_Init(const Sonar_HW_t hw[SONAR_COUNT],
                TIM_HandleTypeDef *htim_ic,
                TIM_HandleTypeDef *htim_seq);

/** Uruchamia ciągłe pomiary (round-robin). */
void Sonar_Start(void);

/** Zatrzymuje pomiary i wyłącza wszystkie piny TRIG. */
void Sonar_Stop(void);

/**
 * @brief  Wywoływać z HAL_TIM_IC_CaptureCallback().
 * @param  htim     Uchwyt timera z callbacka.
 * @param  channel  htim->Channel (HAL_TIM_ACTIVE_CHANNEL_x).
 */
void Sonar_IC_Callback(TIM_HandleTypeDef *htim, HAL_TIM_ActiveChannel channel);

/**
 * @brief  Wywoływać z HAL_TIM_PeriodElapsedCallback().
 *         Obsługuje koniec impulsu TRIG, timeouty i przerwy między sensorami.
 */
void Sonar_PeriodElapsed_Callback(TIM_HandleTypeDef *htim);

/**
 * @brief  Zwraca ostatni wynik sensora @p idx (0-based).
 *         Bezpieczne dla wątków (chwilowe wyłączenie IRQ).
 */
Sonar_Result_t Sonar_GetResult(uint8_t idx);

/**
 * @brief  Zwraca true i zeruje flagę gdy wszystkie SONAR_COUNT sensorów
 *         skończyły jeden pełny cykl pomiarowy od ostatniego wywołania.
 */
bool Sonar_CycleComplete(void);

#endif /* SONAR_DRIVER_H_ */
