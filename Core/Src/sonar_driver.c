/**
 * @file    sonar_driver.c
 * @brief   HC-SR04 — maszyna stanów sterowana przerwaniami.
 *
 * Diagram stanów (jeden cykl, jeden sensor):
 *
 *  Sonar_Start()
 *       │
 *       ▼
 *  [TRIGGER_HIGH] ──TIM6 10µs──► [WAIT_ECHO_RISE]
 *                                      │
 *                          TIM2 ↑   ──►[WAIT_ECHO_FALL]
 *                                      │               │
 *                          TIM2 ↓   ───┘         TIM6 timeout
 *                                      │               │
 *                               oblicz dist_mm    SONAR_NO_ECHO
 *                                      │               │
 *                                      └──────[GAP]────┘
 *                                               │
 *                                    TIM6 5ms ──► kolejny sensor
 *
 * Uwaga dot. przełączania polaryzacji w ISR:
 *   __HAL_TIM_SET_CAPTUREPOLARITY() to prosty zapis do rejestru CCER — bezpieczny
 *   w kontekście przerwania. Minimalna szerokość echa HC-SR04 (~116 µs przy 2 cm)
 *   jest znacznie większa niż czas obsługi ISR (~1 µs), więc zbocze opadające
 *   nie jest gubione.
 */

#include "sonar_driver.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Stany maszyny
 * -------------------------------------------------------------------------- */
typedef enum {
    S_IDLE          = 0,
    S_TRIGGER_HIGH,     /* TRIG HIGH — czekamy 10 µs (TIM6)         */
    S_WAIT_ECHO_RISE,   /* TRIG LOW  — czekamy na zbocze narastające */
    S_WAIT_ECHO_FALL,   /* Zbocze ↑ zarejestrowane — czekamy na ↓   */
    S_GAP,              /* Przerwa między sensorami (TIM6 5 ms)      */
} SonarState_t;

/* --------------------------------------------------------------------------
 * Zmienne modułu
 * -------------------------------------------------------------------------- */
static const Sonar_HW_t  *s_hw;
static TIM_HandleTypeDef *s_htim_ic;
static TIM_HandleTypeDef *s_htim_seq;

static volatile SonarState_t s_state      = S_IDLE;
static volatile uint8_t      s_idx        = 0;       /* aktywny sensor [0..SONAR_COUNT-1] */
static volatile uint32_t     s_echo_rise  = 0;       /* licznik IC przy zboczu narastającym */
static volatile bool         s_running    = false;
static volatile bool         s_cycle_done = false;
static volatile uint8_t      s_done_cnt   = 0;

static volatile Sonar_Result_t s_results[SONAR_COUNT];

/* --------------------------------------------------------------------------
 * Prywatne: konwersja HAL_TIM_ActiveChannel → TIM_CHANNEL_x
 * -------------------------------------------------------------------------- */
static uint32_t active_to_tim_channel(HAL_TIM_ActiveChannel ch)
{
    switch (ch) {
        case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
        case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
        case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
        case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
        default:                       return 0xFFU;
    }
}

/* --------------------------------------------------------------------------
 * Prywatne: jednorazowy strzał sekcera (TIM6 one-pulse mode)
 * -------------------------------------------------------------------------- */
static void seq_oneshot_us(uint32_t us)
{
    __HAL_TIM_DISABLE(s_htim_seq);
    __HAL_TIM_SET_AUTORELOAD(s_htim_seq, us - 1U);
    __HAL_TIM_SET_COUNTER(s_htim_seq, 0U);
    __HAL_TIM_CLEAR_FLAG(s_htim_seq, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(s_htim_seq, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(s_htim_seq);
}

/* --------------------------------------------------------------------------
 * Prywatne: włączenie IC dla aktualnego sensora (zbocze narastające)
 * -------------------------------------------------------------------------- */
static void ic_start_rising(void)
{
    TIM_IC_InitTypeDef cfg = {0};
    cfg.ICPolarity  = TIM_ICPOLARITY_RISING;
    cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    cfg.ICPrescaler = TIM_ICPSC_DIV1;
    cfg.ICFilter    = 0;  /* bez filtrowania — echo jest już stabilne */
    HAL_TIM_IC_ConfigChannel(s_htim_ic, &cfg, s_hw[s_idx].tim_channel);
    HAL_TIM_IC_Start_IT(s_htim_ic, s_hw[s_idx].tim_channel);
}

/* --------------------------------------------------------------------------
 * Prywatne: zapisanie wyniku + przejście do następnego sensora
 * -------------------------------------------------------------------------- */
static void record_and_advance(uint16_t dist_mm)
{
    /* Zapis wyniku — volatile pola, bezpieczne w ISR */
    s_results[s_idx].distance_mm  = dist_mm;
    s_results[s_idx].timestamp_ms = HAL_GetTick();
    s_results[s_idx].valid        = true;

    /* Sprawdzenie kompletności cyklu */
    s_done_cnt++;
    if (s_done_cnt >= SONAR_COUNT) {
        s_done_cnt   = 0;
        s_cycle_done = true;
    }

    /* Przejście do następnego sensora po przerwie GAP */
    s_idx   = (s_idx + 1U) % SONAR_COUNT;
    s_state = S_GAP;
    seq_oneshot_us(SONAR_GAP_US);
}

/* ========================================================================== */
/* Publiczne API                                                               */
/* ========================================================================== */

void Sonar_Init(const Sonar_HW_t hw[SONAR_COUNT],
                TIM_HandleTypeDef *htim_ic,
                TIM_HandleTypeDef *htim_seq)
{
    s_hw       = hw;
    s_htim_ic  = htim_ic;
    s_htim_seq = htim_seq;

    memset((void *)s_results, 0, sizeof(s_results));

    /* Wszystkie piny TRIG w stan niski */
    for (uint8_t i = 0; i < SONAR_COUNT; i++)
        HAL_GPIO_WritePin(hw[i].trig_port, hw[i].trig_pin, GPIO_PIN_RESET);
}

void Sonar_Start(void)
{
    if (s_running) return;
    s_running  = true;
    s_idx      = 0;
    s_done_cnt = 0;

    /* Impuls TRIG dla pierwszego sensora */
    HAL_GPIO_WritePin(s_hw[0].trig_port, s_hw[0].trig_pin, GPIO_PIN_SET);
    s_state = S_TRIGGER_HIGH;
    seq_oneshot_us(SONAR_TRIG_US);
}

void Sonar_Stop(void)
{
    s_running = false;
    s_state   = S_IDLE;

    __HAL_TIM_DISABLE(s_htim_seq);

    for (uint8_t i = 0; i < SONAR_COUNT; i++) {
        HAL_TIM_IC_Stop_IT(s_htim_ic, s_hw[i].tim_channel);
        HAL_GPIO_WritePin(s_hw[i].trig_port, s_hw[i].trig_pin, GPIO_PIN_RESET);
    }
}

Sonar_Result_t Sonar_GetResult(uint8_t idx)
{
    Sonar_Result_t r;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    r = *(const Sonar_Result_t *)&s_results[idx];
    __set_PRIMASK(primask);
    return r;
}

bool Sonar_CycleComplete(void)
{
    if (!s_cycle_done) return false;
    s_cycle_done = false;
    return true;
}

/* ========================================================================== */
/* Callbacki przerwań                                                          */
/* ========================================================================== */

/**
 * @brief  Wywoływać z HAL_TIM_IC_CaptureCallback().
 *
 *  Zbocze narastające: zapisuje czas startu, przełącza IC na zbocze opadające.
 *  Zbocze opadające:   oblicza szerokość pulsu → odległość, anuluje timeout.
 */
void Sonar_IC_Callback(TIM_HandleTypeDef *htim, HAL_TIM_ActiveChannel channel)
{
    if (htim != s_htim_ic) return;

    uint32_t tim_ch = active_to_tim_channel(channel);
    if (tim_ch != s_hw[s_idx].tim_channel) return;

    switch (s_state) {

    /* --- Zbocze narastające echa ----------------------------------------- */
    case S_WAIT_ECHO_RISE:
        s_echo_rise = HAL_TIM_ReadCapturedValue(htim, tim_ch);

        /* Przełącz na zbocze opadające (zapis do CCER — OK w ISR) */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_ch,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);

        s_state = S_WAIT_ECHO_FALL;
        /* TIM6 timeout nadal biegnie od momentu uruchomienia IC — nie restartujemy */
        break;

    /* --- Zbocze opadające echa ------------------------------------------- */
    case S_WAIT_ECHO_FALL: {
        /* Anuluj timeout TIM6 */
        __HAL_TIM_DISABLE(s_htim_seq);
        __HAL_TIM_DISABLE_IT(s_htim_seq, TIM_IT_UPDATE);

        uint32_t echo_fall = HAL_TIM_ReadCapturedValue(htim, tim_ch);
        HAL_TIM_IC_Stop_IT(htim, tim_ch);

        /* Oblicz szerokość pulsu z obsługą przepełnienia licznika.
         * Dla TIM2 (32-bit) przepełnienie w 30ms nigdy nie nastąpi,
         * ale wzór jest poprawny również dla timerów 16-bit. */
        uint32_t max_cnt  = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
        uint32_t pulse_us = (echo_fall >= s_echo_rise)
                            ? (echo_fall - s_echo_rise)
                            : (max_cnt - s_echo_rise + echo_fall);

        /* Przelicz na mm: dist = pulse * sound / 2 */
        uint32_t dist_mm = (pulse_us * SONAR_SOUND_SPEED_MS) / 2000U;
        if (dist_mm > SONAR_DIST_MAX_MM) dist_mm = SONAR_NO_ECHO;

        record_and_advance((uint16_t)dist_mm);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief  Wywoływać z HAL_TIM_PeriodElapsedCallback().
 *
 *  Obsługuje trzy zdarzenia TIM6:
 *    - Koniec impulsu TRIG (10 µs) → ustaw TRIG LOW, uruchom IC
 *    - Timeout echa (30 ms)        → brak echa, zanotuj SONAR_NO_ECHO
 *    - Koniec przerwy GAP (5 ms)   → uruchom TRIG następnego sensora
 */
void Sonar_PeriodElapsed_Callback(TIM_HandleTypeDef *htim)
{
    if (htim != s_htim_seq) return;

    switch (s_state) {

    /* --- Koniec impulsu TRIG ----------------------------------------------- */
    case S_TRIGGER_HIGH:
        HAL_GPIO_WritePin(s_hw[s_idx].trig_port,
                          s_hw[s_idx].trig_pin, GPIO_PIN_RESET);

        ic_start_rising();           /* uruchom IC na zbocze narastające */
        s_state = S_WAIT_ECHO_RISE;
        seq_oneshot_us(SONAR_TIMEOUT_US); /* ustaw timeout na czas echa */
        break;

    /* --- Timeout echa (brak zbocza narastającego lub opadającego) ---------- */
    case S_WAIT_ECHO_RISE:
    case S_WAIT_ECHO_FALL:
        HAL_TIM_IC_Stop_IT(s_htim_ic, s_hw[s_idx].tim_channel);
        record_and_advance(SONAR_NO_ECHO);
        break;

    /* --- Koniec przerwy GAP — wyzwól następny sensor ----------------------- */
    case S_GAP:
        if (!s_running) { s_state = S_IDLE; return; }

        HAL_GPIO_WritePin(s_hw[s_idx].trig_port,
                          s_hw[s_idx].trig_pin, GPIO_PIN_SET);
        s_state = S_TRIGGER_HIGH;
        seq_oneshot_us(SONAR_TRIG_US);
        break;

    default:
        break;
    }
}
