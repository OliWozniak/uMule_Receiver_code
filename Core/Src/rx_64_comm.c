#include "rx_64_comm.h"

// ---------------------------------------------------------------------------
// Definicje zmiennych globalnych
// ---------------------------------------------------------------------------
Servo right_wheel;
Servo left_wheel;

osMessageQueueId_t dynamixelCmdQueueHandle  = NULL;
osMessageQueueId_t wheelFeedbackQueueHandle = NULL;

const osThreadAttr_t dynamixelTask_attributes = {
    .name       = "dynamixelTask",
    .priority   = (osPriority_t) osPriorityNormal,
    .stack_size = 1024 * 4
};

// ---------------------------------------------------------------------------
// Pomocnicze
// ---------------------------------------------------------------------------

// Ustawia serwomechanizm w tryb koła:
// CW limit = 0 i CCW limit = 0 jednocześnie oznacza tryb koła w Dynamixel.
void DX64_SetupWheelMode(Servo *servo) {
    setCWLimit(0, *servo);
    HAL_Delay(10);
    setCCWLimit(0, *servo);
    HAL_Delay(10);
    // Wyzeruj prędkość startową
    setSpeed(0.0f, *servo);
    HAL_Delay(5);
}

// Odczytuje pozycję, prędkość i obciążenie w jednym wywołaniu.
ServoState_t DX64_GetFullState(Servo *servo) {
    ServoState_t state = {0};
    state.position = getPositionAngle(*servo);
    state.velocity = getSpeedRPM(*servo);
    state.load     = getPresentLoad(*servo);
    return state;
}

// ---------------------------------------------------------------------------
// Task FreeRTOS
// ---------------------------------------------------------------------------
void StartDynamixelTask(void *argument) {

    // Inicjalizacja kolejek (tworzone w main przed startem kernela,
    // ale zabezpieczamy się na wypadek gdyby nie były)
    if (dynamixelCmdQueueHandle == NULL) {
        dynamixelCmdQueueHandle = osMessageQueueNew(10, sizeof(DynamixelCommand_t), NULL);
    }
    if (wheelFeedbackQueueHandle == NULL) {
        wheelFeedbackQueueHandle = osMessageQueueNew(4, sizeof(WheelStateFeedback_t), NULL);
    }

    // Inicjalizacja serwomechanizmów
    // ID 0x01 = prawe koło, ID 0x02 = lewe koło (zgodnie z istniejącym kodem)
    right_wheel = AxelFlow_servo_init(0x01, &huart3, false);
    left_wheel  = AxelFlow_servo_init(0x02, &huart3, false);

    DX64_SetupWheelMode(&right_wheel);
    DX64_SetupWheelMode(&left_wheel);

    DynamixelCommand_t incomingCmd;
    ServoState_t       rightState, leftState;
    WheelStateFeedback_t feedback;

    // Licznik do wysyłania feedbacku co N iteracji (20ms * 5 = 100ms)
    uint8_t feedback_counter = 0;
    const uint8_t FEEDBACK_EVERY_N = 5;

    for (;;) {
        // ----------------------------------------------------------------
        // A. Mruganie diodą: sygnał życia taska
        // ----------------------------------------------------------------
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

        // ----------------------------------------------------------------
        // B. Odbiór i realizacja poleceń prędkości z kolejki
        //    Kolejka jest nieblokująca (timeout=0): jeśli pusta - jedziemy dalej
        // ----------------------------------------------------------------
        while (osMessageQueueGet(dynamixelCmdQueueHandle,
                                 &incomingCmd, NULL, 0) == osOK) {
            if (incomingCmd.id == 1) {
                setSpeed(incomingCmd.speed, right_wheel);
            } else if (incomingCmd.id == 2) {
                setSpeed(incomingCmd.speed, left_wheel);
            }
        }

        // ----------------------------------------------------------------
        // C. Odczyt stanu kół (co FEEDBACK_EVERY_N iteracji = co ~100ms)
        // ----------------------------------------------------------------
        feedback_counter++;
        if (feedback_counter >= FEEDBACK_EVERY_N) {
            feedback_counter = 0;

            rightState = DX64_GetFullState(&right_wheel);
            leftState  = DX64_GetFullState(&left_wheel);

            feedback.data[0] = rightState.position;
            feedback.data[1] = rightState.velocity;
            feedback.data[2] = rightState.load;
            feedback.data[3] = leftState.position;
            feedback.data[4] = leftState.velocity;
            feedback.data[5] = leftState.load;

            // Wrzuć do kolejki; jeśli pełna - nadpisz najstarszy element
            if (osMessageQueueGetSpace(wheelFeedbackQueueHandle) == 0) {
                WheelStateFeedback_t dummy;
                osMessageQueueGet(wheelFeedbackQueueHandle, &dummy, NULL, 0);
            }
            osMessageQueuePut(wheelFeedbackQueueHandle, &feedback, 0, 0);
        }

        // ----------------------------------------------------------------
        // D. Czekaj 20ms (50Hz pętla sterowania)
        // ----------------------------------------------------------------
        osDelay(20);
    }
}
