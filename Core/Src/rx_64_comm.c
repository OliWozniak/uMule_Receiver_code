#include "rx_64_comm.h"

// Faktyczna definicja zmiennych
Servo right_wheel;
Servo left_wheel;
osMessageQueueId_t dynamixelQueueHandle;

const osThreadAttr_t dynamixelTask_attributes = {
  .name = "dynamixelTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};

void DX64_SetupWheelMode(Servo *servo) {
    setCWLimit(0, *servo);
    HAL_Delay(5);
    setCCWLimit(0, *servo);
}

ServoState_t DX64_GetFullState(Servo *servo) {
    ServoState_t state = {0};
    state.position = getPositionAngle(*servo);
    state.velocity = getSpeedRPM(*servo);
    state.load = getPresentLoad(*servo);
    return state;
}

void StartDynamixelTask(void *argument) {
    // Inicjalizacja kolejki (można też w main, ale tu bezpieczniej)
    if(dynamixelQueueHandle == NULL) {
        dynamixelQueueHandle = osMessageQueueNew(10, sizeof(DynamixelCommand_t), NULL);
    }

    right_wheel = AxelFlow_servo_init(0x01, &huart3, false);
    left_wheel = AxelFlow_servo_init(0x02, &huart3, false);

    DX64_SetupWheelMode(&right_wheel);
    DX64_SetupWheelMode(&left_wheel);

    DynamixelCommand_t incomingCmd;
    ServoState_t rightState, leftState;

    for(;;) {
        // A. Odbieranie poleceń
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // Mrugaj diodą, żeby widzieć, że task żyje

        if (osMessageQueueGet(dynamixelQueueHandle, &incomingCmd, NULL, 0) == osOK) {
            if (incomingCmd.id == 1) setSpeed(incomingCmd.speed, right_wheel);
            if (incomingCmd.id == 2) setSpeed(incomingCmd.speed, left_wheel);
        }

        // B. Odczyt stanu
        rightState = DX64_GetFullState(&right_wheel);
        leftState = DX64_GetFullState(&left_wheel);

        // Aby uniknąć ostrzeżeń o nieużywanych zmiennych, dopóki nie wysyłasz ich do ROS:
        (void)rightState;
        (void)leftState;

        osDelay(20);
    }
}
