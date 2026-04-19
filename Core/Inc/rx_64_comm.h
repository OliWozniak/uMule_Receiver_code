#ifndef RX_64_COMM_H_
#define RX_64_COMM_H_

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "AxelFlow.h"

// Struktury danych
typedef struct {
    uint8_t id;
    float speed; // Zmieniono z target_angle na speed
} DynamixelCommand_t;

typedef struct {
    float position;
    float velocity;
    float load;
} ServoState_t;

// Zapowiedzi zmiennych (widoczne dla innych plików)
extern Servo right_wheel;
extern Servo left_wheel;
extern osMessageQueueId_t dynamixelQueueHandle;
extern const osThreadAttr_t dynamixelTask_attributes;

// Prototypy funkcji
void StartDynamixelTask(void *argument);
void DX64_SetupWheelMode(Servo *servo);
ServoState_t DX64_GetFullState(Servo *servo);

#endif
