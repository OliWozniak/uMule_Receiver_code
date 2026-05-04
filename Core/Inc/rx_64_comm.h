#ifndef RX_64_COMM_H_
#define RX_64_COMM_H_

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "AxelFlow.h"

// ---------------------------------------------------------------------------
// Struktury danych
// ---------------------------------------------------------------------------

// Polecenie prędkości przesyłane przez kolejkę FreeRTOS
// id: 1 = prawe koło, 2 = lewe koło
// speed: -100.0 .. +100.0 (procent maksymalnej prędkości, znak = kierunek)
typedef struct {
    uint8_t id;
    float   speed;
} DynamixelCommand_t;

// Stan jednego koła odczytywany z serwomechanizmu
typedef struct {
    float position;   // kąt [stopnie]
    float velocity;   // prędkość [RPM, wartość surowa z dataToDegrees]
    float load;       // obciążenie [%]
} ServoState_t;

// Wiadomość publikowana do micro-ROS (stan obu kół)
// Odpowiada custom msg geometry_msgs/TwistStamped lub własnej strukturze.
// Używamy płaskiej struktury żeby uniknąć zależności od custom msg:
//   float[6]: right_pos, right_vel, right_load, left_pos, left_vel, left_load
typedef struct {
    float data[6];
} WheelStateFeedback_t;

// ---------------------------------------------------------------------------
// Zmienne globalne (definicja w rx_64_comm.c)
// ---------------------------------------------------------------------------
extern Servo right_wheel;
extern Servo left_wheel;

extern osMessageQueueId_t dynamixelCmdQueueHandle;   // PC -> STM32 (komendy)
extern osMessageQueueId_t wheelFeedbackQueueHandle;  // STM32 -> micro-ROS task

extern const osThreadAttr_t dynamixelTask_attributes;

// ---------------------------------------------------------------------------
// Prototypy funkcji
// ---------------------------------------------------------------------------
void StartDynamixelTask(void *argument);
void DX64_SetupWheelMode(Servo *servo);
ServoState_t DX64_GetFullState(Servo *servo);

#endif /* RX_64_COMM_H_ */
