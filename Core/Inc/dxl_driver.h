#ifndef DXL_DRIVER_H_
#define DXL_DRIVER_H_

#include "main.h"
#include <stdbool.h>

// --- Rejestry RX-64 (Protokół 1.0) ---
#define DXL_REG_ID                  0x03
#define DXL_REG_RETURN_DELAY_TIME   0x05  // [EEPROM] 1 unit = 2 µs, domyslnie 250 (500 µs)
#define DXL_REG_CW_ANGLE_LIMIT      0x06
#define DXL_REG_CCW_ANGLE_LIMIT     0x08
#define DXL_REG_STATUS_RETURN_LEVEL 0x10  // [EEPROM] 0=nigdy, 1=tylko READ, 2=zawsze
#define DXL_REG_TORQUE_ENABLE       0x18
#define DXL_REG_TORQUE_LIMIT        0x22  // [RAM] 0-1023, 1023=100% max torque; wczytywany z EEPROM 0x0E przy starcie
#define DXL_REG_GOAL_POSITION       0x1E
#define DXL_REG_MOVING_SPEED        0x20
#define DXL_REG_PRESENT_POSITION    0x24  // 6 kolejnych bajtow: pos(2)+spd(2)+load(2)
#define DXL_REG_PRESENT_SPEED       0x26
#define DXL_REG_PRESENT_LOAD        0x28

// --- Instrukcje ---
#define DXL_INST_READ               0x02
#define DXL_INST_WRITE              0x03

typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* dir_port;
    uint16_t            dir_pin;
} DXL_Port_t;

// --- Funkcje komunikacyjne ---
void     DXL_Init         (DXL_Port_t* port, UART_HandleTypeDef* huart, GPIO_TypeDef* dir_port, uint16_t dir_pin);
void     DXL_WriteByte    (DXL_Port_t* port, uint8_t id, uint8_t reg, uint8_t val);
void     DXL_Write16      (DXL_Port_t* port, uint8_t id, uint8_t reg, uint16_t val);
uint16_t DXL_Read16       (DXL_Port_t* port, uint8_t id, uint8_t reg);

/* Blokowy odczyt pozycji + predkosci + obciazenia w jednym pakiecie (zamiast 3 osobnych).
 * Zwraca true jezeli pakiet odebrany i zweryfikowany (naglowek + ID + brak bledu). */
bool DXL_ReadPresentState (DXL_Port_t* port, uint8_t id,
                            uint16_t *position, uint16_t *speed, uint16_t *load);

// --- Funkcje sterujące (Raw) ---
void DXL_SetTorque       (DXL_Port_t* port, uint8_t id, bool enable);
void DXL_SetWheelMode    (DXL_Port_t* port, uint8_t id);
void DXL_SetGoalSpeedRaw (DXL_Port_t* port, uint8_t id, uint16_t raw_speed);

#endif
