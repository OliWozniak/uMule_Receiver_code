#include "dxl_driver.h"

void DXL_Init(DXL_Port_t* port, UART_HandleTypeDef* huart, GPIO_TypeDef* dir_port, uint16_t dir_pin) {
    port->huart = huart;
    port->dir_port = dir_port;
    port->dir_pin = dir_pin;
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
}

static void DXL_SendPacket(DXL_Port_t* port, uint8_t id, uint8_t inst, uint8_t* params, uint8_t param_len) {
    uint8_t packet[16];
    uint8_t length = param_len + 2;
    packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
    packet[3] = length; packet[4] = inst;
    for(uint8_t i=0; i<param_len; i++) packet[5+i] = params[i];

    uint8_t checksum = 0;
    for(uint8_t i=2; i < 5 + param_len; i++) checksum += packet[i];
    packet[5 + param_len] = ~checksum;

    HAL_GPIO_WritePin(port->dir_port, port->dir_pin, GPIO_PIN_SET);
    HAL_UART_Transmit(port->huart, packet, length + 4, 50);
    while(__HAL_UART_GET_FLAG(port->huart, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(port->dir_port, port->dir_pin, GPIO_PIN_RESET);

    /* Flush ech i zalegajacych bajtow statusu z poprzednich operacji.
     * Bez tego DXL_Read16 czyta stale bajty zamiast odpowiedzi silnika. */
    __HAL_UART_CLEAR_FLAG(port->huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_PEF  | UART_CLEAR_FEF);
    while (__HAL_UART_GET_FLAG(port->huart, UART_FLAG_RXNE)) {
        volatile uint32_t dummy = port->huart->Instance->RDR;
        (void)dummy;
    }
}

void DXL_WriteByte(DXL_Port_t* port, uint8_t id, uint8_t reg, uint8_t val) {
    uint8_t params[2] = { reg, val };
    DXL_SendPacket(port, id, DXL_INST_WRITE, params, 2);
}

void DXL_Write16(DXL_Port_t* port, uint8_t id, uint8_t reg, uint16_t val) {
    uint8_t params[3] = { reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    DXL_SendPacket(port, id, DXL_INST_WRITE, params, 3);
}

uint16_t DXL_Read16(DXL_Port_t* port, uint8_t id, uint8_t reg) {
    uint8_t params[2] = { reg, 0x02 };
    DXL_SendPacket(port, id, DXL_INST_READ, params, 2);
    uint8_t res[8] = {0};
    if(HAL_UART_Receive(port->huart, res, 8, 25) == HAL_OK) {
        if(res[0] == 0xFF && res[1] == 0xFF) return (uint16_t)(res[5] | (res[6] << 8));
    }
    return 0xFFFF;
}

void DXL_SetTorque(DXL_Port_t* port, uint8_t id, bool enable) {
    uint8_t params[2] = { DXL_REG_TORQUE_ENABLE, enable ? 1 : 0 };
    DXL_SendPacket(port, id, DXL_INST_WRITE, params, 2);
}

void DXL_SetWheelMode(DXL_Port_t* port, uint8_t id) {
    DXL_Write16(port, id, DXL_REG_CW_ANGLE_LIMIT, 0);
    HAL_Delay(5);
    DXL_Write16(port, id, DXL_REG_CCW_ANGLE_LIMIT, 0);
}

void DXL_SetGoalSpeedRaw(DXL_Port_t* port, uint8_t id, uint16_t raw_speed) {
    DXL_Write16(port, id, DXL_REG_MOVING_SPEED, raw_speed);
}

bool DXL_ReadPresentState(DXL_Port_t* port, uint8_t id,
                           uint16_t *position, uint16_t *speed, uint16_t *load)
{
    uint8_t params[2] = { DXL_REG_PRESENT_POSITION, 0x06 };
    DXL_SendPacket(port, id, DXL_INST_READ, params, 2);

    uint8_t res[12] = {0};
    if (HAL_UART_Receive(port->huart, res, sizeof(res), 10) != HAL_OK)
        return false;

    if (res[0] != 0xFF || res[1] != 0xFF || res[2] != id || res[4] != 0x00)
        return false;

    uint8_t csum = 0;
    for (uint8_t k = 2; k <= 10; k++) csum += res[k];
    if (res[11] != (uint8_t)(~csum))
        return false;

    *position = (uint16_t)(res[5]  | (res[6]  << 8));
    *speed    = (uint16_t)(res[7]  | (res[8]  << 8));
    *load     = (uint16_t)(res[9]  | (res[10] << 8));
    return true;
}

