#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    /* Czekaj aż poprzedni transfer DMA dobiegnie końca.
     * Przy 1 Mbaud maks. ramka 512 B = 5 ms — timeout 30 ms to duży zapas.
     * Bez tego czekania, gdy DMA jest zajęte, zapis był cicho dropowany,
     * co przy dużej liczbie publisherów powodowało utratę ACK sesji XRCE-DDS. */
    HAL_StatusTypeDef ret;
    uint32_t wait_ms = 0;
    while (uart->gState != HAL_UART_STATE_READY && wait_ms < 30) {
        osDelay(1);
        wait_ms++;
    }
    if (uart->gState != HAL_UART_STATE_READY) {
        return 0;  /* timeout — błąd sprzętowy */
    }

    ret = HAL_UART_Transmit_DMA(uart, buf, len);
    while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
        osDelay(1);
    }

    return (ret == HAL_OK) ? len : 0;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    int ms_used = 0;
    do
    {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();
        ms_used++;
        osDelay(portTICK_RATE_MS);
    } while (dma_head == dma_tail && ms_used < timeout);
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM