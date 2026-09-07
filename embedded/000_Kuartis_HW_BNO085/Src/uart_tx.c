/*
 * uart_tx.c
 *
 *  Created on: 7 Eyl 2026
 *      Author: furkan
 */

#include "uart_tx.h"

#include <stddef.h>


#define UART_TX_BUFFER_SIZE    256U


static UART_HandleTypeDef *uart_tx_handle = NULL;

static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];

static volatile uint16_t uart_tx_head = 0U;
static volatile uint16_t uart_tx_tail = 0U;

static volatile uint16_t uart_tx_active_length = 0U;
static volatile uint8_t uart_tx_active = 0U;


/**
 * @brief Returns the number of free bytes in the TX ring buffer.
 */

static uint16_t UART_TX_GetFreeSpace(void)
{
    uint16_t head;
    uint16_t tail;

    head = uart_tx_head;
    tail = uart_tx_tail;

    if (head >= tail)
    {
		return (uint16_t) (UART_TX_BUFFER_SIZE - (head - tail) - 1U);

    }

    return (uint16_t)(tail - head - 1U);
}

/**
 * @brief Starts transmission of the next contiguous buffer block.
 */
static HAL_StatusTypeDef UART_TX_StartNext(void)
{
    HAL_StatusTypeDef status;
    uint16_t chunk_length;

    if (uart_tx_handle == NULL)
    {
        return HAL_ERROR;
    }

    if (uart_tx_active != 0U)
    {
        return HAL_OK;
    }

    if (uart_tx_head == uart_tx_tail)
    {
        return HAL_OK;
    }

    /*
     * Send only the contiguous section between tail
     * and either head or the end of the ring buffer.
     */
    if (uart_tx_head > uart_tx_tail)
    {
        chunk_length =
            uart_tx_head -
            uart_tx_tail;
    }
    else
    {
        chunk_length =
            UART_TX_BUFFER_SIZE -
            uart_tx_tail;
    }

    uart_tx_active = 1U;
    uart_tx_active_length = chunk_length;

    status =
        HAL_UART_Transmit_IT(uart_tx_handle,
                             &uart_tx_buffer[uart_tx_tail],
                             chunk_length);

    if (status != HAL_OK)
    {
        uart_tx_active = 0U;
        uart_tx_active_length = 0U;
    }

    return status;
}

HAL_StatusTypeDef UART_TX_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    uart_tx_handle = huart;

    uart_tx_head = 0U;
    uart_tx_tail = 0U;

    uart_tx_active_length = 0U;
    uart_tx_active = 0U;

    return HAL_OK;
}


HAL_StatusTypeDef UART_TX_Write(const uint8_t *data,
                                uint16_t length)
{
    uint16_t i;
    uint16_t free_space;
    uint32_t primask;
    HAL_StatusTypeDef status;

    if ((data == NULL) ||
        (length == 0U) ||
        (uart_tx_handle == NULL))
    {
        return HAL_ERROR;
    }

    if (length >= UART_TX_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }

    /*
     * Protect shared head/tail state from UART ISR.
     *
     * The critical section is short:
     * normally only ~20 bytes are copied for one HDM sentence.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    free_space =
        UART_TX_GetFreeSpace();

    /*
     * All-or-nothing write.
     *
     * We never place half an NMEA sentence into the buffer.
     */
    if (length > free_space)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }

        return HAL_BUSY;
    }

    for (i = 0U; i < length; i++)
    {
        uart_tx_buffer[uart_tx_head] =
            data[i];

        uart_tx_head++;

        if (uart_tx_head >= UART_TX_BUFFER_SIZE)
        {
            uart_tx_head = 0U;
        }
    }

    /*
     * If UART is idle, start transmission immediately.
     * Otherwise the active transfer callback will continue it.
     */
    status =
        UART_TX_StartNext();

    if (primask == 0U)
    {
        __enable_irq();
    }

    return status;
}


void UART_TX_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != uart_tx_handle) ||
        (uart_tx_active == 0U))
    {
        return;
    }

    uart_tx_tail =
        (uint16_t)(uart_tx_tail +
                   uart_tx_active_length);

    if (uart_tx_tail >= UART_TX_BUFFER_SIZE)
    {
        uart_tx_tail -=
            UART_TX_BUFFER_SIZE;
    }

    uart_tx_active_length = 0U;
    uart_tx_active = 0U;

    /*
     * Continue automatically with any data
     * remaining in the ring buffer.
     */
    (void)UART_TX_StartNext();
}
