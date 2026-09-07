/*
 * uart_tx.h
 *
 *  Created on: 7 Eyl 2026
 *      Author: furkan
 */

#ifndef UART_TX_H_
#define UART_TX_H_


#include "stm32f7xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef UART_TX_Init(UART_HandleTypeDef *huart);

HAL_StatusTypeDef UART_TX_Write(const uint8_t *data, uint16_t length);

void UART_TX_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* UART_TX_H_ */
