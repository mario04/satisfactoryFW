/*
 * customUart.h
 *
 *  Created on: 15 set 2015
 *      Author: Emil Kallias
 */

#ifndef UART_H_
#define UART_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define MAXSTRING 32
#define MAX_UART_STRING_LENGTH 256

void USART2_UART_Init(void);

int8_t uartWriteLineNoOS(char * inString);
int8_t uartWriteLine(char * inString);

int8_t uartWriteNoOS(char * String, uint16_t Size);

int8_t uartRead(char * String, uint16_t Size);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

int fputc ( int ch, FILE *stream );      // Primary UART for QUAT/ACCEL/GYRO data

#endif /* UART_H_ */
