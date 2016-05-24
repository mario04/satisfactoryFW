/**
 ******************************************************************************
 * File Name          : uart.c
 * Description        : Custom UART functions
 * Author			  : Emil Kallias
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 Istituto Superiore Mario Boella (ISMB)
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "compiler.h"

#include "uart.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Defines -------------------------------------------------------------------*/

#define UART_TIMOUT		1000

/* Private variables ---------------------------------------------------------*/

SemaphoreHandle_t uart2TxSemHandle;
SemaphoreHandle_t uart2RxSemHandle;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

uint8_t uartRxBuffer = '\000'; // where we store that one character that just came in
uint8_t uartRxString[MAXSTRING]; // where we build our string from characters coming in
int uartRxindex = 0; // index for going though uartRxString


/**
 * @brief  Initialises UART2 hardware, registers and OS specific parameters
 */
void MX_UART2_Init(void)
{

	/* Initialise hardware, values taken from STM32CubeMX */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);

	/* Initialise operation system */

	//uart2RxSemHandle = xSemaphoreCreateBinary();
	//xSemaphoreTake(uart2RxSemHandle, osWaitForever);

	uart2TxSemHandle = xSemaphoreCreateBinary();
	xSemaphoreGive(uart2TxSemHandle);

	/* Initialise receiver */
	__HAL_UART_FLUSH_DRREGISTER(&huart2);
	HAL_UART_Receive_DMA(&huart2, &uartRxBuffer, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(huart->Instance==USART2)
	{
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */
		/* Peripheral clock enable */
		__USART2_CLK_ENABLE();

		/**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_usart2_rx.Instance = DMA1_Stream5;
		hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_rx.Init.MemInc = DMA_PINC_DISABLE;
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_rx.Init.Mode = DMA_NORMAL;
		hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		//hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
		//hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		//hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		HAL_DMA_Init(&hdma_usart2_rx);

		__HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

		hdma_usart2_tx.Instance = DMA1_Stream6;
		hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_tx.Init.Mode = DMA_NORMAL;
		hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		/*hdma_usart2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
		hdma_usart2_tx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_usart2_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;*/
		HAL_DMA_Init(&hdma_usart2_tx);

		__HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

		/* USER CODE BEGIN USART2_MspInit 1 */

		/* USER CODE END USART2_MspInit 1 */
	}

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

	if(huart->Instance==USART2)
	{
		/* USER CODE BEGIN USART2_MspDeInit 0 */

		/* USER CODE END USART2_MspDeInit 0 */
		/* Peripheral clock disable */
		__USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(huart->hdmarx);
		HAL_DMA_DeInit(huart->hdmatx);
		/* USER CODE BEGIN USART2_MspDeInit 1 */

		/* USER CODE END USART2_MspDeInit 1 */
	}

}



/**
 * @brief  Sends a sting ended by a newline symbol using blocking mode
 * 		Can be executed without OS does however block he MPU
 * @param  inString: pointer to a sting
 * @retval Number of transmitted bytes
 */
int8_t uartWriteLineNoOS(char * inString) {
	HAL_StatusTypeDef status;
	int32_t txStatus;
	uint8_t length = 0;
	uint8_t outString[256];

	int8_t timeout = UART_TIMOUT;

	//	while(inString[length] != '\n' && inString[length] != '\0') {
	//		outString[length] = inString[length];
	//		length++;
	//	}
	//	outString[length++] = '\n';
	//
	//	while((status = HAL_UART_Transmit(&huart2, outString, length, 1000)) != HAL_OK) {
	//		HAL_Delay(2);
	//		if(timeout-- > 0) return -1;
	//	}

	while(inString[length] != '\n' && inString[length] != '\0') {
		outString[length] = inString[length];
		length++;
	}
	outString[length] = ' ';
	outString[length+1] = '\n';

	while((status = HAL_UART_Transmit(&huart2, outString, length+2, 1000)) != HAL_OK) {
		HAL_Delay(2);
		if(timeout-- > 0) return -1;
	}

	return length;

}


/**
 * @brief  Sends a sting ended by a newline symbol using DMA and semaphores
 * 		Always make sure the FreeRTOS kernel is running when using this function
 * @param  inString: pointer to a sting
 * @retval Number of transmitted bytes
 */
int8_t uartWriteLine(char * inString) {

	HAL_StatusTypeDef status;
	uint8_t length = 0;
	int8_t timeout = UART_TIMOUT;
	char * outString;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	if((xSemaphoreTake(uart2TxSemHandle, 3000)) == pdTRUE) {

		while(inString[length] != '\n' && inString[length] != '\0') {
			length++;
		}

		outString = malloc(sizeof(char)*length+1);
		memcpy(outString,inString,length);

		outString[length++] = '\n';

		while((status = HAL_UART_Transmit_DMA(&huart2,(uint8_t *) outString, length)) != HAL_OK) {
			HAL_Delay(2);
			if(timeout-- > 0) return -1;
		}
	}

	return length;
}


int8_t uartRead(char * String, uint16_t Size){
	return NULL;
}

int8_t uartWriteNoOS(char * String, uint16_t Size) {

	HAL_StatusTypeDef status;

	status = HAL_UART_Transmit(&huart2, String, Size, 1000);
}




/**
 * @brief  Callback function for successful transmission
 * 		(not in use)
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval void
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	xSemaphoreGiveFromISR(uart2TxSemHandle, NULL);
}

/**
 * @brief  Callback function for successful receiption
 * 		(not in use)
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval void
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

	int i = 0;

	if (uartRxBuffer == '\n' || uartRxBuffer == '\r') // If Enter
	{
		uartRxString[uartRxindex++] = uartRxBuffer;
		//TODO executeSerialCommand(uartRxString);
		uartWriteLine(uartRxString);

		uartRxString[uartRxindex] = 0;
		uartRxindex = 0;
		for (i = 0; i < MAXSTRING; i++) uartRxString[i] = 0; // Clear the string buffer
	}

	else
	{
		uartRxString[uartRxindex] = uartRxBuffer; // Add that character to the string
		uartRxindex++;
		if (uartRxindex >= MAXSTRING) // User typing too much, we can't have commands that big
		{
			uartWriteLine(uartRxString);
			uartRxindex = 0;
			for (i = 0; i < MAXSTRING; i++) uartRxString[i] = 0; // Clear the string buffer
		}
	}

}

/**
 * @brief  bytewise output function for data streams
 * 		only available when OS is Running
 * @param  ch: character to send over uart
 * 		FILE: output stream (required but not used)
 * @retval character that has been transmitted or error
 */
int fputc(int ch, FILE *stream )
{
	HAL_StatusTypeDef status;
	uint8_t length = 0;
	uint8_t outString[256];
	int8_t timeout = UART_TIMOUT;


	//if((xSemaphoreTake(uart2TxSemHandle, 3000)) == pdTRUE) {
	while((status = HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 1000)) != HAL_OK) {
		//osDelay(2);
		HAL_Delay(2);
		if(timeout-- > 0) return -1;
	}
	//}

	//* Loop until the end of transmission

	//if(osSemaphoreWait(uart2TxSemHandle, 1000) == osOK){
	//	return ch;
	//}
	//return -1;


}
