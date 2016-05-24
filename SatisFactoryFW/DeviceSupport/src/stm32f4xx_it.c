/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

#include "deca_port.h"
#include "i2c.h"
#include "sf_port.h"
#include "uart.h"
#include "rgbLed.h"

#include "mpuMain.h"
#include <uwbMain.h>

/* External variables --------------------------------------------------------*/
//extern void xPortSysTickHandler(void);

/* external I2C variables */
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

/* external SPI variables */
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;

extern SemaphoreHandle_t uwbInterruptSemHandle;

extern SemaphoreHandle_t spi3TxSemHandle;
extern SemaphoreHandle_t spi3RxSemHandle;

/* external UART variables */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern SemaphoreHandle_t uart2TxSemHandle;
extern SemaphoreHandle_t uart2RxSemHandle;


extern RTC_HandleTypeDef hrtc;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/* |<-------------- Added from decadriver stm32f10x_it.c */

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void pop_registers_from_fault_stack(unsigned int * hardfault_args)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);
	/* Inspect stacked_pc to locate the offending instruction. */
	for( ;; );
}

void HardFault_Handler(void)
{
	{ __asm volatile (  " tst lr, #4 \n"
			" ite eq \n"
			" mrseq r0, msp \n"
			" mrsne r0, psp \n"
			" ldr r1, [r0, #24] \n"
			" ldr r2, handler2_address_const \n"
			" bx r2 \n"
			" handler2_address_const: .word pop_registers_from_fault_stack \n" );




	/* Go to infinite loop when Hard Fault exception occurs */
	}

	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles RTC global interrupt request.
 * @param  None
 * @retval None
 */
void RTC_IRQHandler(void)
{
	//if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
	if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_RESET)
	{
		/* Clear the RTC Second interrupt */
		//RTC_ClearITPendingBit(RTC_IT_SEC);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc,RTC_IT_ALRA); //TODO might not be right

		/* Wait until last write operation on RTC registers has finished */
		HAL_RTC_WaitForSynchro(&hrtc);
	}
}

/* Added from decadriver stm32f10x_it.c -------------->| */

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	//HAL_IncTick();
	//HAL_SYSTICK_IRQHandler();
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	//osSystickHandler();

	HAL_IncTick();
	osSystickHandler();
}

/**
 * @brief This function handles DMA1 Stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
	//static BaseType_t xHigherPriorityTaskWoken;
	HAL_DMA_IRQHandler(&hdma_i2c1_rx);

	if(hdma_i2c1_rx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */
	}
	else if(hdma_i2c1_rx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		hi2c1.State = HAL_I2C_STATE_READY;
	}

	else {
		/* DMS error */
		//osSemaphoreRelease(i2c1RxSemHandle);
		//hi2c1.State = HAL_I2C_STATE_READY;
		//uartWriteLineNoOS("I2C RX gone wrong\n");
	}
}

/**
 * @brief This function handles DMA1 Stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_i2c1_tx);

	if(hdma_i2c1_tx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */
	}
	else if(hdma_i2c1_tx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		//osSemaphoreRelease(i2c1TxSemHandle);
		hi2c1.State = HAL_I2C_STATE_READY;
	}
	else {
		/* DMS error */
		//osSemaphoreRelease(i2c1TxSemHandle);
		//hi2c1.State = HAL_I2C_STATE_READY;
		//uartWriteLineNoOS("I2C TX gone wrong\n");
	}
}


/**
 * @brief This function handles DMA1 Stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
	HAL_DMA_IRQHandler(&hdma_usart2_rx);

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	if(hdma_usart2_rx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */

	}
	else if(hdma_usart2_rx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		//xSemaphoreGiveFromISR(uart2RxSemHandle, NULL);
		//huart2.State = HAL_UART_STATE_READY;
	}
	else {
		/* DMS error */
		//huart2.State = HAL_UART_STATE_READY;
		//osSemaphoreRelease(uart2RxSemHandle);
		uartWriteLineNoOS("UART RX gone wrong\n");
	}


}

/**
 * @brief This function handles DMA1 Stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);

	if(hdma_usart2_tx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */
	}
	else if(hdma_usart2_tx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		xSemaphoreGiveFromISR(uart2TxSemHandle, NULL);
		huart2.State = HAL_UART_STATE_READY;
		//osSemaphoreRelease(uart2TxSemHandle);
	}
	else if(hdma_usart2_tx.State == HAL_DMA_STATE_ERROR) {
		/* DMS error Release UART Port to make sure the next transmission gets trough*/
		huart2.State = HAL_UART_STATE_READY;
		//osSemaphoreRelease(uart2TxSemHandle);
		//xSemaphoreGiveFromISR(uart2TxSemHandle, NULL);
		uartWriteLineNoOS("UART TX gone wrong\n");
	}
	else {
		/* DMA Busy */
	}
}

/**
 * @brief This function handles DMA1 Stream3 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void)
{
	//__IO HAL_SPI_StateTypeDef  state;
	//portBASE_TYPE xTaskWoken = pdFALSE;
	HAL_DMA_IRQHandler(&hdma_spi3_rx);

	if(hdma_spi3_rx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */
	}
	else if(hdma_spi3_rx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//state = hspi3.State;

		hspi3.State = HAL_SPI_STATE_READY;
		//xSemaphoreGiveFromISR(spi3RxSemHandle, NULL);
		//vTaskNotifyGiveFromISR( defaultTaskHandle, &xTaskWoken );
		//port_SPI3_clear_chip_select();
	}
	else if(hdma_spi3_rx.State == HAL_DMA_STATE_ERROR) {
		/* DMS error Release UART Port to make sure the next transmission gets trough*/

	}
	else {
		/* DMA Busy */
	}

	//portEND_SWITCHING_ISR( xTaskWoken );

}

/**
 * @brief This function handles DMA1 Stream4 global interrupt.
 */
void DMA1_Stream7_IRQHandler(void)
{

	HAL_DMA_IRQHandler(&hdma_spi3_tx);

	if(hdma_spi3_tx.State == HAL_DMA_STATE_READY_HALF_MEM0) {
		/* DMS half done */
	}
	else if(hdma_spi3_tx.State == HAL_DMA_STATE_READY_MEM0) {
		/* DMS done */
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//hspi3.State = HAL_SPI_STATE_READY;
		//osSemaphoreRelease(spi3TxSemHandle);
		//port_SPI3_clear_chip_select();
	}
	else if(hdma_spi3_tx.State == HAL_DMA_STATE_ERROR) {
		/* DMS error Release UART Port to make sure the next transmission gets trough*/

	}
	else {
		/* DMA Busy */
	}
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi3);
}


void USARTx_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief  This function handles ScenSor  interrupt request.
 * @param  None
 * @retval None
 */
/*void EXTI0_IRQHandler(void)
{
	process_dwRSTn_irq();
     Clear EXTI Line 0 Pending Bit
    EXTI_ClearITPendingBit(UWB_RSTn_EXTI);
}*/

/*void EXTI3_IRQHandler(void)
{
    process_deca_irq();
     Clear EXTI Line 3 Pending Bit
    EXTI_ClearITPendingBit(EXTI_Line3);
}*/


/* -------------- Added for decadriver TODO --------------
 *
 * EXTI0 Reset interrupt not implemented jet TODO
 * EXTI3 removed since not used for this design
 *
 * */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	/*process_dwRSTn_irq();
     Clear EXTI Line 0 Pending Bit
    EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);*/
}

void EXTI4_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	/*process_dwRSTn_irq();
     Clear EXTI Line 0 Pending Bit
    EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);*/
}


/**
 * @brief This function handles EXTI Line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
	/* Clear EXTI Line 8 Pending Bit */
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI9_5_IRQn);


	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


	if (GPIO_Pin == GPIO_PIN_0) {
		/* DWM1000 Reset Interrupt callback */
		//xSemaphoreGiveFromISR(decaRstIrqSemHandle, NULL);
		process_dwRSTn_irq();
	}
	else if (GPIO_Pin == GPIO_PIN_4) {
		/* DWM1000 Interrupt callback */

		uwb_interrupt_cb();

	}
	else if (GPIO_Pin == GPIO_PIN_6) {
		/* MPU Interrupt callback */
		//gyro_data_ready_cb();

	}
	else if (GPIO_Pin == GPIO_PIN_8) {
		/* MPU Interrupt callback */

	}
}


/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
