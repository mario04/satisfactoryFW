/*
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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

//#include <mpuMain.h>
#include "compiler.h"

#include "cmsis_os.h"

#include "stm32f4xx_hal.h"

#include "sf_port.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "dma.h"
#include "gpio.h"
#include "rgbLed.h"
//#include "deca_port.h"
//#include "deca_spi.h"
//#include "deca_device_api.h"
//#include "instance.h"

//#include "mpuMain.h"
#include "uwbMain.h"


/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId permanentBlinkTaskHandle;
osThreadId permanentBlinkTaskHandle2;
/*
osThreadId blinkTaskHandle;
osThreadId readI2CTaskHandle;
osThreadId readTemperatureTaskHandle;


osSemaphoreId semHandle;


extern volatile uint8_t i2cRxBuffer;

extern I2C_HandleTypeDef hi2c1;
*/

extern SPI_HandleTypeDef hspi3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/*
void StartUWBTask(void const * argument);
void StartMPUTask(void const * argument);
*/

void StartDefaultTask(void const * argument);
void PermanentBlinkTask(void const * argument);
/*void BlinkTask(void const * argument);
void ReadI2CTask(void const *argument);
void ReadTemperatureTask(void const *argument);

void GPIO_BlinkPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

uint64 geteui();

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} blinkTaskDataStucture_t;
*/



int main(void){

	//uint8_t mpuWhoAmI[1];
	//char outString[MAX_UART_STRING_LENGTH];

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	//MX_RTC_Init();
	MX_DMA_Init();
	MX_GPIO_Init();

	MX_UART2_Init();
	MX_SPI3_Init();
	//MX_I2C1_Init();

	MX_TIM1_Init();

	//decaPeripheralInit();


	SetRGBLed(RED, 50);

	/* Test UART Transmit without semaphores */
	uartWriteLineNoOS("SatisFactory MPU9250 no OS Test\n");

	/* Create the threads and semaphore */
	//osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	//defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	osThreadDef(permanentBlinkTask, PermanentBlinkTask, osPriorityNormal, 0, 128);
	permanentBlinkTaskHandle = osThreadCreate(osThread(permanentBlinkTask), NULL);

	//osThreadDef(permanentBlinkTask2, PermanentBlinkTask, osPriorityNormal, 0, 128);
	//permanentBlinkTaskHandle2 = osThreadCreate(osThread(permanentBlinkTask2), 200);

	osThreadDef(uwbInitTask, UwbInitTask, osPriorityNormal, 0, 512); //128
	uwbInitTaskHandle = osThreadCreate(osThread(uwbInitTask), NULL);

	//osThreadDef(readI2CTask, ReadI2CTask, osPriorityNormal, 0, 512); //128
	//readI2CTaskHandle = osThreadCreate(osThread(readI2CTask), NULL);

	/* Start scheduler */
	osKernelStart();

	//StartDefaultTask(NULL);
	//StartUWBTask(NULL);
	//StartMPUTask(NULL);
	//ReadI2CTask(NULL);

	/* We should never get here as control is now taken by the scheduler */


	/* Infinite loop */
	while (1){
		//RGBLedFade(1);
	}

}



/* StartDefaultTask function */
void StartDefaultTask(void const *argument)
{
	UNUSED(argument);

	uint8_t pTxData = 0x00;
	uint8_t pRxData[4];
	char outString[MAX_UART_STRING_LENGTH];
	uint32_t regval = -1;

	uartWriteLineNoOS("SatisFactory Default Task\n");
	//SetRGBLed(ORANGE,100);

	while (1){
		osDelay(1000);

		HAL_GPIO_WritePin(DWM_CS_GPIO_Port,DWM_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_DMA(&hspi3, &pTxData, pRxData, 5);
		osDelay(1);
		HAL_GPIO_WritePin(DWM_CS_GPIO_Port,DWM_CS_Pin,GPIO_PIN_SET);

		for (int j = 4 ; j > 0 ; j--) {
			regval = (regval << 8) + pRxData[j] ;        // sum
		}

		//sprintf(outString, "Decawave chip ID: %08X",(int) regval);
		//uartWriteLineNoOS(outString); //send some data

		//printf("Decawave chip ID: %08X\n",(int) regval);


	}

}


void PermanentBlinkTask(void const *argument)
{
	UNUSED(argument);

	uartWriteLine("SatisFactory Permanent Blink Task\n");

	while(1) {

		SetRGBLed(ORANGE,70);
		osDelay(500);
		SetRGBLed(REDMAGENTA,0);
		osDelay(500);

		//uartWriteLine("UART Test\n");

	}
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}


#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
