/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

/*
 * DWM1000_MOSI (SPI3)	PC12
 * DWM1000_MISO (SPI3)	PC11
 * DWM1000_SCK  (SPI3)	PC10
 * DWM1000_NSS  (SPI3)	PA4
 * DWM1000_IRQ			PB4
 * DWM1000_RST			PA0
 */


#include "deca_port.h"

#include "../../User/inc/compiler.h"
#include "cmsis_os.h"

/*
#define interrupt_init(x)			NVIC_Configuration(x)
#define spi_init(x)					SPI_Configuration(x)
*/
#define gpio_init(x)				GPIO_Configuration(x)

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;


extern SPI_HandleTypeDef hspi3;

/*DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;*/

SemaphoreHandle_t spi3TxSemHandle;
SemaphoreHandle_t spi3RxSemHandle;

int No_Configuration(void)
{
	return -1;
}

int portGetTickCnt(void)
{
	//return time32_incr;
	//return HAL_GetTick();
	return osKernelSysTick();
}


int NVIC_DisableDECAIRQ(void)
{
	HAL_NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);

	return 0;
}


/*int NVIC_Configuration(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStruct.Pin = DECAIRQ;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStruct);

	 EXTI interrupt init
	 * Enable and set EXTI Interrupt to the lowest priority
	 *
	HAL_NVIC_SetPriority(DECAIRQ_EXTI_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);

	HAL_GPIO_WritePin(DECAIRQ_GPIO, DECAIRQ, GPIO_PIN_RESET);

	return 0;
}*/

/**
 * @brief  Checks whether the specified EXTI line is enabled or not.
 * @param  EXTI_Line: specifies the EXTI line to check.
 *   This parameter can be:
 *     @arg EXTI_Linex: External interrupt line x where x(0..19)
 * @retval The "enable" state of EXTI_Line (SET or RESET).
 */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
	ITStatus bitstatus = RESET;
	uint32_t enablestatus = 0;
	/* Check the parameters */
	assert_param(IS_GET_EXTI_LINE(EXTI_Line));

	enablestatus =  EXTI->IMR & EXTI_Line;
	if (enablestatus != (uint32_t)RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}


void SPI_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void SPI_ConfigFastRate(uint16_t scalingfactor)
{

	__SPI3_CLK_ENABLE();

	hspi3.Instance = SPI3;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi3.Init.BaudRatePrescaler = scalingfactor;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&hspi3);


}

/*int SPI_Configuration(void)
{

	__SPI3_CLK_ENABLE();

	hspi3.Instance = SPI3;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	//hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
	//hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	hspi3.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&hspi3);

	//vSemaphoreCreateBinary(spi3RxSemHandle);
	//xSemaphoreTake(spi3RxSemHandle, osWaitForever);

	//vSemaphoreCreateBinary(spi3TxSemHandle);
	//xSemaphoreTake(spi3TxSemHandle, osWaitForever);

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_SET);

	return 0;
}*/

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStruct.Pin = DW1000_RSTn;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW1000_RSTn_GPIO, DW1000_RSTn, GPIO_PIN_SET);
	Sleep(2); //TODO ok?

	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = DW1000_RSTn;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);


	//HAL_GPIO_WritePin(DW1000_RSTn_GPIO, DW1000_RSTn, GPIO_PIN_SET);

}


void setup_DW1000RSTnIRQ(int enable)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(enable)
	{
		// Enable GPIO used as DECA IRQ for interrupt
		GPIO_InitStruct.Pin = DECARSTIRQ;
		//GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
		GPIO_InitStruct.Pull = GPIO_MODE_INPUT;
		HAL_GPIO_Init(DECARSTIRQ_GPIO, &GPIO_InitStruct);

		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(DECARSTIRQ_EXTI_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DECARSTIRQ_EXTI_IRQn);

	}
	else
	{
		//put the pin back to tri-state ... as input
		GPIO_InitStruct.Pin = DW1000_RSTn;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);

		HAL_NVIC_DisableIRQ(DECARSTIRQ_EXTI);
	}
}





int is_IRQ_enabled(void)
{
	return ((   NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]
						   & (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
}

/*
int decaPeripheralInit (void)
{
	//rcc_init();
	//gpio_init();
	//rtc_init();
	//systick_init();
	//interrupt_init();
	//spi_init();
#if (DMA_ENABLE == 1)
	//dma_init();	//init DMA for SPI only. Connection of SPI to DMA in read/write functions
#endif

	return 0;
}

void decaSpiPeripheralInit()
{
	//spi_init();

}
*/
