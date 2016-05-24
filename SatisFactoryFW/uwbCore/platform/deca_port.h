/*! ----------------------------------------------------------------------------
 * @file	port.h
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


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../../User/inc/compiler.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/*****************************************************************************************************************//*
 * To enable Direct Memory Access for SPI set this option to (1)
 * This option will increase speed of spi transactions but it will use an extra RAM memory buffer
 */
#define DMA_ENABLE	(1)


/*****************************************************************************************************************//*
**/
#if (DMA_ENABLE == 1)
 #define writetospi		writetospi_dma
 #define readfromspi	readfromspi_dma
 void dma_init(void);
#else

 extern int writetospi_serial( uint16_t headerLength,
 			   	    const uint8_t *headerBuffer,
 					uint32_t bodylength,
 					const uint8_t *bodyBuffer
 				  );

 extern int readfromspi_serial( uint16_t	headerLength,
 			    	 const uint8_t *headerBuffer,
 					 uint32_t readlength,
 					 uint8_t *readBuffer );

 #define writetospi		writetospi_serial
 #define readfromspi	readfromspi_serial
#endif

#define SPIx_PRESCALER				SPI_BAUDRATEPRESCALER_16

#define SPIx						SPI3
#define SPIx_GPIO					GPIOC
#define SPIx_CS						DWM_CS_Pin
#define SPIx_CS_GPIO				DWM_CS_GPIO_Port
#define SPIx_SCK					DWM_CLK_Pin
#define SPIx_MISO					DWM_MISO_Pin
#define SPIx_MOSI					DWM_MOSI_Pin

#define DW1000_RSTn					DWM_RST_Pin
#define DW1000_RSTn_GPIO			DWM_RST_GPIO_Port

#define DECARSTIRQ                  GPIO_PIN_0
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI0_IRQn // TODO check: EXTI_Line0
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource0
#define DECARSTIRQ_EXTI_IRQn        EXTI0_IRQn

#define DECAIRQ                     GPIO_PIN_4
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI4_IRQn //EXTI_Line5
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource4
#define DECAIRQ_EXTI_IRQn           EXTI4_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE


#define port_SPI3_busy_sending()		(__HAL_SPI_GET_FLAG((&hspi3),(SPI_FLAG_TXE))==(RESET))
#define port_SPI3_no_data()				(__HAL_SPI_GET_FLAG((&hspi3),(SPI_FLAG_RXNE))==(RESET))
#define port_SPI3_send_data(x)			SPI_I2S_SendData((SPIx),(x)) // TODO
#define port_SPI3_receive_data()		SPI_I2S_ReceiveData(SPIx) // TODO
#define port_SPI3_disable()				__HAL_SPI_DISABLE(&hspi3)
#define port_SPI3_enable()              __HAL_SPI_ENABLE(&hspi3)
#define port_SPI3_set_chip_select()		HAL_GPIO_WritePin(SPIx_CS_GPIO,SPIx_CS,GPIO_PIN_RESET)
#define port_SPI3_clear_chip_select()	HAL_GPIO_WritePin(SPIx_CS_GPIO,SPIx_CS,GPIO_PIN_SET)

// Legacy related defines
#define port_SPIx_set_chip_select()		port_SPI3_set_chip_select()
#define port_SPIx_clear_chip_select()	port_SPI3_clear_chip_select()

#define port_GET_stack_pointer()		__get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_AUDIBLE_enable()			// not used
#define port_AUDIBLE_disable()			// not used
#define port_AUDIBLE_set_interval_ms(x)	// not used
#define port_AUDIBLE_get_interval_ms(x)	// not used
#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()				NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 HAL_GPIO_ReadPin(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

void __weak process_dwRSTn_irq(void);
void __weak process_deca_irq(void);

int is_IRQ_enabled(void);

#define gpio_set(x)				0
#define gpio_reset(x)				0
#define is_gpio_out_low(x)			0
#define is_gpio_out_high(x)			0

/*
int decaPeripheralInit(void);
void decaSpiPeripheralInit(void);
*/

void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

int portGetTickCnt(void); //TODO

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
