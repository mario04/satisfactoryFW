/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  : 
 ********************************** Includes ***********************************/

/* Includes ------------------------------------------------------------------*/

#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "uart.h"


/* Defines -------------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/



/* Private variables ---------------------------------------------------------*/

/*
osThreadId i2cReadTaskHandle;
osThreadId callingreadI2CTaskHandle;*/

SemaphoreHandle_t  i2c1RxSemHandle;
SemaphoreHandle_t  i2c1TxSemHandle;
SemaphoreHandle_t  i2c1LockSemHandle;

I2C_HandleTypeDef hi2c1;

DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;


typedef struct {
	unsigned char Address;
	unsigned char RegisterAddr;
	unsigned short RegisterLen;
	unsigned char *RegisterValue;
} readWriteTaskDataStucture_t;

volatile uint8_t i2cRxBuffer = '\000'; // where we store that one character that just came in

/* Public functions ----------------------------------------------------------*/

/* I2C1 init function */
void MX_I2C1_Init(void)
{
	hi2c1.Instance = SENSORS_I2C;
	hi2c1.Init.ClockSpeed = I2C_BUS_SPEED;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = I2C_OWN_ADDRESS;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);

	/* Create receive semaphore and lock it until the first reception */
	vSemaphoreCreateBinary(i2c1RxSemHandle);
	xSemaphoreTake(i2c1RxSemHandle, osWaitForever);

	/* Create transmission semaphore and leave it unlock to allow first transmission */
	vSemaphoreCreateBinary(i2c1TxSemHandle);
	xSemaphoreGive(i2c1TxSemHandle);

	//	vSemaphoreCreateBinary(i2c1LockSemHandle);
	//	xSemaphoreGive(i2c1LockSemHandle);




}


void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(hi2c->Instance==I2C1)
	{
		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		/**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__I2C1_CLK_ENABLE();

		/* Peripheral DMA init*/

		hdma_i2c1_rx.Instance = DMA1_Stream0;
		hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
		hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_rx.Init.MemInc = DMA_MINC_DISABLE;
		hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_i2c1_rx);

		__HAL_LINKDMA(hi2c,hdmarx,hdma_i2c1_rx);

		hdma_i2c1_tx.Instance = DMA1_Stream1;
		hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_0;
		hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_tx.Init.MemInc = DMA_MINC_DISABLE;
		hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_i2c1_tx);

		__HAL_LINKDMA(hi2c,hdmatx,hdma_i2c1_tx);

		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

	if(hi2c->Instance==I2C1)
	{
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(hi2c->hdmarx);
		HAL_DMA_DeInit(hi2c->hdmatx);
		/* USER CODE BEGIN I2C1_MspDeInit 1 */

		/* USER CODE END I2C1_MspDeInit 1 */
	}
}



int I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr,
		unsigned short RegisterLen, unsigned char *RegisterValue) {

	HAL_StatusTypeDef status;
	/* bitshift adress by one bit since the ST HAL function assumes the
	 * address is given in 8 bit and the motion driver assume 7 bit */
	//status = HAL_I2C_Mem_Read_DMA(&hi2c1, (Address<<1) , RegisterAddr, 1, RegisterValue, RegisterLen);
	status = HAL_I2C_Mem_Read(&hi2c1, (Address<<1) , RegisterAddr, 1, RegisterValue, RegisterLen, 1000);
	//HAL_Delay(1);

	/*if(status == HAL_OK) {
		 lock semaphore until the rx complete interrupt unlocks it to make sure
	 * there is a register value to read once the function is exited
		if (xSemaphoreTake(i2c1RxSemHandle, 1000) == pdTRUE) {
			return 0;
		}
	}*/

	return((int) status);
	//return -1;
}


int I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr,
		unsigned short RegisterLen, const unsigned char *RegisterValue) {

	HAL_StatusTypeDef status;

	//status = HAL_I2C_Mem_Write_DMA(&hi2c1, (Address<<1), RegisterAddr, 1, RegisterValue, RegisterLen);
	status = HAL_I2C_Mem_Write(&hi2c1, (Address<<1), RegisterAddr, 1, RegisterValue, RegisterLen, 1000);

	return((int) status);

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {

	//xSemaphoreGiveFromISR(i2c1TxSemHandle, NULL);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	//xSemaphoreGiveFromISR(i2c1RxSemHandle, NULL);

}
