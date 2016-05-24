/*********************************************************************
File    : i2c.h
Purpose : 
**********************************************************************/
#ifndef __I2C_H__
#define __I2C_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Defines -------------------------------------------------------------------*/
#define SENSORS_I2C               I2C1

#define I2C_BUS_SPEED             100000
#define I2C_OWN_ADDRESS           0x00

#define I2C_Config() MX_I2C1_Init();

#define I2C_BUFFERSIZE			  32

/* Public variables ----------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;

/* Public function prototypes ------------------------------------------------*/

void MX_I2C1_Init(void);

int I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr,
										unsigned short RegisterLen, unsigned char *RegisterValue);

int I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr,
										unsigned short RegisterLen, const unsigned char *RegisterValue);


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif // __I2C_H__


