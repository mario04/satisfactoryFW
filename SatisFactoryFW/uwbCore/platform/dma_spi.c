/*! ----------------------------------------------------------------------------
 * @file	dma_spi.c
 * @brief	realize fast dma spi read/write; in "safe" mode it used extra RAM buffer
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "stm32f4xx_hal_dma.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_port.h"
#include "cmsis_os.h"

/***************************************************************************//**
 * Local variables, private define
**/


extern SPI_HandleTypeDef hspi3;
//extern DMA_HandleTypeDef hdma_spi3_rx;
//extern DMA_HandleTypeDef hdma_spi3_tx;

// Two options of DMA_SPI - with tmp_buf buffer and without
// use  SAFE_DMA_AND_BUFF if you unsure that DMA_SPI works properly
#define SAFE_DMA_AND_BUFF

#ifdef	SAFE_DMA_AND_BUFF
 #define MAX_DMABUF_SIZE	4096
 static uint8 __align4 	tmp_buf[MAX_DMABUF_SIZE];
#endif

/*
 * 	do not use library function cause they are slow
 */
#define DMA_TX_CH	DMA1_Stream7
#define DMA_RX_CH	DMA1_Stream2
#define PORT_DMA_START_TX_FAST 	{__HAL_DMA_ENABLE(&hdma_spi3_tx);}
#define PORT_DMA_STOP_TX_FAST 	{__HAL_DMA_DISABLE(&hdma_spi3_tx);}
#define PORT_DMA_START_RX_FAST 	{__HAL_DMA_ENABLE(&hdma_spi3_rx);}
#define PORT_DMA_STOP_RX_FAST 	{__HAL_DMA_DISABLE(&hdma_spi3_rx);}


#define PORT_SPI_CLEAR_CS_FAST	{SPIx_CS_GPIO->BSRR = SPIx_CS;}
#define PORT_SPI_SET_CS_FAST	{SPIx_CS_GPIO->BSRR = SPIx_CS;}

#define DMA_SPI_MAX_ACCESS_ATTEMPTS		1000

/***************************************************************************//**
 * Exported function prototypes
 */
//void dma_init(void);

int writetospi_dma( uint16 headerLength,
			   	    const uint8 *headerBuffer,
					uint32 bodylength,
					const uint8 *bodyBuffer
				  );

int readfromspi_dma( uint16	headerLength,
			    	 const uint8 *headerBuffer,
					 uint32 readlength,
					 uint8 *readBuffer );

/***************************************************************************//**
 * @fn		dma_init()
 * @brief
 * 			init of dma module. we will not use IRQ for DMA transaction
 * 			spi_init should be executed first
 *
**/
/*void dma_init(void)
{

	 DMA interrupt init for SPI 3
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		/* connect DMA1 clock */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	/* connect SPI1 clock if not enable yet */
	//__DMA1_CLK_ENABLE();
/*
	HAL_DMA_DeInit(&hdma_spi3_tx);
	HAL_DMA_DeInit(&hdma_spi3_rx);


 do not use library function cause it slow
 connect SPI to DMA in write/read functions
 * SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx , ENABLE);
 * SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx , ENABLE);
*
    SPIx->CR2 |= SPI_CR2_TXDMAEN;	//connect Tx DMA_SPI
    SPIx->CR2 &= ~SPI_CR2_RXDMAEN;	//Disconnect Rx DMA_SPI by default

 DMA Channel SPI_TX Configuration
    DMA_TX_CH->CCR =DMA_DIR_PeripheralDST | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |\
    				DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Mode_Normal |\
    				DMA_Priority_High | DMA_M2M_Disable ;

    hdma_spi3_tx.Instance = DMA1_Stream7;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH; // TODO ?
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    //	hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    //	hdma_spi2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    //	hdma_spi2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    //	hdma_spi2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_spi3_tx);

    __HAL_LINKDMA(&hspi3,hdmarx,hdma_spi3_tx);

 Default DMA Channel SPI_RX Configuration
	DMA_RX_CH->CCR =DMA_DIR_PeripheralSRC | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |\
					DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Mode_Normal |\
					DMA_Priority_High | DMA_M2M_Disable ;

	hdma_spi3_rx.Instance = DMA1_Stream2;
	hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
	hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; // TODO ?
	hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi3_rx.Init.Mode = DMA_NORMAL;
	hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
//	hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//	hdma_spi2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
//	hdma_spi2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
//	hdma_spi2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
	HAL_DMA_Init(&hdma_spi3_rx);

	__HAL_LINKDMA(&hspi3,hdmarx,hdma_spi3_rx);



}*/


/***************************************************************************//**
 * @fn	writetospi()
 * @brief
 * 		  Low level function to write to the SPI
 * 		  Takes two separate byte buffers for write header and write data
 * 		  Source data will be transfered from buf to spi via DMA
 *
 * @return
 * 			DWT_SUCCESS or DWT_ERROR
 *
**/
#pragma GCC optimize ("O3")
int writetospi_dma
(
	uint16 headerLength,
	const uint8 *headerBuffer,
	uint32 bodylength,
	const uint8 *bodyBuffer
)
{
    int stat ;

#ifdef SAFE_DMA_AND_BUFF
    if ((headerLength + bodylength) > MAX_DMABUF_SIZE)
    {
    	return DWT_ERROR;
    }
#endif

    stat = decamutexon() ;

    memcpy(&tmp_buf[0], headerBuffer, headerLength);
    memcpy(&tmp_buf[headerLength], bodyBuffer, bodylength);

    port_SPI3_set_chip_select();

    HAL_SPI_Transmit_DMA(&hspi3, tmp_buf, headerLength+bodylength);

    while(DMA_TX_CH->NDTR !=0);	 //pool until last clock from SPI_RX

    while((SPI3->SR & (1 << 1)) == 0);  // wait while TXE flag is 0 (TX is not empty)
    //while((SPI3->SR & (1 << 7)) != 0);  // wait while BSY flag is 1 (SPI is busy)

    port_SPI3_clear_chip_select();
    //PORT_SPI_SET_CS_FAST

    decamutexoff(stat);

    return DWT_SUCCESS;
}


/***************************************************************************//**
 * @fn	readfromspi()
 * @brief Source data will be transfered from buf to spi via DMA
 * 		  Low level abstract function to write to the SPI
 * 		  Takes two separate byte buffers for write header and write data
 *
 * @return
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 * 			DWT_SUCCESS or DWT_ERROR
 *
**/
#pragma GCC optimize ("O3")
int readfromspi_dma
(
	uint16       headerLength,
	const uint8 *headerBuffer,
	uint32       readlength,
	uint8       *readBuffer
)
{

	uint8 spiBuffer[64] = "";
	uint16 counter;

    int stat ;

#ifdef	SAFE_DMA_AND_BUFF
    uint16 	count;
    count = (uint16)(headerLength + readlength);

    if (count > MAX_DMABUF_SIZE )
    {
    	return DWT_ERROR;
    }
#endif

    stat = decamutexon() ;


    //PORT_SPI_CLEAR_CS_FAST	// give extra time for SPI slave device

	port_SPI3_set_chip_select();

    HAL_SPI_TransmitReceive_DMA(&hspi3, headerBuffer, tmp_buf, count);

    while(DMA_RX_CH->NDTR !=0);	 //pool until last clock from SPI_RX

    while((SPI3->SR & (1 << 0)) != 0);  // wait while RXNE flag is not 1 (RX is not empty)
    //while((SPI3->SR & (1 << 7)) != 0);  // wait while BSY flag is 1 (SPI is busy)


     //result of read operation
    for (count=headerLength; count<(headerLength+readlength); count++)
    {
    	*readBuffer++ = tmp_buf[count];
    }

    port_SPI3_clear_chip_select();
    //PORT_SPI_SET_CS_FAST

    decamutexoff(stat);

    return DWT_SUCCESS;
}

/* eof dma_spi */
