/*
 * stm32f401rb_spi_driver.h
 *
 *  Created on: 29-Aug-2023
 *      Author: velmurugan
 */

#ifndef INC_STM32F401RB_SPI_DRIVER_H_
#define INC_STM32F401RB_SPI_DRIVER_H_

#include "stm32f401rb.h"

typedef struct{

	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
	uint32_t DR;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t I2SPR;

}SPIrefdef_t;

typedef struct{

	uint32_t SPI_mode;
	uint32_t SPI_busconfig;
	uint32_t SPI_clkspeed;
	uint32_t SPI_CPOL;
	uint32_t SPI_CPHA;
	uint32_t SPI_SSM;
	uint32_t DFF;

}spi_config_t;


typedef struct{

	SPIrefdef_t *SPIx;
    spi_config_t pconfig;
    uint8_t *Txbuffer;
    uint8_t Txstate;
    uint8_t Txlength;
    uint8_t *Rxbuffer;
    uint8_t Rxlength;
    uint8_t Rxstate;

}spi_handle_t;

#define SPI1 (SPIrefdef_t*)SPI1_I2S1_BASE_ADDRESS
#define SPI2 (SPIrefdef_t*)SPI2_I2S2_BASE_ADDRESS
#define SPI5 (SPIrefdef_t*)SPI3_I2S3_BASE_ADDRESS

#define SPI_FULL_DUPLEX 	0
#define SPI_HALF_DUPLEX 	1
#define SPI_RXONLY			3

#define SPI_DFF_8 		0
#define SPI_DFF_16		1

#define SPI_CLK_SPEED_2	 	0
#define SPI_CLK_SPEED_4		1
#define SPI_CLK_SPEED_8		2
#define SPI_CLK_SPEED_16	3
#define SPI_CLK_SPEED_32	4
#define SPI_CLK_SPEED_64	5
#define SPI_CLK_SPEED_128	6
#define SPI_CLK_SPEED_256	7


#define SPI_MODE_MASTER		1
#define SPI_MODE_SLAVE		0


#define SSM_ENABLED			1
#define SSM_DISABLED		0

#define SSI_SET				1
#define SSI_UNSET				0


#define SPI_CPOL_HIGH			1
#define SPI_CPOL_LOW			0

#define SPI_CPHA_HIGH			1
#define SPI_CPHA_LOW			0

#define SPI_BUSY_TX			0
#define SPI_BUSY_RX			1
#define SPI_READY_TX		2
#define SPI_READY_RX		3


// SPI initializations
void SPI_init(spi_handle_t *pSPIx);
void SPI_deinit(spi_handle_t *pSPIx);
void SPI2_peripheral_enable(SPIrefdef_t *SPIx);
void SPI2_SSI_config(SPIrefdef_t *SPIx);
// SPI Send and Recieve data

void SPI_SendData(SPIrefdef_t *SPIx, char *tx_data ,uint32_t length );
void SPI_RecieveData(SPIrefdef_t *SPIx, uint8_t *rx_data ,uint32_t length);

void SPI_SendDataIT(spi_handle_t *pSPIx, uint8_t *tx_data ,uint32_t length );
void SPI_RecieveDataIT(spi_handle_t *pSPIx, uint8_t *rx_data ,uint32_t length);

// IRQ SPI API's
void SPI_irq_config(uint8_t pin_number, uint8_t priority, uint8_t EnorDis);
void SPI_irq_handler(spi_handle_t *pSPIx);



#endif /* INC_STM32F401RB_SPI_DRIVER_H_ */
