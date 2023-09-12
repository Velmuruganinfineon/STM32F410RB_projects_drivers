/*
 * stm32f401rb_spi_driver.c
 *
 *  Created on: 30-Aug-2023
 *      Author: velmurugan
 */

#include "stm32f401rb_spi_driver.h"

void spi_txe_interrupt_handle(spi_handle_t*);
void spi_rxe_interrupt_handle(spi_handle_t*);
void spi_error_interrupt_handle(spi_handle_t*);
void spi_closereception(spi_handle_t* SPIx);
void spi_closetransmission(spi_handle_t* SPIx);
void spi_applicationcallback();

void SPI_init(spi_handle_t *obj)
{
	uint32_t tempreg = 0;

	if(obj->pconfig.SPI_busconfig == SPI_FULL_DUPLEX)
	{
		tempreg &= ~(1<<15);
	}
	else if(obj->pconfig.SPI_busconfig == SPI_HALF_DUPLEX)
	{
		tempreg |= (1<<15);
	}
	else if(obj->pconfig.SPI_busconfig == SPI_RXONLY)
	{
		tempreg &= ~(1<<15);
		tempreg |= (1<<10);
	}

	if(obj->pconfig.DFF == SPI_DFF_8)
	{
		tempreg &= ~(1<<11);
	}
	else
	{
		tempreg |= (1<<11);
	}

	if(obj->pconfig.SPI_SSM == SSM_ENABLED)
	{
		tempreg |= (1<<9);
	}

	if(obj->pconfig.SPI_mode == SPI_MODE_MASTER)
	{
		tempreg |= (1<<2);
	}

	if(obj->pconfig.SPI_CPOL == SPI_CPOL_HIGH)
	{
		tempreg |= (1<<1);
	}

	if(obj->pconfig.SPI_CPHA == SPI_CPHA_HIGH)
	{
		tempreg |= (1<<0);
	}

	tempreg |= (obj->pconfig.SPI_clkspeed << 3);

	obj->SPIx->CR1 = tempreg;

}

void SPI_deinit(spi_handle_t *pSPIx){

}

// SPI Send and Recieve data

void SPI_SendData(SPIrefdef_t *SPIx, char *tx_data ,uint32_t length ){

	uint32_t length_buffer = length;

	if(length == 0)
	{
		return;
	}
	else{

		while(length_buffer != 0){



				if((SPIx->CR1)& (1<<1))
				{
					SPIx->DR = *((uint16_t*)tx_data);
					length_buffer--;
					length_buffer--;
					(uint16_t*)tx_data++;
				}

				else{

					SPIx->DR = *((uint8_t*)tx_data);
                    length_buffer--;
                    (uint8_t*)tx_data++;

				}



		}


	}

}

void SPI_RecieveData(SPIrefdef_t *SPIx, uint8_t *rx_data ,uint32_t length){

	uint32_t length_buffer = length;

		if(length == 0)
		{
			return;
		}
		else{

			while(length_buffer != 0){



					if((SPIx->CR1)& (1<<0))
					{

						*rx_data = SPIx->DR;
						length_buffer--;
						length_buffer--;
						(uint16_t*)rx_data++;

					}

					else{

						*rx_data = SPIx->DR;
						length_buffer--;
						(uint16_t*)rx_data++;

					}



			}


		}

}

// IRQ SPI API's
void SPI_irq_config(uint8_t irq_pin_number, uint8_t priority, uint8_t EnorDis){

	uint8_t pinx;
	uint8_t pinx_t;

    if (EnorDis)
    {
        if (irq_pin_number <= 31)
        {
        	*NVIC_ISER0 |= (1<< irq_pin_number);
        	//*NVIC_ICER0 &= ~(1<<irq_pin_number);
            // TODO: Enable IRQ in NVIC
        }
        else if (irq_pin_number <= 63 && irq_pin_number >= 32)
        {
        	*NVIC_ISER1 |= (1<< (irq_pin_number%32));
        	//*NVIC_ICER1 &= ~(1<<irq_pin_number);
            // TODO: Enable IRQ in NVIC
        }
        else if (irq_pin_number >= 64 && irq_pin_number < 96)
        {
        	*NVIC_ISER2 |= (1<< (irq_pin_number%64));
        	//*NVIC_ICER2 &= ~(1<<irq_pin_number);
            // TODO: Enable IRQ in NVIC
        }


        pinx = irq_pin_number / 4 ;
        pinx_t = irq_pin_number % 4;
        *(NVIC_PR_BASE_ADDR + (pinx)) |= priority << (pinx_t*8 + 4);


    }
    else
    {
        if (irq_pin_number <= 31)
        {
        	*NVIC_ICER0 &= ~(1<<irq_pin_number);
            // TODO: Disable IRQ in NVIC
        }
        else if (irq_pin_number <= 63 && irq_pin_number >= 32)
        {
        	*NVIC_ICER1 &= ~(1<<(irq_pin_number%32));
            // TODO: Disable IRQ in NVIC
        }
        else if (irq_pin_number >= 64 && irq_pin_number < 96)
        {
        	*NVIC_ICER2 &= ~(1<<(irq_pin_number%64));
            // TODO: Disable IRQ in NVIC
        }
    }


}
void SPI_irq_handler(spi_handle_t *pSPIx){

	uint8_t temp1,temp2;

	temp1 = (pSPIx->SPIx->SR) & (1<<1);
	temp2 = (pSPIx->SPIx->CR2) & (1<<7);

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pSPIx);
	}

	temp1 = (pSPIx->SPIx->SR) & (1<<0);
	temp2 = (pSPIx->SPIx->CR2) & (1<<6);

	if(temp1 && temp2)
	{
		spi_rxe_interrupt_handle(pSPIx);
	}

	temp1 = (pSPIx->SPIx->SR) & (1<<6);
	temp2 = (pSPIx->SPIx->CR2) & (1<<5);

	if(temp1 && temp2)
	{
		spi_error_interrupt_handle(pSPIx);
	}


}

void SPI2_peripheral_enable(SPIrefdef_t *SPIx){

	SPIx->CR1 |= (1<<6);

}

void SPI2_SSI_config(SPIrefdef_t *SPIx){
	SPIx->CR1 |= (1<<8);
}


void SPI_SendDataIT(spi_handle_t *pSPIx, uint8_t *tx_data ,uint32_t length ){

	uint8_t status = pSPIx->Txstate;

	if(status != SPI_BUSY_TX)
	{

		(pSPIx->Txbuffer) = tx_data;
		pSPIx->Txlength = length;
		pSPIx->Txstate  = SPI_BUSY_TX;
		pSPIx->SPIx->CR2 |= (1<<7);


	}






}

void SPI_RecieveDataIT(spi_handle_t *pSPIx, uint8_t *rx_data ,uint32_t length){

	uint8_t status = pSPIx->Rxstate;

		if(status != SPI_BUSY_RX)
		{

			rx_data = (pSPIx->Rxbuffer);
			pSPIx->Rxlength = length;
			pSPIx->Rxstate  = SPI_BUSY_RX;
			pSPIx->SPIx->CR2 |= (1<<6);


		}




}

void spi_txe_interrupt_handle(spi_handle_t* SPIx){

				if((SPIx->SPIx->CR1)&(1<<1))
				{
					SPIx->SPIx->DR = *((uint16_t*)SPIx->Txbuffer);
					SPIx->Txlength--;
					SPIx->Txlength--;
					(uint16_t*)SPIx->Txbuffer++;
				}

				else{

					SPIx->SPIx->DR = *((uint8_t*)SPIx->Txbuffer);
                    SPIx->Txlength--;
                    (uint8_t*)SPIx->Txbuffer++;

				}

				if(!SPIx->Txlength)
				{
					spi_closetransmission(SPIx);
					spi_applicationcallback();
				}



}

void spi_rxe_interrupt_handle(spi_handle_t* SPIx){

						if((SPIx->SPIx->CR1)&(1<<0))
						{

							*((uint16_t*)SPIx->Rxbuffer) = SPIx->SPIx->DR;
							SPIx->Rxlength--;
							SPIx->Rxlength--;
							(uint16_t*)SPIx->Rxbuffer++;

						}

						else{

							*((uint8_t*)SPIx->Rxbuffer) = SPIx->SPIx->DR;
							SPIx->Rxlength--;
							(uint8_t*)SPIx->Rxbuffer++;

						}

						if(!SPIx->Rxlength)
						{
							spi_closereception(SPIx);
							spi_applicationcallback();
						}

}



void spi_closetransmission(spi_handle_t* SPIx){

	SPIx->Txstate = SPI_READY_TX;
	SPIx->Txlength = 0;
	SPIx->Txbuffer = NULL;
	SPIx->SPIx->CR2 &= ~(1<<7);

}

void spi_closereception(spi_handle_t* SPIx){

	SPIx->Rxstate =  SPI_READY_RX;
	SPIx->Rxlength = 0;
	SPIx->Rxbuffer = NULL;
	SPIx->SPIx->CR2 &=- ~(1<<6);

}

 void spi_applicationcallback(){

}

