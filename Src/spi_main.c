/*
 * spi_main.c
 *
 *  Created on: 03-Sep-2023
 *      Author: velmurugan
 */



#include <stdint.h>
#include <string.h>
#include "stm32f401rb.h"
#include "stm32f401rb_spi_driver.h"
#include "stm32f401rb_gpio_driver.h"

/*
 * PB12 - SPI2_SS
 * PB13	- SPI2_CLK
 * PB14 - SPI2-MISO
 * PB15	- SPI2-MOSI
 * Alt-function-mode - 6
 */


void SPI2_GPIO_init(){

	gpio_handle_t SPI_pins;

	GPIOB_PCLK_EN();

	SPI_pins.pGPIOx = GPIOB;

	SPI_pins.pconfig.GPIO_pinAltmode = 6;

	SPI_pins.pconfig.GPIO_pinOptype = GPIO_PUSH_PULL;

	SPI_pins.pconfig.GPIO_pinPupdcontrol = GPIO_PULL_NONE;

	SPI_pins.pconfig.GPIO_pinMode = GPIO_MODE_ALTERNATE;

	SPI_pins.pconfig.GPIO_pinNumber = GPIO_PIN_NO13;

	GPIO_init(&SPI_pins);

	SPI_pins.pconfig.GPIO_pinNumber = GPIO_PIN_NO12;

	GPIO_init(&SPI_pins);

	SPI_pins.pconfig.GPIO_pinNumber = GPIO_PIN_NO14;

	GPIO_init(&SPI_pins);

	SPI_pins.pconfig.GPIO_pinNumber = GPIO_PIN_NO15;

	GPIO_init(&SPI_pins);

}

void SPI2_init(){

	spi_handle_t SPI;

	SPI2_PCLK_EN();

	SPI.SPIx = SPI2;
	SPI.pconfig.DFF = SPI_DFF_8 ;
	SPI.pconfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI.pconfig.SPI_CPOL= SPI_CPOL_LOW;
	SPI.pconfig.SPI_mode = SPI_MODE_MASTER;
	SPI.pconfig.SPI_clkspeed = SPI_CLK_SPEED_2;
	SPI.pconfig.SPI_busconfig = SPI_FULL_DUPLEX;
	SPI.pconfig.SPI_SSM = SSM_ENABLED;

	SPI_init(&SPI);


}


int main(){


char user_data[] = "hellomuthu";

SPI2_GPIO_init();

SPI2_init();

SPI2_SSI_config(SPI2);

SPI2_peripheral_enable(SPI2);

SPI_SendData(SPI2,user_data,strlen(user_data));

while(1);

return 0;

}
