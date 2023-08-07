/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f401rb.h"
#include "stm32f401rb_gpio_driver.h"
#include "string.h"

void EXTI0_IRQHandler();
void delay(){
	for(int i=0; i< 5000 ;i ++);
}

int main(void)
{

	uint8_t x =0;
	gpio_handle_t led,led1;
	gpio_handle_t button;

	memset(&button,0,sizeof(gpio_handle_t));
	memset(&led,0,sizeof(gpio_handle_t));
	memset(&led1,0,sizeof(gpio_handle_t));

	led.pGPIOx = GPIOA;
	led.pconfig.GPIO_pinNumber = GPIO_PIN_NO0;
	led.pconfig.GPIO_pinMode = GPIO_MODE_IT_FT ;
	led.pconfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;
	led.pconfig.GPIO_pinPupdcontrol = GPIO_PULL_NONE;

		led1.pGPIOx = GPIOA;
		led1.pconfig.GPIO_pinNumber = GPIO_PIN_NO5;
		led1.pconfig.GPIO_pinMode = GPIO_MODE_OUTPUT ;
		led.pconfig.GPIO_pinOptype = GPIO_PUSH_PULL;
		led1.pconfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;


	GPIOA_PCLK_EN();
	GPIO_init(&led);


//	button.pGPIOx = GPIOC;
//	button.pconfig.GPIO_pinNumber = GPIO_PIN_NO13;
//	button.pconfig.GPIO_pinMode = GPIO_MODE_IT_FT ;
//	button.pconfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;
//	button.pconfig.GPIO_pinPupdcontrol = GPIO_PULL_NONE;

	GPIOC_PCLK_EN();
	GPIO_init(&button);

	GPIO_irq_config(IRQ_NO_EXTI0,15,1);

    /* Loop forever */
	for(;;){

		if(GPIO_read_pin(GPIOA,GPIO_PIN_NO0)){

			x = !x;
		//	GPIO_toggle_pin(GPIOA,GPIO_PIN_NO13);

	}
	}
}


void EXTI0_IRQHandler(){

	GPIO_toggle_pin(GPIOA,GPIO_PIN_NO13);
	GPIO_irq_handler(0);

}