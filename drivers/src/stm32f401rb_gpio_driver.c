/*
 * stm32f401rb_gpio_driver.c
 *
 *  Created on: 24-Jul-2023
 *      Author: velmurugan
 */

#include "stm32f401rb_gpio_driver.h"

// Clock enable and disable
void GPIO_clock_peri_enable(GPIOref_def_t *pGPIOx, uint8_t EnorDis)
{
    if (EnorDis)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
    }
}

// GPIO initialization
void GPIO_init(gpio_handle_t *pGPIO)
{
    uint32_t port_code;
    uint8_t temp1;
    uint8_t temp2;
    uint8_t temp = 0;

    if (pGPIO->pconfig.GPIO_pinMode <= GPIO_MODE_ALTERNATE)
    {
        temp = (pGPIO->pconfig.GPIO_pinMode << (2 * pGPIO->pconfig.GPIO_pinNumber));
        pGPIO->pGPIOx->MODER &= ~(3 << (2 * pGPIO->pconfig.GPIO_pinNumber));
        pGPIO->pGPIOx->MODER |= temp;
    }
    else
    {
        if (pGPIO->pconfig.GPIO_pinMode == GPIO_MODE_IT_FT)
        {
            EXTI->EXTI_RTSR &= ~(1 << pGPIO->pconfig.GPIO_pinNumber);
            EXTI->EXTI_FTSR |= (1 << pGPIO->pconfig.GPIO_pinNumber);
        }
        else if (pGPIO->pconfig.GPIO_pinMode == GPIO_MODE_IT_RT)
        {
            EXTI->EXTI_FTSR &= ~(1 << pGPIO->pconfig.GPIO_pinNumber);
            EXTI->EXTI_RTSR |= (1 << pGPIO->pconfig.GPIO_pinNumber);
        }
        else
        {
            EXTI->EXTI_FTSR |= (1 << pGPIO->pconfig.GPIO_pinNumber);
            EXTI->EXTI_RTSR |= (1 << pGPIO->pconfig.GPIO_pinNumber);
        }


        temp1 = pGPIO->pconfig.GPIO_pinNumber % 4;
        temp2 = pGPIO->pconfig.GPIO_pinNumber / 4;
        SYSCFG_PCLK_EN();
        port_code = GPIO_BASE_ADDRESS_PORT(pGPIO->pGPIOx);
        SYSCFG->SYSCFG_EXTICR[temp2] = port_code << (temp1 * 4);
        EXTI->EXTI_IMR |= 1 << pGPIO->pconfig.GPIO_pinNumber;
    }

    temp = 0;
    temp = (pGPIO->pconfig.GPIO_pinSpeed << (2 * pGPIO->pconfig.GPIO_pinNumber));
    pGPIO->pGPIOx->OSPEEDER &= ~(3 << (2 * pGPIO->pconfig.GPIO_pinNumber));
    pGPIO->pGPIOx->OSPEEDER |= temp;

    temp = 0;
    temp = (pGPIO->pconfig.GPIO_pinPupdcontrol << (2 * pGPIO->pconfig.GPIO_pinNumber));
    pGPIO->pGPIOx->PUPDR &= ~(3 << (2 * pGPIO->pconfig.GPIO_pinNumber));
    pGPIO->pGPIOx->PUPDR |= temp;

    temp = 0;
    temp = (pGPIO->pconfig.GPIO_pinOptype << (pGPIO->pconfig.GPIO_pinNumber));
    pGPIO->pGPIOx->OTYPER &= ~(1 << pGPIO->pconfig.GPIO_pinNumber);
    pGPIO->pGPIOx->OTYPER |= temp;

    if (pGPIO->pconfig.GPIO_pinAltmode == GPIO_MODE_ALTERNATE)
    {
        uint8_t temp1 = pGPIO->pconfig.GPIO_pinNumber / 8;
        uint8_t temp2 = pGPIO->pconfig.GPIO_pinNumber % 8;

        pGPIO->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIO->pGPIOx->AFR[temp1] |= (pGPIO->pconfig.GPIO_pinAltmode << (4 * temp2));
    }
}

void GPIO_deinit(gpio_handle_t *pGPIO)
{
    if (pGPIO->pGPIOx == GPIOA)
    {
        GPIOA_PERI_DIS();
    }
    else if (pGPIO->pGPIOx == GPIOB)
    {
        GPIOB_PERI_DIS();
    }
    else if (pGPIO->pGPIOx == GPIOC)
    {
        GPIOC_PERI_DIS();
    }
}

// Read and write operations on pins and ports
uint8_t GPIO_read_pin(GPIOref_def_t *pGPIOx, uint16_t pin)
{
    uint8_t temp;
    temp = (uint8_t)((pGPIOx->IDR >> pin) & 0x00000001);
    return temp;
}

void GPIO_write_pin(GPIOref_def_t *pGPIOx, uint16_t pin, uint8_t value)
{
    if (value)
        pGPIOx->ODR |= (1 << pin);
    else
        pGPIOx->ODR &= ~(1 << pin);
}

void GPIO_write_port(GPIOref_def_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
}

uint16_t GPIO_read_port(GPIOref_def_t *pGPIOx)
{
    uint16_t temp;
    temp = (uint16_t)(pGPIOx->IDR);
    return temp;
}

void GPIO_toggle_pin(GPIOref_def_t *pGPIOx, uint16_t pin)
{
    pGPIOx->ODR ^= (1 << pin);
}

// GPIO IRQ handlers

void GPIO_irq_config(uint8_t irq_pin_number, uint8_t priority, uint8_t EnorDis)
{
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

void GPIO_irq_handler(uint8_t pin_number)
{
	if(EXTI->EXTI_PR & (1<<pin_number)){

		EXTI->EXTI_PR = 1<<pin_number;

	}

}

    //
