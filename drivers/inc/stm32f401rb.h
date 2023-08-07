/*
 * stm32f401rb.h
 *
 * Created on: 05-Jul-2023
 * Author: Velmurugan
 */

#ifndef INC_STM32F401RB_H_
#define INC_STM32F401RB_H_

#include <stdint.h>

#define __vo volatile

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

#define GPIO_BASE_ADDRESS_PORT(x)  ((x == GPIOA)? 0 :\
								   (x == GPIOB)? 1 :\
								   (x == GPIOC)? 2 : \
								   (x == GPIOH)? 4:	0)


#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


#define FLASH_BASE         0x08000000u
#define SRAM1              0x20000000u
#define ROM                0x1FFF0000u

#define GPIOA_BASE_ADDRESS 0x40020000u
#define GPIOB_BASE_ADDRESS 0x40020400u
#define GPIOC_BASE_ADDRESS 0x40020800u
#define GPIOH_BASE_ADDRESS 0x40021C00u
#define RCC_BASE_ADDRESS   0x40023800u

// Arm peripheral bus 2
#define RESERVED_BASE_ADDRESS      0x40015400u
#define SPI5_I2S5_BASE_ADDRESS     0x40015000u
#define RESERVED2_BASE_ADDRESS     0x40014C00u
#define TIM11_BASE_ADDRESS         0x40014800u
#define RESERVED3_BASE_ADDRESS     0x40014400u
#define TIM9_BASE_ADDRESS          0x40014000u
#define EXTI_BASE_ADDRESS          0x40013C00u
#define SYSCFG_BASE_ADDRESS        0x40013800u
#define RESERVED4_BASE_ADDRESS     0x40013400u
#define SPI1_I2S1_BASE_ADDRESS     0x40013000u
#define RESERVED5_BASE_ADDRESS     0x40012400u
#define ADC1_BASE_ADDRESS          0x40012000u
#define RESERVED6_BASE_ADDRESS     0x40011800u
#define USART6_BASE_ADDRESS        0x40011400u
#define USART1_BASE_ADDRESS        0x40011000u
#define RESERVED7_BASE_ADDRESS     0x40010400u
#define TIM1_BASE_ADDRESS          0x40010000u

// Arm high-speed bus 1
#define RNG_BASE_ADDRESS           0x40080000u
#define DMA2_BASE_ADDRESS          0x40026400u
#define DMA1_BASE_ADDRESS          0x40026000u
#define FLASH_BASE_ADDRESS         0x40023C00u
#define RCC_BASE_ADDRESS           0x40023800u
#define CRC_BASE_ADDRESS           0x40023000u
#define LPTIM1_BASE_ADDRESS        0x40022400u

// Arm peripheral bus 2
#define DAC_BASE_ADDRESS           0x40007400u
#define PWR_BASE_ADDRESS           0x40007000u
#define I2C4_FMPLUS_BASE_ADDRESS   0x40006000u
#define I2C2_BASE_ADDRESS          0x40005800u
#define I2C1_BASE_ADDRESS          0x40005400u
#define USART2_BASE_ADDRESS        0x40004400u
#define SPI2_I2S2_BASE_ADDRESS     0x40003800u
#define IWDG_BASE_ADDRESS          0x40003000u
#define WWDG_BASE_ADDRESS          0x40002C00u
#define RTC_BKP_BASE_ADDRESS       0x40002800u
#define TIM6_BASE_ADDRESS          0x40001000u
#define TIM5_BASE_ADDRESS          0x40000C00u


// IRQ to EXTI numbers

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDER;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIOref_def_t;

#define GPIOA   ((GPIOref_def_t *)GPIOA_BASE_ADDRESS)
#define GPIOB   ((GPIOref_def_t *)GPIOB_BASE_ADDRESS)
#define GPIOC   ((GPIOref_def_t *)GPIOC_BASE_ADDRESS)
#define GPIOH   ((GPIOref_def_t *)GPIOH_BASE_ADDRESS)

typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t RESERVED1[3];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED2[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t RESERVED9[3];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t RESERVED4[3];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t RESERVED7[2];
    volatile uint32_t DCKCFGR;
    volatile uint32_t RESERVED8;
    volatile uint32_t DCKCFGR2;
} RCCref_def_t;


typedef struct{
 volatile uint32_t EXTI_IMR;
 volatile uint32_t EXTI_EMR;
 volatile uint32_t EXTI_RTSR;
 volatile uint32_t EXTI_FTSR;
 volatile uint32_t EXTI_SWIER;
 volatile uint32_t EXTI_PR;
}EXTIref_def_t;

typedef struct{
	volatile uint32_t SYSCFG_MEMRMP;
	volatile uint32_t SYSCFG_PMC;
	volatile uint32_t SYSCFG_EXTICR[3];
	volatile uint32_t SYSCFG_CFGR2;
	volatile uint32_t SYSCFG_CMPCR;
	volatile uint32_t SYSCFG_CFGR;
}SYSCFGref_def_t;

#define RCC     ((RCCref_def_t *)RCC_BASE_ADDRESS)
#define EXTI     ((EXTIref_def_t*)EXTI_BASE_ADDRESS)
#define SYSCFG   ((SYSCFGref_def_t*)SYSCFG_BASE_ADDRESS)

#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))

// Disable peripheral clock
#define GPIOA_PCLK_DIS()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()    (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()    (RCC->AHB1ENR &= ~(1 << 2))
#define I2C1_PCLK_DIS()     (RCC->APB1ENR &= ~(1 << 21))

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))

// Reset GPIO ports
#define GPIOA_PERI_DIS()    \
    do                      \
    {                       \
        RCC->AHB1RSTR |= (1 << 0);  \
        RCC->AHB1RSTR &= ~(1 << 0); \
    } while (0)

#define GPIOB_PERI_DIS()    \
    do                      \
    {                       \
        RCC->AHB1RSTR |= (1 << 1);  \
        RCC->AHB1RSTR &= ~(1 << 1); \
    } while (0)

#define GPIOC_PERI_DIS()    \
    do                      \
    {                       \
        RCC->AHB1RSTR |= (1 << 2);  \
        RCC->AHB1RSTR &= ~(1 << 2); \
    } while (0)

#endif
