/*
 * stm32f401rb_gpio_driver.h
 *
 * Created on: 24-Jul-2023
 * Author: velmurugan
 */

#ifndef INC_STM32F401RB_GPIO_DRIVER_H_
#define INC_STM32F401RB_GPIO_DRIVER_H_

#include "stm32f401rb.h"

// GPIO pin numbers
#define GPIO_PIN_NO0   0
#define GPIO_PIN_NO1   1
#define GPIO_PIN_NO2   2
#define GPIO_PIN_NO3   3
#define GPIO_PIN_NO4   4
#define GPIO_PIN_NO5   5
#define GPIO_PIN_NO6   6
#define GPIO_PIN_NO7   7
#define GPIO_PIN_NO8   8
#define GPIO_PIN_NO9   9
#define GPIO_PIN_NO10 10
#define GPIO_PIN_NO11 11
#define GPIO_PIN_NO12 12
#define GPIO_PIN_NO13 13
#define GPIO_PIN_NO14 14
#define GPIO_PIN_NO15 15

// GPIO Modes
#define GPIO_MODE_INPUT     0
#define GPIO_MODE_OUTPUT    1
#define GPIO_MODE_ALTERNATE 2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

// GPIO pull-up/pull-down
#define GPIO_PULL_NONE   0
#define GPIO_PULL_UP     1
#define GPIO_PULL_DOWN   2
#define GPIO_PULL_RESERVED 3

// GPIO speeds
#define GPIO_SPEED_LOW        0
#define GPIO_SPEED_MEDIUM     1
#define GPIO_SPEED_HIGH       2
#define GPIO_SPEED_VERY_HIGH  3

// GPIO output type
#define GPIO_OPEN_DRAIN 0
#define GPIO_PUSH_PULL  1

typedef struct {
    uint8_t GPIO_pinNumber;
    uint8_t GPIO_pinMode;
    uint8_t GPIO_pinSpeed;
    uint8_t GPIO_pinPupdcontrol;
    uint8_t GPIO_pinOptype;
    uint8_t GPIO_pinAltmode;
} gpio_config_t;

typedef struct {
    GPIOref_def_t *pGPIOx;
    gpio_config_t pconfig;
} gpio_handle_t;

// Function prototypes

// Clock enable and disable
void GPIO_clock_peri_enable(GPIOref_def_t *pGPIOx, uint8_t EnorDis);

// GPIO initializations
void GPIO_init(gpio_handle_t *pGPIO);
void GPIO_deinit();

// Read and write operations on ports
uint8_t GPIO_read_pin(GPIOref_def_t *pGPIOx, uint16_t pin);
void GPIO_write_pin(GPIOref_def_t *pGPIOx, uint16_t pin, uint8_t value);
void GPIO_write_port(GPIOref_def_t *pGPIOx, uint16_t value);
uint16_t GPIO_read_port(GPIOref_def_t *pGPIOx);
void GPIO_toggle_pin(GPIOref_def_t *pGPIOx, uint16_t pin);

// GPIO IRQ handlers
void GPIO_irq_config(uint8_t pin_number, uint8_t priority, uint8_t EnorDis);
void GPIO_irq_handler(uint8_t pin_number);

#endif /* INC_STM32F401RB_GPIO_DRIVER_H_ */
