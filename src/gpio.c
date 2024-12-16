#include "gpio.h"

#include "tools.h"

#include <stm32h7xx.h>
#include <stdint.h>

void gpio_setup_pin(GPIO_TypeDef *port, uint8_t pin, uint16_t flags)
{
    // Memory mapping of GPIO ports in STM32H742/743/753/750:
    // &GPIOx = &GPIOA + 0x400 * n, where x = A..K corresponds to n = 0..10
    uint8_t port_num = ((uintptr_t)port - (uintptr_t)GPIOA) / 0x400;
    // In RCC_AHB4ENR, bits 0..10 stand for port A..K enable
    SET_RCC_xxxxEN(RCC->AHB4ENR, 1 << port_num);

    uint8_t afsel = (flags & GPIO_FLAGS_AFSEL_Msk) >> GPIO_FLAGS_AFSEL_Pos;
    if (pin < 8) {
        MODIFY_REG(port->AFR[0], 0b1111 << (pin * 4), afsel << (pin * 4));
    } else {
        MODIFY_REG(port->AFR[1], 0b1111 << (pin * 4), afsel << (pin * 4));
    }

    uint8_t pupd = (flags & GPIO_FLAGS_PUPD_Msk) >> GPIO_FLAGS_PUPD_Pos;
    MODIFY_REG(port->PUPDR, 0b11 << (pin * 2), pupd << (pin * 2));

    uint8_t ospeed = (flags & GPIO_FLAGS_OSPEED_Msk) >> GPIO_FLAGS_OSPEED_Pos;
    MODIFY_REG(port->OSPEEDR, 0b11 << (pin * 2), ospeed << (pin * 2));

    uint8_t otype = (flags & GPIO_FLAGS_OTYPE_Msk) >> GPIO_FLAGS_OTYPE_Pos;
    MODIFY_REG(port->OTYPER, 0b1 << pin, otype << pin);

    uint8_t mode = (flags & GPIO_FLAGS_MODE_Msk) >> GPIO_FLAGS_MODE_Pos;
    MODIFY_REG(port->MODER, 0b11 << (pin * 2), mode << (pin * 2));
}

