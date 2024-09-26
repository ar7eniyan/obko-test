#include <stdint.h>

#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"


void setup_i2c(void) {
    // SCL - PB6
    // SDA - PB7
    SET_RCC_xxxxEN(RCC->APB1LENR, RCC_APB1LENR_I2C1EN);
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);

    // Alternate function mode.
    MODIFY_REG(GPIOB->MODER,
        GPIO_MODER_MODE6 | GPIO_MODER_MODE7,
        (0b10 << GPIO_MODER_MODE6_Pos | 0b10 << GPIO_MODER_MODE7_Pos));
    // Output open-drain.
    MODIFY_REG(GPIOB->OTYPER,
        GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7,
        (1 << GPIO_OTYPER_OT6_Pos | 1 << GPIO_OTYPER_OT7_Pos));
    // High speed. @note as in datasheet.
    MODIFY_REG(GPIOB->OSPEEDR,
        GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7,
        (0b10 << GPIO_OSPEEDR_OSPEED6_Pos | 0b10 << GPIO_OSPEEDR_OSPEED7_Pos));
    // No pull-up, pull-down.
    MODIFY_REG(GPIOB->PUPDR,
        GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7,
        (0b00 << GPIO_PUPDR_PUPD6_Pos | 0b00 << GPIO_PUPDR_PUPD7_Pos));
    // AF4
    MODIFY_REG(GPIOB->AFR[0],
        GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7,
        (0b0100 << GPIO_AFRL_AFSEL6_Pos | 0b0100 << GPIO_AFRL_AFSEL6_Pos));

    I2C1->CR1 &= ~(I2C_CR1_GCEN | I2C_CR1_NOSTRETCH);   // General call disable; Clock stretching enabled
    I2C1->CR1 &= ~I2C_CR1_PE;                           // Peripheral Disable.
    I2C1->TIMINGR = (uint32_t)0x00B03FDB;               // 400kHz, From CubeMX.
    I2C1->OAR1 |= I2C_OAR1_OA1EN |                      // Own address 1 enable.
        (0x33) << 1;                                    // Interface own slave address. 7-bit
    I2C1->CR1 |= I2C_CR1_PE;                            // Peripheral Enaable.
    I2C1->CR2 = I2C_CR2_AUTOEND |                       // Automatic STOP.
        (1 << 16) |
        (I2C_CR2_SADD << 1);
}

