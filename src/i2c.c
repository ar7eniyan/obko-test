#include "i2c.h"


void setup_i2c(void) {
    // I2C1:
    //   SCL - PB6
    //   SDA - PB7
    // I2C3:
    //   SCL - PA8
    //   SDA - PC9
    // I2C4:
    //   SCL - PB8
    //   SDA - PB9
    SET_RCC_xxxxEN(RCC->APB1LENR, RCC_APB1LENR_I2C1EN);
    SET_RCC_xxxxEN(RCC->APB1LENR, RCC_APB1LENR_I2C3EN);
    SET_RCC_xxxxEN(RCC->APB4ENR, RCC_APB4ENR_I2C4EN);
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);

    // Alternate function mode.
    GPIOB->MODER |= (0b10 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE7_Pos) |
                    (0b10 << GPIO_MODER_MODE8_Pos) | (0b10 << GPIO_MODER_MODE9_Pos);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODE8_Pos);
    GPIOC->MODER |= (0b10 << GPIO_MODER_MODE9_Pos);
    // Output open-drain.
    GPIOB->OTYPER |= (1 << GPIO_OTYPER_OT6_Pos) | (1 << GPIO_OTYPER_OT7_Pos) |
                     (1 << GPIO_OTYPER_OT8_Pos) | (1 << GPIO_OTYPER_OT9_Pos);
    GPIOA->OTYPER |= (1 << GPIO_OTYPER_OT8_Pos);
    GPIOC->OTYPER |= (1 << GPIO_OTYPER_OT9_Pos);
    // High speed. @note as in datasheet.
    GPIOB->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED6_Pos) | (0b10 << GPIO_OSPEEDR_OSPEED7_Pos) |
                      (0b10 << GPIO_OSPEEDR_OSPEED8_Pos) | (0b10 << GPIO_OSPEEDR_OSPEED9_Pos);
    GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED8_Pos);
    GPIOC->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED9_Pos);
    // No pull-up, pull-down.
    GPIOB->PUPDR &= ~(0b00 << GPIO_PUPDR_PUPD6_Pos | 0b00 << GPIO_PUPDR_PUPD7_Pos |
                      0b00 << GPIO_PUPDR_PUPD8_Pos | 0b00 << GPIO_PUPDR_PUPD9_Pos);
    GPIOA->PUPDR &= ~(0b00 << GPIO_PUPDR_PUPD8_Pos);
    GPIOC->PUPDR &= ~(0b00 << GPIO_PUPDR_PUPD9_Pos);
    // AF4.
    GPIOB->AFR[0] |= (0b0100 << GPIO_AFRL_AFSEL6_Pos) | (0b0100 << GPIO_AFRL_AFSEL7_Pos);
    GPIOB->AFR[1] |= (0b0100 << GPIO_AFRH_AFSEL8_Pos) | (0b0100 << GPIO_AFRH_AFSEL9_Pos);
    // AF6.
    GPIOB->AFR[1] |= (0b0110 << GPIO_AFRH_AFSEL8_Pos) | (0b0110 << GPIO_AFRH_AFSEL9_Pos);

    // I2C1
    I2C1->CR1 &= ~I2C_CR1_PE;                   // Peripheral Disable.

    I2C1->CR1 &= ~(I2C_CR1_ANFOFF |             // Analog noise filter disabled.
        (0b0000 << I2C_CR1_DNF_Pos));           // Digital filter disabled.
    I2C1->TIMINGR = I2C_TIMINGR;                // 400kHz, From CubeMX.
    I2C1->CR1 |= I2C_CR1_NOSTRETCH;             // Clock stretching disabled.
    I2C1->CR2 |= I2C_CR2_AUTOEND;               // Automatic end mode (auto STOP).

    I2C1->OAR1 |= I2C_OAR1_OA1EN;               // Own address 1 enabled. The received slave address OA1 is ACKed.
    I2C1->OAR1 &= I2C_OAR1_OA1MODE;             // Own address 1 is a 7-bit address.
    I2C1->OAR1 |= 0x00 << I2C_OAR1_OA1_Pos;     // Own slave address is 0x00.

    I2C1->CR1 |= I2C_CR1_PE;                    // Peripheral Enable.

    // I2C3
    I2C3->CR1 &= ~I2C_CR1_PE;                   // Peripheral Disable.

    I2C3->CR1 &= ~(I2C_CR1_ANFOFF |             // Analog noise filter disabled.
        (0b0000 << I2C_CR1_DNF_Pos));           // Digital filter disabled.
    I2C3->TIMINGR = I2C_TIMINGR;                // 400kHz, From CubeMX.
    I2C3->CR1 |= I2C_CR1_NOSTRETCH;             // Clock stretching disabled.
    I2C3->CR2 |= I2C_CR2_AUTOEND;               // Automatic end mode (auto STOP).

    I2C3->OAR1 |= I2C_OAR1_OA1EN;               // Own address 1 enabled. The received slave address OA1 is ACKed.
    I2C3->OAR1 &= I2C_OAR1_OA1MODE;             // Own address 1 is a 7-bit address.
    I2C3->OAR1 |= 0x00 << I2C_OAR1_OA1_Pos;     // Own slave address is 0x00.

    I2C3->CR1 |= I2C_CR1_PE;                    // Peripheral Enable.

    // I2C4
    I2C4->CR1 &= ~I2C_CR1_PE;                   // Peripheral Disable.

    I2C4->CR1 &= ~(I2C_CR1_ANFOFF |             // Analog noise filter disabled.
        (0b0000 << I2C_CR1_DNF_Pos));           // Digital filter disabled.
    I2C4->TIMINGR = I2C_TIMINGR;                // 400kHz, From CubeMX.
    I2C4->CR1 |= I2C_CR1_NOSTRETCH;             // Clock stretching disabled.
    I2C4->CR2 |= I2C_CR2_AUTOEND;               // Automatic end mode (auto STOP).

    I2C4->OAR1 |= I2C_OAR1_OA1EN;               // Own address 1 enabled. The received slave address OA1 is ACKed.
    I2C4->OAR1 &= I2C_OAR1_OA1MODE;             // Own address 1 is a 7-bit address.
    I2C4->OAR1 |= 0x00 << I2C_OAR1_OA1_Pos;     // Own slave address is 0x00.

    I2C4->CR1 |= I2C_CR1_PE;                    // Peripheral Enable.
}

void i2c_master_transmit(I2C_TypeDef *i2c_no, uint8_t addr, const char * data, uint8_t len) {
    uint8_t i = 0;

    for (i = 0; i < len; i++) {
    i2c_no->TXDR = data[i];
    i2c_no->CR2 |= (len << I2C_CR2_NBYTES_Pos |
        (addr << 1) << I2C_CR2_SADD_Pos |       // Shifted addr << 1 to WRITE.
        I2C_CR2_RD_WRN |                        // Master requests a read transfer.
        I2C_CR2_AUTOEND |                       // Automatic end mode (auto STOP).
        I2C_CR2_START);
    // while (!I2C1->ISR | I2C_ISR_TXE);
    }
}


void i2c_encoder_setup(void) {
    
}

