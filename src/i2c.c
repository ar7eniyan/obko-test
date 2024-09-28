#include "i2c.h"


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
    // AF4.
    MODIFY_REG(GPIOB->AFR[0],
        GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7,
        (0b0100 << GPIO_AFRL_AFSEL6_Pos | 0b0100 << GPIO_AFRL_AFSEL6_Pos));

    I2C1->CR1 &= ~I2C_CR1_PE;                   // Peripheral Disable.
    
    I2C1->CR1 &= ~(I2C_CR1_ANFOFF |             // Analog noise filter disabled.
        (0b0000 << I2C_CR1_DNF_Pos));           // Digital filter disabled.
    I2C1->TIMINGR = I2C_TIMINGR;                // 400kHz, From CubeMX.
    I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;            // Clock stretching enabled.
    I2C1->CR2 |= I2C_CR2_AUTOEND;               // Automatic end mode (auto STOP).
        
    I2C1->CR1 |= I2C_CR1_PE;                    // Peripheral Enable.
}

void i2c_master_transmit(uint8_t addr, const char *data, uint32_t num, bool xfer_pending) {
    // 1. Master generates START condition
    // 2. Master addresses the Slave as Master Transmitter
    // 3. Master transmits data to the addressed Slave
    // 4. Master generates STOP condition (if xfer_pending is "false")

    I2C1->CR2 |= (num << I2C_CR2_NBYTES_Pos |
        ((addr << 1) << I2C_CR2_SADD_Pos) |     // Shifted addr << 1 to WRITE.
        (I2C_CR2_RD_WRN));                      // Master requests a read transfer.
    I2C1->CR2 |= I2C_CR2_START;
    // while (!I2C1->ISR | I2C_ISR_TXE);
    I2C1->TXDR = data;
}
