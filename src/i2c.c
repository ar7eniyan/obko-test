#include 

void setup_i2c(void) {
	// SCL - PB6
	// SDA - PB7
	SET_RCC_xxxxEN(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
	SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);

	MODIFY_REG(GPIOB->MODER,														// Alternate function mode.
		GPIO_MODER_MODE6 | GPIO_MODER_MODE7,
		(0b10 << GPIO_MODER_MODE6_Pos | 0b10 << GPIO_MODER_MODE7_Pos));
	MODIFY_REG(GPIOB->OTYPER,														// Output open-drain.
		GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7,
		(1 << GPIO_OTYPER_OT6_Pos | 1 << GPIO_OTYPER_OT7_Pos));
	MODIFY_REG(GPIOB->OSPEEDR,														// High speed. @note as in datasheet.
		GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7,
		(0b10 << GPIO_OSPEEDR_OSPEEDR6_Pos | 0b10 << GPIO_OSPEEDR_OSPEEDR7_Pos));
	MODIFY_REG(GPIOB->PUPDR,														// No pull-up, pull-down.
		GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7,
		(0b00 << GPIO_PUPDR_PUPDR6_Pos | 0b00 << GPIO_PUPDR_PUPDR7_Pos));
    MODIFY_REG(GPIOB->AFR[0],														// AF4.
		GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7,
		(0b0100 << GPIO_AFRL_AFSEL6_Pos | 0b0100 << GPIO_AFRL_AFSEL6_Pos));
	
	I2C1->CR1 &= ~(I2C_CR1_GCEN | I2C_CR1_NOSTRETCH);	// General call disable; Clock stretching enabled
	I2C1->CR1 &= ~I2C_CR1_PE;							// Peripheral Disable.
	I2C1->TIMINGR = (uint32_t)0x00B03FDB;				// 400kHz, From CubeMX.
	I2C1->OA1 |= I2C_OA1_OA1EN |						// Own address 1 enable.
		(0x33) << 1;									// Interface own slave address. 7-bit
	I2C1->CR1 |= I2C_CR1_PE;							// Peripheral Enaable.
	I2C1->CR2 = I2C_CR2_AUTOEND |						// Automatic STOP.
		(1 << 16) |
		(I2C_CR2_SADD << 1);
}
