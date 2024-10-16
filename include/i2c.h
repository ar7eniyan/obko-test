#ifndef INCLUDE_I2C_H
#define INCLUDE_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"
#include "config.h"

// #define I2C_RISE_TIME   300
// #define I2C_FALL_TIME   300
// #define F_I2CCLK        9e6             // 9e6 - DNF=1, 8e6 - DNF=0
// #define T_I2CCLK        1/F_I2CCLK
#define I2C_TIMINGR     0x00B045E1      // 400kHz, From CubeMX.


void setup_i2c(void);
void i2c_master_transmit(I2C_TypeDef *i2c_no, uint8_t addr, const char * data, uint8_t len);

void i2c_encoder_setup(void);           // Absolute Encoder AS5600 Initialization.

#endif  // #ifndef INCLUDE_I2C_H
