#ifndef INCLUDE_GPIO_H
#define INCLUDE_GPIO_H

#include <stdint.h>
#include <stm32h743xx.h>
#include <stm32h7xx.h>

#define GPIO_FLAGS_MODE_Pos 0
#define GPIO_FLAGS_MODE_Msk (0b11 << GPIO_FLAGS_MODE_Pos)
#define GPIO_FLAGS_MODE_IN     (0b00 << GPIO_FLAGS_MODE_Pos)
#define GPIO_FLAGS_MODE_OUT    (0b01 << GPIO_FLAGS_MODE_Pos)
#define GPIO_FLAGS_MODE_AF     (0b10 << GPIO_FLAGS_MODE_Pos)
#define GPIO_FLAGS_MODE_ANALOG (0b11 << GPIO_FLAGS_MODE_Pos)

#define GPIO_FLAGS_OTYPE_Pos 2
#define GPIO_FLAGS_OTYPE_Msk (0b1 << GPIO_FLAGS_OTYPE_Pos)
#define GPIO_FLAGS_OTYPE_PP (0b0 << GPIO_FLAGS_OTYPE_Pos)
#define GPIO_FLAGS_OTYPE_OD (0b1 << GPIO_FLAGS_OTYPE_Pos)

#define GPIO_FLAGS_OSPEED_Pos 3
#define GPIO_FLAGS_OSPEED_Msk (0b11 << GPIO_FLAGS_OSPEED_Pos)
#define GPIO_FLAGS_OSPEED_LOW       (0b00 << GPIO_FLAGS_OSPEED_Pos)
#define GPIO_FLAGS_OSPEED_MEDIUM    (0b01 << GPIO_FLAGS_OSPEED_Pos)
#define GPIO_FLAGS_OSPEED_HIGH      (0b10 << GPIO_FLAGS_OSPEED_Pos)
#define GPIO_FLAGS_OSPEED_VERY_HIGH (0b11 << GPIO_FLAGS_OSPEED_Pos)

#define GPIO_FLAGS_PUPD_Pos 5
#define GPIO_FLAGS_PUPD_Msk (0b11 << GPIO_FLAGS_PUPD_Pos)
#define GPIO_FLAGS_PUPD_NONE (0b00 << GPIO_FLAGS_PUPD_Pos)
#define GPIO_FLAGS_PUPD_PU   (0b01 << GPIO_FLAGS_PUPD_Pos)
#define GPIO_FLAGS_PUPD_PD   (0b10 << GPIO_FLAGS_PUPD_Pos)

#define GPIO_FLAGS_AFSEL_Pos 7
#define GPIO_FLAGS_AFSEL_Msk (0b1111 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF0  (0b0000 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF1  (0b0001 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF2  (0b0010 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF3  (0b0011 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF4  (0b0100 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF5  (0b0101 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF6  (0b0110 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF7  (0b0111 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF8  (0b1000 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF9  (0b1001 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF10 (0b1010 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF11 (0b1011 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF12 (0b1100 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF13 (0b1101 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF14 (0b1110 << GPIO_FLAGS_AFSEL_Pos)
#define GPIO_FLAGS_AFSEL_AF15 (0b1111 << GPIO_FLAGS_AFSEL_Pos)


// Enable the port in RCC, change pin's settings according to the flags.
// Does not validate any of the arguments.
void gpio_setup_pin(GPIO_TypeDef *port, uint8_t pin, uint16_t flags);

#endif  // #ifndef INCLUDE_GPIO_H
