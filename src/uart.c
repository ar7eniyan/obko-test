#include "uart.h"


void setup_uart(void) {
    // TX - PB14
    // RX - PB15
    SET_RCC_xxxxEN(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);

    // Alternate function mode.
    MODIFY_REG(GPIOB->MODER,
        GPIO_MODER_MODE14 | GPIO_MODER_MODE15,
        (0b10 << GPIO_MODER_MODE14_Pos | 0b10 << GPIO_MODER_MODE15_Pos));
    // Output open-drain.
    MODIFY_REG(GPIOB->OTYPER,
        GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15,
        (1 << GPIO_OTYPER_OT14_Pos | 1 << GPIO_OTYPER_OT15_Pos));
    // High speed. @note as in datasheet.
    MODIFY_REG(GPIOB->OSPEEDR,
        GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15,
        (0b10 << GPIO_OSPEEDR_OSPEED14_Pos | 0b10 << GPIO_OSPEEDR_OSPEED15_Pos));
    // No pull-up, pull-down.
    MODIFY_REG(GPIOB->PUPDR,
        GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15,
        (0b00 << GPIO_PUPDR_PUPD14_Pos | 0b00 << GPIO_PUPDR_PUPD15_Pos));
    // AF7.
    MODIFY_REG(GPIOB->AFR[1],
        GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15,
        (0b0111 << GPIO_AFRH_AFSEL14_Pos | 0b0111 << GPIO_AFRH_AFSEL15_Pos));


    USART1->CR1 &= ~USART_CR1_UE;			    // USART disabled.

    // OVER8 = 0:   Oversampling by 16.
    // PCE = 0:     Parity control disabled.
    // STOP = 00:   1 stop bit.
    USART1->CR1 |= (0b0 << USART_CR1_M1_Pos) |	// 1 start bit, 8 Data bits, n Stop bit.
        (0b0 << USART_CR1_M0_Pos) |
        (1 << USART_CR1_TE) |                   // Transmitter is enabled.
        (1 << USART_CR1_RE);                    // Receiver is enabled and begins searching for a start bit.
    USART1->PRESC = 0b0000;                     // input clock not divided.
    USART1->BRR = USARTDIV;
    
    USART1->CR1 |= USART_CR1_UE;                // USART enaabled.
}

void uart_send_byte(char data) {
    USART1->TDR |= data;
}

void uart_send_string(char* str) {
    uint8_t = 0;

    while (str[i]) {
        uart_send_byte (str[i++]);
    }
    while(!(USART1->ISR & USART_ISR_TC))        // Wait Transmission Complete.
}
