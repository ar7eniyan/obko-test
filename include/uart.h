#ifndef INCLUDE_UART_H
#define INCLUDE_UART_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"


#define UART_CLK    120e6   // usart_ker_ck
#define UART_BAUD   1e6
#define USARTDIV    UART_CLK / UART_BAUD    // if oversampling = 16.

void setup_uart(void);
void uart_send_byte(char data);
void uart_send_string(char* str);

#endif	// #ifndef INCLUDE_UART_H
