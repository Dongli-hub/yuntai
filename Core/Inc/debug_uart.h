#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "main.h"
#include "usart.h"

void debug_uart_init(void);
void debug_print(const char *msg);
void debug_println(const char *msg);

#endif
