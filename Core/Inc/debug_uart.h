#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "main.h"
#include "usart.h"

void debug_uart10_init(void);
uint8_t debug_uart10_send(const char *msg);

#endif
