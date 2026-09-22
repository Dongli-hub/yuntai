#include "debug_uart.h"
#include <stdio.h>
#include <string.h>

void debug_uart_init(void) {}

void debug_print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

void debug_println(const char *msg)
{
    debug_print(msg);
    debug_print("\r\n");
}
