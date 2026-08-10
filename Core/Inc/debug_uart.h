#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "main.h"
#include "usart.h"

void debug_uart_init(void);
void debug_print(const char *msg);
void debug_println(const char *msg);
void debug_print_imu(float yaw, float pitch, float roll, float gx, float gy, float gz, float ax, float ay, float az);

#endif
