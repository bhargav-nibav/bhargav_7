/*
 * debug_print.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */

#include "usart.h"
#include "common.h"
#include "debug_print.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void debug_print(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)buff, strlen(buff), 100);
    va_end(args);
}

void print_debug_message(const char *buf, int n)
{
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) buf, n, 100);
}
