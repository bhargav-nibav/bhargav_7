/*
 * debug_print.c
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */


#include "usart.h"

#include "debug_print.h"
#include "led_indication.h"
#include "common.h"

void debug_print_n(const char* fmt, ...) {
	write_debug_led(false);
    char buff[256];
    va_list args;
    va_start(args, fmt);
    int buff_size = vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)buff, buff_size, PRINT_TIMEOUT);
    va_end(args);
    write_debug_led(true);
}
