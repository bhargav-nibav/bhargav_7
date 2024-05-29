/*
 * debug_print.h
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */

#include "common.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define ENABLE_PRINT	1
#define PRINT_TIMEOUT 	100

#define DEBUG_OVERLOAD_ACTIVE	 	"overload_triggered\n"
#define DEBUG_OVERLOAD_INACTIVE	 	"overload_released\n"
#define DEBUG_POWER_AVAILABLE		"power_available\n"
#define DEBUG_POWER_UNAVAILABLE		"power_unavailable\n"
#define DEBUG_EMERGENCY_ON          "emergency pressed\n"
#define DEBUG_EMERGENCY_OFF          "emergency released\n"

#define SIREN_PRESSED               "Siren released\n"
#define SIREN_RELEASED              "Siren Pressed\n"
#include "usart.h"

void debug_print_n(const char* fmt, ...);
