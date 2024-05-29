/*
 * overload_detection.c
 *
 *  Created on: Oct 18, 2023
 *      Author: ADMIN
 */


#include "overload_detection.h"
#include "debug_print.h"
#include "led_indication.h"
#include "serial_handler.h"

#include "gpio.h"

#define OVERLOAD_CHECK_INTERVAL	 100
#define OVERLOAD_ACTIVE			0
#define OVERLOAD_INACTIVE		1

bool monitor_overload()
{
	static bool ov_state = OVERLOAD_INACTIVE;
	bool current_state = (bool)!HAL_GPIO_ReadPin(overload_input_GPIO_Port, overload_input_Pin);
	if(ov_state !=current_state )
	{
		ov_state=current_state;
		write_overload_led(current_state);
		if(ov_state == (bool)SET)//(GPIO_PinState)OVERLOAD_ACTIVE)
		{
			debug_print_n(DEBUG_OVERLOAD_ACTIVE);
		}
		else
		{
			debug_print_n(DEBUG_OVERLOAD_INACTIVE);
		}
		set_device_availability(OVERLOAD_DEVICE, current_state);
	}
	return ov_state;
}
