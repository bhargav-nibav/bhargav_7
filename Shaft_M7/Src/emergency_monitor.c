/*
 * emergency_monitor.c
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */


#include "emergency_monitor.h"
#include "led_indication.h"
#include "serial_handler.h"

#include "gpio.h"
#include "debug_print.h"

bool readEmergencyShaftInput()
{

	//	return false;
		static bool prevstate = true;
		bool curState = (bool)HAL_GPIO_ReadPin(emergency_input_GPIO_Port, emergency_input_Pin);//readEmergencyShaftInput;
		if(prevstate!=curState)
		{
			debug_print_n("Emergency button is now %d\n", curState);
			prevstate=curState;
			write_emergency_led(curState);
			set_device_availability(EMERGENCY_DEVICE, curState);
		}
		 return curState;//(readEmergencyCabinInput() || readEmergencyShaftInput());

//	static boo
//	return HAL_GPIO_ReadPin(emergency_input_GPIO_Port, emergency_input_Pin);
}
