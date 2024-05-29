/*
 * emergency_input.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */


/// @brief read the input from emergency button. On button press, send emergency on and release send emergency release.

#include "gpio.h"
#include <stdbool.h>



#include "debug_print.h"
#include "android_handler.h"

void Emergency_Read()
{
	static bool prevEmergencyState = false;
	  bool state = HAL_GPIO_ReadPin(emergency_GPIO_Port, emergency_Pin);
	  if (prevEmergencyState != state)
	  {
		  prevEmergencyState = state;
		  debug_print("Emergency : %d\n", state);
		  androidDataframe(EMERGENCY_MESSAGE, state);
	  }
}
