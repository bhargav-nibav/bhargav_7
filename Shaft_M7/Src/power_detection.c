/*
 * power_detection.c
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */


#include "power_detection.h"
#include "led_indication.h"
#include "debug_print.h"
#include "serial_handler.h"
#include "gpio.h"
#include <stdint.h>


bool readPowerFailure()
{
  static bool preValue = true;
//  bool curValue = !HAL_GPIO_ReadPin(power_input_GPIO_Port, power_input_Pin);
  bool curValue = !HAL_GPIO_ReadPin(current_sense_input_GPIO_Port, current_sense_input_Pin);
//  debug_print_n(" power val : %d \n",curValue);
  if(preValue != curValue)
  {
    preValue = curValue;
    write_power_led(!curValue);
    set_device_availability(POWER_DEVICE, curValue);
    if((bool)curValue)
    {
    	debug_print_n(DEBUG_POWER_UNAVAILABLE);
    }
    else
    {
    	debug_print_n(DEBUG_POWER_AVAILABLE);
    }
  }
  return curValue;
}
