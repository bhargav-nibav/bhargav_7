/*
 * contact_charger.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */


#include "contact_charger.h"
#include "gpio.h"
#include "debug_print.h"
#include "android_handler.h"


#include <stdbool.h>

void displayCCicon()
{

  static long int interV = 0;
  static bool prevBatteryState = false;
  bool chargingState = HAL_GPIO_ReadPin(contact_charger_GPIO_Port, contact_charger_Pin);
  if(HAL_GetTick()- interV >= CONTACT_CHARGER_MONITOR_TIMER)
  {

    interV = HAL_GetTick();
    if(prevBatteryState != chargingState)
    {
      prevBatteryState = chargingState;
      toggleChargeIcon(chargingState);

      if (chargingState)
      {
	  HAL_GPIO_WritePin(CONTACT_INDICATOR_GPIO_Port, CONTACT_INDICATOR_Pin, 1);
	  debug_print("Contact charger is available\n");
    	  //print_debug_message(contactChargerOn, sizeof(contactChargerOn));
      }
      else
      {
	  HAL_GPIO_WritePin(CONTACT_INDICATOR_GPIO_Port, CONTACT_INDICATOR_Pin, 0);
	  debug_print("Contact charger is not available\n");
//    	  print_debug_message(contactChargerOff, sizeof(contactChargerOff));
      }
    }
  }

}

void toggleChargeIcon(int _State)
{
  androidDataframe(CONTACT_CHARGER, (uint8_t)_State);
}
