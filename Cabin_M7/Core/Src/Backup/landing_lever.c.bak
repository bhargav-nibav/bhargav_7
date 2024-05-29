/*
 * landing_lever.c
 *
 *  Created on: Oct 31, 2023
 *      Author: ADMIN
 */


#include "landing_lever.h"
#include "debug_print.h"
#include "android_handler.h"

#define LL_device 0

bool landingLeverStatus = false;
long int llMaxTimer = 0;

void set_landing_lever(bool state)
{
	static bool prev_state = false;
	if(prev_state!=state)
	{
		prev_state=state;
		landingLeverStatus=state;
		llMaxTimer = HAL_GetTick();
		HAL_GPIO_WritePin(LL_PIN_GPIO_Port, LL_PIN_Pin, state);
		HAL_GPIO_WritePin(LL_INDICATION_GPIO_Port, LL_INDICATION_Pin, state);
		set_output_variable(state, LL_device);
		debug_print("Landing lever set %d\n", (int)state);
	}
}

void monitor_ll()
{
  if (landingLeverStatus)
  {
    if (HAL_GetTick() - llMaxTimer >= LANDING_LEVER_TIMER)
    {
      set_landing_lever(false);
      llMaxTimer = HAL_GetTick();
      debug_print("Manual LL Off\n");
    }
  }
}
