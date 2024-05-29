/*
 * led_indication.c
 *
 *  Created on: Dec 1, 2023
 *      Author: ADMIN
 */


#include "led_indication.h"
#include "gpio.h"
#include "common.h"

void device_health_indication()
{
  static long int timer_expired = 0;
  if(HAL_GetTick() - timer_expired > 300)
    {
      timer_expired = HAL_GetTick();
      HAL_GPIO_TogglePin(DEVICE_HEALTH_GPIO_Port, DEVICE_HEALTH_Pin);
    }
}
