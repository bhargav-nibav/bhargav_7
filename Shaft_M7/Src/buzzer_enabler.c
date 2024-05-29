/*
 * buzzer_enabler.c
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */
#include "gpio.h"

#include "buzzer_enabler.h"
#include "serial_handler.h"
#include "debug_print.h"

#include "common.h"

TIMEOUT_CALCULATION siren_timeout = DEVICE_SET_TIMEOUT;

void EmergencyOutput_On()
{
	siren_timeout = DEVICE_START_TIMER;
}

void EmergencyOutput_Off()
{
	siren_timeout = DEVICE_SET_TIMEOUT;
}


void monitor_alarm()
{
	static bool prev_alarm_state = false;
	static long int siren_timeout_timer  = 0;

	if(prev_alarm_state != siren_input())
	{
		prev_alarm_state = siren_input();
		if(prev_alarm_state)
		{
			siren_timeout = DEVICE_START_TIMER;
		}
		else
		{
			siren_timeout = DEVICE_SET_TIMEOUT;
		}
	}

	switch(siren_timeout)
	{
		case DEVICE_START_TIMER:
			{
				siren_timeout_timer = HAL_GetTick();
				siren_timeout = DEVICE_WAIT_UNTIL_COUNTDOWN;
				HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, (GPIO_PinState)SET);
				debug_print_n(SIREN_PRESSED);
			}
			break;
		case DEVICE_WAIT_UNTIL_COUNTDOWN:
			if(HAL_GetTick() - siren_timeout_timer > 10000)
			{
				siren_timeout_timer = HAL_GetTick();
				siren_timeout = DEVICE_SET_TIMEOUT;
			}
			break;
		case DEVICE_SET_TIMEOUT:
			{
				HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, (GPIO_PinState)RESET);
				debug_print_n(SIREN_RELEASED);
				siren_timeout = DEVICE_IDLE;
			}
			break;
		case DEVICE_IDLE:
			{
				//do nothing
			}
			break;
		default:
			{

			}
		break;
	}
}
