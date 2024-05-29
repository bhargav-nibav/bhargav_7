/*
 * pwm_devices.c
 *
 *  Created on: Nov 25, 2023
 *      Author: ADMIN
 */

#include "pwm_devices.h"
#include "android_handler.h"
#include "tim.h"
//#include "pwd.h"
#include "gpio.h"
#include "debug_print.h"
#include "common.h"

PWM_DEVICE_STATE fan_pwm_state = PWM_STOP;
PWM_DEVICE_STATE light_pwm_state = PWM_STOP;
PWM_DEVICE_STATE logo_pwm_state = PWM_STOP;

uint8_t pwm_duration[4] = {0};

const long int FAN_TIMEOUT[]={NORMAL_PWM_DURATION,CABIN_BOOKED_DURATION};
const long int LIGHT_TIMEOUT[]={NORMAL_PWM_DURATION,CABIN_BOOKED_DURATION};
const long int LOGO_TIMEOUT[]={NORMAL_PWM_DURATION,CABIN_BOOKED_DURATION};

void init_pwm()
{
	setPWM(FAN_PWM, INIT_VALUE, 0);
	setPWM(LIGHT_PWM, INIT_VALUE, 0);
	setPWM(LOGO_PWM, INIT_VALUE, 0);
}

void fan_duration_control();
void light_duration_control();
void logo_duration_control();

void setPWM(PWM_DEVICE_TYPES device_id, int device_intensity, int duration_type)
{
	debug_print("PWM message received for %d value is %d duration type is %d \n", device_id, device_intensity, duration_type);
	pwm_duration[device_id]=duration_type;
	if(device_intensity)
	  {
	    set_output_variable(1, device_id);
	  }
	else
	  {
	    set_output_variable(0, device_id);
	  }
	device_intensity = 100-device_intensity;
	switch(device_id)
	{
		case FAN_PWM:
		{
			fan_pwm_state = PWM_START;
			TIM4->CCR2 = device_intensity;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		}
		break;
		case LIGHT_PWM:
		{
			light_pwm_state = PWM_START;
			TIM4->CCR3 = device_intensity;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		}
		break;
		case LOGO_PWM:
		{
			logo_pwm_state = PWM_START;
			TIM4->CCR4 = device_intensity;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		}
		break;
		default:
		  {

		  }
		  break;
	}
}

void monitor_pwm()
{
  fan_duration_control();
  light_duration_control();
  logo_duration_control();
}


void fan_duration_control()
{
  static long int timer_expired = 0;
  switch(fan_pwm_state)
  {
    case PWM_STOP:
      setPWM(FAN_PWM, INIT_VALUE, 0);
      fan_pwm_state = PWM_IDLE;
      break;
    case PWM_START:
      timer_expired = HAL_GetTick();
      fan_pwm_state = PWM_TIMER_COUNTDOWN;
      break;
    case PWM_TIMER_COUNTDOWN:
      {
	if(HAL_GetTick() - timer_expired > FAN_TIMEOUT[pwm_duration[FAN_PWM]])
	  {
	    timer_expired=HAL_GetTick();
	    fan_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_TIMER_EXPIRE:
      {
	if(HAL_GetTick() - timer_expired > 5*1000)
	  {
	    timer_expired=HAL_GetTick();
	    fan_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_IDLE:
      //do nothing
      break;
    default:
      fan_pwm_state = PWM_STOP;
      break;
  }
}

void light_duration_control()
{
  static long int timer_expired = 0;
  switch(light_pwm_state)
  {
    case PWM_STOP:
	setPWM(LIGHT_PWM, INIT_VALUE, 0);
	light_pwm_state  = PWM_IDLE;
      break;
    case PWM_START:
      timer_expired = HAL_GetTick();
      light_pwm_state = PWM_TIMER_COUNTDOWN;
      break;
    case PWM_TIMER_COUNTDOWN:
      {
	if(HAL_GetTick() - timer_expired > LIGHT_TIMEOUT[pwm_duration[LIGHT_PWM]])
	  {
//	    debug_print("manually stopping light after %ds\n",LIGHT_TIMEOUT[pwm_duration[LIGHT_PWM]]);
	    timer_expired=HAL_GetTick();
	    light_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_TIMER_EXPIRE:
      {
	if(HAL_GetTick() - timer_expired > 5*1000)
	  {
	    timer_expired=HAL_GetTick();
	    light_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_IDLE:
      //do nothing
      break;
    default:
      light_pwm_state = PWM_STOP;
      break;
  }
}

void logo_duration_control()
{
  static long int timer_expired = 0;
  switch(logo_pwm_state)
  {
    case PWM_STOP:
	  setPWM(LOGO_PWM, INIT_VALUE, 0);
	  logo_pwm_state= PWM_IDLE;
      break;
    case PWM_START:
      timer_expired = HAL_GetTick();
      logo_pwm_state = PWM_TIMER_COUNTDOWN;
      break;
    case PWM_TIMER_COUNTDOWN:
      {
	if(HAL_GetTick() - timer_expired > LOGO_TIMEOUT[pwm_duration[LOGO_PWM]])
	  {
	    timer_expired=HAL_GetTick();
	    logo_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_TIMER_EXPIRE:
      {
	if(HAL_GetTick() - timer_expired > 5*1000)
	  {
	    timer_expired=HAL_GetTick();
	    logo_pwm_state = PWM_STOP;
	  }
      }
      break;
    case PWM_IDLE:
      //do nothing
      break;
    default:
      logo_pwm_state = PWM_STOP;
      break;
  }
}
