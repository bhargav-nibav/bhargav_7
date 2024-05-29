/*
 * android_handler.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */

#include "android_handler.h"
#include "usart.h"
#include "common.h"
#include "pwm_devices.h"
#include "landing_lever.h"
#include "debug_print.h"

uint8_t cabin_to_tab[]={0xD5,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF};
long int timer_expired = 0;

uint8_t device_current_time[] = {0,0,0,0};

void resetTimer();

void clearAllOutput()
{
  CLEAR_REG(cabin_to_tab[DEVICE_DETAILS]);
}

void setDevice(int deviceID)
{
  SET_BIT_n(cabin_to_tab[DEVICE_DETAILS], deviceID);
  debug_print("Set value is %d\n", cabin_to_tab[DEVICE_DETAILS]);
  resetTimer();
}

void clearDevice(int deviceID)
{
  CLR_BIT(cabin_to_tab[DEVICE_DETAILS], deviceID);
  debug_print("clear value is %d\n", cabin_to_tab[DEVICE_DETAILS]);
//  resetTimer();
}

void androidDataframe(MESSAGE_TYPE msg_type, uint8_t data)
{
   cabin_to_tab[msg_type] = data;
}


void resetTimer()
{
  timer_expired = timer_expired - CABIN_MESSAGE_INTERVAL;
}

void transmit_cabin_msg()
{
  if(HAL_GetTick() - timer_expired > CABIN_MESSAGE_INTERVAL)
    {
      timer_expired = HAL_GetTick();
      HAL_UART_Transmit(&WIFI_UART, cabin_to_tab, LENGTH_OF_ARRAY(cabin_to_tab), 100);
    }

}


void set_output_variable(bool type, int device_id)
{
  device_current_time[device_id]= HAL_GetTick();
  if(type)
    {
      setDevice(device_id);
    }
  else
    {
      clearDevice(device_id);
    }
}
