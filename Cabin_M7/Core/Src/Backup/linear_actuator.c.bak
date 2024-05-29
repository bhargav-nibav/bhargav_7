/*
 * linear_actuator.c
 *
 *  Created on: Jan 10, 2024
 *      Author: ADMIN
 */

#include "linear_actuator.h"
#include "gpio.h"
#include "debug_print.h"

typedef enum
{
    DISENGAGE_SEQUENCE,
    ENGAGE_START,
    WAIT_FOR_FULL_RELEASE,
    WAIT_FULL_RELEASE_TIMEOUT,
    WAIT_FOR_FULL_RETRACTION,
    WAIT_FULL_RETRACTION_TIMEOUT,
    LINEAR_ACTUATOR_IDLE,
}LINEAR_ACTUATOR_STAGES;

LINEAR_ACTUATOR_STAGES la_state = WAIT_FOR_FULL_RETRACTION;

typedef enum
{
    RELEASE_OFF,
    RELEASE_ON,
}RELEASE_CONTROLLER_STATE;

typedef enum
{
    RETRACT_OFF,
    RETRACT_ON,
}RETRACT_CONTROLLER_STATE;

RELEASE_CONTROLLER_STATE rel_control = RELEASE_OFF;
RETRACT_CONTROLLER_STATE ret_control = RETRACT_OFF;

void set_controller_state(RELEASE_CONTROLLER_STATE _rel, RETRACT_CONTROLLER_STATE _ret);

void setLinearActuator(bool status)
{
  static bool prev_val  = false;
  if(prev_val != status)
    {
      prev_val = status;
      la_state = status;
      debug_print("[linear_actuator] [%s]", status?"retract":"release");
    }
}


void process_la_sequence()
{
  static uint32_t la_timer = 0;
  switch(la_state)
  {
    case DISENGAGE_SEQUENCE:
      {
        la_state = WAIT_FOR_FULL_RETRACTION;
      }
      break;
    case ENGAGE_START:
      {
        la_state = ENGAGE_START;
      }
      break;
    case WAIT_FOR_FULL_RELEASE:
      {
	set_controller_state(RELEASE_ON, RETRACT_OFF);
        la_timer = HAL_GetTick();
        la_state = WAIT_FULL_RELEASE_TIMEOUT;
        debug_print("[linear_actuator] [init release]\n");
      }
      break;
    case WAIT_FULL_RELEASE_TIMEOUT:
      {
  	if(HAL_GetTick() - la_timer >= RELEASE_TIME)
  	  {
  	      set_controller_state(RELEASE_ON, RETRACT_ON);
  	      la_timer = HAL_GetTick();
  	      la_state = WAIT_FOR_FULL_RETRACTION;
  	      debug_print("[linear_actuator] [%ds released]\n", RELEASE_TIME);
  	  }
      }
      break;
    case WAIT_FOR_FULL_RETRACTION:
      {
        la_state = WAIT_FULL_RETRACTION_TIMEOUT;
        la_timer = HAL_GetTick();
        debug_print("[linear_actuator] [init retract]\n");
        set_controller_state(RELEASE_OFF, RETRACT_ON);
      }
      break;
    case WAIT_FULL_RETRACTION_TIMEOUT:
      {
  	if(HAL_GetTick() - la_timer >= RETRACT_TIME)
  	  {
  	      la_timer = HAL_GetTick();
  	      la_state = LINEAR_ACTUATOR_IDLE;
  	      debug_print("[linear_actuator] [%ds retracted]\n",RETRACT_TIME);
  	      set_controller_state(RELEASE_OFF, RETRACT_OFF);
  	  }
      }
      break;
    case LINEAR_ACTUATOR_IDLE:
      {
        //do nothing here
      }
      break;
    default:
      {
        la_state = WAIT_FOR_FULL_RETRACTION;
      }
      break;
  }
}

void set_controller_state(RELEASE_CONTROLLER_STATE _rel, RETRACT_CONTROLLER_STATE _ret)
{
  //set gpio here
}
