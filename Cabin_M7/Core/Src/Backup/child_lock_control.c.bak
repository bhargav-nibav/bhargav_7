/*
 * child_lock_control.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */
#include "gpio.h"

#include "child_lock_control.h"
#include "debug_print.h"
#include "android_handler.h"


#include <stdbool.h>
#include <stdint.h>

void checkChildLock();
void initChildLock();
void disableChildLock();
void enableChildLock();

void sendCLStatus(bool clState);

/// @brief Continously monitor the child lock button status
/// @details If the button is pressed or released, send the relevant message to display as well as shaft
/// @author Balaji M
void checkChildLock()
{
  //todo fix this delay
  static long int clTimer = 0;
  static bool prevState = false;
  if(HAL_GetTick() - clTimer > CHILD_LOCK_MONITOR_TIMER)
  {
    bool childLockStatus = HAL_GPIO_ReadPin(childLock_GPIO_Port, childLock_Pin);
    if(prevState!=childLockStatus)
    {
    	androidDataframe(CHILDLOCK_STATUS, childLockStatus);
	prevState=childLockStatus;
	if (!childLockStatus)
	{
	    debug_print("Child lock off \n");
	}
	else
	{
	    debug_print("Child lock on \n");
	}
    }
    clTimer=HAL_GetTick();
  }
}


void sendCLStatus(bool clState)
{
  if(clState)
  {
    enableChildLock();
  }
  else
  {
    disableChildLock();
  }
}


/// @brief Send child lock disable message to android display
/// @details 0x0B disable Child Lock
void disableChildLock()
{
	androidDataframe(CHILDLOCK_STATUS, CHILD_LOCK_DISABLED);
}

/// @brief Send child lock enable message to android display
/// @details 0x0C disable Child Lock
void enableChildLock()
{
	androidDataframe(CHILDLOCK_STATUS, CHILD_LOCK_ENABLED);
}
