/*
 * call_reg.c
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */


#include "call_reg.h"
#include "serial_handler.h"

#include <stdio.h>
typedef struct callReg
{
  volatile uint32_t cabinCallRequest;     /* from cabin*/
  volatile uint32_t cabinCallsRegistered; /* to cabin*/
  uint8_t registeredCalls[MAX_FLOOR_CONST];
  uint16_t ledOnTimer[MAX_FLOOR_CONST];
  bool allowMultipleCallsRegistration;
  uint32_t callsRegistered;
  bool blockCallRegistration;
} CALL_REG;

CALL_REG callRegistration;
bool safetyFailed;
bool getCallSafety();


uint16_t callRegDebug = 0;

void initCallRegistration()
{
  memset(&callRegistration, false, sizeof(callRegistration));
}

void registerHALcall(FLOOR_NUM flrNum)
{
  debug_print_n("register hall call for %d\n", flrNum);
  SET_BIT_n(callRegistration.registeredCalls[flrNum], CALL_REGISTERED);
  SET_BIT_n(callRegistration.registeredCalls[flrNum], CALL_FROM_HAL);
  SET_BIT_n(callRegistration.callsRegistered, flrNum);
}

void cancelHALcall(FLOOR_NUM flrNum)
{
  //  CANCALL_DEBUG("HAL call cncl =%d \r\n",(uint8_t)flrNum);
  //  CLR_BIT(callRegistration.registeredCalls[flrNum], CALL_REGISTERED);
  CLR_BIT(callRegistration.registeredCalls[flrNum], CALL_FROM_HAL);
  CLR_BIT(callRegistration.callsRegistered, flrNum);
  turnHALcallLEDoffInTimer(flrNum);
}

bool checkHALcallregistered(FLOOR_NUM flrNum)
{
  return (bool)CHECK_BIT(callRegistration.registeredCalls[flrNum], CALL_FROM_HAL);
}

void checkCabinCalls()
{
//	debug_print_n("checkCabinCalls\n");
  if (callRegistration.cabinCallRequest)
  {
  	debug_print_n("New calls available\n");
    if (callRegistration.blockCallRegistration == true || (callRegistration.allowMultipleCallsRegistration == false && callRegistration.callsRegistered))
    {
      debug_print_n("1: cab call %d ...\r\n", callRegistration.blockCallRegistration);
      callRegistration.cabinCallRequest = false;
      return;
    }
    // setCallReg();
    // callReqSend=true;
//      if (!getCallSafety())
//      {
//        //send confirmation here
//    	  setCallBookedMessage(MAX_FLOOR);
//    	  clearCallBookedMessage(MAX_FLOOR);
//    	  callReqSend = false;
//      }

    callRegistration.cabinCallsRegistered |= callRegistration.cabinCallRequest;
    callRegistration.cabinCallRequest = false;
    for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++)
    {
      if (CHECK_BIT(callRegistration.cabinCallsRegistered, floorNum))
      {
      	debug_print_n("cabin call registed for %d\n", floorNum);
        SET_BIT_n(callRegistration.registeredCalls[floorNum], CALL_REGISTERED);
        SET_BIT_n(callRegistration.registeredCalls[floorNum], CALL_CABIN);
        SET_BIT_n(callRegistration.callsRegistered, floorNum);
      }
    }
  }
}
//void PB_LEDtimerCallback();
//void PB_LEDtimerCallback()
//{
//  static uint32_t prevTime;
//
//  if (tickDiff(prevTime, HAL_GetTick()) > ONE_SECOND)
//  {
//    prevTime = HAL_GetTick();
//    for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++)
//    {
//      if (callRegistration.ledOnTimer[floorNum])
//      {
//        callRegistration.ledOnTimer[floorNum]--;
//        if (false == callRegistration.ledOnTimer[floorNum])
//        {
//          //clear message LOP
//        }
//      }
//    }
//  }
//}

void turnHALcallLEDoffInTimer(FLOOR_NUM floorNum)
{
  if (MAX_FLOOR != floorNum)
  {
    callRegistration.ledOnTimer[floorNum] = PB_LED_OFF_TIME;
  }
}

bool checkCabinCallRegisterd(FLOOR_NUM floorNum)
{
  return CHECK_BIT(callRegistration.registeredCalls[floorNum], CALL_CABIN);
}

bool checkCallRegisterd(FLOOR_NUM floorNum, uint8_t checkbit)
{
  return CHECK_BIT(callRegistration.registeredCalls[floorNum], checkbit);
}

void clearRegisteredCalls(FLOOR_NUM floorNum)
{
//	debug_print_n("Call is cleared --- \n");
	callRegistration.registeredCalls[floorNum] = false;
  /**
   * clear current call.
   */
  turnHALcallLEDoffInTimer(floorNum);
  CLR_BIT(callRegistration.cabinCallsRegistered, floorNum);
  CLR_BIT(callRegistration.callsRegistered, floorNum);
}

void registerCabinCall(uint32_t calls)
{
	debug_print_n( "cabin set call %d\n", (int)calls);
	callRegistration.cabinCallRequest = calls;
}

void cancelAllcalls()
{
	debug_print_n( "all call clear\n");
  callRegistration.cabinCallRequest = false;
  callRegistration.cabinCallsRegistered = false;
  callRegistration.callsRegistered = false;
  for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++)
  {
    clearRegisteredCalls((FLOOR_NUM)floorNum);
  }
}

bool checkBusyCondition()
{
  for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++)
  {
    // Serial.println(callRegistration.registeredCalls[floorNum]);
    if (callRegistration.registeredCalls[floorNum] != false)
    {
      return true;
    }
  }
  return false;
}

int checkBookedCondition()
{
  if (getCallSafety())
  {
    return false;
  }
  else
  {
    for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++)
    {
      // Serial.println(callRegistration.registeredCalls[floorNum]);
      if (callRegistration.registeredCalls[floorNum] != false)
      {
        return floorNum + 1;
      }
    }
    return false;
  }
}

void configMultipleCallsRegistration(bool state)
{
  callRegistration.allowMultipleCallsRegistration = state;
}

void ConfigDisableCallRegistration(bool state)
{

  static bool prevValue = true;
  if (prevValue != state)
  {
    prevValue = state;
  }
  callRegistration.blockCallRegistration = state;
}

bool getCallRegistrationDisabledStatus(void)
{
  return callRegistration.blockCallRegistration;
}

bool getCallSafety()
{
  return safetyFailed;
}

void setCallSafety(bool status)
{
  safetyFailed = status;
}


void checkFirstChange()
{
//	debug_print_n("checkFirstChange \n");
	static int prevVal = -1;
	if(prevVal != callRegistration.registeredCalls[1])
	{
		prevVal = callRegistration.registeredCalls[1];
		debug_print_n( "value is %d \n", prevVal);
	}
}
