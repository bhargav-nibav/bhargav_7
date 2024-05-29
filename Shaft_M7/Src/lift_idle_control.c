/*
 * lift_idle_control.c
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */


#include "lift_idle_control.h"
#include "gpio.h"

typedef struct liftIdleContol
{
    uint32_t idleStartTime;
    uint32_t idleTimeValueSeconds;
    bool isLiftIdle;
}liftIdleContol_t;

static liftIdleContol_t idleControl;


void liftIdleInit(uint8_t time_sec)
{
    idleControl.idleTimeValueSeconds = time_sec;
    idleControl.isLiftIdle = false;
}

bool isLiftIdle()
{
//	debug_print_n("idleControl.isLiftIdle : %d\n",idleControl.isLiftIdle);
    return idleControl.isLiftIdle;
}

void triggerLiftBusySignal()
{
    idleControl.idleStartTime = HAL_GetTick();
    idleControl.isLiftIdle = false;
}

void checkLiftIdleProcess()
{
    if(!idleControl.isLiftIdle)
    {
        if (HAL_GetTick() - idleControl.idleStartTime > idleControl.idleTimeValueSeconds *ONE_SECOND)
        {
            idleControl.isLiftIdle = true;
            idleControl.idleStartTime = HAL_GetTick();
        }
    }
}
