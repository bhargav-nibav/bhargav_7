/*
 * lift_idle_control.h
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */

#ifndef INC_LIFT_IDLE_CONTROL_H_
#define INC_LIFT_IDLE_CONTROL_H_

#include "stdint.h"
#include "common.h"
#include <stdbool.h>
void liftIdleInit(uint8_t time_sec);
bool isLiftIdle();
void triggerLiftBusySignal();
void checkLiftIdleProcess();

#endif /* INC_LIFT_IDLE_CONTROL_H_ */
