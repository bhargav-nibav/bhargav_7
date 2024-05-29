/*
 * lop_door_control.c
 *
 *  Created on: Oct 26, 2023
 *      Author: ADMIN
 */

/*********************************************************************************************/
/*********************************************************************************************/
#include "lop_door_control.h"
#include "power_detection.h"

#include "lift_idle_control.h"
#include "lidar_reading.h"

#include "overload_detection.h"
#include "gpio.h"
#include "debug_print.h"
#include "motor_controller.h"
#include "common.h"
#include "stdio.h"
#include "serial_handler.h"
/*********************************************************************************************/
/*********************************************************************************************/

DOOR_STATE_em doorStateMachine = DOOR_KEEP_CLOSED;
DOOR_Stus doorstates;
ML_Stus mlstates;
static uint8_t doorState;
FLOOR_NUM doorfloornumber;
bool doorCloseReq = true;
/*********************************************************************************************/

/*********************************************************************************************/
void openDoor(FLOOR_NUM floorNumber);
void closeDoor();
void doorStateProcess();
void test_door_inputs();
void Check_ML_inputs();
void ML_Error_inputs_outputs();
void Door_Error_inputs_outputs();
void Doorswitch_Mechanicallock_Leds();

void door_status_function(uint8_t fir, uint8_t statuss);
bool prevEmergencyState = false;
/*********************************************************************************************/

/*********************************************************************************************/
/*********************************************************************************************/
void doorInit(uint8_t flr) {
	doorStateMachine = DOOR_IDLE_ST;
	doorfloornumber = flr; //(FLOOR_NUM) getFloorNumberByLIDARValue();//GND_FLOOR;
}

void doorKeepClosed() {
	for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++) {
		if (CHECK_BIT(doorState, floorNum)) {
			setDoorLockState((FLOOR_NUM) floorNum, false);
		}
	}
}

bool checkAllDoorLockClosed() {
	for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++) {
		if (!setDoorLockState((FLOOR_NUM) floorNum, false)) {
			return false;
		}
	}
	return true;
}

bool checkDoorAllClosed() {
	for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++) {
		if (!readDoorSwitchStatus((FLOOR_NUM) floorNum)) {
//			debug_print_n("door failed here %d", floorNum);
			return false;
		}
	}
//	debug_print_n("door closed here ");
	return true;
}

bool checkMLAllClosed() {
	for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++) {
		if (readMechanicalLock((FLOOR_NUM) floorNum)) {
			return false;
		}
	}
	return true;
}

void openDoor(FLOOR_NUM floorNumber) {
	//addToQueue("LOP_OPEN_DOOR_"+String(floorNumber));
	doorStateMachine = DOOR_OPEN_REQ;
	doorfloornumber = floorNumber;
	doorCloseReq = false;
}

void closeDoor() {
	setDoorLockState(MAX_FLOOR, false);
	doorStateMachine = DOOR_CLOSED_ST;
	doorCloseReq = true;
}

void doorStateProcess() {
//	debug_print_n(" doorStateProcess \n");
	static uint32_t startTime;

	switch (doorStateMachine) {
	case DOOR_IDLE_ST: {
//    	debug_print_n("DOOR_IDLE_ST   *************\n");
		if (checkKeepDoorOpen(doorfloornumber)) {
			openDoor(doorfloornumber);
			triggerLiftBusySignal();
		}
	}
		break;
	case DOOR_OPEN_REQ: {
		doorStateMachine = DOOR_WAIT_FOR_OPEN;
		setDoorLockState(doorfloornumber, true);
		debug_print_n("unlock dor foor =%d\r\n", doorfloornumber);
		startTime = HAL_GetTick();
		triggerLiftBusySignal();
	}
		break;
	case DOOR_CLOSE_REQ: {
		doorStateMachine = DOOR_WAIT_FOR_CLOSE;
		startTime = HAL_GetTick();
	}
		break;
	case DOOR_WAIT_FOR_OPEN: {
		if ((HAL_GetTick() - startTime) < DOOR_OPEN_TIMES) {
			/** Door is opened now ?*/
			if (false == readDoorSwitchStatus(doorfloornumber)) {
				/** door opened now go to door closing**/
				debug_print_n("[Door] [Timeout] [Door opened]\n");
				doorStateMachine = DOOR_OPEN_MSG_TO_CAB;
				startTime = HAL_GetTick();
			}
		} else {
			/**Door is not opened till the timeout*/
			doorStateMachine = DOOR_OPEN_TIMEOUT;
			debug_print_n("[Door] [Timeout] [DoorClosed already]\n");
			startTime = HAL_GetTick();
		}
	}
		break;

	case DOOR_OPEN_MSG_TO_CAB: {
		doorStateMachine = DOOR_WAIT_FOR_CLOSE;
//    	lockDoorSolenoidAtFloor(doorfloornumber, true);
	}
		break;

	case DOOR_WAIT_FOR_CLOSE: {
		/** door is closed now?*/
//		debug_print_n("[door] doorCloseReq ******: %d diff : %d\n",
//				doorCloseReq, (HAL_GetTick() - startTime));
		//  if(((HAL_GetTick() - startTime > 20000) || doorCloseReq) &&(true == readDoorSwitchStatus(doorfloornumber)))
		if (true == readDoorSwitchStatus(doorfloornumber)) {
			if ((HAL_GetTick() - startTime > 30000) || doorCloseReq) {

				doorCloseReq = false;
				/** Door is closed now - set lock*/
				debug_print_n("[door] [door kept open more than 20s]\n");
				doorStateMachine = DOOR_KEEP_CLOSED;
			}

		}
		else if(false == readDoorSwitchStatus(doorfloornumber))
		{
			startTime = HAL_GetTick();
		}
		else if (HAL_GetTick() - startTime > 30000) {
			debug_print_n("[door] [door closed, trigger alarm]\n");
			/**Door not closed within timeout - enable door open alarm*/
			doorStateMachine = DOOR_OPEN_ALARM_ACT;
		}

	}
		break;
	case DOOR_OPEN_TIMEOUT: {
		/** Door is closed now - set lock*/
		debug_print_n("[door] [timeout] [init]");
		doorStateMachine = DOOR_KEEP_CLOSED;
	}
		break;
	case DOOR_KEEP_CLOSED: {
		if (setDoorLockState(doorfloornumber, false)) {
			debug_print_n("[door] [door solenoid close]\n");
			doorStateMachine = DOOR_CLOSED_ST;
		} else {
			/** make door open alarm*/
			debug_print_n("[door] [door still open]\n");
			doorStateMachine = DOOR_OPEN_ALARM_ACT;
		}
	}
		break;

	case DOOR_OPEN_ALARM_ACT: {
		doorStateMachine = DOOR_WAIT_ALARM_CLR_STATE;
	}
		break;

	case DOOR_WAIT_ALARM_CLR_STATE: {
		if (setDoorLockState(doorfloornumber, true)) {
			debug_print_n("[door][door closed after a long time]\n");
			doorStateMachine = DOOR_OPEN_ALARM_CLR;
		} else {
			startTime = HAL_GetTick();
			doorStateMachine = DOOR_OPEN_REQ;
			doorCloseReq = true;
			debug_print_n("[door] [init close sequence again]\n");
		}
	}
		break;
	case DOOR_OPEN_ALARM_CLR: {
		debug_print_n("[door][secondary timeout triggered]\n");
		doorStateMachine = DOOR_KEEP_CLOSED;
	}
		break;
	case DOOR_CLOSED_ST: {
		doorStateMachine = DOOR_IDLE_ST;
	}
		break;
	default:
		doorStateMachine = DOOR_IDLE_ST;
		break;
	}
	//test_door_inputs();

}

/*********************************************************************************************/

bool checkCabinDoorStatus(FLOOR_NUM flr) {
	bool status = false;
	if (readDoorSwitchStatus(flr)) {
		status = true;
	}
	return status;
}

bool isDoorAlarmActive() {
	return (DOOR_OPEN_ALARM_ACT <= doorStateMachine
			&& DOOR_WAIT_ALARM_CLR_STATE >= doorStateMachine) ? true : false;
}

void test_door_inputs() {
	static uint32_t prevTime;
	uint16_t cabLocation = getDistance();
	if ((HAL_GetTick() - prevTime) > 1000) {
		prevTime = HAL_GetTick();
		for (uint8_t flr = false; flr < MAX_FLOOR; flr++) {
			debug_print_n(
					"dorm=%d floor =%d door =%d mechlck =%d lidarIntensity = %d\n",
					doorStateMachine, flr,
					readDoorSwitchStatus((FLOOR_NUM) flr),
					readMechanicalLock((FLOOR_NUM) flr), getSigStrenth());
		}
		debug_print_n(" pwr = %d OVin=%d dis= %d speed =%d CAN %d lidar %d\n",
				readPowerFailure(), monitor_overload(), cabLocation,
				(int) getSpeed(), lop_disconnected(), isLIDARWorks());
	}
}

/// @brief function to check whether doors are opened on floors where cabin is not
/// @param flr
/// @return true if door open false if doors are closed
bool checkOtherDoors(FLOOR_NUM flr) {
	bool status = false;
	for (uint8_t i = 0; i < MAX_FLOOR; i++) {
		if (i != flr) {
			if (!readDoorSwitchStatus((FLOOR_NUM) flr)) {
				status = true;
			}
		}
	}
	return status;
}

bool keepDoorOpen(FLOOR_NUM flr) {
	bool status = false;
	return status; //
	return checkOtherDoors(flr);
}
