/*
 * auto_calibration.c
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */

#include "gpio.h"
#include "main.h"
#include "auto_calibration.h"
#include "led_indication.h"
#include "call_serving.h"
#include "lop_door_control.h"
#include "call_serving.h"
//#include "lop_data.h"
#include "configuration.h"
#include "serial_handler.h"
#include "shaft_controller.h"
#include "common.h"
#include "pid.h"

#define CHECK_LIFT_MOVED_UP_OFFSET(floor_pos,lidar_pos,offset) ((floor_pos-offset) >= lidar_pos)
#define CHECK_LIFT_ABOVE_OFFSET(floor_pos,lidar_pos,offset) ((floor_pos-offset) >= lidar_pos)
#define CHECK_LIFT_BELOW_OFFSET(floor_pos,lidar_pos,offset) ((floor_pos+offset) < lidar_pos)
#define CHECK_LIFT_ABOVE_FLOOR(floor_pos,lidar_pos)         ((floor_pos) > lidar_pos)

#define LIFT_TOP_FLOOR__LIDAR_VALUE       105

//#define UPCALL_TOP_POINT_OFFSET           5

/**
 *
 */
//#define DOWNCALL_BOTTOM_POINT_OFFSET     8

#define LIFT_MIN_FLOORS                  2

#define FLOOR_RANGE_IN_DIST              80

extern CONFIG_FILE config1;

// unlock ll
// move lift to top
// top reached ?
// ll lock
// turn off motor.
// lift up offset
//  u ll
// below offset
// ll
// go
static uint32_t prevStateTime;
static uint16_t prevLidarDistance;
static uint8_t validateCount;
extern uint16_t send_to_esp32[12];
extern uint16_t LIDARAtFloorPosition[4];
uint32_t speedCont;

AUTO_FLR_STS_em autoFloorMeasurement_Init();
AUTO_FLR_STS_em autoFloorMeasurement_unlockLL();
AUTO_FLR_STS_em autoFloorMeasurement_RunMotor();
AUTO_FLR_STS_em autoFloorMeasurement_TopReached();
AUTO_FLR_STS_em autoFloorMeasurement_lockLL();
AUTO_FLR_STS_em autoFloorMeasurement_GetLidarVal();
AUTO_FLR_STS_em autoFloorMeasurement_MoveAboveFloor();
AUTO_FLR_STS_em autoFloorMeasurement_MoveBelowFloor();
AUTO_FLR_STS_em autoFloorMeasurement_CheckLiftStopped();
AUTO_FLR_STS_em autoFloorMeasurement_LiftBottomReached();
AUTO_FLR_STS_em autoFloorMeasurement_checkMotorSpeed();
AUTO_FLR_STS_em autoFloorMeasurement_StopMotor();
AUTO_FLR_STS_em autoFloorMeasurement_LiftMovedAboveFloor();
AUTO_FLR_STS_em autoFloorMeasurement_LiftMovedBelowFloor();
AUTO_FLR_STS_em autoFloorMeasurement_AutofloorCalibReset();
AUTO_FLR_STS_em autoFloorMeasurement_AutofloorReportSuccess();
AUTO_FLR_STS_em autoFloorMeasurement_AutofloorReportError();

void reverseLIDARfloorValues();
typedef AUTO_FLR_STS_em (*AUTOCALL_STATES)(void);

typedef enum {
	AUTO_FLR_INIT_STATE,
	AUTO_FLR_UNLOCK_LL,
	AUTO_FLR_RUN_MOTOR,
	AUTO_FLR_CHK_MOTOR,
	AUTO_FLR_TOP_REACHED,
	AUTO_FLR_STOP_MOTOR,
	AUTO_FLR_LOCK_LL,
	AUTO_FLR_LIFT_STOPPED,
	AUTO_FLR_FLR_LIDAR,
	AUTO_FLR_BOTTOM_REACHED,
	AUTO_FLR_NXT_UNLOCK_LL,
	AUTO_FLR_RUN_MOTOR_ABVE_FLR,
	AUTO_FLR_LIFT_CHK_ABVE_FLR,
	AUTO_FLR_STOP_MOTOR_ABVE_FLR,
	AUTO_FLR_BLW_FLR,
	AUTO_FLR_RESET_STATE,
	AUTO_FLR_RESET_LL,
	AUTO_FLR_EXIT_ERRR,
	AUTO_FLR_EXIT_SUCCESS,
} AUTO_FLOOR_SATES;

struct {
	AUTOCALL_STATES state;
	AUTO_FLOOR_SATES passSate;
	AUTO_FLOOR_SATES bustSate;
	AUTO_FLOOR_SATES failSate;
} autoCalibFloor[] = {

/*AUTO_FLR_INIT_STATE,        */{ autoFloorMeasurement_Init, AUTO_FLR_UNLOCK_LL,
		AUTO_FLR_INIT_STATE, AUTO_FLR_EXIT_ERRR },
/*AUTO_FLR_UNLOCK_LL,         */{ autoFloorMeasurement_unlockLL,
		AUTO_FLR_RUN_MOTOR, AUTO_FLR_UNLOCK_LL, AUTO_FLR_EXIT_ERRR },
/*AUTO_FLR_RUN_MOTOR,         */{ autoFloorMeasurement_RunMotor,
		AUTO_FLR_CHK_MOTOR, AUTO_FLR_RUN_MOTOR, AUTO_FLR_EXIT_ERRR },
/*AUTO_FLR_CHK_MOTOR,         */{ autoFloorMeasurement_checkMotorSpeed,
		AUTO_FLR_TOP_REACHED, AUTO_FLR_CHK_MOTOR, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_TOP_REACHED,       */{ autoFloorMeasurement_TopReached,
		AUTO_FLR_STOP_MOTOR, AUTO_FLR_RUN_MOTOR, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_STOP_MOTOR,        */{ autoFloorMeasurement_StopMotor,
		AUTO_FLR_LOCK_LL, AUTO_FLR_STOP_MOTOR, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_LOCK_LL,           */{ autoFloorMeasurement_lockLL,
		AUTO_FLR_LIFT_STOPPED, AUTO_FLR_LOCK_LL, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_LIFT_STOPPED,      */{ autoFloorMeasurement_CheckLiftStopped,
		AUTO_FLR_FLR_LIDAR, AUTO_FLR_LIFT_STOPPED, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_FLR_LIDAR,         */{ autoFloorMeasurement_GetLidarVal,
		AUTO_FLR_BOTTOM_REACHED, AUTO_FLR_FLR_LIDAR, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_BOTTOM_REACHED,    */{ autoFloorMeasurement_LiftBottomReached,
		AUTO_FLR_EXIT_SUCCESS, AUTO_FLR_NXT_UNLOCK_LL, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_NXT_UNLOCK_LL,     */{ autoFloorMeasurement_unlockLL,
		AUTO_FLR_RUN_MOTOR_ABVE_FLR, AUTO_FLR_NXT_UNLOCK_LL,
		AUTO_FLR_RESET_STATE },
/*AUTO_FLR_RUN_MOTOR_ABVE_FLR,*/{ autoFloorMeasurement_RunMotor,
		AUTO_FLR_LIFT_CHK_ABVE_FLR, AUTO_FLR_RUN_MOTOR_ABVE_FLR,
		AUTO_FLR_RESET_STATE },
/*AUTO_FLR_LIFT_CHK_ABVE_FLR,  */{ autoFloorMeasurement_LiftMovedAboveFloor,
		AUTO_FLR_STOP_MOTOR_ABVE_FLR, AUTO_FLR_LIFT_CHK_ABVE_FLR,
		AUTO_FLR_RESET_STATE },
/*AUTO_FLR_STOP_MOTOR_ABVE_FLR*/{ autoFloorMeasurement_StopMotor,
		AUTO_FLR_BLW_FLR, AUTO_FLR_STOP_MOTOR_ABVE_FLR, AUTO_FLR_RESET_STATE },
/*AUTO_FLR_BLW_FLR            */{ autoFloorMeasurement_LiftMovedBelowFloor,
		AUTO_FLR_LOCK_LL, AUTO_FLR_BLW_FLR, AUTO_FLR_LIFT_STOPPED },
/*AUTO_FLR_RESET_STATE        */{ autoFloorMeasurement_AutofloorCalibReset,
		AUTO_FLR_RESET_LL, AUTO_FLR_RESET_STATE, AUTO_FLR_RESET_LL },
/*AUTO_FLR_RESET_LL           */{ autoFloorMeasurement_unlockLL,
		AUTO_FLR_EXIT_ERRR, AUTO_FLR_RESET_LL, AUTO_FLR_EXIT_ERRR },
/*AUTO_FLR_EXIT_ERRR          */{ autoFloorMeasurement_AutofloorReportError,
		AUTO_FLR_EXIT_ERRR, AUTO_FLR_EXIT_ERRR, AUTO_FLR_EXIT_ERRR },
/*AUTO_FLR_EXIT_SUCCESS       */{ autoFloorMeasurement_AutofloorReportSuccess,
		AUTO_FLR_EXIT_SUCCESS, AUTO_FLR_EXIT_SUCCESS, AUTO_FLR_EXIT_SUCCESS }, };

//static bool state = false;

bool readAutoCalibrationReq() {
	//	return false;
	static bool prevstate = true;
	bool curState = (bool) HAL_GPIO_ReadPin(AUTO_CALIB_GPIO_Port,
			AUTO_CALIB_Pin); //readEmergencyShaftInput;
	if (prevstate != curState) {
		debug_print_n("Calibration button is now %d\n", curState);
		prevstate = curState;
		//write_calibration_led(curState);

	}
	return curState; //(readEmergencyCabinInput() || readEmergencyShaftInput());
}

void autoCalibration(void) {
	static long int auto_timer_expired = 0;
	uint8_t autoCalSet = readAutoCalibrationReq();
	if (autoCalSet == true) {
		if (HAL_GetTick() - auto_timer_expired > 500) {
			processAutoFloorCalib();
			auto_timer_expired = HAL_GetTick();
		}
	} else {
		auto_timer_expired = HAL_GetTick() - 500;
	}
}

AUTO_MEASURE_STATE_SATUS runAutoFloorMeasurement() {
	uint8_t runstate;
	AUTO_FLR_STS_em sateMachinestatus;
	AUTO_MEASURE_STATE_SATUS status;

	runstate = (uint8_t) AUTO_FLR_INIT_STATE;
	status = AMS_BUSY;

	do {
		indicate_health();
		doorGetIOstate();
		getMechLockStatus();
//	updateLidarValue();
		getLidarDisAndLiftSpeed();
		controlMotorSpeedProcess();
		setcabinvalueupdated();
		send_broadcast_message();
		update_wifi_value();
//	updateWiFiValue();
		sateMachinestatus = autoCalibFloor[runstate].state();

		// debug_print_n("state =%d runstate =%d \r\n",sateMachinestatus,runstate);
		// DEBUG_AUTO(debugAutoFloor,1,"state =%d",sateMachinestatus);
		switch (sateMachinestatus) {
		case AUTO_FLR_STS_PROCESS_DONE: {
			status = AMS_SUCCESS;
		}
			break;
		case AUTO_FLR_STS_PROCESS_ERROR: {
			status = AMS_FAIL;
		}
			break;
		case AUTO_FLR_STS_COMPLETE: {
			runstate = autoCalibFloor[runstate].passSate;
		}
			break;
		case AUTO_FLR_STS_BUSY: {
			CALL_SERVE_STS preChecked = preCheck();
//    	  debug_print_n("preChecked ********** =%d  \r\n",preChecked);
			if (preChecked == CALL_SERVE_STS_NOFAIL) {
				runstate = autoCalibFloor[runstate].bustSate;
				setcalibrationStatus(CALIBRATION_START);
			} else {
				resetSpeedControlLogic();
				status = AMS_FAIL;
				setcalibrationStatus(CALIBRATION_STOP);
				setErrorMessage(preChecked);
			}

		}
			break;
		case AUTO_FLR_STS_ERROR: {
			runstate = autoCalibFloor[runstate].failSate;
		}
			break;
		default: {
			status = AMS_FAIL;
		}
		}
	} //while((status == AMS_BUSY) ||(!checkAllDoorLockClosed()) || (!isLIDARWorks())); //while((status == AMS_BUSY) && (checkAllDoorLockClosed()!=CALL_SERVE_STS_NOFAIL));
	while (status == AMS_BUSY); // ||(!doorClosed() || (!isLIDARWorks()))); //while((status == AMS_BUSY) && (checkAllDoorLockClosed()!=CALL_SERVE_STS_NOFAIL));

	return status;
}

AUTO_FLR_STS_em autoFloorMeasurement_unlockLL() {
	setLLState(false);
	debug_print_n("state unlock LL \r\n");
	return AUTO_FLR_STS_COMPLETE;

}

AUTO_FLR_STS_em autoFloorMeasurement_RunMotor() {
	uint16_t lidarDistance;

	lidarDistance = getDistance();
	speedCont++;
//	 setMotorSpeed(7000);
//	 if(speedCont <= 500)
//	 {
//		 debug_print_n("speedCont : %d \n",speedCont);
//		 pidUpControl(2, 1000, 0.1, 5, lidarDistance);
//	 }
//	 else
//	 {
	pidUpControl(2, 60, 0.1, 10, lidarDistance);
//	 }

//  setMotorControlSpeed(CONTROL_UP_DIR ,(int16_t)10);
	debug_print_n("Set Speed \r\n");
//  debug_print_n(debugAutoFloor,2,"Set Speed \r\n");
	prevStateTime = HAL_GetTick();
	return AUTO_FLR_STS_COMPLETE;
}

AUTO_FLR_STS_em autoFloorMeasurement_checkMotorSpeed() {
	debug_print_n("check motor speed \r\n");
//  debug_print_n(debugAutoFloor,3,"check motor speed \r\n");
//  if(getControlStatus() == CONTROL_ACHIVED)
//  {
//    prevStateTime = HAL_GetTick();
//    debug_print_n(" AUTO_FLR_STS_COMPLETE ****\r\n");
//    return AUTO_FLR_STS_COMPLETE;
//  }
//  else if(getControlStatus() == CONTROL_ERROR)
//  {
//    prevStateTime = HAL_GetTick();
//    debug_print_n("AUTO_FLR_STS_ERROR\r\n");
//    return AUTO_FLR_STS_ERROR;
//  }
	return AUTO_FLR_STS_COMPLETE;
	//debug_print_n(" AUTO_FLR_STS_BUSY getControlStatus %d : \r\n", getControlStatus());
//  return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_TopReached() {
	uint16_t lidarDistance;
	debug_print_n("TopReached \r\n");
	if (isLIDARWorks()) {
		//debug_print_n("LIDAR measu \r\n");
//	debug_print_n(debugAutoFloor,4,"LIDAR measu \r\n");
		lidarDistance = getDistance();
		debug_print_n("LIDAR measu %d\r\n", lidarDistance);
//   	pidUpControl(2, 50, 0.1, 10, lidarDistance);
		if (CHECK_LIFT_ABOVE_OFFSET(LIFT_TOP_FLOOR__LIDAR_VALUE, lidarDistance,
				UPCALL_TOP_POINT_OFFSET)) {
//      debug_print_n(debugAutoFloor,20,"20 Top reached \r\n");
			debug_print_n("20 Top reached \r\n");
			prevStateTime = HAL_GetTick();
			return AUTO_FLR_STS_COMPLETE;
		}
	} else {
		prevStateTime = HAL_GetTick();
		// debug_print_n(debugAutoFloor,21,"21 Top Error \r\n");
		debug_print_n("21 Top Error \r\n");
		return AUTO_FLR_STS_ERROR;
	}
	return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_lockLL() {
	setLLState(true);
	// debug_print_n(debugAutoFloor,5,"LOCK LL \r\n");
	debug_print_n("LOCK LL \r\n");
	return AUTO_FLR_STS_COMPLETE;

}

AUTO_FLR_STS_em autoFloorMeasurement_CheckLiftStopped() {
	uint16_t lidarDistance;
	uint16_t lidarDiff;

//  debug_print_n(debugAutoFloor,6,"check LIFT stopped \r\n");
//  debug_print_n("check LIFT stopped \r\n");
	if (validateCount < LIFT_STOPPED_CHECK_MAX_COUNT) {
		if ((HAL_GetTick() - prevStateTime) > TWO_SECOND) {
			debug_print_n("Lift stop counter\r\n");
//    	debug_print_n(debugAutoFloor,20,"Lift stop counter\r\n");
			prevStateTime = HAL_GetTick();
			if (isLIDARWorks()) {
				lidarDistance = getDistance();
			} else {
				prevStateTime = HAL_GetTick();
				prevLidarDistance = false;
				validateCount = false;
				return AUTO_FLR_STS_ERROR;
			}

			lidarDiff =
					(prevLidarDistance > lidarDistance) ?
							(prevLidarDistance - lidarDistance) :
							(lidarDistance - prevLidarDistance);
			prevLidarDistance = lidarDistance;
			if (lidarDiff < LIDR_DIFF_VAL_ON_LIFT_STOPPED) {
				validateCount++;
				debug_print_n("validateCount=%d \r\n", validateCount);
			} else {
				validateCount = 0;
				debug_print_n("else validate count \r\n");
			}
		}
	} else {
		prevStateTime = HAL_GetTick();
		prevLidarDistance = false;
		validateCount = false;
		return AUTO_FLR_STS_COMPLETE;
	}
	return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_GetLidarVal() {
	CONFIG_FILE *config = getConfig();
//  debug_print_n(debugAutoFloor,7,"Get LIDAR val \r\n");
	speedCont = 0;
	debug_print_n("Get LIDAR val \r\n");
	if (config->numberOfFloors < MAX_FLOOR_CONST) {
		config->LIDAR[config->numberOfFloors] = getDistance();
		debug_print_n("floor lidar val=%d floor num =%d \r\n",
				config->LIDAR[config->numberOfFloors], config->numberOfFloors);
		config->numberOfFloors++;
		prevStateTime = HAL_GetTick();
		return AUTO_FLR_STS_COMPLETE;
	} else {
		prevStateTime = HAL_GetTick();
		return AUTO_FLR_STS_ERROR;
	}
}

AUTO_FLR_STS_em autoFloorMeasurement_LiftMovedAboveFloor() {
	uint16_t lidarDistance;
	CONFIG_FILE *config = getConfig();
	debug_print_n("Lift abve floor \r\n");
	//debug_print_n(debugAutoFloor,8,"Lift abve floor \r\n");
	if (isLIDARWorks() || config->numberOfFloors <= false) {
		lidarDistance = getDistance();
	} else {
		prevStateTime = HAL_GetTick();
		return AUTO_FLR_STS_ERROR;
	}
	// get current floor LIDAR value.
	if (CHECK_LIFT_ABOVE_OFFSET(config->LIDAR[config->numberOfFloors - 1],
			lidarDistance, DOWNCALL_BOTTOM_POINT_OFFSET)) {
		prevStateTime = HAL_GetTick();
		return AUTO_FLR_STS_COMPLETE;
	}
	return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_LiftMovedBelowFloor() {
	uint16_t lidarDistance;
	uint16_t lidarDiff;
	CONFIG_FILE *config = getConfig();
	debug_print_n("Lift below floor \r\n");
	//debug_print_n(debugAutoFloor,9,"Lift below floor \r\n");
	if (isLIDARWorks() || config->numberOfFloors <= false) {
		lidarDistance = getDistance();
	} else {
		prevStateTime = HAL_GetTick();
		prevLidarDistance = false;
		validateCount = false;
		return AUTO_FLR_STS_ERROR;
	}
	// get current floor LIDAR value.
	if (CHECK_LIFT_BELOW_OFFSET(config->LIDAR[config->numberOfFloors - 1],
			lidarDistance, DOWNCALL_BOTTOM_POINT_OFFSET)) {
		prevStateTime = HAL_GetTick();
		prevLidarDistance = false;
		validateCount = false;
		return AUTO_FLR_STS_COMPLETE;
	} else if ((HAL_GetTick() - prevStateTime) > ONE_SECOND) {
		prevStateTime = HAL_GetTick();
		lidarDiff =
				(prevLidarDistance > lidarDistance) ?
						(prevLidarDistance - lidarDistance) :
						(lidarDistance - prevLidarDistance);
		prevLidarDistance = lidarDistance;
		if (lidarDiff < LIDR_DIFF_VAL_ON_LIFT_STOPPED) {
			if (validateCount < LIFT_STOPPED_CHECK_MAX_COUNT) {
				validateCount++;
			} else {
				prevStateTime = HAL_GetTick();
				prevLidarDistance = false;
				validateCount = false;
				return AUTO_FLR_STS_ERROR;
			}
		} else {
			validateCount = false;
		}
	}
	return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_LiftBottomReached() {
	uint16_t dist_diff;
	uint16_t top;
	uint16_t bottom;
	CONFIG_FILE *config = getConfig();
	debug_print_n("Lift bottom reachec \r\n");
//  debug_print_n(debugAutoFloor,10,"Lift bottom reachec \r\n");
	if (config->numberOfFloors > LIFT_MIN_FLOORS) {
		top = config->LIDAR[config->numberOfFloors - 1];
		bottom = config->LIDAR[config->numberOfFloors - 2];
		dist_diff = (top < bottom) ? (bottom - top) : (top - bottom);
		if (dist_diff <= FLOOR_RANGE_IN_DIST) {
			// remove last try from count.
			config->numberOfFloors--;
			prevStateTime = HAL_GetTick();
			return AUTO_FLR_STS_COMPLETE;
		}
	}
	return AUTO_FLR_STS_BUSY;
}

AUTO_FLR_STS_em autoFloorMeasurement_StopMotor() {
	debug_print_n("Reset stop motor \r\n");
	resetSpeedControlLogic();
	prevStateTime = HAL_GetTick();
	return AUTO_FLR_STS_COMPLETE;
}

AUTO_FLR_STS_em autoFloorMeasurement_AutofloorCalibReset() {
	debug_print_n("Reset motor logic \r\n");
	//debug_print_n(debugAutoFloor,12,"Reset motor logic \r\n");
	resetSpeedControlLogic();
	prevStateTime = HAL_GetTick();
	return AUTO_FLR_STS_COMPLETE;
}

AUTO_FLR_STS_em autoFloorMeasurement_AutofloorReportError() {
	// debug_print_n(debugAutoFloor,13,"Report error \r\n");
	debug_print_n("Report error \r\n");
	return AUTO_FLR_STS_PROCESS_ERROR;
}

AUTO_FLR_STS_em autoFloorMeasurement_AutofloorReportSuccess() {
	CONFIG_FILE *config = getConfig();
	setcalibrationStatus(CALIBRATION_SUCCESS);
	// HAL_Delay(5000);

//  debug_print_n(debugAutoFloor,14,"Reverse Floor values \r\n");
	debug_print_n("Reverse Floor values \r\n");
	reverseLIDARfloorValues();
	// setcalibrationStatus(CALIBRATION_IDLE);
	for (uint8_t flr = false; flr < config->numberOfFloors; flr++) {
		debug_print_n("Floor =%d Lidar val = %d \r\n", flr, config->LIDAR[flr]);
	}

	return AUTO_FLR_STS_PROCESS_DONE;
}

void reverseLIDARfloorValues() {
	uint8_t idx;
	uint16_t temp;
	uint8_t end;
	CONFIG_FILE *config = getConfig();

	if (config->numberOfFloors) {
		end = config->numberOfFloors - 1;
		for (idx = false; idx < config->numberOfFloors / 2; idx++) {
			temp = config->LIDAR[idx];
			config->LIDAR[idx] = config->LIDAR[end];
			config->LIDAR[end] = temp;
			end--;
		}
	}
	writeConfigFile(false);
	update_lidar_values(true);
	send_lidar_value();
	store_values();
}

AUTO_FLR_STS_em autoFloorMeasurement_Init() {
	CONFIG_FILE *config = getConfig();

	config->numberOfFloors = false;
	memset(config->LIDAR, false, sizeof(config->LIDAR));
	debug_print_n("auto floor initialialistion \r\n");
	return AUTO_FLR_STS_COMPLETE;
}

//void reset_timer()
//{
//	//timer_expired = HAL_GetTick() - 500;
//}

