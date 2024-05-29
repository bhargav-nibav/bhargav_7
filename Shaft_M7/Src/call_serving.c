/*
 * call_serving.c
 *
 *  Created on: Oct 27, 2023
 *      Author: ADMIN
 */
//#include "stm32h7xx_hal.h"
#include "call_serving.h"
#include "auto_calibration.h"
#include "motor_controller.h"
#include "configuration.h"
#include "lop_door_control.h"
#include "call_reg.h"
#include "lift_idle_control.h"
#include "overload_detection.h"
#include "power_detection.h"
#include "buzzer_enabler.h"
#include "led_indication.h"
#include "serial_handler.h"
#include "emergency_monitor.h"
#include "pid.h"
#include "common.h"

/*********************************************************************************************/
#define SEC_TO_MS(sec) (sec * 1000)
//bool readChildlockCabinInput();
/*********************************************************************************************/
#define ERR_LOG // Serial.println
#define MECHLOCK_CHANGE_DIS 25
#define EMERGENCY_TIME_STEPS 1000

#define ERROR_DIRECTION 0x03

#define LIFT_UP_OFFSET_TIME (5 * 1000)
#define CHECK_LIFT_MOVED_UP_OFFSET(floor_pos, lidar_pos, offset) ((floor_pos - offset) >= lidar_pos)

#define CHECK_LIFT_ABOVE_OFFSET(floor_pos, lidar_pos, offset) ((floor_pos - offset) >= lidar_pos)

#define CHECK_LIFT_BELOW_OFFSET(floor_pos, lidar_pos, offset) ((floor_pos + offset) < lidar_pos)

#define CHECK_LIFT_ABOVE_FLOOR(floor_pos, lidar_pos) ((floor_pos) > lidar_pos)

#define CHECK_LIFT_ABOVE_BOTTOM(floor_pos, lidar_pos, offset) ((floor_pos + offset) > lidar_pos)

#define CHECK_LIFT_ABOVE_TOP(floor_pos, lidar_pos, offset) ((floor_pos - offset) > lidar_pos)

#define CHECK_LIFT_IN_SOFT_ZONE(floor_pos, lidar_pos, offset) ((floor_pos - offset) > lidar_pos)

#define AUTO_CALIB_DOOR_CLOSE_WAIT_TIME 30000
#define AUTO_CALIB_ENABLE_TIME           5000
/*********************************************************************************************/
bool callClearFail = false;
uint32_t callClearFailedValue = 0;

static long int failedTimer = 0;
bool checkKeepDoorOpen(FLOOR_NUM flr);

bool clearCallOnSafetyFail = false;
const int DOWNCALL_TOP_POINT_OFFSET[MAX_FLOOR] = { 150, 150, 150, 150 };
//const int DOWNCALL_TOP_POINT_OFFSET[MAX_FLOOR] = { 35, 30, 25, 15 };
//const int DOWNCALL_TOP_POINT_OFFSET[MAX_FLOOR] = { 200, 250 };
/*********************************************************************************************/
#define SPEED_MODE_LOW
//PID COntrol Initialization

//#define SPEED_LOW		0
//#define SPEED_MODERATE	1
//#define SPEED_HIGH		2

//#ifdef SPEED_MODE_LOW
/************   Up Call  ******************/
//volatile int upcallSoftStart[2] = { 80, 5 };
//volatile int upcallMaxSpeed[2] = { 100, 12 };
//volatile int upcallSoftLand1[2] = { 100, 7 };
//volatile int upcallSoftLand2[2] = { 100, 4 };
//
//volatile int downcallSoftStart[2] = { 30, 5 };
//volatile int downcallMaxSpeed[2] = { 100, -12 };
//volatile int downcallSoftLand1[2] = { 100, -7 };
//volatile int downcallSoftLand2[2] = { 100, -4 };
//#elif SPEED_MODE_MODERATE
///************   Up Call  ******************/
volatile int upcallSoftStart[2] = { 10, 10 };
volatile int upcallMaxSpeed[2] = { 100, 20 };
volatile int upcallSoftLand1[2] = { 100, 10 };
volatile int upcallSoftLand2[2] = { 100, 5 };

/************   Down Call  ******************/
volatile int downcallSoftStart[2] = { 100, 10 };
volatile int downcallMaxSpeed[2] = { 100, -20 };
volatile int downcallSoftLand1[2] = { 100, -10 };
volatile int downcallSoftLand2[2] = { 100, -4 };

volatile int fixedPIDIVal = 5;
//
//#else
//volatile int upcallSoftStart[2] = { 60, 8 };
//volatile int upcallMaxSpeed[2] = { 100, 22 };
//volatile int upcallSoftLand1[2] = { 100, 10 };
//volatile int upcallSoftLand2[2] = { 100, 5 };
//
///************   Down Call  ******************/
//volatile int downcallSoftStart[2] = { 40, 5 };
//volatile int downcallMaxSpeed[2] = { 100, -22 };
//volatile int downcallSoftLand1[2] = { 100, -12 };
//volatile int downcallSoftLand2[2] = { 100, -5 };
///************   Up Call  ******************/
//const int upcallSoftStart[2] = { 100, 5 };
//const int upcallMaxSpeed[2] = { 100, 23 };
//const int upcallSoftLand1[2] = { 100, 10 };
//const int upcallSoftLand2[2] = { 100, 5 };
//
///************   Down Call  ******************/
//const int downcallSoftStart[2] = { 30, -5 };
//const int downcallMaxSpeed[2] = { 100, -23 };
//const int downcallSoftLand1[2] = { 100, -10 };
//const int downcallSoftLand2[2] = { 100, -5 };
//
//#endif

int errStatement = 0;
bool enCabDerailCheck;
volatile bool mech_lckerr = false;

// extern bool Overload_safety;
void setCallSafety(bool status);
typedef enum {
	EMGCY_STATE_IDLE,
	EMGCY_STATE_CHECKS,
	EMGCY_STATE_UNLOCK_LL,
	EMGCY_STATE_RUN_MOTOR,
	EMGCY_STATE_WAIT_FOR_RELEASE,
	EMGCY_STATE_FLOOR_LVL,
	EMGCY_STATE_LOCK_LL,
	EMGCY_STATE_DEST,
	EMGCY_STATE_OPEN_DOOR,
	EMGCY_STATE_RESET,
} EMERGENCY_STATES;

typedef struct cabinApplicationRequest {
	bool startAutoFloorCalib;
// AUTO_MEASURE_STATE_SATUS autoFloorCalibStatus;
} CABIN_APP_REQ;

CABIN_APP_REQ cabinAppReq;

LIFT_STATUS_st liftStatus;
uint8_t stateDebug, estateDebug;
static AUTO_CALIB_APP_em autoCalibState;
uint16_t LIDARAtFloorPosition[MAX_FLOOR] = { 10800, 6760, 3485, 220 };
//uint16_t LIDARAtFloorPosition[MAX_FLOOR] = { 9860, 6650, 3448, 256 };
//uint16_t LIDARAtFloorPosition[MAX_FLOOR] = { 2560, 160 };

bool calibration_state = false;
bool power_state = false;
bool overload_state = false;
bool emergency_state = false;
bool lidar_state = false;
bool cabin_booking_allowed = false;
bool cabin_schedule_lock = false;
int people_boarded_count = 0;
bool over_speed_triggered = false;
bool device_under_update = false;
uint8_t softStartCounter = 0;

extern volatile double upCallKp, upCallKi, upCallKd;
extern volatile double upCallSetpoint;
extern volatile uint16_t downCallOutputAdjusted, upCallOutputAdjusted;
extern volatile double downCallKp, downCallKi, downCallKd;
extern volatile double downCallSetpoint;
/*********************************************************************************************/

void upDateCabinFloorStatus();
void serveCalls();
void resetLiftServeStateMachine();
FLOOR_NUM getNextUpCall(uint8_t currentFloor);
FLOOR_NUM getNextDownCall(uint8_t currentFloor);

CALL_SERVE_STS serveCallOnSameFloor(FLOOR_NUM floorNum);
int getCallDirection();
MECH_LOCK_STATUS findCabinPostion(FLOOR_NUM *floorNum);
CALL_SERVE_STS serviceUpcall(uint8_t currentFloor, uint8_t destFloor);
CALL_SERVE_STS serviceDowncall(uint8_t currentFloor, uint8_t destFloor);
CALL_TYPE getFloorCallType();
CALL_SERVE_STS serveDownCall(uint8_t currentFloor, uint8_t destFloor);
CALL_SERVE_STS serveUpCall(uint8_t currentFloor, uint8_t destFloor);
bool handleLandingLeverFailure();
bool handle_lift_service_error(CALL_SERVE_STS sts);
bool handleOverloadFailure();
bool handleDoorOpenTooLong();
bool handleClearCall();
bool handleLIDARfailure();
bool handleCabinDerailfailure();
void cancelAllRegCallsInShaftandCabin();
void handleCallsRegistered();
bool get_mech_lock_error1(uint8_t currentFlr);
bool getEmergencyInput();
bool get_mech_lock_error(uint8_t currentFlr, uint8_t destFloor,
		CALL_TYPE direction);
bool callServeCheckLiftStopped();
uint8_t getFloorNumberByLIDARValue();
bool handleSafetyFailure();
bool handleCANSafetyFailure(CALL_SERVE_STS serveSt);
bool handlePowerFailure();
bool handleOverloadSafetyFailure();
bool handle_light_curtain_safety();
bool handleMechLockFailureOnCallReg();
bool handleSafetyLockFailureOnCallServe();
AUTO_FLR_STS_em callserve_CheckLiftStopped();
REQ_STATUS updateFloorNumberMsgToCabin();
bool handleEspComFailure();
void debug_serving_state();

uint8_t mech_lock_prev;
/*********************************************************************************************/

void update_lidar_values(bool _status) {
	CONFIG_FILE *config = getConfig();
	for (int i = 0; i < 4; i++) {
		if (_status) {
			LIDARAtFloorPosition[i] = config->LIDAR[i];
		} else {
			config->LIDAR[i] = LIDARAtFloorPosition[i];
		}

	}
	send_to_shaft_esp32(config->LIDAR);
}

void set_lidar(uint8_t *modified_lidar) {
	CONFIG_FILE *configFile = getConfig();
	debug_print_n("[lidar received] ");
	for (int i = 0; i < 4; i++) {
		configFile->LIDAR[i] = modified_lidar[2 * i] * 256
				+ modified_lidar[2 * i + 1];
		debug_print_n("set_lidar %d\t", (int) configFile->LIDAR[i]);
	}
	debug_print_n("\n");
	update_lidar_values(true);
}
void initLiftStateMachine() {
	liftStatus.liftRunStatus = LIFT_IDLE;
	liftStatus.direction = NO_DIR;
	liftStatus.destCallFloor = MAX_FLOOR;
	liftStatus.MsgCabinfloorNum = MAX_FLOOR;
	liftStatus.liftNearDestination = false;
	set_motor_count(MSPEED_STOP);
	resetLiftServeStateMachine();
}

void monitor_devices() {
//	debug_print_n("monitor_devices \n");
	static long int timer_expired = 0;
	calibration_state = readAutoCalibrationReq();
	power_state = readPowerFailure();
	overload_state = monitor_overload();
	emergency_state = readEmergencyShaftInput();
	lidar_state = isLIDARWorks();
	cabin_booking_allowed = cabin_battery_acceptable();
	cabin_schedule_lock = downtime_status();
	people_boarded_count = people_count();
	device_under_update = return_update();
//	over_speed_triggered	= cabin_freefalling();
	if (HAL_GetTick() - timer_expired > 1000) {
		timer_expired = HAL_GetTick();
		debug_print_n(
				"[calib]:[%s]\t[power]:[%s]\t[overload]:[%s]\t[emergency]:[%s]\t[lidar]:[%s]\n",
				calibration_state ? "Pressed" : "Released",
				power_state ? "Fail" : "Available",
				overload_state ? "Active" : "Inactive",
				emergency_state ? "Pressed" : "Release",
				lidar_state ? "Active" : "Inactive");
		debug_print_n("[downtime]:[%s] [child_lock]:[%s]\n",
				cabin_schedule_lock ? "active" : "inactive",
				child_lock_input() ? "Pressed" : "Released");
	}
}

void serveCalls() {
//	debug_print_n(" serveCalls \n");
	static bool emergencyStatus;
	if (liftStatus.emergencyOn) {
		if (emergencyStatus == false) {
			emergencyStatus = true;
		}
	} else {
		if (emergencyStatus == true) {
			emergencyStatus = false;
		}
		handleCallsRegistered();
	}
}

void handleCallsRegistered() {
	static CALL_SERVE_STS serveStatus;
	static CALL_SERVE_STS preCheckPrevValue = CALL_SERVE_STS_COMPLETE;
	FLOOR_NUM floorNum;
	switch (liftStatus.liftRunStatus) {
	case LIFT_IDLE: {
//		set_motor_count(MSPEED_STOP);
//		setEVstate(false);
#ifdef SERVO_ENABLE
				setEvoServoDuty(EVO_SERVO_CLOSE);
#else
		setEVstate(EVO_CLOSE);
#endif

		setMotorSpeed(MOTOR_STOP);
		liftStatus.destCallFloor = MAX_FLOOR;
		liftStatus.liftNearDestination = false;
		liftStatus.liftMovingStarted = false;
		ConfigDisableCallRegistration(false);
		enCabDerailCheck = true;
		liftStatus.liftRunStatus = LIFT_INITIAL_FLOOR_UPDATE;
		PidInitializeUpCall();
//		PidInitializeDownCall();
		debug_print_n("LIFT_IDLE\n");
		cancelAllcalls();
	}
		break;

	case LIFT_INITIAL_FLOOR_UPDATE: {
		static FLOOR_NUM liftStatus_CabinCurrentfloorNum_prev = 0;
		liftStatus.liftRunStatus = LIFT_CHECK_IDLE;
		if (liftStatus.CabinCurrentfloorNum
				!= liftStatus_CabinCurrentfloorNum_prev) {
			debug_print_n("liftSts.CbCurrentfloor 1 : %d \n",
					liftStatus.CabinCurrentfloorNum);
		}
		liftStatus_CabinCurrentfloorNum_prev = liftStatus.CabinCurrentfloorNum;

		setCabinFloorNumber(liftStatus.CabinCurrentfloorNum);
		setLLState(false);
		setCabinCallBooked(MAX_FLOOR);
		setLOPCallBookedMessage(MAX_FLOOR);
		setErrorMessage(0xAF);
	}
		break;

	case LIFT_CHECK_IDLE: {
		FLOOR_NUM cabin_flr = liftStatus.CabinCurrentfloorNum;
		checkLiftIdleProcess();
		if (!readDoorSwitchStatus(cabin_flr)) {
			triggerLiftBusySignal();
		}
		if (liftStatus.isLiftMsgSentCabin != isLiftIdle()) {
			liftStatus.isLiftMsgSentCabin = isLiftIdle();
		}
		liftStatus.liftRunStatus = LIFT_CALL_REG_VALIDATION;
	}
		break;
	case LIFT_CALL_REG_VALIDATION: {
		CALL_SERVE_STS preChecked = preCheck();
		if (preCheckPrevValue != preChecked) {
			preCheckPrevValue = preChecked;
		}
		if (preChecked == CALL_SERVE_STS_NOFAIL) {
//			debug_print_n("Call serve no fail\n");
			setCallSafety(false);
		} else {
//			debug_print_n("safety failed\n");
			setCallSafety(true);
		}
		liftStatus.liftRunStatus = LIFT_CHECK_NEWCALLS;
	}
		break;

	case LIFT_CHECK_NEWCALLS: {
		for (uint8_t floorNum = 0; floorNum < MAX_FLOOR; floorNum++) {
			if (checkCallRegisterd((FLOOR_NUM) floorNum, CALL_CABIN)) {
				debug_print_n("*********cab call booked : %d\n", floorNum);
				if (getCallSafety()) {
					debug_print_n("send fail\n");
					liftStatus.liftRunStatus = LIFT_SEND_FAIL_REASON;
					break;
				} else {
					liftStatus.destCallFloor = (FLOOR_NUM) floorNum;
					liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
					liftStatus.startTime = HAL_GetTick();
					setCabinCallBooked(liftStatus.destCallFloor);
					//					setCallBookedMessage();
					debug_print_n("cab : predict dir\n");
					break;
				}
			} else if (checkCallRegisterd((FLOOR_NUM) floorNum,
			CALL_FROM_HAL)) {
				if (liftStatus.destCallFloor == MAX_FLOOR) {
					if (getCallSafety()) {
						debug_print_n("lop send fail\n");
						liftStatus.liftRunStatus = LIFT_SEND_FAIL_REASON;
						break;
					} else {
						liftStatus.destCallFloor = (FLOOR_NUM) floorNum;
						liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
						liftStatus.startTime = HAL_GetTick();
						debug_print_n("50: lop predict dir\n");
						setLOPCallBookedMessage(liftStatus.destCallFloor);
						break;
					}
				}
			}
		}
		if ((liftStatus.liftRunStatus != LIFT_PREDICT_DIR)
				&& (liftStatus.liftRunStatus != LIFT_SEND_FAIL_REASON)
				&& (liftStatus.liftRunStatus != LIFT_SERVED)
				&& (liftStatus.liftRunStatus != LIFT_STM_RESET)) {
			liftStatus.liftRunStatus = LIFT_CHECK_IDLE;
		}
	}
		break;
	case LIFT_SEND_FAIL_REASON: {
		// send error status to cabin
		debug_print_n("error reason %d \n", preCheckPrevValue);
		setErrorMessage(preCheckPrevValue - 1);
		liftStatus.liftRunStatus = LIFT_SERVED;
	}
		break;
	case LIFT_PREDICT_DIR: {

		liftStatus.direction = getCallDirection();
		liftStatus.sourceCallFloor = liftStatus.CabinCurrentfloorNum;
		if (liftStatus.direction != NO_DIR) {
			liftStatus.liftRunStatus = LIFT_CLOSE_DOOR;
			debug_print_n("disengage ll %d \n", liftStatus.direction);
		} else {
			liftStatus.liftRunStatus = LIFT_SERVING;
			debug_print_n("serving ll %d \n", liftStatus.direction);
		}
		debug_print_n("initiate call serving %d \n", liftStatus.direction);
		PidInitializeUpCall();
//		PidInitializeDownCall();
		resetLiftServeStateMachine();
		ConfigDisableCallRegistration(true);
		liftStatus.startTime = HAL_GetTick();
		triggerLiftBusySignal();
	}
		break;

	case LIFT_CLOSE_DOOR: {
		closeDoor();
//		lockDoorSolenoidAtFloor(MAX_FLOOR,false);
		liftStatus.liftRunStatus = LIFT_DISENGAGE_LL;
		debug_print_n("lift close door\n");
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case LIFT_CHECK_DOOR_LOCK: {
		if (checkAllDoorLockClosed()) {
			liftStatus.liftRunStatus = LIFT_CHECK_LOCKS;
		} else if (HAL_GetTick() - liftStatus.startTime > 3000) {
			liftStatus.startTime = HAL_GetTick();
			liftStatus.liftRunStatus = LIFT_ERROR;
			serveStatus = CALL_SERVE_STS_DOORLOCK_FAIL;
		} else {
			liftStatus.liftRunStatus = LIFT_ERROR;
			serveStatus = CALL_SERVE_STS_DOORLOCK_FAIL;
		}
	}
		break;

	case LIFT_CHECK_LOCKS: {
		if (liftStatus.direction != NO_DIR) {
			if (MECH_LOCK_NO_ERROR == liftStatus.lckStatus) {
				// lock may not be valid ...
				if (setDoorLockState(liftStatus.CabinCurrentfloorNum, false)) {
					liftStatus.liftRunStatus = LIFT_DISENGAGE_LL;
					// printf("idle to new call chk \r\n");
				} else {
					//          DEBUG(stateDebug, 7, "7: open not cls \r\n");
				}
			} else {
				// printf("8: mech lck error \r\n");
				liftStatus.liftRunStatus = LIFT_ERROR;
				serveStatus = CALL_SERVE_STS_ML_FAIL_BEFORE_CALL;
			}
		} else {
			liftStatus.liftRunStatus = LIFT_DISENGAGE_LL;
		}
	}
		break;
	case LIFT_DISENGAGE_LL: {
		debug_print_n("disengage ll");
		// Send ll message false message to cabin
		liftStatus.liftRunStatus = LIFT_SERVING;
		setLLState(false);
	}
		break;
	case LIFT_SERVING: {

		if (UP_DIR == liftStatus.direction) {
			serveStatus = serviceUpcall(liftStatus.sourceCallFloor,
					liftStatus.destCallFloor);
			// debug_print(buf, v);

			if (CALL_SERVE_STS_COMPLETE == serveStatus) {
				liftStatus.liftRunStatus = LIFT_SERVED;
			} else if (CALL_SERVE_STS_NOFAIL == serveStatus) {
			} else {
				liftStatus.liftRunStatus = LIFT_ERROR;
				//setMotorSpeed(MOTOR_STOP);
			}
		} else if (DOWN_DIR == liftStatus.direction) {
			serveStatus = serviceDowncall(liftStatus.sourceCallFloor,
					liftStatus.destCallFloor);
			if (CALL_SERVE_STS_COMPLETE == serveStatus) {
				liftStatus.liftRunStatus = LIFT_SERVED;
			} else if (CALL_SERVE_STS_NOFAIL == serveStatus) {
			} else {
				liftStatus.liftRunStatus = LIFT_ERROR;
				//	setMotorSpeed(MOTOR_STOP);
			}
		} else {

			serveStatus = serveCallOnSameFloor(liftStatus.destCallFloor);
			if (CALL_SERVE_STS_COMPLETE == serveStatus) {
				liftStatus.liftRunStatus = LIFT_SERVED;
			} else if (CALL_SERVE_STS_NOFAIL == serveStatus) {
			} else {
				liftStatus.liftRunStatus = LIFT_ERROR;
			}
		}
	}
		break;
	case LIFT_SERVED: {
		liftStatus.CabinCurrentfloorNum = liftStatus.destCallFloor;
		clearRegisteredCalls(liftStatus.destCallFloor);
		liftStatus.destCallFloor = MAX_FLOOR;
		resetLiftServeStateMachine();
		liftStatus.liftRunStatus = LIFT_PROCESS_SERVED;
	}
		break;
	case LIFT_PROCESS_SERVED: {
		if (handleClearCall()) {
			liftStatus.liftRunStatus = LIFT_PROCESS_UPDATE_FLOOR_NUM;
		}
	}
		break;

	case LIFT_PROCESS_UPDATE_FLOOR_NUM: {
		// update cabin floor number
		setCabinFloorNumber(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftRunStatus = LIFT_STM_RESET;
	}
		break;

	case LIFT_NEXT_CALL: {
		if (UP_DIR == liftStatus.direction) {
			floorNum = getNextUpCall(liftStatus.CabinCurrentfloorNum);
			if (MAX_FLOOR != floorNum) {
				liftStatus.destCallFloor = floorNum;
				liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
			} else {
				floorNum = getNextDownCall(liftStatus.CabinCurrentfloorNum);
				if (MAX_FLOOR != floorNum) {
					liftStatus.destCallFloor = floorNum;
					liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
				} else {
					liftStatus.destCallFloor = MAX_FLOOR;
					liftStatus.liftRunStatus = LIFT_STM_RESET;
				}
			}
		} else if (DOWN_DIR == liftStatus.direction) {
			floorNum = getNextDownCall(liftStatus.CabinCurrentfloorNum);
			if (MAX_FLOOR != floorNum) {
				liftStatus.destCallFloor = floorNum;
				liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
			} else {
				floorNum = getNextUpCall(liftStatus.CabinCurrentfloorNum);
				if (MAX_FLOOR != floorNum) {
					liftStatus.destCallFloor = floorNum;
					liftStatus.liftRunStatus = LIFT_PREDICT_DIR;
				} else {
					liftStatus.destCallFloor = MAX_FLOOR;
					liftStatus.liftRunStatus = LIFT_STM_RESET;
				}
			}
		} else {
			liftStatus.liftRunStatus = LIFT_STM_RESET;
		}
	}
		break;
	case LIFT_ERROR: {
		debug_print_n("Error happened here- %d\n", (int) serveStatus - 1);
		resetLiftServeStateMachine();
		liftStatus.liftRunStatus = LIFT_ERROR_SERVICE; // LIFT_ERROR_SERVICE;
		liftStatus.startTime = HAL_GetTick();
		setErrorMessage((int) serveStatus - 1);
	}
		break;
	case LIFT_SEND_ERROR_MESSAGE: {
		liftStatus.liftRunStatus = LIFT_STM_RESET;
	}
		break;
	case LIFT_ERROR_SERVICE: {
		if (handle_lift_service_error(serveStatus)) {
			liftStatus.liftRunStatus = LIFT_SEND_ERROR_MESSAGE;
		}
	}
		break;
	case LIFT_STM_RESET: {
		cancelAllcalls();
		initLiftStateMachine();
		resetLiftServeStateMachine();
		resetSpeedControlLogic();
//		setEVstate(EVO_CLOSE);
//		setMotorSpeed(MOTOR_STOP);
		liftStatus.liftRunStatus = LIFT_IDLE;
	}
		break;
	default: {
	}
		break;
	}
}

void processAutoFloorCalib() {
//	debug_print_n("processAutoFloorCalib \n");
	static uint32_t startTime;

	static uint32_t autocalibTimer;
	AUTO_MEASURE_STATE_SATUS status;
	// static AUTO_CALIB_APP_em passState, failState;
	//REQ_STATUS espRequest;
	switch (autoCalibState) {
	case AUTO_CALIB_IDLE: {
		if (readAutoCalibrationReq()) {
			if ((HAL_GetTick() - autocalibTimer) > AUTO_CALIB_ENABLE_TIME) {
				enCabDerailCheck = false;
				PidInitializeUpCall();
				cabinAppAutoCalibStateSet(false);
				autoCalibState = AUTO_CALIB_WAIT_DOOR_CLOSE;
				startTime = HAL_GetTick();
				autocalibTimer = HAL_GetTick();
			}

		} else {
			autocalibTimer = HAL_GetTick();
		}
	}
		break;

	case AUTO_CALIB_WAIT_DOOR_CLOSE: {
		if ((HAL_GetTick() - startTime) < AUTO_CALIB_DOOR_CLOSE_WAIT_TIME) {
			if (checkAllDoorLockClosed()) {
				autoCalibState = AUTO_CALIB_CHECK_INPUTS;
				startTime = HAL_GetTick();
			}
		} else {
			autoCalibState = AUTO_CALIB_ERROR;
		}
	}
		break;

	case AUTO_CALIB_CHECK_INPUTS: {
		if (checkLiftIsIdle() && MECH_LOCK_NO_ERROR == mechLockStatus()
				&& checkAllDoorLockClosed()) {
			autoCalibState = AUTO_CALIB_MSG_PROCESSING;
			startTime = HAL_GetTick();
		} else {
			autoCalibState = AUTO_CALIB_ERROR;
		}
	}
		break;

	case AUTO_CALIB_MSG_PROCESSING: {
		status = runAutoFloorMeasurement();
		// cabinUpdateFloorCalibStatus(status);
		if (status == AMS_SUCCESS) {
			autoCalibState = AUTO_CALIB_MSG_COMPLETE;
			startTime = HAL_GetTick();
		} else if (status == AMS_FAIL) {
			autoCalibState = AUTO_CALIB_MSG_ERROR;
			startTime = HAL_GetTick();
		} else {
		}
	}
		break;

	case AUTO_CALIB_MSG_COMPLETE: {
		autoCalibState = AUTO_CALIB_COMPLETE;
	}
		break;

	case AUTO_CALIB_MSG_ERROR: {
		autoCalibState = AUTO_CALIB_ERROR;
	}
		break;

	case AUTO_CALIB_ERROR:
	case AUTO_CALIB_COMPLETE: {
		autoCalibState = AUTO_CALIB_IDLE;
	}
		break;

	default:
		break;
	}
}

void resetLiftServeStateMachine() {
	liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
}

FLOOR_NUM getNextUpCall(uint8_t currentFloor) {
	for (uint8_t floorNum = currentFloor; floorNum < MAX_FLOOR; floorNum++) {
		if (checkCallRegisterd((FLOOR_NUM) floorNum, CALL_REGISTERED)) {
			return (FLOOR_NUM) floorNum;
		}
	}
	return MAX_FLOOR;
}

FLOOR_NUM getNextDownCall(uint8_t currentFloor) {
	for (uint8_t floorNum = currentFloor; floorNum == 0; floorNum--) {
		if (checkCallRegisterd((FLOOR_NUM) floorNum, CALL_REGISTERED)) {
			return (FLOOR_NUM) floorNum;
		}
	}
	return MAX_FLOOR;
}

void initLiftStatus() {
	memset(&liftStatus, false, sizeof(liftStatus));
}

CALL_SERVE_STS serveCallOnSameFloor(FLOOR_NUM floorNum) {
	CALL_SERVE_STS status;

	status = CALL_SERVE_STS_NOFAIL;

	switch (liftStatus.liftCallServeState) {
	case CALL_SERVICE_CHECK_LOCKS: {
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;
	case CALL_SERVICE_OPEN_DOOR: {
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
		openDoor(floorNum);
	}
		break;
		break;
	case CALL_SERVICE_COMPLETE: {
		liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
		status = CALL_SERVE_STS_COMPLETE;
	}
		break;

	default: {
	}
		break;
	}
	return status;
}

int getCallDirection() {
	debug_print_n("dest floor = %d current floor = %d \n",
			(int) liftStatus.destCallFloor,
			(int) liftStatus.CabinCurrentfloorNum);
	CALL_TYPE call_direction = NO_DIR;
	if (liftStatus.destCallFloor < liftStatus.CabinCurrentfloorNum) {
		call_direction = DOWN_DIR;
		//		return 2;
	} else if (liftStatus.destCallFloor > liftStatus.CabinCurrentfloorNum) {
		call_direction = UP_DIR;
		//		return 1;
	} else if (liftStatus.destCallFloor == liftStatus.CabinCurrentfloorNum) {
		call_direction = NO_DIR;
		//		return 0;
	}
	return call_direction;
}

MECH_LOCK_STATUS findCabinPostion(FLOOR_NUM *floorNum) {
	// mechanical lock - high
	// sensor check
	uint8_t lockCheckCounter;
	uint8_t mechanicalLockStatus;
	MECH_LOCK_STATUS mechLockStatus;
	static FLOOR_NUM previousFloorNum;
	FLOOR_NUM currentFloorNumber;
	mechLockStatus = MECH_LOCK_NO_ERROR;
	lockCheckCounter = 0;
	mechanicalLockStatus = false;
	for (uint8_t floors = 0; floors < MAX_FLOOR; floors++) {
		if (readMechanicalLock(floors) != MECH_LOCK_CLOSED) {
			SET_BIT_n(mechanicalLockStatus, floors);
			if (readMechanicalLock(floors) == MECH_LOCK_SEMI) {
				lockCheckCounter = lockCheckCounter + 2;
			} else {
				lockCheckCounter++;
			}
		}
	}

	currentFloorNumber = (FLOOR_NUM) getFloorNumberByLIDARValue();
	if ((currentFloorNumber != MAX_FLOOR)
			&& (previousFloorNum != currentFloorNumber)) {
		*floorNum = currentFloorNumber;
		previousFloorNum = currentFloorNumber;
		debug_print_n("Current floor number  = %d  \n", currentFloorNumber);
		setCabinFloorNumber(liftStatus.CabinCurrentfloorNum);
	} else {
		*floorNum = previousFloorNum;
	}
//	debug_print_n("Current floor number  = %d  \n", currentFloorNumber);
	if (lockCheckCounter == 0) {
		mechLockStatus = MECH_ERROR_NO_LOCK_OPENED;
	} else if (lockCheckCounter > 1) {
		mechLockStatus = MECH_ERROR_LOCK_OPEN_MANY;
	} else {
		mechLockStatus = MECH_LOCK_NO_ERROR;
	}
	//debug_print_n(" mechLockStatus ********** : %X \n ",mechLockStatus);
//	liftStatus.mechLockIOstate = mechanicalLockStatus;
	return mechLockStatus;
}

uint8_t mechLockGetIOstate() {
	return liftStatus.mechLockIOstate;
}

void mechIOState() {
	static uint8_t prevMechLock = 0;
	if (prevMechLock != liftStatus.mechLockIOstate) {
		//debug_print_n("mechIOState : %X \n",liftStatus.mechLockIOstate);
		prevMechLock = liftStatus.mechLockIOstate;
		set_mech_state(liftStatus.mechLockIOstate);
	}
}

uint8_t getMechLockStatus() {
//	debug_print_n("getMechLockStatus \n");
	int ml_status[MAX_FLOOR];
	static int prev_ml_status[MAX_FLOOR];
	uint8_t mech_lock_status = 0;
	for (int i = 0; i < MAX_FLOOR; i++) {
		ml_status[i] = readMechanicalLock(i);
//		debug_print_n(" %d ml_status[i] %X \n ",i,ml_status[i]);
		if (ml_status[i] != 0x02) {
			mech_lckerr = true;
		} else {
			mech_lckerr = false;
		}

		if (prev_ml_status[i] != ml_status[i]) {

			prev_ml_status[i] = ml_status[i];
			write_mech_led(i, ml_status[i]);
		}
		mech_lock_status =
				(uint8_t) (ml_status[i] << (2 * i) | mech_lock_status);

	}
//	if (HAL_GetTick() - timer_expired > 300) // changed from 300 to 500
//	{
//		debug_print_n("ml st1 %d  ml_st2 = %d mech_lock_status = %d \n", ml_status[0], ml_status[1], mech_lock_status);
//		set_mech_state(mech_lock_status);
//		timer_expired = HAL_GetTick();
//	}
//	debug_print_n(" mech_lock_status********************* %X \n ",mech_lock_status);
	if (mech_lock_prev != mech_lock_status) {

		debug_print_n("mech lock now %X mech_lock_prev %X  ****************\n",
				mech_lock_status, mech_lock_prev);
		mech_lock_prev = mech_lock_status;
		set_mech_state(mech_lock_status);
	}
	return mech_lock_status;
}

void upDateCabinFloorStatus() {
//	debug_print_n("upDateCabinFloorStatus \n");
	liftStatus.lckStatus = findCabinPostion(&liftStatus.CabinCurrentfloorNum);
	// updateLopFloorNumber(liftStatus.CabinCurrentfloorNum);
}

CALL_SERVE_STS serviceUpcall(uint8_t currentFloor, uint8_t destFloor) {
	CALL_SERVE_STS status;
	uint16_t distance;
	distance = getDistance();
	status = CALL_SERVE_STS_NOFAIL;

	bool door_status;
	bool mech_status;
	bool chlk_status;
	door_status = checkDoorAllClosed();
	mech_status = get_mech_lock_error(currentFloor, destFloor, UP_DIR);
	chlk_status = child_lock_input();
	if ((liftStatus.liftNearDestination == false) & chlk_status) {
		debug_print_n("Cl lock error");
		return CALL_SERVE_CHILD_LOCK_FAILURE;
	}
	if ((liftStatus.liftNearDestination == false) && !door_status) {
		debug_print_n("Door lock error");
		return CALL_SERVE_STS_SAFETY_FAILURE;
	}
	if ((liftStatus.liftNearDestination == false) && mech_status) {
		debug_print_n("mech lock error");
		return CALL_SERVE_STS_SAFETY_FAILURE;
	}
//	if(light_curtain_triggered())
//	{
//		debug_print_n("light curtain triggered");
//		return CALL_SERVE_CABIN_LIGHTCURTAIN_SAFETY;
//	}

	if (overload_state) {
		debug_print_n("DV OV error");
		return CALL_SERVE_STS_OVER_LOAD;
	}
	if (!lidar_state) {
		return CALL_SERVE_STS_LIDAR_FAIL;
	}
	if (power_state) {
		debug_print_n("power error");
		return CALL_SERVE_STS_POWER_FAILURE;
	}
	if (!getAndroidState()) {
		return CALL_SERVE_STS_ANDROID_FAILURE;
	}
	if (lop_disconnected()) {
		return CALL_SERVE_CAN_STS_SAFETY_FAILURE;
	}

	switch (liftStatus.liftCallServeState) {
	case CALL_SERVICE_CHECK_LOCKS: {
		debug_print_n("CALL_SERVICE_CHECK_LOCKS \n");
		liftStatus.liftCallServeState = CALL_SERVICE_CHK_CURRENT_FLR_RNG;
		liftStatus.liftNearDestination = false;
		liftStatus.startTime = HAL_GetTick();
		setLLState(false);
	}
		break;
	case CALL_SERVICE_CHK_CURRENT_FLR_RNG: {
		softStartCounter = 0;
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_FLOOR_RANGE(LIDARAtFloorPosition[currentFloor],
					FLOOR_RANGE_OFFSET_VAL, distance)) {
				liftStatus.liftCallServeState = CALL_SERVICE_SOFT_START;
			} else {
				status = CALL_SERVE_STS_CAB_DERAIL;
			}
		} else {
			status = CALL_SERVE_STS_LIDAR_FAIL;

		}
	}
		break;
	case CALL_SERVICE_SOFT_START:

		if (lidar_state) {
			distance = getDistance();
			if (CHECK_LIFT_IN_SOFT_ZONE(LIDARAtFloorPosition[currentFloor],
					distance, SOFT_ZONE_OFFESET))//soft start end at current floor + 300mm;
					{
				liftStatus.liftCallServeState = CALL_SERVICE_MAX_SPEED;
			} else {
				setMotorSpeed(6000);
//				softStartCounter++;
//				if (softStartCounter < 150) { // to remove ubnormal start
//					pidUpControl(2, 5, 0.1, 5, distance);
//				} else {
//				if()
//					pidUpControl(2, upcallSoftStart[0], 0.1, upcallSoftStart[1],
//							distance);
//				}
			}
		} else {
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}

		break;
	case CALL_SERVICE_MAX_SPEED: {
		//	debug_print_n("[upcall] CALL_SERVICE_MAX_SPEED \n");
		if (lidar_state) {
			distance = getDistance();
//			debug_print_n("[upcall] distance = %d \n", distance);
			if (CHECK_LIFT_ABOVE_BOTTOM(LIDARAtFloorPosition[destFloor],
					distance, UPCALL_DEST_OFFSET_CTRL)) {

				liftStatus.liftCallServeState = CALL_SERVICE_SOFT_LANDING;

			} else {
				bool speedControl = getSpeedPIDCtrlStatus(upcallMaxSpeed[1]);
				if (speedControl == CONTROL_REACHED) {
					pidUpControl(2, fixedPIDIVal, 0.1, upcallMaxSpeed[1],
							distance);
				} else {
					pidUpControl(2, upcallMaxSpeed[0], 0.1, upcallMaxSpeed[1],
							distance);
				}

			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;
	case CALL_SERVICE_SOFT_LANDING: {
		//	debug_print_n("[upcall] CALL_SERVICE_SOFT_LANDING \n");
		if (lidar_state) {
			distance = getDistance();

			if (((distance
					< (LIDARAtFloorPosition[destFloor] + UPCALL_DEST_OFFSET_CTRL))
					&& (distance > (LIDARAtFloorPosition[destFloor] + 100)))) {
				bool speedControl = getSpeedPIDCtrlStatus(upcallSoftLand1[1]);
				if (speedControl == CONTROL_REACHED) {
					pidUpControl(2, fixedPIDIVal, 0.1, upcallSoftLand1[1],
							distance);
				} else {
					pidUpControl(2, upcallSoftLand1[0], 0.1, upcallSoftLand1[1],
							distance);
				}

//				debug_print_n("[upcall] if CALL_SERVICE_SOFT_LANDING \n");
			} else {
				liftStatus.liftCallServeState = CALL_SERVICE_WAIT_FOR_TOP_POINT;

			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;
	case CALL_SERVICE_WAIT_FOR_TOP_POINT: {
		//	debug_print_n("[upcall] CALL_SERVICE_WAIT_FOR_TOP_POINT \n");
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_LIFT_ABOVE_TOP(LIDARAtFloorPosition[destFloor], distance,
					UPCALL_OFFSET_VALUE)) //|| distance < 80) //changed from 10
					{
				debug_print_n(
						"[upcall] top point reached *************** %d \n",
						distance);
				//	upCallOutputAdjusted = 9000; //MOTOR_STOP;
				liftStatus.liftCallServeState = CALL_SERVICE_STOP_MOTOR;
				liftStatus.espComRetries = 0;
			} else if (HAL_GetTick() - liftStatus.startTime > 6 * 1000) {
				status = CALL_SERVE_STS_CAB_DERAIL;
			} else {
				bool speedControl = getSpeedPIDCtrlStatus(upcallSoftLand2[1]);
				if (speedControl == CONTROL_REACHED) {
					pidUpControl(2, fixedPIDIVal, 0.1, upcallSoftLand2[1],
							distance);
				} else {
					pidUpControl(2, upcallSoftLand2[0], 0.1, upcallSoftLand2[1],
							distance);
				}
//				pidUpControl(2, upcallSoftLand2[0], 0.1, upcallSoftLand2[1],
//						distance);
			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			//      DEBUG(stateDebug, 56, "56: LIDAR read failed \r\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;

	case CALL_SERVICE_STOP_MOTOR: {
		debug_print_n("CALL_SERVICE_STOP_MOTOR : %d \n", upCallOutputAdjusted);
		liftStatus.liftCallServeState = CALL_SERVICE_MAX_VALID;
		resetSpeedControlLogic();
		setLLState(true);
		liftStatus.startTime = HAL_GetTick();
	}
		break;
	case CALL_SERVICE_MAX_VALID: {
		if (HAL_GetTick() - liftStatus.startTime > 1000) {
			liftStatus.startTime = HAL_GetTick();
			liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
		}
	}
		break;
	case CALL_SERVICE_SOFT_WAIT_TIME: {
		//	debug_print_n("CALL_SERVICE_SOFT_WAIT_TIME : %d \n",
//				upCallOutputAdjusted);
		//	if(distance <)
//		if ((distance > (LIDARAtFloorPosition[destFloor] - 50))
//				&& (distance < (LIDARAtFloorPosition[destFloor] - 20))) {
		pidUpControl(2, 20, 0.1, -1, distance);
//			debug_print_n("CALL_SERVICE_SOFT_WAIT_TIME : if ********** \n");
//		} else
		if ((distance > (LIDARAtFloorPosition[destFloor] - 70))
				&& (distance < (LIDARAtFloorPosition[destFloor] - 30))) {
			resetSpeedControlLogic();
			liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
			liftStatus.startTime = HAL_GetTick();
//			debug_print_n("CALL_SERVICE_SOFT_WAIT_TIME : else if ** \n");
		}
//			else if(distance < (LIDARAtFloorPosition[destFloor]-50))
//			{
//				//resetSpeedControlLogic();
//			}
		else if (distance < (LIDARAtFloorPosition[destFloor] - 70)) {
			resetSpeedControlLogic();
		} else {
			resetSpeedControlLogic();
//			debug_print_n("CALL_SERVICE_SOFT_WAIT_TIME :(** else ** ** \n");
		}

		//	resetSpeedControlLogic();
		//	setLLState(true);

	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (lidar_state) {
			distance = getDistance();
//			pidUpControl(2, 10, 0.1, -3, distance);
			//	debug_print_n("[upcall]CALL_SERVICE_TO_DESTINATION\n");
			if (CHECK_FLOOR_RANGE(
					LIDARAtFloorPosition[liftStatus.destCallFloor],
					DEST_FLOOR_RANGE_OFFSET_VAL, distance)) {
				debug_print_n("[upcall] cabin within landing range\n");
				resetSpeedControlLogic();
				liftStatus.liftCallServeState = CALL_SERVICE_CABIN_DEST;
			} else if (HAL_GetTick() - liftStatus.startTime > 6 * 1000) {
//				debug_print_n("[upcall] cabin out of landing range\n");
				status = CALL_SERVE_STS_CAB_DERAIL;
			} else {
				debug_print_n(
						"inside pid control CALL_SERVICE_TO_DESTINATION %d\n",
						upCallOutputAdjusted);
//				setMotorSpeed(8200);
//				pidUpControl(2, 100, 0.1, -5, distance);
			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			debug_print_n("[upcall] lidar fail\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
			liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
		}
	}
		break;

	case CALL_SERVICE_CABIN_DEST: {
		liftStatus.startTime = HAL_GetTick();
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		liftStatus.liftNearDestination = true;
	}
		break;
	case CALL_SERVICE_OPEN_DOOR: {
		debug_print_n("[upcall] CALL_SERVICE_OPEN_DOOR : %d\n", destFloor);
		openDoor((FLOOR_NUM) destFloor);
		liftStatus.liftCallServeState = CALL_SERVICE_LIFT_STOPPED;
	}
		break;

	case CALL_SERVICE_LIFT_STOPPED: {
		AUTO_FLR_STS_em lift_status = callserve_CheckLiftStopped();
		if (AUTO_FLR_STS_COMPLETE == lift_status) {
			liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_DOOR;
		} else if (AUTO_FLR_STS_ERROR == lift_status) {
			status = CALL_SERVE_STS_LIDAR_FAIL;
			liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
		} else {
		}
	}
		break;

	case CALL_SERVICE_CLOSE_DOOR: {
		debug_print_n("CALL_SERVICE_CLOSE_DOOR \n ");
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;
	case CALL_SERVICE_COMPLETE: {
		debug_print_n("CALL_SERVICE_COMPLETE \n ");
		status = CALL_SERVE_STS_COMPLETE;
	}
		break;

//	case CALL_SERVICE_MAX_SPEED: {
//		liftStatus.liftCallServeState = CALL_SERVICE_MAX_VALID;
//		distance = getDistance();
//		upCallKp = 2, upCallKi = 50, upCallKd = 0.1;
//		SetTuningsUpCall(upCallKp, upCallKi, upCallKd, P_ON_E);
//		upCallSetpoint = 30;
//		getVelocity(distance);
//		ComputeUpCall();
//		setMotorPWMDutyCycle(upCallOutputAdjusted);
//		if (CHECK_LIFT_ABOVE_BOTTOM(LIDARAtFloorPosition[destFloor], distance,
//				UPCALL_DEST_OFFSET_CTRL))	//lidar_value < 2200) {
//				{
//			liftStatus.liftCallServeState = CALL_SERVICE_MAX_SPEED;
//		}
////		if (currentFloor == TOP_FLOOR - 1) {
////			setMotorControlSpeedMotor(2, CONTROL_UP_DIR, (int16_t) 25);
////		} else {
////			setMotorControlSpeedMotor(2, CONTROL_UP_DIR, (int16_t) 25);
////		}
//	}
//		break;
//		case CALL_SERVICE_MAX_VALID:
//		{
//			if (getControlStatus() == CONTROL_ACHIVED) {
//				printf("[upcall] speed control achieved \n");
//				liftStatus.liftCallServeState =
//						CALL_SERVICE_WAIT_FOR_BOTTOM_POINT;
//			} else if (getControlStatus() == CONTROL_ERROR) {
//				printf("[upcall] speed control failed \n");
//				status = CALL_SERVE_STS_OVER_LOAD;
//			}
//		}
//		break;

//		case CALL_SERVICE_FLOOR_UPDATE:
//		{
//			// send floor number to cabin
//			liftStatus.liftCallServeState = CALL_SERVICE_MAX_SPEED;
//		}
//		break;

//		case CALL_SERVICE_SOFT_WAIT_TIME:
//		{
////		setMotorControlSpeed(CONTROL_UP_DIR, 20);
//			liftStatus.liftCallServeState = CALL_SERVICE_SOFT_LANDING;
//		}
//		break;

//		case CALL_SERVICE_SOFT_LANDING:
//		{
//			if (lidar_state) {
//				distance = getDistance();
//
//				if (getControlStatus() == CONTROL_ACHIVED) {
//					liftStatus.startTime = HAL_GetTick();
//					if (CHECK_LIFT_ABOVE_TOP(LIDARAtFloorPosition[destFloor],
//							distance, TOP_FLOOR_STOP_OFFSET)) {
//						liftStatus.liftCallServeState = CALL_SERVICE_STOP_MOTOR;
//					} else {
//						liftStatus.liftCallServeState =
//								CALL_SERVICE_SOFT_WAIT_TIME;
//					}
//				} else if (getControlStatus() == CONTROL_ERROR) {
//					//        setOverloadType(2);
//					status = CALL_SERVE_STS_OVER_LOAD;
//				} else {
//					if (CHECK_LIFT_ABOVE_TOP(LIDARAtFloorPosition[destFloor],
//							distance, TOP_FLOOR_STOP_OFFSET)) {
//						liftStatus.liftCallServeState = CALL_SERVICE_STOP_MOTOR;
//					}
//				}
//			} else {
//				status = CALL_SERVE_STS_LIDAR_FAIL;
//			}
//		}
//		break;

	default:
		break;
	}
	return status;
}

FLOOR_NUM getCabinFloorNumber() {
	return liftStatus.CabinCurrentfloorNum;
}

CALL_SERVE_STS serviceDowncall(uint8_t currentFloor, uint8_t destFloor) {
	CALL_SERVE_STS status;
	uint16_t distance;
	status = CALL_SERVE_STS_NOFAIL;
	bool door_status;
	bool mech_status;
	bool chlk_status;
//	int n = snprintf(buf, sizeof(buf), "serviceDowncall \n");
//	debug_print(buf, n);

	door_status = checkDoorAllClosed();
	mech_status = get_mech_lock_error(currentFloor, destFloor, DOWN_DIR);
	chlk_status = child_lock_input();
	if ((liftStatus.liftNearDestination == false) & chlk_status) {
		debug_print_n("Child lock error");
		return CALL_SERVE_CHILD_LOCK_FAILURE;
	}
	if ((liftStatus.liftNearDestination == false) && !door_status) {
		debug_print_n("Door lock error");
		return CALL_SERVE_STS_SAFETY_FAILURE;
	}
	if ((liftStatus.liftNearDestination == false) && mech_status) {
		debug_print_n("mech lock error");
		return CALL_SERVE_STS_SAFETY_FAILURE;
	}
	if (lop_disconnected()) {
		return CALL_SERVE_CAN_STS_SAFETY_FAILURE;
	}
// bool ov_state = false;
	if (overload_state) {
		// setOverloadType(ov_state);
		debug_print_n("OV device error");
		return CALL_SERVE_STS_OVER_LOAD;
	}
	if (liftStatus.liftMovingStarted == true && callServeCheckLiftStopped()) {
//    setOverloadType(2);
		return CALL_SERVE_STS_OVER_LOAD;
	}
	if (power_state) {
		debug_print_n("power error");
		return CALL_SERVE_STS_POWER_FAILURE;
	}
	if (!lidar_state) {
		return CALL_SERVE_STS_LIDAR_FAIL;
	}
	if (!getAndroidState()) {
		return CALL_SERVE_STS_ANDROID_FAILURE;
	}
//	if(light_curtain_triggered())
//	{
//		debug_print_n("light curtain triggered");
//		return CALL_SERVE_CABIN_LIGHTCURTAIN_SAFETY;
//	}
	switch (liftStatus.liftCallServeState) {
	case CALL_SERVICE_CHECK_LOCKS: {
		liftStatus.liftNearDestination = false;
		liftStatus.liftMovingStarted = false;
		liftStatus.liftCallServeState = CALL_SERVICE_CHK_CURRENT_FLR_RNG;
		setLLState(false);
		liftStatus.startTime = HAL_GetTick();
		debug_print_n("[downcall] setLLState(false) : \n", false);

	}
		break;
	case CALL_SERVICE_CHK_CURRENT_FLR_RNG: {
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_FLOOR_RANGE(LIDARAtFloorPosition[currentFloor],
					FLOOR_RANGE_OFFSET_VAL, distance)) {
				softStartCounter = 0;
				liftStatus.liftCallServeState = CALL_SERVICE_LIFT_UP;
				//debug_print_n("[downcall] call service min speed \n");
			} else {
				//debug_print_n("[downcall] cab derail error\n");
				status = CALL_SERVE_STS_CAB_DERAIL;
			}
		} else {
			//	debug_print_n("[downcall] lidar fail\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;
	case CALL_SERVICE_LIFT_UP: { //lift up for LL disengage
		///debug_print_n("[downcall] CALL_SERVICE_LIFT_UP \n");
		if (lidar_state) {
			distance = getDistance();
			getVelocity(distance);
			//	debug_print_n("[downcall] lidar distance : %d \n", distance);
			if (distance
					> (LIDARAtFloorPosition[currentFloor]
							- LIFT_UP_DOWNCALL_OFFSET)) //|| (distance > 150))
					{
				setMotorSpeed(6000);
//				softStartCounter++;
//				if (softStartCounter < 50) {
//					pidUpControl(2, 5, 0.1, 5, distance);
//				} else {
//
//					//	debug_print_n("[downcall] if distance > \n");
//					pidUpControl(2, downcallSoftStart[0], 0.1,
//							downcallSoftStart[1], distance);
//
//					liftStatus.startTime = HAL_GetTick();
//				}

			} else {
				debug_print_n("[downcall] else distance < \n");
				//upCallOutputAdjusted = MOTOR_STOP;
				setMotorSpeed(MOTOR_STOP);
				liftStatus.liftCallServeState = CALL_SERVICE_EVO_OPEN;
				liftStatus.startTime = HAL_GetTick();
			}
		} else {
			debug_print_n("[downcall] lidar fail\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}

//		debug_print_n("[downcall] set upward speed\n");
		//	setMotorControlSpeed(CONTROL_UP_DIR, (int16_t) 10);
//		liftStatus.liftCallServeState = CALL_SERVICE_MIN_VALID;

	}
		break;

	case CALL_SERVICE_EVO_OPEN:
//		debug_print_n("inside CALL_SERVICE_EVO_OPEN \n");
		if (HAL_GetTick() - liftStatus.startTime > 1000) {
			liftStatus.liftMovingStarted = true;
			//	setEvoServoDuty(EVO_SERVO_OPEN);
			debug_print_n("******** CALL_SERVICE_EVO_OPEN \n");
#ifdef SERVO_ENABLE
				setEvoServoDuty(EVO_SERVO_OPEN);
#else
			setEVstate(EVO_OPEN);
#endif

			liftStatus.liftCallServeState = CALL_SERVICE_RUN_MOTOR;
			//debug_print_n("[downcall] run evo \n");
		}
		break;

	case CALL_SERVICE_LIFTUP_OFFSET: {
		debug_print_n("inside CALL_SERVICE_LIFTUP_OFFSET \n");
		if (lidar_state) {
			distance = getDistance();

			if (CHECK_LIFT_BELOW_OFFSET(LIDARAtFloorPosition[currentFloor],
					distance, LIFT_UP_FLOOR_OFFSET)) {
//				if (get_mech_lock_error(currentFloor, destFloor, DOWN_DIR)) {
//					debug_print_n(
//							"[downcall] mech lock error above lift off at %dcm\n",
//							distance);
//					status = CALL_SERVE_STS_SAFETY_FAILURE;
//				}
			}
			liftStatus.liftCallServeState = CALL_SERVICE_RUN_MOTOR;
			liftStatus.startTime = HAL_GetTick();
//			n = snprintf(buf, sizeof(buf), "downcall call service max speed \n");
//			debug_print(buf, n);
		} else {
			debug_print_n("[downcall] lidar fail\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;
	case CALL_SERVICE_RUN_MOTOR: {
		if (HAL_GetTick() - liftStatus.startTime > 1000) {
			liftStatus.liftMovingStarted = true;
			//setEVstate(EVO_OPEN);
			liftStatus.liftCallServeState = CALL_SERVICE_MAX_SPEED;
			debug_print_n("[downcall] CALL_SERVICE_RUN_MOTOR \n");
		}
	}
		break;

	case CALL_SERVICE_MAX_SPEED: {
//		debug_print_n("inside CALL_SERVICE_MAX_SPEED \n");
		distance = getDistance();
		//	getVelocity(distance);
		bool speedControl = getSpeedPIDCtrlStatus(downcallMaxSpeed[1]);
		if (speedControl == CONTROL_REACHED) {
			pidUpControl(2, fixedPIDIVal, 0.1, downcallMaxSpeed[1], distance);
		} else {
			pidUpControl(2, downcallMaxSpeed[0], 0.1, downcallMaxSpeed[1],
					distance);
		}

//		if (getControlStatus() == CONTROL_ACHIVED) {
//			if ((LIDARAtFloorPosition[destFloor] - DOWNCALL_MOTORCTRL_DIS_OFFSET)
//					< distance) {
//				debug_print_n("[downcall] cabin reached 1m aboce dest\n");
//				liftStatus.liftCallServeState = CALL_SERVICE_WAIT_FOR_TOP_POINT;
//				liftStatus.startTime = HAL_GetTick();
//			}
//		}
		if ((LIDARAtFloorPosition[destFloor] - DOWNCALL_MOTORCTRL_DIS_OFFSET)
				< distance) {
			liftStatus.liftCallServeState = CALL_SERVICE_WAIT_FOR_TOP_POINT;
			liftStatus.startTime = HAL_GetTick();
		} else if (getControlStatus() == CONTROL_ERROR) {
			//      setOverloadType(2);
			status = CALL_SERVE_STS_OVER_LOAD;
		}
	}
		break;

//	case CALL_SERVICE_MIN_VALID: {
//		if (lidar_state) {
//			distance = getDistance();
//			getVelocity(distance);
//			pidUpControl(2, 50, 0.1, -15, distance);
//			if (distance > 0 && distance < 2000) {
//				if ((HAL_GetTick() - liftStatus.startTime
//						> (uint32_t) (OVERLOAD_DETECTION_TIME * 1000))
//						|| (getControlStatus() == CONTROL_ERROR)) {
////					setOverloadType(2);
//					debug_print_n("[downcall error] user overload 1\n");
//					status = CALL_SERVE_STS_OVER_LOAD;
//				} else if (CHECK_LIFT_MOVED_UP_OFFSET(
//						LIDARAtFloorPosition[currentFloor], distance,
//						LIFT_UP_FLOOR_OFFSET) || (distance < 12)) {
//					debug_print_n(
//							"[downcall] lift moved 10cm above current floor \n");
//					liftStatus.liftCallServeState = CALL_SERVICE_RUN_MOTOR;
//					resetSpeedControlLogic();
//					pidUpControl(2, 20, 0.1, -1, distance);
//					liftStatus.startTime = HAL_GetTick();
//				} else {
////					debug_print_n("[downcall error] user overload 2\n");
////					status = CALL_SERVE_STS_OVER_LOAD;
//				}
//			} else {
//				debug_print_n("[downcall error] user overload 3\n");
//				status = CALL_SERVE_STS_OVER_LOAD;
//			}
//		} else {
////			      DEBUG(stateDebug, 92, "92: Lidar fail\r\n");
//			debug_print_n("[downcall] lidar fail\n");
//			status = CALL_SERVE_STS_LIDAR_FAIL;
//		}
//	}
//		break;

	case CALL_SERVICE_WAIT_FOR_TOP_POINT: {
		if (lidar_state) {
			distance = getDistance();
			if ((LIDARAtFloorPosition[destFloor] - distance)
					< DOWNCALL_TOP_POINT_OFFSET[destFloor]) {
				debug_print_n("[downcall] top point reached **************\n");
				//	upCallOutputAdjusted = 9000; //MOTOR_STOP;
				setEVstate(EVO_CLOSE);
				liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_LANDING_GEAR;
				liftStatus.espComRetries = 0;
			} else if (HAL_GetTick() - liftStatus.startTime > 6 * 1000) {
				status = CALL_SERVE_STS_CAB_DERAIL;
			} else {
				bool speedControl = getSpeedPIDCtrlStatus(downcallSoftLand1[1]);
				if (speedControl == CONTROL_REACHED) {
					pidUpControl(2, fixedPIDIVal, 0.1, downcallSoftLand1[1],
							distance);
				} else {
					pidUpControl(2, downcallSoftLand1[0], 0.1,
							downcallSoftLand1[1], distance);
				}
//				pidUpControl(2, downcallSoftLand1[1], 0.1, downcallSoftLand1[1],
//						distance);
			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			//      DEBUG(stateDebug, 56, "56: LIDAR read failed \r\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}

	}
		break;

	case CALL_SERVICE_CLOSE_LANDING_GEAR: {
		debug_print_n("[downcall] engage landing lever\n");
		setLLState(true);
		liftStatus.liftCallServeState = CALL_SERVICE_SOFT_WAIT_TIME;
	}
		break;
	case CALL_SERVICE_SOFT_WAIT_TIME: {
		if (lidar_state) {
			debug_print_n("[downcall]CALL_SERVICE_SOFT_WAIT_TIME ****\n");
			distance = getDistance();
//			bool speedControl = getSpeedPIDCtrlStatus(downcallSoftLand2[1]);
//			if (speedControl == CONTROL_REACHED) {
//				pidUpControl(2, fixedPIDIVal, 0.1, downcallSoftLand2[1],
//						distance);
//			} else {
//				pidUpControl(2, downcallSoftLand2[0], 0.1, downcallSoftLand2[1],
//						distance);
//			}
//			pidUpControl(2, downcallSoftLand2[0], 0.1, downcallSoftLand2[1],
//					distance);
			if ((LIDARAtFloorPosition[destFloor] - DOWNCALL_SOFTLAND_DIS_OFFSET)
					< distance) {
				resetSpeedControlLogic();
				debug_print_n("[downcall] reset lift logic\n");
				liftStatus.espComRetries = 0;
				liftStatus.liftCallServeState = CALL_SERVICE_SOFT_LANDING;
			} else if (HAL_GetTick() - liftStatus.startTime > 10 * 1000) {
				status = CALL_SERVE_STS_CAB_DERAIL;
			} else {
			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			status = CALL_SERVE_STS_LIDAR_FAIL;
		}
	}
		break;

	case CALL_SERVICE_SOFT_LANDING: {
		debug_print_n("[downcall] timeout for soft land\n");
		liftStatus.liftMovingStarted = false;
		liftStatus.liftNearDestination = true;
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_FLOOR_RANGE(
					LIDARAtFloorPosition[liftStatus.destCallFloor],
					DEST_FLOOR_RANGE_OFFSET_VAL, distance)) {
				debug_print_n("[]\n");
				liftStatus.liftCallServeState = CALL_SERVICE_CABIN_DEST;
			}
			liftStatus.startTime = HAL_GetTick();
		} else {
			//      DEBUG(stateDebug, 64, "64: lidar fail \r\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
			liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
		}
	}
		break;

	case CALL_SERVICE_CABIN_DEST: {
		if ((currentFloor == destFloor)
				|| (HAL_GetTick() - liftStatus.startTime > 1000)) {
			/** slows down the speed */
			//setEVstate(EVO_CLOSE);
			//	set_motor_count(MSPEED_STOP);
			resetSpeedControlLogic();
			liftStatus.startTime = HAL_GetTick();
			//      DEBUG(stateDebug, 65, "65: dest \r\n");
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		debug_print_n("[downcall] open door solenoid\n");
		openDoor((FLOOR_NUM) destFloor);
		liftStatus.liftCallServeState = CALL_SERVICE_LIFT_STOPPED;
		//    DEBUG(stateDebug, 66, "66: lift open door ... \r\n");
	}
		break;

	case CALL_SERVICE_LIFT_STOPPED: {
		AUTO_FLR_STS_em lift_status = callserve_CheckLiftStopped();
		if (AUTO_FLR_STS_COMPLETE == lift_status) {
			debug_print_n("[downcall] wait for lift to stop\n");
			liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_DOOR;
		} else if (AUTO_FLR_STS_ERROR == lift_status) {
			//      DEBUG_SAFTY(debugfailure, 107, "107: lift stoped Error \r\n");
			status = CALL_SERVE_STS_LIDAR_FAIL;
			liftStatus.liftCallServeState = CALL_SERVICE_CHECK_LOCKS;
		} else {
			// wait
		}
	}
		break;

	case CALL_SERVICE_CLOSE_DOOR: {
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;
	case CALL_SERVICE_COMPLETE: {
		status = CALL_SERVE_STS_COMPLETE;
	}
		break;
	default:
		break;
	}
	return status;
}

/**
 *
 */
bool handle_lift_service_error(CALL_SERVE_STS sts) {
	bool status;
	status = false;
	switch (sts) {
	case CALL_SERVE_STS_NOFAIL: {
		status = true;
	}
		break;
	case CALL_SERVE_STS_DOORLOCK_FAIL: {
		if (handleDoorOpenTooLong()) {
			debug_print_n("Door open too long\n");
			status = true;
		}
	}
		break;

	case CALL_SERVE_STS_SAFETY_FAILURE: {
		if (handleSafetyFailure()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_DEVICE_UNDER_UPDATE: {
		if (handleSafetyFailure()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_CHILD_LOCK_FAILURE: {
		if (handleSafetyFailure()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_CAN_STS_SAFETY_FAILURE: {
		if (handleCANSafetyFailure(CALL_SERVE_CAN_STS_SAFETY_FAILURE)) {
			status = true;
		}
	}
		break;

	case CALL_SERVE_STS_OVER_LOAD: {
		if (handleOverloadSafetyFailure()) {
			status = true;
		}
	}
		break;

	case CALL_SERVE_CABIN_LIGHTCURTAIN_SAFETY: {
		if (handle_light_curtain_safety()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_STS_POWER_FAILURE: {
		if (handleCANSafetyFailure(CALL_SERVE_STS_POWER_FAILURE)) {
			status = true;
		}
	}

		break;
	case CALL_SERVE_STS_ANDROID_FAILURE: {
		if (handleCANSafetyFailure(CALL_SERVE_STS_ANDROID_FAILURE)) {
			status = true;
		}
	}
		break;
//	case CALL_SERVE_STS_LL_FAIL:
//	{
//		if (handleLandingLeverFailure())
//		{
//			status = true;
//		}
//	}
//	break;
	case CALL_SERVE_STS_LIDAR_FAIL: {
		if (handleLIDARfailure()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_STS_CAB_DERAIL: {
		if (handleCabinDerailfailure()) {
			status = true;
		}
	}
		break;
	case CALL_SERVE_STS_OVER_SPEED: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		status = true;
	}
		break;
	case CALL_SERVE_STS_ML_FAIL_BEFORE_CALL: {
		if (handleMechLockFailureOnCallReg()) {
			status = true;
		}
	}
		break;

	case CALL_SERVE_STS_ESP_COM_FAILURE: {
		if (handleEspComFailure()) {
			status = true;
		}
	}
		break;

	case CALL_SERVE_STS_COMPLETE: {
		status = true;
	}
		break;
	default: {
		status = true;
	}
		break;
	}
	return status;
}

bool handleClearCall() {
	uint32_t cabinCalls;
	bool status;
	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		liftStatus.liftCallServeState = CALL_SERVICE_EEROR_CANCEL_CALLS;
		liftStatus.startTime = HAL_GetTick();
		setLOPCallBookedMessage(MAX_FLOOR);
		setEVstate(EVO_CLOSE);
	}
		break;
	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		cabinCalls = false;
		// door open message to cabin
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
		callClearFail = true;
		failedTimer = HAL_GetTick();
		callClearFailedValue = cabinCalls;
		setLLState(false);
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		status = true;
	}
		break;
	}
	return status;
}

void cancelAllRegCallsInShaftandCabin() {
	cancelAllcalls();
// Cancel call update is a broadcast message.
}

void handleEmergency() {
	static EMERGENCY_STATES emergencyState;
	uint16_t distance;
	static uint16_t emergencyInputTime;
	bool stop_status = false;
//	CALL_SERVE_STS preChecked = preCheck();
//	debug_print_n("EMGCY_STATE_WAIT_FOR_RELEASE : %d\n ",preChecked);
	switch (emergencyState) {
	case EMGCY_STATE_IDLE: {
		emergencyState = EMGCY_STATE_CHECKS;
		emergencyInputTime = 0;
	}
		break;
	case EMGCY_STATE_CHECKS: {
		distance = getDistance();
		if (((getEmergencyInput() == true) && isLiftIdle()
				&& !child_lock_input() && checkDoorAllClosed()
				&& (liftStatus.lckStatus != MECH_LOCK_NO_ERROR))) {
			if (emergencyInputTime++ > 10) {
				emergencyInputTime = 0;
				emergencyState = EMGCY_STATE_UNLOCK_LL;
				liftStatus.emergencyOn = true;
				liftStatus.startTime = HAL_GetTick();
			}
		} else {
			emergencyState = EMGCY_STATE_IDLE;
		}
	}
		break;
	case EMGCY_STATE_UNLOCK_LL: {
		setLLState(false);
		emergencyState = EMGCY_STATE_RUN_MOTOR;
	}
		break;

	case EMGCY_STATE_RUN_MOTOR: {
		EmergencyOutput_On();
		setMotorSpeed(5000);
		//set_motor_count(MOTOR_FIVE);
		emergencyState = EMGCY_STATE_WAIT_FOR_RELEASE;
		//    DEBUG_EM(estateDebug, 6, "6:run motor\r\n");
	}
		break;
	case EMGCY_STATE_WAIT_FOR_RELEASE: {
		distance = getDistance();
		bool mech_l = false;
//		uint8_t mech_lck = get_mech_lock_error(currentFloor, destFloor, UP_DIR);
//		 if((liftStatus.liftNearDestination == false) && mech_lck)
//		{
//			mech_l = true;
//		}
		debug_print_n(
				"getEmergencyInput %d child_lock_input %d mech_lckerr %d power_state %d\n",
				getEmergencyInput(), child_lock_input(), mech_lckerr,
				power_state);
		if ((false == getEmergencyInput()) || child_lock_input() || power_state
				|| EMG_FLOOR_RANGE_OFFSET_VAL >= distance) {
			if ( EMG_FLOOR_RANGE_OFFSET_VAL >= distance) {
				debug_print_n("maximum top point emergency reached\n");
				liftStatus.emergencyLastTime = HAL_GetTick();
			}
			stop_status = true;
			debug_print_n("inside false == getEmergencyInput() \n");
		}
		if (stop_status) {
			liftStatus.emergencyLastTime = HAL_GetTick();
			emergencyState = EMGCY_STATE_FLOOR_LVL;
			setMotorSpeed(MOTOR_STOP);
			//set_motor_count(MSPEED_STOP);
			EmergencyOutput_Off();
		}
	}
		break;
	case EMGCY_STATE_FLOOR_LVL: {
		emergencyState = EMGCY_STATE_DEST;
		setLLState(true);
	}
		break;

	case EMGCY_STATE_DEST: {
		if (callServeCheckLiftStopped()) {
			emergencyState = EMGCY_STATE_OPEN_DOOR;
		}
	}
		break;

	case EMGCY_STATE_OPEN_DOOR: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		emergencyState = EMGCY_STATE_RESET;
	}
		break;

	case EMGCY_STATE_RESET: {
		initLiftStateMachine();
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		liftStatus.emergencyOn = false;
		EmergencyOutput_Off();
		emergencyState = EMGCY_STATE_IDLE;
		setLLState(false);
	}
		break;
	default: {
	}
		break;
	}

//  uint8_t emInp = readEmergencyCabinInput();
//  static uint8_t prevEmInp;
//  if (prevEmInp != emInp)
//  {
//    prevEmInp = emInp;
//  }
}

bool get_mech_lock_error1(uint8_t currentFlr) {
	uint16_t distance;
	distance = getDistance();
	if (CHECK_LIFT_ABOVE_OFFSET(LIDARAtFloorPosition[currentFlr], distance,
			MECHLOCK_CHANGE_DIS)) {
		if (readMechanicalLock((FLOOR_NUM) currentFlr) != MECH_LOCK_CLOSED) {
			return true;
		}
	}
	return false;
}

bool get_mech_lock_error(uint8_t currentFlr, uint8_t destFloor,
		CALL_TYPE direction) {
	uint16_t distance;
	distance = getDistance();

	for (uint8_t floor = 0; floor < MAX_FLOOR; floor++) {
		// skip current floor untill the floor moved offset.

		if (direction == UP_DIR) {
			if (floor == currentFlr) {
				// check lift have moved offset
				if (CHECK_LIFT_MOVED_UP_OFFSET(LIDARAtFloorPosition[floor],
						distance, UPCALL_TOP_POINT_OFFSET_MODIFIED)) {
					if (readMechanicalLock(
							(FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
						return true;
					}
				}
			} else if (floor == destFloor) {
				if (CHECK_FLOOR_RANGE(LIDARAtFloorPosition[floor],
						FLOOR_RANGE_OFFSET_VAL, distance)) {
// SKIP ERROR
				} else {
					if (readMechanicalLock(
							(FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
						// Serial.println("out of offset destination");
						return true;
					}
				}
			} else {
				if (readMechanicalLock((FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
					return true;
				}
			}
		} else {
			if (floor == currentFlr) {
				// check lift have moved offset
				if (CHECK_LIFT_MOVED_UP_OFFSET(LIDARAtFloorPosition[floor],
						distance, UPCALL_LIFT_MOVED_OFFSET) ||
						CHECK_LIFT_BELOW_OFFSET(LIDARAtFloorPosition[floor],
								distance, UPCALL_LIFT_MOVED_OFFSET)) {
					if (readMechanicalLock(
							(FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
						//            DEBUG_PRINT_1("*4>failed here =%d  \r\n", floor);
						return true;
					}
				}
			} else if (floor == destFloor) {
				if (CHECK_FLOOR_RANGE(LIDARAtFloorPosition[floor], distance,
						FLOOR_RANGE_OFFSET_VAL)) {
// SKIP ERROR
				} else {
					if (readMechanicalLock(
							(FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
						// Serial.println("out of offset destination");
						return true;
					}
				}
			} else {
				if (readMechanicalLock((FLOOR_NUM) floor) != MECH_LOCK_CLOSED) {
//          DEBUG_PRINT_1("*4failed here =%d  \r\n", floor);
					return true;
				}
			}
		}
	}
	return false;
}

uint16_t debugStop;

AUTO_FLR_STS_em callserve_CheckLiftStopped() {
	uint16_t lidarDistance;
	uint16_t lidarDiff;
	static uint32_t prevStateTime;
	static uint16_t prevLidarDistance;
	static uint8_t validateCount;

//  DEBUG_AUTO(debugStop, 6, "check LIFT stopped \r\n");
//debug_print_n("check LIFT stopped *******  \n");
	if (validateCount < LIFT_STOPPED_CHECK_MAX_COUNT) {
		if (HAL_GetTick() - prevStateTime > 1000) {
//debug_print_n("(HAL_GetTick() - prevStateTime > 1000)  \n");
			prevStateTime = HAL_GetTick();
			if (lidar_state) {
				lidarDistance = getDistance();
			} else {
				prevStateTime = HAL_GetTick();
				prevLidarDistance = false;
				validateCount = false;
				//        DEBUG_STOP_LIFT("1>Lidat Error \r\n");
				return AUTO_FLR_STS_ERROR;
			}

			lidarDiff =
					(prevLidarDistance > lidarDistance) ?
							(prevLidarDistance - lidarDistance) :
							(lidarDistance - prevLidarDistance);
			prevLidarDistance = lidarDistance;
			if (lidarDiff < LIDR_DIFF_VAL_ON_LIFT_STOPPED) {
				validateCount++;
			} else {
				validateCount = 0;
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

bool callServeCheckLiftStopped() {
	uint16_t lidarDistance;
	uint16_t lidarDiff;
	static uint16_t prevLidarDistance;
	static uint8_t validateCount;
	static uint32_t prevStateTime;

	if (validateCount < LIFT_STOPPED_CHECK_MAX_COUNT) {
		if (HAL_GetTick() - prevStateTime > 1000) {
			prevStateTime = HAL_GetTick();
			lidarDistance = getDistance();

			lidarDiff =
					(prevLidarDistance > lidarDistance) ?
							(prevLidarDistance - lidarDistance) :
							(lidarDistance - prevLidarDistance);
			prevLidarDistance = lidarDistance;

			// DEBUG_PRINT_1("lidarDiff =%d \r\n",lidarDiff);
			if (lidarDiff < LIDR_DIFF_VAL_ON_LIFT_STOPPED) {
				validateCount++;
			} else {
				validateCount = false;
			}
		}
	} else {
		prevLidarDistance = false;
		validateCount = false;
		prevStateTime = false;
		prevLidarDistance = false;
		return true;
	}
	return false;
}

uint8_t getFloorNumberByLIDARValue() {
	uint16_t lidarDistance;
	lidarDistance = getDistance();
	for (uint8_t floorNubmer = false; floorNubmer < MAX_FLOOR; floorNubmer++) {
		if (CHECK_FLOOR_RANGE(LIDARAtFloorPosition[floorNubmer],
				FLOOR_RANGE_OFFSET_VAL, lidarDistance)) {
			return floorNubmer;
		}
	}
	return (uint8_t) MAX_FLOOR;
}

bool checkLiftIsIdle() {
	if (liftStatus.liftRunStatus < LIFT_PREDICT_DIR) {
		return true;
	}
	return false;
}

MECH_LOCK_STATUS mechLockStatus() {
	return liftStatus.lckStatus;
}

REQ_STATUS sendFloorNumOnReq() {
	REQ_STATUS espRequest = ESP_CMD_REQ_STATUS_COMPLETE; // = espCabinRequest(&sendFlooorNumberToCabin, (uint32_t)liftStatus.CabinCurrentfloorNum);
	return espRequest;
}

REQ_STATUS updateFloorNumberMsgToCabin() {
	REQ_STATUS espRequest;
	if (liftStatus.MsgCabinfloorNum != liftStatus.CabinCurrentfloorNum) {
		// send floor number message
		liftStatus.MsgCabinfloorNum = liftStatus.CabinCurrentfloorNum;
	} else {
		espRequest = ESP_CMD_REQ_STATUS_COMPLETE;
	}
	return espRequest;
}

void debug_serving_state() {

	static uint32_t st_time;

// if(prevState != liftStatus.liftRunStatus)
// {
//   prevState = liftStatus.liftRunStatus;
//   CALLSERVE_DEBUG_PRINT("run status =%d \r\n",liftStatus.liftRunStatus);
// }

	if (HAL_GetTick() - st_time > 500) {
		st_time = HAL_GetTick();
		// CALLSERVE_DEBUG_PRINT(" input=%d \r\n",getEmergencyInput());
	}
}

void initDoor() {
	doorInit(getFloorNumberByLIDARValue());
}

/*********************************************************************************************
 HANDLE SAFETY ERRORS
 *********************************************************************************************/
bool handleDoorOpenTooLong() {
	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		liftStatus.liftCallServeState = CALL_SERVICE_EEROR_CANCEL_CALLS;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		/**
		 * send message to cabin to clear all calls registered.
		 */
		// sendAllCallCancelledNotification((uint32_t) false);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;
	case CALL_SERVICE_OPEN_DOOR: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;
	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handleSafetyFailure() {
	bool status;
	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_CLOSE_LANDING_GEAR: {
		// ll true
		setLLState(true);
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll false
		setLLState(false);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handleCANSafetyFailure(CALL_SERVE_STS serveSt) {

	bool status;
	status = false;
	char tag[20]; // use it identify the failure type in statement
	switch (serveSt) {
	case CALL_SERVE_CAN_STS_SAFETY_FAILURE:
		snprintf(tag, sizeof(tag), "[lop failure]:");
		break;
	case CALL_SERVE_STS_POWER_FAILURE:
		snprintf(tag, sizeof(tag), "[power failure]:");
		break;
	case CALL_SERVE_STS_ANDROID_FAILURE:
		snprintf(tag, sizeof(tag), "[cabin failure]:");
		break;
	default:
		snprintf(tag, sizeof(tag), "[invalid failure]:");
		break;
	}
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		setLLState(false);
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;
	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		// DEBUG_PRINT_1("CAN safety failure flr =%d \r\n",liftStatus.CabinCurrentfloorNum);
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handlePowerFailure() {

	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll false
		setLLState(false);
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handleOverloadSafetyFailure() {
	bool status;
	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		//    DEBUG(stateDebug, 80, "80: check locks \r\n");
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		EmergencyOutput_On();
		liftStatus.startTime = HAL_GetTick();

		liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_LANDING_GEAR;
	}
		break;

	case CALL_SERVICE_CLOSE_LANDING_GEAR: {
		// ll true
		setLLState(true);
		liftStatus.liftCallServeState = CALL_SERVICE_LIFT_STOPPED;
	}
		break;
	case CALL_SERVICE_LIFT_STOPPED: {
		if (callServeCheckLiftStopped()) {
			//      DEBUG(stateDebug, 64, "64: lift stopped \r\n");
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		uint16_t distance;
		EmergencyOutput_Off();
		//    DEBUG_PRINT_1("Overload safety flr =%d \r\n", liftStatus.CabinCurrentfloorNum);
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_FLOOR_RANGE(
					LIDARAtFloorPosition[liftStatus.CabinCurrentfloorNum],
					FLOOR_RANGE_OFFSET_VAL, distance)) {
				openDoor(liftStatus.CabinCurrentfloorNum);
			}
		}
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handle_light_curtain_safety() {
	bool status;
	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		//    DEBUG(stateDebug, 80, "80: check locks \r\n");
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		EmergencyOutput_On();
		liftStatus.startTime = HAL_GetTick();

		liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_LANDING_GEAR;
	}
		break;

	case CALL_SERVICE_CLOSE_LANDING_GEAR: {
		// ll true
		setLLState(true);
//		setLinearActuator(true);
		liftStatus.liftCallServeState = CALL_SERVICE_LIFT_STOPPED;
	}
		break;
	case CALL_SERVICE_LIFT_STOPPED: {
		if (callServeCheckLiftStopped()) {
			//      DEBUG(stateDebug, 64, "64: lift stopped \r\n");
//			setLinearActuator(false);
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		uint16_t distance;
		EmergencyOutput_Off();
		//    DEBUG_PRINT_1("Overload safety flr =%d \r\n", liftStatus.CabinCurrentfloorNum);
		if (lidar_state) {
			distance = getDistance();
			if (CHECK_FLOOR_RANGE(
					LIDARAtFloorPosition[liftStatus.CabinCurrentfloorNum],
					FLOOR_RANGE_OFFSET_VAL, distance)) {
				openDoor(liftStatus.CabinCurrentfloorNum);
			}
		}
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handleLandingLeverFailure() {
	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	case CALL_SERVICE_CHECK_LOCKS: {
		//    DEBUG(stateDebug, 100, "100: LL check \r\n");
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_EEROR_CANCEL_CALLS;
		}
	}
		break;

	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		/**
		 * send message to cabin to clear all calls registered.
		 */
		// sendAllCallCancelledNotification((uint32_t) false);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		//    DEBUG(stateDebug, 109, "109: opndor\r\n");
		openDoor((FLOOR_NUM) liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		//    DEBUG(stateDebug, 111, "111: cmplet\r\n");
		status = true;
	}
		break;

	default:
		openDoor((FLOOR_NUM) liftStatus.CabinCurrentfloorNum);
		//    DEBUG(stateDebug, 110, "110: LL defa\r\n");
		resetSpeedControlLogic();
		status = true;
	}
	return status;
}

bool handleLIDARfailure() {
	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);

		liftStatus.liftCallServeState = CALL_SERVICE_CLOSE_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_CLOSE_LANDING_GEAR: {
		// ll false message
		liftStatus.liftCallServeState = CALL_SERVICE_EEROR_MESSAGE_SEND;
	}
		break;
	case CALL_SERVICE_EEROR_MESSAGE_SEND: {
		// lidar fail message
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (readMechanicalLock(0) == MECH_LOCK_OPEN) {
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;

	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		/**
		 * send message to cabin to clear all calls registered.
		 */
		// sendAllCallCancelledNotification((uint32_t) false);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll false message
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		//    DEBUG(stateDebug, 109, "109: opndor\r\n");
		openDoor(0);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		//    DEBUG(stateDebug, 111, "111: cmplet\r\n");
		status = true;
	}
		break;

	default:
		openDoor(0);
		//    DEBUG(stateDebug, 110, "110: LIDAR defa\r\n");
		resetSpeedControlLogic();
		status = true;
	}
	return status;
}

bool handleCabinDerailfailure() {
	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	case CALL_SERVICE_CHECK_LOCKS: {
		//    DEBUG(stateDebug, 100, "100: LL check \r\n");
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
	}
		break;
	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll false;
		liftStatus.liftCallServeState = CALL_SERVICE_STOP_MOTOR;
	}
		break;

	case CALL_SERVICE_STOP_MOTOR: {
		//    DEBUG(stateDebug, 108, "108: stop serv2\r\n");
		resetSpeedControlLogic();
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		liftStatus.liftCallServeState = CALL_SERVICE_CABIN_DEST;
	}
		break;
	case CALL_SERVICE_CABIN_DEST: {
		liftStatus.liftCallServeState = CALL_SERVICE_EEROR_CANCEL_CALLS;
		//    DEBUG(stateDebug, 107, "107: in cabin dst2\r\n");
		cancelAllRegCallsInShaftandCabin();
	}
		break;

	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		// send all call clear
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;
	case CALL_SERVICE_OPEN_DOOR: {
		//    DEBUG(stateDebug, 109, "109: opndor\r\n");
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;
	case CALL_SERVICE_COMPLETE: {
		//    DEBUG(stateDebug, 111, "111: cmplet\r\n");
		status = true;
	}
		break;

	default:
		openDoor(liftStatus.CabinCurrentfloorNum);
		//    DEBUG(stateDebug, 110, "110: DERAIL defa\r\n");
		resetSpeedControlLogic();
		status = true;
	}
	return status;
}

bool handleMechLockFailureOnCallReg() {
	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll message false
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
		}
	}
		break;
	case CALL_SERVICE_OPEN_DOOR: {
		//    DEBUG_PRINT_1("CAN safety failure flr =%d \r\n", liftStatus.CabinCurrentfloorNum);
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool handleEspComFailure() {

	bool status;

	status = false;
	switch (liftStatus.liftCallServeState) {
	/**
	 * clear all call in shaft.
	 */
	case CALL_SERVICE_CHECK_LOCKS: {
		cancelAllRegCallsInShaftandCabin();
		resetSpeedControlLogic();
		setEVstate(EVO_CLOSE);
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_LANDING_GEAR;
		liftStatus.startTime = HAL_GetTick();
	}
		break;

	case CALL_SERVICE_OPEN_LANDING_GEAR: {
		// ll message false
		liftStatus.liftCallServeState = CALL_SERVICE_TO_DESTINATION;
	}
		break;

	case CALL_SERVICE_TO_DESTINATION: {
		if (callServeCheckLiftStopped()) {
			liftStatus.liftCallServeState = CALL_SERVICE_EEROR_CANCEL_CALLS;
		}
	}
		break;

	case CALL_SERVICE_EEROR_CANCEL_CALLS: {
		// ll message false
		liftStatus.liftCallServeState = CALL_SERVICE_OPEN_DOOR;
	}
		break;

	case CALL_SERVICE_OPEN_DOOR: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		liftStatus.liftCallServeState = CALL_SERVICE_COMPLETE;
	}
		break;

	case CALL_SERVICE_COMPLETE: {
		status = true;
	}
		break;

	default: {
		openDoor(liftStatus.CabinCurrentfloorNum);
		status = true;
	}
		break;
	}
	return status;
}

bool checkKeepDoorOpen(FLOOR_NUM flr) {
	bool status = false;
	if (MECH_LOCK_NO_ERROR == liftStatus.lckStatus) {
		status = keepDoorOpen(flr);
	}
	return false;
	return status;
}
/********************************************************************************************************/
CALL_SERVE_STS preCheck() {
	uint16_t dist = 0;
	CALL_SERVE_STS initCheck = CALL_SERVE_STS_NOFAIL;
	dist = getDistance();
	if (power_state) {
		initCheck = CALL_SERVE_STS_POWER_FAILURE;
		return initCheck;
	} else if (!lidar_state) {
		initCheck = CALL_SERVE_STS_LIDAR_FAIL;
		return initCheck;
	} else if (overload_state) {
		initCheck = CALL_SERVE_STS_OVER_LOAD;
		return initCheck;
	} else if (!doorClosed()) {
		initCheck = CALL_SERVE_STS_DOORLOCK_FAIL;
		return initCheck;
	} else if (MECH_LOCK_NO_ERROR != liftStatus.lckStatus) {

		initCheck = CALL_SERVE_STS_SAFETY_FAILURE;
		return initCheck;
	} else if (!getAndroidState()) {
		initCheck = CALL_SERVE_STS_ANDROID_FAILURE;
		return initCheck;
	} else if (enCabDerailCheck == true) {
		if (!(CHECK_FLOOR_RANGE(
				LIDARAtFloorPosition[liftStatus.CabinCurrentfloorNum],
				FLOOR_RANGE_OFFSET_VAL, dist))) {
			initCheck = CALL_SERVE_STS_CAB_DERAIL;
			return initCheck;
		}

	} else if (lop_disconnected() == true) {
		initCheck = CALL_SERVE_CAN_STS_SAFETY_FAILURE;
		return initCheck;
	} else if (cabin_booking_allowed == false) {
		initCheck = CALL_SERVE_CABIN_BATTERY_LOW;
//		debug_print_n("cabin battery low\n");
		return initCheck;
	} else if (cabin_schedule_lock == true) {
		initCheck = CALL_SERVE_CABIN_SCHEDULED_DOWN;
	}
//	debug_print_n("MECH_LOCK_NO_ERROR %d\n",initCheck);
	return initCheck;
}

bool getEmergencyInput() {
	return (bool) (cabin_emergency_input() || readEmergencyShaftInput());
}

void cabinAppAutoCalibStateSet(bool state) {
	cabinAppReq.startAutoFloorCalib = state;
}

/*********************************************************
 * Used to set the speed mode. 0 - Low Speed, 1-Moderate 2-High Speed
 * Function Name  : setSpeedMode
 * @arg           : none
 * return         : none
 **********************************************************/
void setSpeedMode() {
	CONFIG_FILE *temp_config = getConfig();
	if (temp_config->speed_mode == 0x01) { // Moderate speed
		upcallSoftStart[0] = 8;
		upcallSoftStart[1] = 10;
		upcallMaxSpeed[0] = 100;
		upcallMaxSpeed[1] = 18;
		upcallSoftLand1[0] = 100;
		upcallSoftLand1[1] = 10;
		upcallSoftLand2[0] = 100;
		upcallSoftLand2[1] = 7;

		downcallSoftStart[0] = 50;
		downcallSoftStart[1] = -8;
		downcallMaxSpeed[0] = 100;
		downcallMaxSpeed[1] = -18;
		downcallSoftLand1[0] = 100;
		downcallSoftLand1[1] = -10;
		downcallSoftLand2[0] = 100;
		downcallSoftLand2[1] = -7;

//		upcallSoftStart[0] = 50;
//		upcallSoftStart[1] = 5;
//		upcallMaxSpeed[0] = 100;
//		upcallMaxSpeed[1] = 12;
//		upcallSoftLand1[0] = 100;
//		upcallSoftLand1[1] = 7;
//		upcallSoftLand2[0] = 100;
//		upcallSoftLand2[1] = 3;
//
//		downcallSoftStart[0] = 50;
//		downcallSoftStart[1] = -5;
//		downcallMaxSpeed[0] = 100;
//		downcallMaxSpeed[1] = -12;
//		downcallSoftLand1[0] = 100;
//		downcallSoftLand1[1] = -7;
//		downcallSoftLand2[0] = 100;
//		downcallSoftLand2[1] = -3;
	} else if (temp_config->speed_mode == 0x02) { // High Speed
		upcallSoftStart[0] = 80;
		upcallSoftStart[1] = 15;
		upcallMaxSpeed[0] = 100;
		upcallMaxSpeed[1] = 23;
		upcallSoftLand1[0] = 100;
		upcallSoftLand1[1] = 8;
		upcallSoftLand2[0] = 100;
		upcallSoftLand2[1] = 5;

		downcallSoftStart[0] = 80;
		downcallSoftStart[1] = -15;
		downcallMaxSpeed[0] = 100;
		downcallMaxSpeed[1] = -23;
		downcallSoftLand1[0] = 100;
		downcallSoftLand1[1] = -8;
		downcallSoftLand2[0] = 100;
		downcallSoftLand2[1] = -5;

	} else {  // Low Speed
//		upcallSoftStart[0] = 50;
//		upcallSoftStart[1] = 5;
//		upcallMaxSpeed[0] = 100;
//		upcallMaxSpeed[1] = 12;
//		upcallSoftLand1[0] = 100;
//		upcallSoftLand1[1] = 7;
//		upcallSoftLand2[0] = 100;
//		upcallSoftLand2[1] = 3;
//
//		downcallSoftStart[0] = 50;
//		downcallSoftStart[1] = -5;
//		downcallMaxSpeed[0] = 100;
//		downcallMaxSpeed[1] = -12;
//		downcallSoftLand1[0] = 100;
//		downcallSoftLand1[1] = -7;
//		downcallSoftLand2[0] = 100;
//		downcallSoftLand2[1] = -3;

		upcallSoftStart[0] = 50;
		upcallSoftStart[1] = 8;
		upcallMaxSpeed[0] = 100;
		upcallMaxSpeed[1] = 18;
		upcallSoftLand1[0] = 100;
		upcallSoftLand1[1] = 10;
		upcallSoftLand2[0] = 100;
		upcallSoftLand2[1] = 7;

		downcallSoftStart[0] = 50;
		downcallSoftStart[1] = -8;
		downcallMaxSpeed[0] = 100;
		downcallMaxSpeed[1] = -18;
		downcallSoftLand1[0] = 100;
		downcallSoftLand1[1] = -10;
		downcallSoftLand2[0] = 100;
		downcallSoftLand2[1] = -7;
	}
}
