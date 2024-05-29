/*
 * motor_controller.c
 *
 *  Created on: Oct 18, 2023
 *      Author: ADMIN
 */

#include "motor_controller.h"
#include <tim.h>
#include "lidar_reading.h"
#include "gpio.h"
#include "overload_detection.h"
#include <stdio.h>
#include "debug_print.h"
#include "serial_handler.h"
#include "stm32h7xx_hal_gpio.h"

#define CONTROL_INIT_LOGIC_TIMING 5
#define CONTROL_LOGIC_TIMING      20
#define CONTROL_ERROR_DISTANCE    8
#define NO_OF_RELAYS_USED         6
#define CONTROL_TOLERANCE         5

#define OVERLOAD_INPUT_HOLD_TIME  2

volatile int lift_speed;
volatile long int prev_distance;
extern volatile double Input;
extern volatile uint16_t upCallOutputAdjusted;

typedef struct speedControl {
	uint16_t distance;
	int16_t speed;
	int16_t expectedSpeed;
	bool lidarFail;
	bool motorOverLoad;
	bool runMotorControl;
	uint8_t speedOperating;
	CONTROL_DIR controlDirection;
	CONTROL_DIR liftDir;
	uint16_t prevLidarDis;
	uint8_t timeCounter;
	uint8_t intervalCounter;
	uint8_t overloadInputDebonceCounter;
	uint16_t avgDistance;
	CONTROL_DIR controlStatus;

} SPEEDCONTROL_st;

SPEEDCONTROL_st speedControl;

/*********************************************************
 * Used to read the data from lidar UART and calculate the velocity
 * Function Name  : getVelocity
 * @arg           : none
 * return         : int - return the velocity
 **********************************************************/

float getVelocity(uint16_t lider_val) {
	static long int speed_timer = 0;

	if (HAL_GetTick() - speed_timer > SPEED_INTERVAL) {
		speed_timer = HAL_GetTick();
		lift_speed = ((prev_distance - lider_val) / (SPEED_INTERVAL / 100));
//		lift_speed = ((prev_distance - lider_val));

//		debug_print_n("  lift_speed data : %d\n", lift_speed);
		debug_print_n(" lider_val ** : %d prev_distance : %d \n", lider_val,
				prev_distance);
		Input = (double) lift_speed;
//		debug_print_n(" Input  : %f %d \n", Input, lift_speed);

		prev_distance = lider_val;
		return lift_speed;
	}
	return lift_speed;
}

bool getSpeedPIDCtrlStatus(int speed) {
	int speedVal = 0;
	static uint8_t count1 = 0;
	static uint8_t count2 = 0;
	if (speed >= lift_speed) {
		speedVal = abs(speed - lift_speed);
//		debug_print_n("speed <= lift_speed *************** speedVal diff %d speed %d lift_speed %d\n",speedVal,speed,lift_speed);
	} else {
		speedVal = abs(lift_speed - speed);
//		debug_print_n("else  *************** speedVal diff %d speed %d lift_speed %d\n",speedVal,speed,lift_speed);
	}

	if ((speedVal > 5)) //|| (speedVal <  - 5))
		{
//		debug_print_n("inside else ************* count1 %d \n",count1);
		count2 = 0;
		count1++;
		if (count1 > 2) {
			count1=3;
//			debug_print_n(" CONTROL_NOT_REACHED ************************* diff %d speed %d lift_speed %d\n",speedVal,speed,lift_speed);
			return CONTROL_NOT_REACHED;
		}
	} else {
//		debug_print_n("inside else ************* count2 %d \n",count2);
		count1 = 0;
		count2++;
		if (count2 > 2) {
			count2=3;
//			debug_print_n(" CONTROL_REACHED ************************* diff %d speed %d lift_speed %d\n",speedVal,speed,lift_speed);
			return CONTROL_REACHED;
		}
		else
		{

		}
	}

}

/*********************************************************
 * Used to initialize the servo motor.
 * Function Name  : servoInit
 * @arg           : uint8_t - dutyCycle
 * return         : none
 **********************************************************/

//void servoInit(void) {
//
////	HAL_TIM_PWM_Start(&SERVO_TIMER, TIM_CHANNEL_4);
//	setEvoServoDuty(100);
//}
/*********************************************************
 * Used to set the duty cycle of the servo motor.
 * Function Name  : setServoDuty
 * @arg           : uint8_t - dutyCycle
 * return         : none
 **********************************************************/

//void setEvoServoDuty(uint8_t dutyCycle) {
//	TIM2->CCR4 = dutyCycle;
//	//debug_print_n(" setServoDuty ****** \n ");
//}
/*********************************************************
 * Start the zero crossing.
 * Function Name  : zeroCrossStart
 * @arg           : none
 * return         : none
 **********************************************************/

void zeroCrossStart(void) {
	HAL_TIM_OnePulse_Start(&ZEROCROSS_TIMER, TIM_CHANNEL_1);
	setMotorSpeed(MOTOR_STOP);
}

/*********************************************************
 * Used to set the control the motor speed with duty cycle.
 * Function Name  : setMotorPWMDutyCycle
 * @arg           : DutyCycle
 * return         : none
 **********************************************************/

void setMotorSpeed(uint16_t DutyCycle) {
//	debug_print_n("motor duty cycle : %d \n",DutyCycle);
	__HAL_TIM_SET_COMPARE(&ZEROCROSS_TIMER, TIM_CHANNEL_1, DutyCycle);
}

/*********************************************************
 * check the motor speed is reached required speed .
 * Function Name  : setMotorPWMDutyCycle
 * @arg           : DutyCycle
 * return         : none
 **********************************************************/

void motorSpeedControl(uint16_t DutyCycle) {
	__HAL_TIM_SET_COMPARE(&ZEROCROSS_TIMER, TIM_CHANNEL_1, DutyCycle);
}

void set_motor_count(motor_count mt_count) {
//	debug_print_n("****Motor count = %d\n", (int) mt_count);
//	static uint8_t prev_count = 0;
//	if (prev_count != (uint8_t) mt_count) {
//		prev_count = (uint8_t) mt_count;
//		setMotorNum((uint8_t) mt_count);
//	}
//
//	switch (mt_count) {
//	case MSPEED_STOP: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)RESET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_ONE: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_TWO: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_THREE: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_FOUR: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_FIVE: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) RESET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_SIX: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) RESET);
//	}
//		break;
//	case MOTOR_SEVEN: {
////		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin, (GPIO_PinState)SET);
//		HAL_GPIO_WritePin(motor_relay_01_GPIO_Port, motor_relay_01_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_02_GPIO_Port, motor_relay_02_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_03_GPIO_Port, motor_relay_03_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_04_GPIO_Port, motor_relay_04_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_05_GPIO_Port, motor_relay_05_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_06_GPIO_Port, motor_relay_06_Pin,
//				(GPIO_PinState) SET);
//		HAL_GPIO_WritePin(motor_relay_07_GPIO_Port, motor_relay_07_Pin,
//				(GPIO_PinState) SET);
//	}
//		break;
//	default:
//		break;
//	}
}

void setEVstate(evServoState state) {
	evServoState prevState = EVO_IDLE;
	if (prevState != state) {
		prevState = state;
		setEVstatus(state);
		debug_print_n("Evo state is %d\n", state);
		HAL_GPIO_WritePin(evo_relay_GPIO_Port, evo_relay_Pin,
				(GPIO_PinState) state);
	}
}

void resetSpeedControlLogic() {
	speedControl.controlDirection = CONTROL_NO_OPERATION;
	speedControl.controlStatus = CONTROL_NO_OPERATION;
	speedControl.speedOperating = false;
	speedControl.timeCounter = false;
	speedControl.intervalCounter = false;
	speedControl.motorOverLoad = false;
	speedControl.overloadInputDebonceCounter = false;
	speedControl.runMotorControl = false;
	setMotorSpeed(MOTOR_STOP);
#ifdef SERVO_ENABLE
				setEvoServoDuty(EVO_SERVO_CLOSE);
#else
	setEVstate(EVO_CLOSE);
#endif

	upCallOutputAdjusted = 9000;
	//set_motor_count(MSPEED_STOP);
}

void setMotorControlSpeed(CONTROL_DIR liftDir, int16_t speed) {
	if (speedControl.runMotorControl) {
		speedControl.expectedSpeed = speed;
		speedControl.liftDir = liftDir;
		speedControl.controlStatus = CONTROL_STARTED;
	} else {
		speedControl.controlStatus = CONTROL_STARTED;
		speedControl.controlDirection = CONTROL_STARTED;
		speedControl.expectedSpeed = speed;
		speedControl.timeCounter = false;
		speedControl.liftDir = liftDir;
		speedControl.timeCounter = false;
		speedControl.intervalCounter = false;
		speedControl.overloadInputDebonceCounter = false;
		speedControl.runMotorControl = true;
	}
}

void setMotorControlSpeedMotor(uint16_t initial_Motors, CONTROL_DIR liftDir,
		int16_t speed) {
	if (speedControl.runMotorControl) {
		speedControl.expectedSpeed = speed;
		speedControl.liftDir = liftDir;
		speedControl.controlStatus = CONTROL_STARTED;
	} else {
		speedControl.speedOperating = initial_Motors;
		speedControl.controlStatus = CONTROL_STARTED;
		speedControl.controlDirection = CONTROL_STARTED;
		speedControl.expectedSpeed = speed;
		speedControl.timeCounter = false;
		speedControl.liftDir = liftDir;
		speedControl.timeCounter = false;
		speedControl.intervalCounter = false;
		speedControl.overloadInputDebonceCounter = false;
		speedControl.runMotorControl = true;
		set_motor_count(speedControl.speedOperating);

	}

}

const uint8_t logicDecTiming[] = { 5, 5, 5, 5, 5, 5 };

const uint8_t logicUpIncTiming[] = { 5, 5, 10, 20, 30, 40 };
const uint8_t logicUPDecTiming[] = { 60, 50, 40, 40, 30, 20 };

const uint8_t logicDownIncTiming[] = { 10, 20, 30, 40, 50, 60 };
const uint8_t logicDownDecTiming[] = { 50, 50, 50, 40, 30, 20, 10 };

const uint8_t controlUplogicDecTiming[NO_OF_RELAYS_USED] = { 10, 50, 50, 50, 50,
		50 };

uint8_t multiplication_factor[] = { 4, 2 };

//void evoControlProcess() {
//	int speedDiff = lift_speed - speed;
//	if (speedDiff < CONTROL_TOLERANCE) {
//      controlReached++;
//	}
//}

void controlMotorSpeedProcess() {
//	debug_print_n(" controlMotorSpeedProcess \n");
	int16_t errorSpeed;
	const uint8_t *MotorIncTiming;
	const uint8_t *MotorDecTiming;
	int factor_consider = -1;
	if (speedControl.runMotorControl) {
		if (speedControl.liftDir == CONTROL_UP_DIR) {
			errorSpeed = speedControl.expectedSpeed - speedControl.speed;
			MotorIncTiming = logicUpIncTiming;
			MotorDecTiming = logicUPDecTiming;
			factor_consider = 0;
		} else {
			MotorIncTiming = logicDownIncTiming;
			MotorDecTiming = logicDownDecTiming;
			errorSpeed = speedControl.expectedSpeed - speedControl.speed;
			factor_consider = 1;
		}

		if (speedControl.controlDirection == CONTROL_ACHIVED) {
			if ((errorSpeed > CONTROL_ERROR_DISTANCE)
					|| (errorSpeed < -CONTROL_ERROR_DISTANCE)) {
				speedControl.controlDirection = CONTROL_STARTED;
				speedControl.timeCounter = false;
			} else {
				speedControl.controlStatus = CONTROL_ACHIVED;
			}
		}

		if ((speedControl.controlDirection == CONTROL_STARTED)
				|| (speedControl.controlDirection == CONTROL_UP_DIR)
				|| (speedControl.controlDirection == CONTROL_DOWN_DIR)) {
			speedControl.timeCounter++;
//			debug_print_n("ep =%d exp =%d rlys=%d counter =%d \r\n", errorSpeed,speedControl.expectedSpeed, speedControl.speedOperating,speedControl.timeCounter);
			if (errorSpeed >= CONTROL_ERROR_DISTANCE) {
				if (speedControl.speedOperating < NO_OF_RELAYS_USED) {
					if (speedControl.timeCounter
							>= (multiplication_factor[factor_consider]
									* MotorIncTiming[speedControl.speedOperating])) {
						speedControl.timeCounter = false;
						speedControl.speedOperating++;

						if (speedControl.controlDirection == CONTROL_STARTED) {
							speedControl.controlDirection = CONTROL_UP_DIR;
						} else if (speedControl.controlDirection
								== CONTROL_DOWN_DIR) {
							speedControl.controlDirection = CONTROL_ACHIVED;
							speedControl.controlStatus = CONTROL_ACHIVED;
						}
						set_motor_count(speedControl.speedOperating);
					}
				} else {
					if ((speedControl.overloadInputDebonceCounter
							> OVERLOAD_INPUT_HOLD_TIME)
							|| ((speedControl.speed == 0)
									&& (speedControl.intervalCounter
											>= OVERLOAD_DETECTION_TIME))) {
						speedControl.controlDirection = CONTROL_ERROR;
						speedControl.controlStatus = CONTROL_ERROR;
						speedControl.motorOverLoad = true;
						debug_print_n(
								"!3.Control sped=%d rel=%d intCoutr=%d debCountr =%d \n",
								speedControl.speed, speedControl.speedOperating,
								speedControl.intervalCounter,
								speedControl.overloadInputDebonceCounter);
					} else if (speedControl.speed == 0) {
						speedControl.intervalCounter++;
					} else if (speedControl.speed) {
						speedControl.intervalCounter = 0;
					}
				}

				if (monitor_overload()) {
					speedControl.overloadInputDebonceCounter++;
				} else {
					speedControl.overloadInputDebonceCounter = false;
				}
			} else if (errorSpeed <= -CONTROL_ERROR_DISTANCE) {
				if (speedControl.speedOperating) {
					if (speedControl.timeCounter
							>= (multiplication_factor[factor_consider]
									* MotorDecTiming[speedControl.speedOperating])) {
						speedControl.timeCounter = false;
						speedControl.speedOperating--;
						if (speedControl.controlDirection == CONTROL_STARTED) {
							speedControl.controlDirection = CONTROL_DOWN_DIR;
						} else if (speedControl.controlDirection
								== CONTROL_UP_DIR) {
							speedControl.controlDirection = CONTROL_ACHIVED;
							speedControl.controlStatus = CONTROL_ACHIVED;
							speedControl.speedOperating = false;
						}
						set_motor_count(speedControl.speedOperating);
					}
				} else if ((speedControl.speed == 0)
						&& (speedControl.intervalCounter
								>= OVERLOAD_DETECTION_TIME)
						&& (speedControl.speedOperating == false)) {
					speedControl.controlDirection = CONTROL_ERROR;
					speedControl.controlStatus = CONTROL_ERROR;
				} else if (speedControl.speed == 0) {
					speedControl.intervalCounter++;
				} else if (speedControl.speed) {
					speedControl.intervalCounter = 0;
				} else {
					speedControl.speedOperating = 0;
				}
			} else if ((errorSpeed < CONTROL_ERROR_DISTANCE)
					&& (errorSpeed > -CONTROL_ERROR_DISTANCE)) {
				speedControl.controlDirection = CONTROL_ACHIVED;
				speedControl.controlStatus = CONTROL_ACHIVED;
			}
		} else {

		}

	}
}

void decreaseMotorByOne() {
	if (speedControl.speedOperating > 0) {
		speedControl.speedOperating--;
		set_motor_count(speedControl.speedOperating);
	}
}

void increaseMotorByOne() {
	if (speedControl.speedOperating < NO_OF_RELAYS_USED) {
		speedControl.speedOperating++;
		set_motor_count(speedControl.speedOperating);
	}
}

CONTROL_DIR getControlStatus() {
	return speedControl.controlStatus;
}

bool getLidarDisAndLiftSpeed() {
//	debug_print_n(" getLidarDisAndLiftSpeed \n");
	static uint32_t timeprev = 0;
	int16_t LiftSpeed;
	static uint16_t lidarFailCounter;
	if (isLIDARWorks()) {
		speedControl.lidarFail = false;
		speedControl.distance = getDistance();
		// speedControl.avgDistance = (uint16_t)iir((int16_t) speedControl.distance);
		speedControl.avgDistance = (uint16_t) firLIDAR(speedControl.distance);
		if (speedControl.distance < 200) {
			speedControl.avgDistance = speedControl.distance;
		}

		if (HAL_GetTick() - timeprev > 80) {
			// 100 - 80 = 20 -> move up
			// 80 - 100 = -20 -> move down
			LiftSpeed = (int16_t) ((float) (1000
					* ((speedControl.prevLidarDis - speedControl.avgDistance)))
					/ (float) (HAL_GetTick() - timeprev));
			lidarFailCounter = false;
			speedControl.speed = (int16_t) ((float) (LiftSpeed
					+ speedControl.speed) / 2.0f);
			speedControl.speed = fir(speedControl.speed);

//			debug_print_n("Lift speed =%d pvdis =%d cdis =%d \r\n",	speedControl.speed, speedControl.prevLidarDis, speedControl.avgDistance);
			speedControl.prevLidarDis = speedControl.avgDistance;
			timeprev = HAL_GetTick();
		}
	} else {
		if (lidarFailCounter++ >= LIDAR_COM_TIMEOUT) {
			speedControl.lidarFail = true;
			lidarFailCounter = LIDAR_COM_TIMEOUT;        //to avoid overflow
//			debug_print_n("lidar failure = %d\n",speedControl.lidarFail);
		}
		return false;
	}
	return true;
}

int16_t getSpeed() {
	return speedControl.speed;
}

/*********************************************************
 * Used to re-init the timer1 parameters (changed OCMODE_TIMING in sConfigOC.OCMode to TIM_OCMODE_PWM2;
 * Function Name  : tim1_reinit
 * @arg           : none
 * return         : none
 **********************************************************/

void tim1_reinit() {

	/* USER CODE END TIM1_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 180 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

