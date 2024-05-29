/*
 * motor_controller.h
 *
 *  Created on: Oct 18, 2023
 *      Author: ADMIN
 */

#ifndef INC_MOTOR_CONTROLLER_H_
#define INC_MOTOR_CONTROLLER_H_
#include "common.h"
#include "filter.h"
#include "debug_print.h"
#include "lidar_reading.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

typedef enum
{
	MSPEED_STOP,
	MOTOR_ONE,
	MOTOR_TWO,
	MOTOR_THREE,
	MOTOR_FOUR,
	MOTOR_FIVE,
	MOTOR_SIX,
	MOTOR_SEVEN
}motor_count;

typedef enum
{   EVO_CLOSE,
    EVO_OPEN,
	EVO_IDLE
}evServoState;

typedef enum
{
	CONTROL_NOT_REACHED,
	CONTROL_REACHED
}PID_CONTROL;


#define OVERLOAD_DETECTION_TIME   50

float getVelocity(uint16_t lider_val);
bool getSpeedPIDCtrlStatus(int speed);
void servoInit(void);
void setEvoServoDuty(uint8_t dutyCycle);
void setMotorSpeed(uint16_t DutyCycle);
void zeroCrossStart(void);


void set_motor_count(motor_count mt_count);
void setEVstate(evServoState state);
void resetSpeedControlLogic();
void setMotorControlSpeed(CONTROL_DIR liftDir, int16_t speed);
void setMotorControlSpeedMotor(uint16_t initial_Motors, CONTROL_DIR liftDir,
		int16_t speed);
/**
* Call this function in a periodic loop
*/
void controlMotorSpeedProcess();
void decreaseMotorByOne();
void increaseMotorByOne();
CONTROL_DIR getControlStatus();
bool getLidarDisAndLiftSpeed();
int16_t getSpeed();
void tim1_reinit();











#endif /* INC_MOTOR_CONTROLLER_H_ */
