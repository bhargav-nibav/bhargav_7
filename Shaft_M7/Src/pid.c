

#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "debug_print.h"
#include "motor_controller.h"
#include "pid.h"

/*working variables*/
unsigned long upCallLastTime, downCallLastTime;
volatile double Input, upCallOutput, upCallSetpoint;
volatile double downCallOutput, downCallSetpoint;
double upCallOutputSum, upCallLastInput, downCallOutputSum, downCallLastInput;

volatile double upCallKp, upCallKi, upCallKd;
double upCallOutMin, upCallOutMax;
double *upCallMyOutput;
double *upCallMyInput;
double *upCallMySetpoint;

int SampleTime = 100; //1 sec
//double upCallOutMin, upCallOutMax;

bool inAuto = false;
double ITerm;

//////////////////////////// Down Call Init
volatile double downCallKp, downCallKi, downCallKd;
double downCallOutMin, downCallOutMax;
double *downCallMyOutput;
double *downCallMyInput;
double *downCallMySetpoint;

int controllerDirection = DIRECT;

bool pOnE = true, pOnM = false;
double pOnEKp, pOnMKp;

volatile uint16_t downCallOutputAdjusted, upCallOutputAdjusted;
extern volatile int lift_speed;
extern volatile long int prev_distance;
extern volatile uint16_t lidar_value;
extern bool downCallEnable;

/*********************************************************
 * Used to compute the output from Kp,Ki and Kd in lift upcall
 * Function Name  : ComputeUpCall
 * @arg           : none
 * return         : none
 **********************************************************/

void ComputeUpCall() {
	if (!inAuto)
		return;
	unsigned long now = HAL_GetTick();
	int timeChange = (now - upCallLastTime);

	if (timeChange >= SampleTime) {

		/*Compute all the working error variables*/
		double error = upCallSetpoint - Input;
		double dInput = (Input - upCallLastInput);
		upCallOutputSum += (upCallKi * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if (pOnM) {
			upCallOutputSum -= pOnMKp * dInput;
		}

		if (upCallOutputSum > upCallOutMax)
			upCallOutputSum = upCallOutMax;
		else if (upCallOutputSum < upCallOutMin)
			upCallOutputSum = upCallOutMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		if (pOnE)
			upCallOutput = pOnEKp * error;
		else
			upCallOutput = 0;

		/*Compute Rest of PID Output*/
		upCallOutput += upCallOutputSum - upCallKd * dInput;

		if (upCallOutput > upCallOutMax) {
			upCallOutput = upCallOutMax;
		} else if (upCallOutput < upCallOutMin) {
			upCallOutput = upCallOutMin;
		}
//		upCallOutput++;
		upCallOutputAdjusted = 10000 - upCallOutput;
		debug_print_n(
				" upCallOutputAdjusted : %d Input : %d  upCallOutput : %d  Setpoint : %d upCallOutMin : %d upCallOutMax: %d\n",
				upCallOutputAdjusted, (int) Input, (int) upCallOutput,
				(int) upCallSetpoint, (int) upCallOutMin, (int) upCallOutMax);
		/*Remember some variables for next time*/
		upCallLastInput = Input;
		upCallLastTime = now;
		//prev_distance = lidar_value;
	}
}

/*********************************************************
 * Used to compute the output from Kp,Ki and Kd in lift down call
 * Function Name  : ComputeDownCall
 * @arg           : none
 * return         : none
 **********************************************************/

void ComputeDownCall() {
	if (!inAuto)
		return;
	unsigned long now = HAL_GetTick();
	int timeChange = (now - downCallLastTime);
//
	if (timeChange >= SampleTime) {

		/*Compute all the working error variables*/
		double error = downCallSetpoint - Input;
		double dInput = (Input - downCallLastInput);
		downCallOutputSum += (downCallKi * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if (pOnM) {
			downCallOutputSum -= pOnMKp * dInput;
		}

		if (downCallOutputSum > downCallOutMax)
			downCallOutputSum = downCallOutMax;
		else if (downCallOutputSum < downCallOutMin)
			downCallOutputSum = downCallOutMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		if (pOnE)
			downCallOutput = pOnEKp * error;
		else
			downCallOutput = 0;

		/*Compute Rest of PID Output*/
		downCallOutput += downCallOutputSum - downCallKd * dInput;

		if (downCallOutput > downCallOutMax)
			downCallOutput = downCallOutMax;
		else if (downCallOutput < downCallOutMin)
			downCallOutput = downCallOutMin;

		downCallOutputAdjusted = downCallOutput;

		debug_print_n(
				" downCallOutputAdjusted : %d Input : %d lift_speed %d Setpoint : %d\n",
				downCallOutputAdjusted, (int) Input, lift_speed,
				(int) downCallSetpoint);
		/*Remember some variables for next time*/
		downCallLastInput = Input;
		downCallLastTime = now;
	}
}

/*********************************************************
 * Used to tune the PID parameter (Kp,Ki and Kd) in upcall
 * Function Name  : SetTuningsUpCall
 * @arg           : double,double,double (Proportional , Integral, Derivative)
 * return         : none
 **********************************************************/

void SetTuningsUpCall(double Kp, double Ki, double Kd, double pOn) {
	if (Kp < 0 || Ki < 0 || Kd < 0 || pOn < 0 || pOn > 1)
		return;

	pOnE = pOn > 0; //some p on error is desired;
	pOnM = pOn < 1; //some p on measurement is desired;

	double SampleTimeInSec = ((double) SampleTime) / 1000;
	upCallKp = Kp;
	upCallKi = Ki * SampleTimeInSec;
	upCallKd = Kd / SampleTimeInSec;

	if (controllerDirection == REVERSE) {
		upCallKp = (0 - upCallKp);
		upCallKi = (0 - upCallKi);
		upCallKd = (0 - upCallKd);
	}

	pOnEKp = pOn * upCallKp;
	pOnMKp = (1 - pOn) * upCallKp;
}

/*********************************************************
 * Used to tune the PID parameter (Kp,Ki and Kd) in down call
 * Function Name  : SetTuningsDownCall
 * @arg           : double,double,double (Proportional , Integral, Derivative)
 * return         : none
 **********************************************************/

void SetTuningsDownCall(double Kp, double Ki, double Kd, double pOn) {
	if (Kp < 0 || Ki < 0 || Kd < 0 || pOn < 0 || pOn > 1)
		return;

	pOnE = pOn > 0; //some p on error is desired;
	pOnM = pOn < 1; //some p on measurement is desired;

	double SampleTimeInSec = ((double) SampleTime) / 1000;
	downCallKp = Kp;
	downCallKi = Ki * SampleTimeInSec;
	downCallKd = Kd / SampleTimeInSec;

	if (controllerDirection == REVERSE) {
		downCallKp = (0 - downCallKp);
		downCallKi = (0 - downCallKi);
		downCallKd = (0 - downCallKd);
	}

	pOnEKp = pOn * downCallKp;
	pOnMKp = (1 - pOn) * downCallKp;
}

/*********************************************************
 * Used to set the sample time for upcall
 * Function Name  : SetSampleTimeUpCall
 * @arg           : int - NewSampleTime
 * return         : none
 **********************************************************/

void SetSampleTimeUpCall(int NewSampleTime) {
	if (NewSampleTime > 0) {
		double ratio = (double) NewSampleTime / (double) SampleTime;
		upCallKi *= ratio;
		upCallKd /= ratio;
		SampleTime = (unsigned long) NewSampleTime;
	}
}

/*********************************************************
 * Used to set the sample time for down call
 * Function Name  : SetSampleTimeDownCall
 * @arg           : int - NewSampleTime
 * return         : none
 **********************************************************/

void SetSampleTimeDownCall(int NewSampleTime) {
	if (NewSampleTime > 0) {
		double ratio = (double) NewSampleTime / (double) SampleTime;
		downCallKi *= ratio;
		downCallKd /= ratio;
		SampleTime = (unsigned long) NewSampleTime;
	}
}

/*********************************************************
 * Used to set the output limits for upcall
 * Function Name  : SetOutputLimitsUpCall
 * @arg           : double - minimum limit, double - maximum limit
 * return         : none
 **********************************************************/

void SetOutputLimitsUpCall(double Min, double Max) {
	if (Min > Max)
		return;
	upCallOutMin = Min;
	upCallOutMax = Max;

	if (upCallOutput > upCallOutMax)
		upCallOutput = upCallOutMax;
	else if (upCallOutput < upCallOutMin)
		upCallOutput = upCallOutMin;

	if (upCallOutputSum > upCallOutMax)
		upCallOutputSum = upCallOutMax;
	else if (upCallOutputSum < upCallOutMin)
		upCallOutputSum = upCallOutMin;
}

/*********************************************************
 * Used to set the output limits for down call
 * Function Name  : SetOutputLimitsDownCall
 * @arg           : double - minimum limit, double - maximum limit
 * return         : none
 **********************************************************/

void SetOutputLimitsDownCall(double Min, double Max) {
	if (Min > Max)
		return;
	downCallOutMin = Min;
	downCallOutMax = Max;

	if (downCallOutput > downCallOutMax)
		downCallOutput = downCallOutMax;
	else if (downCallOutput < downCallOutMin)
		downCallOutput = downCallOutMin;

	if (downCallOutputSum > downCallOutMax)
		downCallOutputSum = downCallOutMax;
	else if (downCallOutputSum < downCallOutMin)
		downCallOutputSum = downCallOutMin;
}

/*********************************************************
 * Used to set the mode of the PID Control in up call
 * Function Name  : SetModeUpCall
 * @arg           : int - mode ( Automatic / Manual)
 * return         : none
 **********************************************************/

void SetModeUpCall(int Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) { /*we just went from manual to auto*/
		InitializeUpCall();
	}
	inAuto = newAuto;
}

/*********************************************************
 * Used to set the mode of the PID Control in down call
 * Function Name  : SetModeDownCall
 * @arg           : int - mode ( Automatic / Manual)
 * return         : none
 **********************************************************/

void SetModeDownCall(int Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) { /*we just went from manual to auto*/
		InitializeDownCall();
	}
	inAuto = newAuto;
}

/*********************************************************
 * Used to initialize the PID Control in up call
 * Function Name  : InitializeUpCall
 * @arg           : none
 * return         : none
 **********************************************************/

void InitializeUpCall() {
	upCallLastInput = Input;
	upCallOutputSum = upCallOutput;
	if (upCallOutputSum > upCallOutMax)
		upCallOutputSum = upCallOutMax;
	else if (upCallOutputSum < upCallOutMin)
		upCallOutputSum = upCallOutMin;
}

/*********************************************************
 * Used to initialize the PID Control in down call
 * Function Name  : InitializeDownCall
 * @arg           : none
 * return         : none
 **********************************************************/

void InitializeDownCall() {
	downCallLastInput = Input;
	downCallOutputSum = downCallOutput;
	if (downCallOutputSum > downCallOutMax)
		downCallOutputSum = downCallOutMax;
	else if (downCallOutputSum < downCallOutMin)
		downCallOutputSum = downCallOutMin;
}

/*********************************************************
 * Used to set the control direction
 * Function Name  : SetControllerDirection
 * @arg           : int - direction (DIRECT / REVERSE)
 * return         : none
 **********************************************************/

void SetControllerDirection(int Direction) {
	controllerDirection = Direction;
}

/*********************************************************
 * Used to initialize the up call parameters
 * Function Name  : PidInitializeUpCall
 * @arg           : none
 * return         : none
 **********************************************************/

void PidInitializeUpCall(void) {

	upCallKp = 2, upCallKi = 5, upCallKd = 0.1;
	//Setpoint = 30;
	PIDUpCall(upCallKp, upCallKi, upCallKd, P_ON_E, DIRECT);

	SetModeUpCall(AUTOMATIC);

}

/*********************************************************
 * Used to initialize the down call parameters
 * Function Name  : PidInitializeDownCall
 * @arg           : none
 * return         : none
 **********************************************************/

void PidInitializeDownCall(void) {
	downCallKp = 2, downCallKi = 3, downCallKd = 0.1;
	//Setpoint = 30;
	PIDDownCall(downCallKp, downCallKi, downCallKd, P_ON_E, DIRECT);

	SetModeDownCall(AUTOMATIC);
}

/*********************************************************
 * Used to set the PID up call parameters
 * Function Name  : PIDUpCall
 * @arg           : input,output,setpoint,Kp,Ki,Kd,POn, direction
 * return         : none
 **********************************************************/

void PIDUpCall(double Kp, double Ki, double Kd, int POn,
		int ControllerDirection) {
	inAuto = false;
	SetOutputLimitsUpCall(10, 9990);//default output limit corresponds to the STM32H7 pwm limits
	SetControllerDirection(ControllerDirection);
	SetTuningsUpCall(Kp, Ki, Kd, POn);
	upCallLastTime = HAL_GetTick() - SampleTime;
	upCallOutputAdjusted = 9500;
}

/*********************************************************
 * Used to set the PID down call parameters
 * Function Name  : PIDUpCall
 * @arg           : input,output,setpoint,Kp,Ki,Kd,POn, direction
 * return         : none
 **********************************************************/

void PIDDownCall(double Kp, double Ki, double Kd, int POn,
		int ControllerDirection) {
	inAuto = false;
	SetOutputLimitsDownCall(25, 125);//default output limit corresponds to the STM32H7 pwm limits
	SetControllerDirection(ControllerDirection);
	SetTuningsDownCall(Kp, Ki, Kd, POn);
	downCallLastTime = HAL_GetTick() - SampleTime;
}


void pidUpControl(double kp, double ki, double kd,double speed, uint16_t lidarDistance) {
	getVelocity(lidarDistance);
	upCallSetpoint = speed; // Set the speed of the lift
	SetTuningsUpCall(kp, ki, kd, P_ON_E);
	getVelocity(lidarDistance);
	ComputeUpCall();
	setMotorSpeed(upCallOutputAdjusted);
}

void pidDownControl(double kp, double ki, double kd, double speed, uint16_t lidarDistance) {

	downCallKp = kp, downCallKi = ki, downCallKd = kd; // Down Call soft start PID parameters
	downCallSetpoint = speed; // Set the speed of the lift
	SetTuningsDownCall(kp, ki, kd, P_ON_E);
	//SetTuningsDownCall(downCallKp, downCallKi, downCallKd, P_ON_E);
	getVelocity(lidarDistance);
	ComputeDownCall();
	//setEvoServoDuty(downCallOutputAdjusted);
}


void liftSpeedControl()
{
	//if
	if(lift_speed == upCallSetpoint)
		{

		}
}

