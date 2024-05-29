/*
 * pid.h
 *
 *  Created on: Jan 25, 2024
 *      Author: Balamurugan S
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define AUTOMATIC	1
#define MANUAL		0
#define DIRECT 		0
#define REVERSE  	1
#define P_ON_M 		0
#define P_ON_E 		1

void ComputeUpCall(void);
void ComputeDownCall(void);
void SetTuningsUpCall(double Kp, double Ki, double Kd, double pOn);
void SetTuningsDownCall(double Kp, double Ki, double Kd, double pOn);
//void SetTunings(double Kp, double Ki, double Kd);
void SetSampleTimeUpCall(int NewSampleTime);
void SetSampleTimeDownCall(int NewSampleTime);
void SetOutputLimitsUpCall(double Min, double Max);
void SetOutputLimitsDownCall(double Min, double Max);
void SetModeUpCall(int Mode);
void SetModeDownCall(int Mode);
void InitializeUpCall(void);
void InitializeDownCall();
void SetControllerDirection(int Direction);
void PidInitializeUpCall(void);
void PIDUpCall(double Kp,
		double Ki, double Kd, int POn, int ControllerDirection);
void PidInitializeDownCall(void);
void PIDDownCall(double Kp, double Ki, double Kd, int POn,
		int ControllerDirection);
void pidUpControl(double kp, double ki, double kd,double speed, uint16_t lidarDistance);
void pidDownControl(double kp, double ki, double kd, double speed, uint16_t lidarDistance);
#endif /* INC_PID_H_ */

///*
// * pid.h
// *
// *  Created on: Jan 25, 2024
// *      Author: Balamurugan S
// */
//
//#ifndef INC_PID_H_
//#define INC_PID_H_
//
//#define AUTOMATIC	1
//#define MANUAL		0
//#define DIRECT 		0
//#define REVERSE  	1
//#define P_ON_M 		0
//#define P_ON_E 		1
//
//void ComputeUpCall(void);
//void ComputeDownCall(void);
//void SetTuningsUpCall(double Kp, double Ki, double Kd, double pOn);
//void SetTuningsDownCall(double Kp, double Ki, double Kd, double pOn);
////void SetTunings(double Kp, double Ki, double Kd);
//void SetSampleTimeUpCall(int NewSampleTime);
//void SetSampleTimeDownCall(int NewSampleTime);
//void SetOutputLimitsUpCall(double Min, double Max);
//void SetOutputLimitsDownCall(double Min, double Max);
//void SetModeUpCall(int Mode);
//void SetModeDownCall(int Mode);
//void InitializeUpCall(void);
//void InitializeDownCall();
//void SetControllerDirection(int Direction);
//void PidInitializeUpCall(void);
//void PIDUpCall(double Kp, double Ki,
//		double Kd, int POn, int ControllerDirection);
//void PidInitializeDownCall(void);
//void PIDDownCall(double Kp, double Ki,
//		double Kd, int POn, int ControllerDirection);
//void pidDownControl(double kp,double ki,double kd,uint16_t lidarDistance);
//void pidUpControl(double kp,double ki,double kd,uint16_t lidarDistance);
//
//#endif /* INC_PID_H_ */
