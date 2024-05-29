/*
 * pwm_devices.h
 *
 *  Created on: Nov 25, 2023
 *      Author: ADMIN
 */

#ifndef INC_PWM_DEVICES_H_
#define INC_PWM_DEVICES_H_

typedef enum
{
	FAN_PWM=0X01,
	LIGHT_PWM,
	LOGO_PWM,
}PWM_DEVICE_TYPES;

#define NORMAL_PWM_DURATION		(30*1000)
#define CABIN_BOOKED_DURATION	(120*1000)

void setPWM(PWM_DEVICE_TYPES device_id, int device_intensity, int duration_type);
void init_pwm();
void monitor_pwm();
#endif /* INC_PWM_DEVICES_H_ */
