/*
 * common.h
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_


#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include <stdbool.h>
#define SET_BIT_n(reg, n) (reg |= (1 << n))
#define CLR_BIT(reg, n) (reg &= ~(1 << n))
#define CHECK_BIT(reg, n) (reg & (1 << n))

#define LENGTH_OF_ARRAY(ARR) (sizeof(ARR) / sizeof(ARR[0]))

#define ONE_SECOND 1000

#define CABIN_MINOR 0x01
#define CABIN_MAJOR 0x01

#define SHAFT_MINOR 0x00
#define SHAFT_MAJOR 0x01

#define DEBUG_UART huart5
#define WIFI_UART huart1
#define BATTERY_ADC_CHANNEL hadc1
#define INIT_VALUE 0

typedef enum
    {
      PWM_STOP,
      PWM_START,
      PWM_TIMER_COUNTDOWN,
      PWM_TIMER_EXPIRE,
      PWM_IDLE
    }PWM_DEVICE_STATE;

uint32_t tickDiff(uint32_t startTime, uint32_t currentTime);

/*
 *
 * linear actuator release and retract timing
 */
#define RELEASE_TIME			(2000)
#define RETRACT_TIME			(2000)

/*
 * landing automatic turn off timer
 */
#define LANDING_LEVER_TIMER		(60000)


/*
 * child lock monitoring timer
 */
#define CHILD_LOCK_MONITOR_TIMER	(30)

/*
 * contact chargermonitoring timer
 */
#define CONTACT_CHARGER_MONITOR_TIMER	(200)


/*
 * android message delivery timer
 */
#define CABIN_MESSAGE_INTERVAL		(500)
#endif /* INC_COMMON_H_ */
