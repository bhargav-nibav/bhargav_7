/*
 * android_handler.h
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */

#ifndef INC_ANDROID_HANDLER_H_
#define INC_ANDROID_HANDLER_H_


#include "common.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum{
    HEADER_BYTE=0X00,
    CALL_BOOKING,
    SIREN_MESSAGE,
    CHILDLOCK_STATUS,
    EMERGENCY_MESSAGE,
    BATTERY_PERCENTAGE,
    CONTACT_CHARGER,
    DEVICE_DETAILS,
    UPDATE_STATUS,
    FOOTER_BYTE
}MESSAGE_TYPE;

typedef enum
{
    CHILD_LOCK_DISABLED,
    CHILD_LOCK_ENABLED
}CHILD_LOCK_STATE;


void androidDataframe(MESSAGE_TYPE msg_type, uint8_t data);
void transmit_cabin_msg();
void setDevice(int deviceID);
void clearDevice(int deviceID);
void clearAllOutput();
void set_output_variable(bool type, int device_id); //true for set, false for clear

#endif /* INC_ANDROID_HANDLER_H_ */
