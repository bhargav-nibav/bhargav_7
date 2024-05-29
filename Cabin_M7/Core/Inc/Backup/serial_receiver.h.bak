/*
 * serial_receiver.h
 *
 *  Created on: Oct 25, 2023
 *      Author: ADMIN
 */

#ifndef INC_SERIAL_RECEIVER_H_
#define INC_SERIAL_RECEIVER_H_

#define MAX_BUFFER_SIZE 50
typedef enum {
	cabin_current_floor = 0X01,
	call_booking_confirmation,
	error_status,
	update_status,
	lidar_value,
	ll_state,
	call_clear,
	door_lock_state,
	BROADCAST_HEADER = 0x60,
	BROADCAST_FOOTER = 0xff,
} BROADCAST_DATAFRAME;


void monitor_wifi();
#endif /* INC_SERIAL_RECEIVER_H_ */
