/*
 * serial_handler.h
 *
 *  Created on: Oct 26, 2023
 *      Author: ADMIN
 */

#ifndef INC_SERIAL_HANDLER_H_
#define INC_SERIAL_HANDLER_H_

#include "common.h"

#define MAX_BUFFER_SIZE 50


/*
 *  esp32 to shaft header definition
 */
#define CABIN_MESSAGE	0xD5
#define LOP_MESSAGE		0x55
#define ESP32_QUERY		0x95
#define RASP_MESSAGE	0xE5
#define LIDAR_CONFIG	0xc1
#define LOP_DURATION	0xc2

//#define MAX_ALLOWED

#define MINIMUM_ALLOWED_BATTERY 20

void disable_rxTemporarily();

void updateWiFiValue();
void update_wifi_value();
void init_wifi_idle();
void setCabinFloorNumber(uint8_t flr_num);
void setLOPCallBookedMessage(uint8_t flr_num);
void setErrorMessage(uint8_t error_message);
void setcalibrationStatus(uint8_t status);
void setLLState(int ll_status);
void setLinearActuator(int ll_status);
void setEVstatus(uint8_t state);
void setMotorNum(uint8_t num);
void set_power_value(bool state);
void setCabinCallBooked(uint8_t flr_num);
void cabin_set_booking_message(uint8_t regValue);
void door_lock(uint8_t flr_num);
void send_broadcast_message();
void set_door_state(uint8_t door_msg);
void set_mech_state(uint8_t mech_msg);
void setcabinvalueupdated();
void lockDoorSolenoidAtFloor(FLOOR_NUM floorNum,bool state);

bool cabin_emergency_input();
bool siren_input();
bool child_lock_input();
bool downtime_status();
bool getAndroidState();
uint8_t getbatteryValue();
bool getContactChargerValue();

void set_lop_state(uint8_t * flr_num);
bool readDoorSwitchStatus(uint8_t floorNum);
int readMechanicalLock(uint8_t flr);

bool doorClosed();
bool setDoorLockState(FLOOR_NUM flr, bool state);
uint8_t doorGetIOstate();

bool lop_disconnected();
void monitor_lop_timer();
void set_device_availability(int device_type, bool state);
void set_network_availability(int device_type, bool state);
bool cabin_battery_acceptable();

/*
 * RPi data
 * */
bool light_curtain_triggered();
int people_count();
bool return_update();
void device_ota_progress(uint8_t status);

/*
 * callback
 * */

void send_lidar_value();
void set_device_under_update();
void clear_device_under_update();
void send_latest_value();
void store_values();
void device_restart();
void update_lidar_values(bool _status);
void send_to_shaft_esp32(uint32_t* data);
#endif /* INC_SERIAL_HANDLER_H_ */
