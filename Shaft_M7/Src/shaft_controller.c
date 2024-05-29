/*
 * shaft_controller.c
 *
 *  Created on: Oct 26, 2023
 *      Author: ADMIN
 */
#include "shaft_controller.h"
#include "configuration.h"
#include "power_detection.h"
#include "emergency_monitor.h"
#include "buzzer_enabler.h"
#include "motor_controller.h"
#include "lop_door_control.h"

#include "call_serving.h"
#include "call_reg.h"
#include "lidar_reading.h"
#include "serial_handler.h"
#include "led_indication.h"
#include "auto_calibration.h"
#include "lift_idle_control.h"
#include "overload_detection.h"
#include "pid.h"
//#include "power_reading.h"

uint16_t i = 9001;
extern uint32_t autoCalData[5];
extern uint16_t LIDARAtFloorPosition[4];

//uint16_t send_to_esp32[12] = { 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0xff };

//extern volatile double upCallKp, upCallKi, upCallKd;
//extern volatile double upCallSetpoint;
//extern volatile uint16_t downCallOutputAdjusted, upCallOutputAdjusted;
//extern volatile double downCallKp, downCallKi, downCallKd;
//extern volatile double downCallSetpoint;


void setup()
{
	configFileInit();
	liftIdleInit(5);
	initDoor();
	led_indication_init();
	setLLState(false);
	setupLidarConfig();
//	setSpeedMode();
	HAL_Delay(1000);
	tim1_reinit();
	//servoInit();
	zeroCrossStart();

	disable_rxTemporarily();
	//setEvoServoDuty(124);
//	PidInitializeUpCall();
//	PidInitializeDownCall();

}

void loop() {

 //long int timer_expired_count = 0;
	indicate_health();
	monitor_devices();
//
//	updateLidarValue();
//	updateWiFiValue();
	update_wifi_value();
	upDateCabinFloorStatus();
	checkCabinCalls();
	serveCalls();
	monitor_lop_timer();

	doorStateProcess();
	getLidarDisAndLiftSpeed();
	controlMotorSpeedProcess();
	doorGetIOstate();
	getMechLockStatus();
	//mechIOState();
	checkFirstChange();
	processAutoFloorCalib();
	setcabinvalueupdated();

	monitor_alarm();
	setlidarvalueupdated();
	send_broadcast_message();

	test_door_inputs();
	handleEmergency();
//	testCodeHere();

//	debug_print_n("[***********************************duration %d]\n", HAL_GetTick() - timer_expired_count);
}

void testCodeHere() {
static long int timer_expired = 0;
char buffer[] = "new_messsage\n";
if (HAL_GetTick() - timer_expired > 1000) {
	timer_expired = HAL_GetTick();
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) buffer, sizeof(buffer), 100);
//		HAL_GPIO_TogglePin(test_toggle_pin_GPIO_Port, test_toggle_pin_Pin);
}
}
void zeroCrossTestCode() {
i++;
if (i > 9800) {
	i = 1000;
}
setMotorSpeed(i);
debug_print_n("i = %d \n", i);
HAL_Delay(1);
}

void servoTestCode() {
i++;
if (i > 125) {
	i = 25;
}
//setEvoServoDuty(i);
debug_print_n("i = %d \n", i);
HAL_Delay(100);
}

void test_motor() {
set_motor_count(1);
HAL_Delay(1000);
set_motor_count(2);
HAL_Delay(1000);
set_motor_count(3);
HAL_Delay(1000);
set_motor_count(4);
HAL_Delay(1000);
set_motor_count(5);
HAL_Delay(1000);
set_motor_count(6);
HAL_Delay(1000);
set_motor_count(7);
HAL_Delay(1000);

set_motor_count(0);
HAL_Delay(1000);

setEVstate(EVO_OPEN);
HAL_Delay(2000);
setEVstate(EVO_CLOSE);
HAL_Delay(2000);
}
void test_ll() {
static long int timer_expired = 0;
static bool state = false;
if (HAL_GetTick() - timer_expired > 2000) {
	timer_expired = HAL_GetTick();
	state = !state;
	setLLState(state);
}
}

void test_callBookingClear() {
static long int timer_expired = 0;
//static bool state = false;
static int i = 0;
if (HAL_GetTick() - timer_expired > 1000) {
	timer_expired = HAL_GetTick();
	test_door_solenoid(i);
	i++;
	i = i % 4;
}
}

void switchMessages(int state) {
switch (state) {
case 0: {
	setLOPCallBookedMessage(0);
	setCabinFloorNumber(0);
	break;
}
case 1: {
	setLOPCallBookedMessage(MAX_FLOOR);
	break;
}
case 2: {
	setLOPCallBookedMessage(1);
	setCabinFloorNumber(1);
	break;
}
case 3: {
	setLOPCallBookedMessage(MAX_FLOOR);
	break;
}
default:
	break;
}
}

void test_door_solenoid(int i) {
char buf[20];
switch (i) {
case 0: {
	snprintf(buf, sizeof(buf), "All lock low\n");
	door_lock(MAX_FLOOR);
	break;
}
case 1: {
	snprintf(buf, sizeof(buf), "Ground lock High\n");
	door_lock(0);
	break;
}
case 2: {
	snprintf(buf, sizeof(buf), "All lock low\n");
	door_lock(MAX_FLOOR);
	break;
}
case 3: {
	snprintf(buf, sizeof(buf), "First lock High\n");
	door_lock(1);
	break;
}
default:
	break;
}
debug_print_n(buf);
}

void indicate_health() {
//	debug_print_n("indicate_health %\n");
static long int timer_expired = 0;
if (HAL_GetTick() - timer_expired > 500) {
	timer_expired = HAL_GetTick();
	HAL_GPIO_TogglePin(m7_health_GPIO_Port, m7_health_Pin);
	HAL_GPIO_TogglePin(public_health_GPIO_Port, public_health_Pin);
}
}
