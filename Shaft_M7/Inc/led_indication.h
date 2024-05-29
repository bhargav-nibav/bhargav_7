#include "common.h"


#define number_of_74hc595s 1
#define numOfRegisterPins number_of_74hc595s * 8

#define EMERGENCY_LED 	4
#define CALIBRATION_LED 5
#define OVERLOAD_LED	6
#define POWER_LED		7

#define	DEBUG_LED		0
#define WIFI_LED		1
#define LIDAR_LED		2
#define CABIN_LED		3



/// @brief REGISTER 0 for door and mech lock status, REGISTER 1 for input and output REGISTER 2 for process going on, WRITE_ALL for writing to all
typedef enum{
        REGISTER0,
        REGISTER1,
        REGISTER2,
        WRITE_ALL,
}REGISTER_NUM;

void led_indication_init();

void writeLED(int pin, int state);

void led_indication_handler(REGISTER_NUM register_num, unsigned int Error_code, int state);

void write_door_led(int door_pin, int state);		//logic added
void write_mech_led(int mech_number, int state);	//logic added

void write_emergency_led(bool state);		//logic added
void write_calibration_led(bool state);		//logic added
void write_overload_led(bool state); 		//logic added
void write_power_led(bool state);			//logic added

void write_debug_led(bool state);			//logic added
void write_wifi_led(bool state);			//logic added
void write_lidar_led(bool state);			//logic added
void write_cabin_led(bool state);			//
void write_lop_led(int flr, bool state);	//

void set_device_register_01(int index, int value);
void set_device_register_02(int index, int value);
void set_device_register_03(int index, int value);


void writeRegisters01();
void writeRegisters02();
void writeRegisters03();
