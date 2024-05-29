#include "gpio.h"
#include "led_indication.h"


#include <stdbool.h>
static void clearRegisters();
static void writeRegisters();



int registers_01[numOfRegisterPins];
int registers_02[numOfRegisterPins];
int registers_03[numOfRegisterPins];

void led_indication_init()
{
    /*Init register*/
    clearRegisters();
    writeRegisters();
}

void clearRegisters()
{

    /* Clear registers variables */
	  for(int i = numOfRegisterPins-1; i >=   0; i--)
	  {
	    registers_01[i] = GPIO_PIN_RESET;
	    registers_02[i] = GPIO_PIN_RESET;
	    registers_03[i] = GPIO_PIN_RESET;
	  }
}

void writeRegisters()
{
	writeRegisters01();
	writeRegisters02();
	writeRegisters03();
}

void writeRegisters01()
{
    HAL_GPIO_WritePin(shift_reg1_rclk_GPIO_Port, shift_reg1_rclk_Pin, GPIO_PIN_RESET);

    for (int i = (numOfRegisterPins - 1); i >= 0; i--)
    {
        HAL_GPIO_WritePin(shift_reg1_srclk_GPIO_Port, shift_reg1_srclk_Pin, GPIO_PIN_RESET);

        int val = registers_01[i];

        HAL_GPIO_WritePin(shift_reg1_data_GPIO_Port, shift_reg1_data_Pin, val);

        HAL_GPIO_WritePin(shift_reg1_srclk_GPIO_Port, shift_reg1_srclk_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(shift_reg1_rclk_GPIO_Port, shift_reg1_rclk_Pin, GPIO_PIN_SET);
}

void writeRegisters02()
{
    HAL_GPIO_WritePin(shift_reg2_rclk_GPIO_Port, shift_reg2_rclk_Pin, GPIO_PIN_RESET);

    for (int i = (numOfRegisterPins - 1); i >= 0; i--)
    {
        HAL_GPIO_WritePin(shift_reg2_srclk_GPIO_Port, shift_reg2_srclk_Pin, GPIO_PIN_RESET);

        int val = registers_02[i];

        HAL_GPIO_WritePin(shift_reg2_data_GPIO_Port, shift_reg2_data_Pin, (GPIO_PinState)val);

        HAL_GPIO_WritePin(shift_reg2_srclk_GPIO_Port, shift_reg2_srclk_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(shift_reg2_rclk_GPIO_Port, shift_reg2_rclk_Pin,GPIO_PIN_SET);
}

void writeRegisters03()
{
    HAL_GPIO_WritePin(shift_reg3_rclk_GPIO_Port, shift_reg3_rclk_Pin, GPIO_PIN_RESET);

    for (int i = (numOfRegisterPins - 1); i >= 0; i--)
    {
        HAL_GPIO_WritePin(shift_reg3_srclk_GPIO_Port, shift_reg3_srclk_Pin, GPIO_PIN_RESET);

        int val = registers_03[i];

        HAL_GPIO_WritePin(shift_reg3_data_GPIO_Port, shift_reg3_data_Pin, (GPIO_PinState)val);

        HAL_GPIO_WritePin(shift_reg3_srclk_GPIO_Port, shift_reg3_srclk_Pin,GPIO_PIN_SET);

    }

    HAL_GPIO_WritePin(shift_reg3_rclk_GPIO_Port, shift_reg3_rclk_Pin, GPIO_PIN_SET);
}


void set_device_register_01(int index, int value)
{
    /*Set register variable to HIGH or LOW  */
    registers_01[index] = value;
}
void set_device_register_02(int index, int value)
{
    /*Set register variable to HIGH or LOW  */
	registers_02[index] = value;
}
void set_device_register_03(int index, int value)
{
    /*Set register variable to HIGH or LOW  */
	registers_03[index] = value;
}

void write_lop_led(int flr, bool state)
{
	set_device_register_03(flr+4, state);
	writeRegisters03();
}

void write_door_led(int door_pin, int state)
{
	set_device_register_01(door_pin, state);
	writeRegisters01();
}

void write_mech_led(int mech_number, int state)
{
	switch(state)
	{
	case 0:
		set_device_register_01(mech_number+4, state);
		set_device_register_02(mech_number, 0);
		writeRegisters01();
		writeRegisters02();
		break;
	case 1:
		set_device_register_01(mech_number+4, state);
		set_device_register_02(mech_number, 0);
		writeRegisters01();
		writeRegisters02();
		break;
	case 2:
		set_device_register_01(mech_number+4, 0);
		set_device_register_02(mech_number, state);
		writeRegisters01();
		writeRegisters02();
		break;
	default:
		writeRegisters01();
		break;
	}
}

void write_emergency_led(bool state)
{
	set_device_register_02(EMERGENCY_LED, state);
	writeRegisters02();
}

void write_calibration_led(bool state)
{
	set_device_register_02(CALIBRATION_LED, state);
	writeRegisters02();
}

void write_overload_led(bool state)
{
	set_device_register_02(OVERLOAD_LED, state);
	writeRegisters02();
}
void write_power_led(bool state)
{
	set_device_register_02(POWER_LED, state);
	writeRegisters02();
}

void write_debug_led(bool state)
{
	set_device_register_03(DEBUG_LED, state);
	writeRegisters03();
}

void write_wifi_led(bool state)
{
	set_device_register_03(WIFI_LED, state);
	writeRegisters03();
}

void write_lidar_led(bool state)
{
	set_device_register_03(LIDAR_LED, state);
	writeRegisters03();
}

void write_cabin_led(bool state)
{
	set_device_register_03(CABIN_LED, state);
	writeRegisters03();
}
