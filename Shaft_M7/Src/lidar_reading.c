/*
 * lidar_reading.c
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */
#include <stdio.h>
#include "lidar_reading.h"
#include "usart.h"
#include "common.h"
#include "serial_handler.h"
#include <stdio.h>
#include "filter.h"
#include "debug_print.h"
#include "led_indication.h"

uint16_t lidar_value = 0;
uint16_t lidar_strength = 0;
bool lidar_status = false;
uint16_t lidar_speed = 0;
uint8_t tfminiData[9];
const uint8_t lidar_output_format_mm[5] = { 0x5A, 0x05, 0x05, 0x06, 0x6A };
const uint8_t saveSettings[4] = { 0x5A, 0x04, 0x11, 0x6F };
const uint8_t lidarFrameRate[6] = { 0x5A, 0x03, 0x06, 0xe8, 0x03, 0x4E };
const uint8_t lidarBaudRate[8] = { 0x5A, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x00,
		0x53 };
//uint8_t lidar_rx_buffer[LIDAR_INCOMING_SIZE];

TIMEOUT_CALCULATION lidar_state_em = DEVICE_SET_TIMEOUT;
static bool data_received = false;
void lidar_data_cb();
void lidar_error_cb();
void getTFminiData();

void lidar_data_cb()
{
	static long int lidar_timer = 0;
	HAL_UART_Receive_IT(&LIDAR_UART, (uint8_t *)tfminiData, LIDAR_INCOMING_SIZE);
//	debug_print_n("[lidar delay : %d]\n", HAL_GetTick() - lidar_timer);
	lidar_timer=HAL_GetTick();
	data_received=true;
	getTFminiData();
}

void lidar_error_cb()
{
	HAL_UART_AbortReceive_IT(&LIDAR_UART);
	HAL_UART_Receive_IT(&LIDAR_UART, (uint8_t *)tfminiData, LIDAR_INCOMING_SIZE);
//	debug_print_n("[lidar error]\n");
	data_received=false;
}

void getTFminiData()
{
	uint8_t j = 0;
	int checksum = 0;
//	debug_print_n("[lidar] ");
//	for(int i=0;i<9;i++)
//	{
//		debug_print_n("%x\t", tfminiData[i]);
//	}
//	debug_print_n("\n");
	if (tfminiData[0] == 0x59 && tfminiData[1] == 0x59)
	{
		for (j = 0; j < 8; j++)
		{
			checksum += tfminiData[j];
		}
		if (tfminiData[8] == (checksum % 256))
		{
//			debug_print_n("checksum valid\n");
			lidar_value = tfminiData[2] + (tfminiData[3] << 8);
			lidar_strength = tfminiData[4] + (tfminiData[5] << 8);
			lidar_state_em = DEVICE_START_TIMER;
		}
	}
	else
	{
		lidar_error_cb();
		return;
	}
}

bool lidar_available() {
	bool status = true;
	return status;
}

void setDistance(uint16_t val) {
	lidar_value = val;
}

void setStrength(uint16_t strength) {
	lidar_strength = strength;
}

uint16_t getDistance() {
	return lidar_value;
}

uint16_t getSigStrenth() {
	return lidar_strength;
}

bool isLIDARWorks() {
	static bool prev_status = false;
	if (prev_status != lidar_status) {
		prev_status = lidar_status;
		write_lidar_led(lidar_status);
		debug_print_n("Lidar status is now %d\n", lidar_status);
		set_device_availability(LIDAR_DEVICE, lidar_status);
	}
	return lidar_status;
}

void setLidarWorking(bool status) {
	lidar_status = status;
}

void setLidarInMmMode() {
	HAL_UART_Transmit(&LIDAR_UART, lidar_output_format_mm,LENGTH_ARRAY(lidar_output_format_mm), 100); // set output format millimeter
}

void setLidarBaudRate() {

	uint8_t ret = HAL_BUSY;
	ret = HAL_UART_Transmit(&LIDAR_UART, lidarBaudRate, sizeof(lidarBaudRate),
			50);
	if (ret == HAL_OK) {
		debug_print_n("lidar Baudrate command send successfully \n");
	} else {
		debug_print_n("lidar Baudrate command send error \n");
	}

}

void setLidarITMode() {
	if (HAL_UART_Receive_IT(&LIDAR_UART, tfminiData, sizeof(tfminiData))
			!= HAL_OK) {
		Error_Handler();
	}
}

void setupLidarDMA() {
	if (HAL_UART_Receive_DMA(&huart7, tfminiData, sizeof(tfminiData))
			!= HAL_OK) {
		Error_Handler();
	}
}

void updateLidarValue() {
	static long int lidar_timer_expired = 0;
	uint8_t ret = HAL_BUSY;
	if (HAL_GetTick() - lidar_timer_expired > LIDAR_UPDATE_TIMER) {
			lidar_timer_expired = HAL_GetTick();
		ret = HAL_UART_Receive_IT(&LIDAR_UART, tfminiData, LIDAR_INCOMING_SIZE);
		//ret = HAL_UART_Receive(&LIDAR_UART, tfminiData, LIDAR_INCOMING_SIZE,100);
//		debug_print_n("tfminiData : %x  %x  %x  %x  %x  %x  %x  %x  %x \n", tfminiData[0],tfminiData[1],tfminiData[2],tfminiData[3],tfminiData[4],tfminiData[5],tfminiData[6],tfminiData[7],tfminiData[8]);
		if (ret == HAL_OK) {
				lidar_timer_expired = HAL_GetTick();

			getTFminiData();
		}
	}
}

void lidarSaveSettings() {
	uint8_t ret = HAL_BUSY;
	ret = HAL_UART_Transmit(&LIDAR_UART, saveSettings, LENGTH_ARRAY(saveSettings),
			50);
	if (ret == HAL_OK) {
		debug_print_n("lidar Save command send successfully \n");
	} else {
		debug_print_n("lidar command send error \n");
	}
}

void setLidarFrameRate() {
	uint8_t ret = HAL_BUSY;
	ret = HAL_UART_Transmit(&LIDAR_UART, lidarFrameRate, sizeof(lidarFrameRate),
			50);
	if (ret == HAL_OK) {
		debug_print_n("lidar framerate command send successfully \n");
	} else {
		debug_print_n("lidar command send error \n");
	}
}

void setlidarvalueupdated() {
	static long int lidar_timer = 0;
	switch (lidar_state_em) {
	case DEVICE_START_TIMER: {
//			debug_print_n("[lidar] timer start\n");
		lidar_timer = HAL_GetTick();
		lidar_state_em = DEVICE_WAIT_UNTIL_COUNTDOWN;
		setLidarWorking(true);
	}
		break;
	case DEVICE_WAIT_UNTIL_COUNTDOWN: {
		if (HAL_GetTick() - lidar_timer > TIMEOUT_LIDAR) {
//				debug_print_n("[lidar] timeout\n");
			lidar_state_em = DEVICE_SET_TIMEOUT;
		}
	}
		break;

	case DEVICE_SET_TIMEOUT:
		debug_print_n("[lidar] initiate failure sequence\n");
		lidar_timer = HAL_GetTick();
		setLidarWorking(false);
		lidar_state_em = DEVICE_IDLE;
		break;

	case DEVICE_IDLE:
		break;
	default: {
		lidar_state_em = DEVICE_SET_TIMEOUT;
	}
		break;
	}
}

void setupLidarConfig()
{
	debug_print_n("\ninit lidar sequence\n");
	if(HAL_OK == HAL_UART_RegisterCallback(&LIDAR_UART,HAL_UART_RX_COMPLETE_CB_ID, lidar_data_cb))
	{
		debug_print_n("[lidar] [uart rx complete]\n");
	}
	if(HAL_OK == HAL_UART_RegisterCallback(&LIDAR_UART,HAL_UART_ERROR_CB_ID,lidar_error_cb))
	{
		debug_print_n("[lidar] [uart rx error]\n");
	}
	HAL_UART_Receive_IT(&LIDAR_UART, (uint8_t *)tfminiData, LIDAR_INCOMING_SIZE);

	setLidarInMmMode();
	lidarSaveSettings();
	//	setLidarBaudRate();
	//	setLidarFrameRate();
}
