/*
 * voltage_control.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */

#include "gpio.h"
#include "adc.h"

#include <inttypes.h>
#include "math.h"
#include "voltage_control.h"
#include "android_handler.h"
#include "debug_print.h"

#include "common.h"
#include <stdbool.h>

long int batTimer = 0;

#define ADC_RES 16
#define VOLTAGE_PIN 7
#define max_voltage 25
float COMPENSATION[] = { -0.05f, 0.05f };
//#define COMPENSATION 0.28f;
int bat_percentage = 0;
int prev_percentage = 1; //
float input_volt = 0.0;
float temp = 0.0;
float r1 = 10000.0; // r1 value
float r2 = 1100.0;  // r2 value

float vout = 0;

bool compare_float(float x, float y);
int movingAverage(int _input_percentage);

#define BATTERY_TIMER 3
#define maxSample (10 * BATTERY_TIMER)
bool currentChargingState;
uint32_t batteryDisableTime = 0;
uint32_t chargerDisableTime = 0;
float bat_percentage_2 = 0;
int sampleData = 0;
float samplePercentage[maxSample] = {0};

#define FIFTY_PERCENTAGE_REF 22.7f
#define LESSER_HALF_FACTOR 0.044f
#define GREATER_HALF_FACTOR 0.044f

#define LESSER_PERCENTAGE(x) (50 + ((x - FIFTY_PERCENTAGE_REF) / LESSER_HALF_FACTOR))
#define GREATER_PERCENTAGE(x) (50 + ((x - FIFTY_PERCENTAGE_REF) / GREATER_HALF_FACTOR))

/************************************/
#define battery_TIMEOUT BATTERY_TIMER * 1000
#define charger_TIMEOUT 2500
bool batteryStatus = true;
bool batteryTimerStart = false;

void displayPercentage();
void vol_measurement();

uint32_t getADCval();

void vol_measurement() {
	if (HAL_GetTick() - batTimer > 100) {
		batTimer = HAL_GetTick();
		float mV = getADCval() * 3.3 / 65535.0;
		float Vout = (mV / (r2 / (r1 + r2))); //0.0f;
		Vout = ceil(Vout * 100.0) / 100.0;
		if (Vout <= FIFTY_PERCENTAGE_REF) {
			Vout = Vout + COMPENSATION[0];
		} else {
			Vout = Vout + COMPENSATION[1];
		}

		if (Vout == FIFTY_PERCENTAGE_REF) {
			bat_percentage = 50;
		} else if (Vout > FIFTY_PERCENTAGE_REF) {
			bat_percentage = (int) GREATER_PERCENTAGE(Vout); //more than 50%
		} else {
			bat_percentage = (int) LESSER_PERCENTAGE(Vout); //less than 50%
		}
		if (bat_percentage < 0) {
			bat_percentage = 0;
		} else if (bat_percentage > 100) {
			bat_percentage = 100;
		}
//		num = snprintf(buf, sizeof(buf),"mv = %.2f vout = %.2f Vout = %.2f percentage is %d\n", mV,vout, Vout, bat_percentage);
//		print_debug_message(buf, num);
//		bat_percentage = movingAverage(bat_percentage);
	}
}

int movingAverage(int _input_percentage)
{
	if (sampleData >= maxSample)
	  {
		sampleData = 0;
//		averagePercentage = accumulate(samplePercentage, samplePercentage + maxSample, 0.0);
//		Serial.printf("average percentage = %d max sample = %d sampledata =%d\n", averagePercentage, maxSample, sampleData);
//		prev_percentage = ceil((averagePercentage / maxSample) * 100.0) / 100.0;
//		prev_percentage = (averagePercentage / maxSample);
		if (prev_percentage > 98) {
			prev_percentage = 100;
		}
	} else {
		samplePercentage[sampleData] = _input_percentage;
		sampleData++;
	}

	return prev_percentage;
}

void checkbatteryStatus() {
	vol_measurement();
	bool battery_timer = tickDiff(batteryDisableTime, HAL_GetTick()) > battery_TIMEOUT;
	bool charger_timer = tickDiff(chargerDisableTime, HAL_GetTick()) > charger_TIMEOUT;
	if (battery_timer)
	{
		batteryDisableTime = HAL_GetTick();
		displayPercentage();
	}
	if (charger_timer)
	{
		chargerDisableTime = HAL_GetTick();
	}
}

void displayPercentage()
{
//    debug_print("Battery percentage is %d\n", bat_percentage);
    androidDataframe(BATTERY_PERCENTAGE, bat_percentage);
}

uint32_t getADCval()
{
	uint32_t adc_val = 0;
	HAL_ADC_Start(&BATTERY_ADC_CHANNEL); // start the adc
	HAL_ADC_PollForConversion(&BATTERY_ADC_CHANNEL, 10); // poll for conversion

	adc_val = HAL_ADC_GetValue(&BATTERY_ADC_CHANNEL); // get the adc value
	HAL_ADC_Stop(&BATTERY_ADC_CHANNEL); // stop adc
	return adc_val;
}
