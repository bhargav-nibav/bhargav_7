/*
 * serial_receiver.c
 *
 *  Created on: Oct 26, 2023
 *      Author: ADMIN
 */

#include "usart.h"

#include "serial_handler.h"
#include "lidar_reading.h"
#include "common.h"
#include "call_reg.h"
#include "led_indication.h"
#include "configuration.h"
#include "call_serving.h"

#include "stm32h7xx_hal_uart_ex.h"

#define RESET_TIMER_OFFSET 	100
#define MESSAGE_QUERY_TIMER 600
static long int timer_expired = 0;
typedef enum {
	cabin_current_floor=0x01,
	call_booking_confirmation,
	error_status,
	update_status,
	lidar_value,
	ll_state,
	cabin_call_confirmation,
	door_lock_state,
	door_switch,
	mech_lock,
	motor_count,
	evo_status,
	device_availability,
	network_availability,
	BROADCAST_HEADER = 0x60,
	BROADCAST_FOOTER = 0xff,
} BROADCAST_DATAFRAME;

typedef enum {
	call_booking_message = 0x01,
	siren_data,
	child_lock_data,
	cabin_emergency ,
	battery_percentage,
	contact_charger,
	device_details,
	cabin_update_status,
	scheduled_down_status,
	cabin_header = 0x65,
	cabin_footer = 0xff
} CABIN_DATAFORMAT;

typedef enum {
	reserved_data_01 = 0x01,
	reserved_data_02,
	reserved_data_03,
	reserved_data_04 ,
	reserved_data_05,
	reserved_data_06,
	total_people_inside,
	light_curtain_status,
	rpi_header = 0xe5,
	rpi_footer = 0xff
} RPI_DATAFORMAT;

typedef struct
{
  uint8_t booking_request;
  uint8_t current_flr_display;
  uint8_t door_sensor;
  uint8_t mlck_data;
  uint8_t ms_data;
  uint8_t ds_status;
  uint8_t ds_output;
  uint8_t processed_mech_data;
  uint8_t update_status;
}LOP_DEVICE_DATA;


static uint8_t wifi_rx_buffer[10];
static bool message_changed = false;

static TIMEOUT_CALCULATION cabin_connected_state = DEVICE_SET_TIMEOUT;
uint8_t send_to_esp32[] = { 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,MAJOR_VERSION, MINOR_VERSION, 0xff };
static uint8_t shaft_to_common[] = {0x60, 0x00, 0x00, 0x00, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00,0x00, 0xff };// changed to 12 bytes
static uint8_t cabin_to_shaft[] = {0xD5 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0xff };

static uint8_t rpi_to_shaft[] = {0xE5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
static bool cabinConnected = false;

static uint8_t doorState;

static LOP_DEVICE_DATA lop_data[MAX_FLOOR_CONST];
static TIMEOUT_CALCULATION lop_timeout[MAX_FLOOR] = {DEVICE_SET_TIMEOUT,DEVICE_SET_TIMEOUT,DEVICE_SET_TIMEOUT,DEVICE_SET_TIMEOUT};
int lop_available[MAX_FLOOR]={0};
long int lop_timer[MAX_FLOOR]={0};
volatile bool autoCalibDataReq=false;


static void (*esp32_query[6])(void)= {send_lidar_value,set_device_under_update,clear_device_under_update,send_latest_value,store_values,device_restart};
static void wifi_process(uint8_t *data_rc, uint8_t n);
static void set_cabin_state(uint8_t *cab_data, int data_size);
static void checkLOPConnection(int lop_floor);

static void reset_timer();
static void setcabinWorking(bool state)
{
	cabinConnected = state;
}

bool getAndroidState()
{
	return cabinConnected;
}

static void wifi_process(uint8_t *data_rc, uint8_t n)
{
	if(data_rc[n-1]==0xff)
	{
		switch(data_rc[0])
		{
			case CABIN_MESSAGE:
			{
				set_cabin_state(&data_rc[1], n-2);
			}
			break;
			case LOP_MESSAGE:
			{
				set_lop_state(&data_rc[1]);
			}
			break;
			case ESP32_QUERY:
			{
				debug_print_n("esp device req %d\n", data_rc[1]);
				esp32_query[data_rc[1]]();
			}
			break;
			case LIDAR_CONFIG:
			{
				debug_print_n(" set_lidar : %d\n", data_rc[1]);
				set_lidar(&data_rc[1]);
			}
			break;
			case RASP_MESSAGE:
			{
				//		debug_print_n("RPi data received\n");
				//		rpi_to_shaft[total_people_inside] = data_rc[total_people_inside];
				//		rpi_to_shaft[light_curtain_status] = data_rc[light_curtain_status];
			}
			break;
			case LOP_DURATION:
			{
				CONFIG_FILE *temp_config = getConfig();
				if(data_rc[3]==2)
				{
					temp_config->door_solenoid_open_duration = data_rc[4]*1000;
				}
				if(data_rc[1]==1)
				{
					temp_config->lidar_range_check_offset = data_rc[2]*10;
				}
				if(data_rc[5]==3)
				{
					temp_config->speed_mode = data_rc[6];
					setSpeedMode();
				}
				debug_print_n("door solenoid %d\tlidar range %d speed mode %d\n", data_rc[4], data_rc[2], data_rc[6]);
//				set_door_open_duration((int) data_rc[2]);
			}
			break;
			default:
				break;
		}
	}

}

void setCabinFloorNumber(uint8_t flr_num)
{
	if (shaft_to_common[cabin_current_floor] != flr_num) {
		shaft_to_common[cabin_current_floor] = flr_num;
//		message_changed = true;
	}
}

// lidar values
void send_to_shaft_esp32(uint32_t* data)
{
	for(int i=0;i<4;i++)
	{
		send_to_esp32[2*i+1] = (data[i] & 0xFF);
		send_to_esp32[2*(i+1)] = (data[i] & 0xFF00)>>8;
	}
}

void setLOPCallBookedMessage(uint8_t flr_num) {
	uint8_t setBooked = 0;
	if(flr_num == MAX_FLOOR)
	{
		shaft_to_common[call_booking_confirmation] = setBooked;
	}
	else
	{
		setBooked = (1 << flr_num);
		shaft_to_common[call_booking_confirmation] = setBooked;
		message_changed =true;
	}
}

void setErrorMessage(uint8_t error_message) {
	if (shaft_to_common[error_status] != error_message) {
		shaft_to_common[error_status] = error_message;
//		message_changed = true;
	}
}


void setcalibrationStatus(uint8_t status)
{
	if(shaft_to_common[update_status] != status)
	{
		shaft_to_common[update_status] = status;
		message_changed = true;
	}
//	debug_print_n("shaft_to_common[update_status] %d \n", shaft_to_common[update_status]);

}

void device_ota_progress(uint8_t status)
{
	if(shaft_to_common[update_status] != status)
	{
		shaft_to_common[update_status] = status;
		message_changed = true;
	}
}
void setLLState(int ll_status) {
	message_changed = true;
//	debug_print_n("ll state is %d \n", shaft_to_common[ll_state]);
	message_changed = true;
	if (shaft_to_common[ll_state] != ll_status) {
		shaft_to_common[ll_state] = ll_status;
		debug_print_n("ll state is %d \n", ll_status);
	}
//	if(ll_status)
//	{
//		SET_BIT_n(shaft_to_common[ll_state], 0);
//	}
//	else
//	{
//		CLEAR_BIT(shaft_to_common[ll_state], 0);
//	}
	//debug_print_n("ll state value is %d \n", ll_status);
//	debug_print_n("ll state is %d \n", shaft_to_common[ll_state]);
}

void setLinearActuator(int la_status)
{
	message_changed = true;
	if(la_status)
	{
//		SET_BIT_n(shaft_to_common[la_state], 4);
	}
	else
	{
//		CLEAR_BIT(shaft_to_common[la_state], 4);
	}
	debug_print_n("la state is %d \n", la_status);
}

void setCabinCallBooked(uint8_t flr_num) {

	uint8_t setBooked =0;
	if(flr_num >=MAX_FLOOR)
	{
		shaft_to_common[cabin_call_confirmation]=setBooked;
	}
	else
	{
		setBooked = (1<<flr_num);
		shaft_to_common[cabin_call_confirmation]=setBooked;
		message_changed = true;
	}
}

void setMotorNum(uint8_t num)
{
	shaft_to_common[motor_count]=num;
//	message_changed = true;
}

void set_device_availability(int device_type, bool state)
{
	if(state)
	{
		SET_BIT_n(shaft_to_common[device_availability], device_type);
	}
	else
	{
		CLR_BIT(shaft_to_common[device_availability], device_type);
	}
}

/*
 *
 * device id: 0-> cabin 1-4 -> LOP, state 0- > disconnect 1 -> connected
 */
void set_network_availability(int device_type, bool state)
{
	if(state)
	{
		SET_BIT_n(shaft_to_common[network_availability], device_type);
	}
	else
	{
		CLR_BIT(shaft_to_common[network_availability], device_type);
	}
}
void setEVstatus(uint8_t state)
{
	shaft_to_common[evo_status]= state;
//	message_changed = true;
}

void send_broadcast_message()
{
	reset_timer();
	if (( HAL_GetTick() - timer_expired >= MESSAGE_QUERY_TIMER))
	{
		// 200 to 500 changed

		int current_distance = getDistance();
		int _distance = current_distance/50;
		shaft_to_common[lidar_value] = _distance;
		send_to_esp32[9]=(uint8_t)current_distance & 0xff;
		send_to_esp32[10]=(current_distance&0xff00)>>8;
		CONFIG_FILE *temp_config = getConfig();
		send_to_esp32[11]=temp_config->lidar_range_check_offset/10;
		send_to_esp32[12]=temp_config->door_solenoid_open_duration/1000;
		timer_expired = HAL_GetTick();
//		debug_print_n("Requesting device ID %d\n", device_id);
		HAL_StatusTypeDef err = HAL_UART_Transmit(&WIFI_UART, shaft_to_common,LENGTH_ARRAY(shaft_to_common), 100);
		// debug_print_n("general print %d \n", err);
		if(autoCalibDataReq)
		{
			debug_print_n("LENGTH_ARRAY(send_to_esp32) %d\n", LENGTH_ARRAY(send_to_esp32));
			autoCalibDataReq = false;
			HAL_UART_Transmit(&WIFI_UART,send_to_esp32,LENGTH_ARRAY(send_to_esp32),100);
		}
		if(shaft_to_common[update_status]==SHAFT_OTA_FAIL || shaft_to_common[update_status]==SHAFT_OTA_SUCESS)
		{
			shaft_to_common[update_status]=0xDF;
		}
	}

}

bool cabin_emergency_input()
{
	static bool prev_state = false;
	if(prev_state!=cabin_to_shaft[cabin_emergency])
	{
		prev_state=cabin_to_shaft[cabin_emergency];
		set_device_availability(CABIN_EMERGENCY, cabin_to_shaft[cabin_emergency]);
		debug_print_n("*************Cabin Emergency is %d\n",prev_state);
	}
	return cabin_to_shaft[cabin_emergency];
}

bool child_lock_input() {
	static bool prevVal = false;
	if(prevVal != cabin_to_shaft[child_lock_data])
	{
		prevVal = cabin_to_shaft[child_lock_data];
		set_device_availability(CHILD_LOCK_DEVICE, cabin_to_shaft[child_lock_data]);
		debug_print_n("Child lock is %s\n", prevVal?"Pressed":"Released");
	}
	return cabin_to_shaft[child_lock_data];
}

bool siren_input()
{
	return cabin_to_shaft[siren_data];
}

bool downtime_status()
{
	return cabin_to_shaft[scheduled_down_status];
}

bool light_curtain_triggered()
{
	return rpi_to_shaft[light_curtain_status];
}

int people_count()
{
	return rpi_to_shaft[total_people_inside];
}

void cabin_set_booking_message(uint8_t regValue)
{
	if(regValue)
	{
		debug_print_n("[5] : booking [%d]\n", regValue);
	}
	//registerCabinCall(cabin_to_shaft[call_booking_message]);
	registerCabinCall(regValue);
}

void set_cabin_state(uint8_t *cab_data, int data_size)
{
//	debug_print_n("[cabin ++]");
	//debug_print_n("Cabin call_booking_message = %d \n", cab_data[0]);
	if(data_size == LENGTH_ARRAY(cabin_to_shaft)-2)
	{
		cabin_to_shaft[call_booking_message] = cab_data[0];
		cabin_to_shaft[scheduled_down_status]= (bool)CHECK_BIT(cab_data[1], 4);
		cabin_to_shaft[siren_data] = (bool)CHECK_BIT(cab_data[1], 0);
		cabin_to_shaft[child_lock_data] = cab_data[2];
		cabin_to_shaft[cabin_emergency] = cab_data[3];
		cabin_to_shaft[battery_percentage] = cab_data[4];
		cabin_to_shaft[contact_charger] = cab_data[5];
		cabin_to_shaft[device_details] = cab_data[6];
		cabin_to_shaft[cabin_update_status] = cab_data[7];
		cabin_set_booking_message(cabin_to_shaft[call_booking_message]);
		cabin_connected_state=DEVICE_START_TIMER;
	}
}

void door_lock(uint8_t flr_num) {

	uint8_t setBooked = 0;
	if(flr_num < MAX_FLOOR)
	{
		setBooked = (1 << flr_num);
	}
	if (shaft_to_common[door_lock_state] != setBooked) {
		shaft_to_common[door_lock_state] = setBooked;
//		message_changed = true;
	}
}

void set_door_state(uint8_t door_msg)
{
	debug_print_n( "door value = %d\n", door_msg);
	shaft_to_common[door_switch] = door_msg;
//	message_changed = true;
//	debug_print_n("triggered by this 10\n");
}

void set_mech_state(uint8_t mech_msg)
{
//	char buf[30];
	//debug_print_n("mech lock ******= %X \n", mech_msg);
//	debug_print(buf, n);
	shaft_to_common[mech_lock] = mech_msg;
}

void updateWiFiValue()
{
	static uint8_t reserved_buf[10][10]={0};
	static int i =0;
	if(HAL_UART_Receive(&WIFI_UART, wifi_rx_buffer, LENGTH_ARRAY(wifi_rx_buffer),20)==HAL_OK)
	{
		//debug_print_n("wifi_rx_buffer : wifi_rx_buffer[0]: %X wifi_rx_buffer[1]: %X  wifi_rx_buffer[9]: %X  \n",wifi_rx_buffer[0], wifi_rx_buffer[1],wifi_rx_buffer[9]);
		write_wifi_led(true);
		uint8_t size_received = (uint8_t) WIFI_UART.RxXferSize;
		memcpy(reserved_buf[i], wifi_rx_buffer, size_received);
		//debug_print_n("reserved_buf : reserved_buf[0]: %X reserved_buf[1]: %X  reserved_buf[9]: %X  \n",reserved_buf[0], reserved_buf[1],reserved_buf[9]);
		wifi_process((reserved_buf[i]), size_received);
		memset(reserved_buf[i],0,10);
		write_wifi_led(false);
		i++;
		i=i%10;
	}
}

/**
 * uart idle line configuration parameters
 *
 * */
static uint8_t received_buffer[10][10]={0};
static uint8_t previous_count = 0;
static uint8_t current_count = 0;
static uint8_t received_size[30]={0};
static uint8_t received_idle_data[30];
static uint8_t received_idle_dummy[30];

void disable_rxTemporarily()
{
	HAL_UART_MspDeInit(&WIFI_UART);
	HAL_UART_MspInit(&WIFI_UART);
	MX_USART6_UART_Init();
	init_wifi_idle();
}
void init_wifi_idle()
{
	HAL_StatusTypeDef err=HAL_UARTEx_ReceiveToIdle_IT(&WIFI_UART, received_idle_data, LENGTH_ARRAY(received_idle_data));
//	debug_print_n("[wifi] idle err code = %d\n", err);
}

void update_wifi_value()
{
//	debug_print_n("update_wifi_value\n");
	if(previous_count != current_count)
	{
		previous_count = (previous_count+1)%10;
		write_wifi_led(true);
		wifi_process((received_buffer[previous_count]), received_size[previous_count]);
		memset(received_buffer[previous_count],0,received_size[previous_count]);
		write_wifi_led(false);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == WIFI_UART.Instance)
	{
		current_count = (current_count+1)%10;
//		debug_print_n("Current count = %d Previous Count = %d\n", current_count, previous_count);
		memcpy(received_buffer[current_count], received_idle_data, Size);
		received_size[current_count]= Size;
//		disable_rxTemporarily();
		init_wifi_idle();
	}
}

uint8_t getbatteryValue()
{
	return cabin_to_shaft[battery_percentage];
}

bool getContactChargerValue()
{
	return cabin_to_shaft[contact_charger];
}

void setcabinvalueupdated()
{
//	debug_print_n("setcabinvalueupdated \n");
	static long int cabin_timer = 0;
	switch(cabin_connected_state)
	{
	case DEVICE_START_TIMER:
	{
		cabin_timer = HAL_GetTick();
		cabin_connected_state=DEVICE_WAIT_UNTIL_COUNTDOWN;
		setcabinWorking(true);
		write_cabin_led(true);
		debug_print_n("[cabin] [available]\n");
		set_network_availability(0,1);
	}
		break;
	case DEVICE_WAIT_UNTIL_COUNTDOWN:
	{
		if(HAL_GetTick() - cabin_timer > TIMEOUT_WIFI_DEVICES)
		{
			cabin_timer = HAL_GetTick();
			cabin_connected_state=DEVICE_SET_TIMEOUT;
			debug_print_n("[cabin] [timeout exceeded]\n");
		}
	}
		break;
	case DEVICE_SET_TIMEOUT:
	{
		setcabinWorking(false);
		write_cabin_led(false);
		cabin_connected_state=DEVICE_IDLE;
		debug_print_n("[cabin] [disconnected]\n");
		set_network_availability(0,0);
	}
		break;
	case DEVICE_IDLE:
		break;
	default:
	{
		cabin_connected_state=DEVICE_SET_TIMEOUT;
	}
		break;
	}
}

void lockDoorSolenoidAtFloor(FLOOR_NUM floorNum,bool state)
{
//	message_changed = true;

  if (MAX_FLOOR != floorNum)
  {
    if(state)
    {
    	SET_BIT_n(shaft_to_common[door_lock_state], floorNum);
    }
    else
    {
    	shaft_to_common[door_lock_state]=0;
    }
  }
  else
  {
	  shaft_to_common[door_lock_state]=0;
  }
  debug_print_n("[door value]:[%d]\n", shaft_to_common[door_lock_state]);
}

void set_lop_state(uint8_t * flr_num)
{
	int lop_flr_num = flr_num[0];
	if(lop_flr_num < MAX_FLOOR)
	{
//			debug_print_n("[LOP : %d] [Mech Lock %d]\n", lop_flr_num, flr_num[4]);
			lop_data[lop_flr_num].booking_request=flr_num[1];
			lop_data[lop_flr_num].current_flr_display=flr_num[2];
			lop_data[lop_flr_num].door_sensor=flr_num[3];
			lop_data[lop_flr_num].processed_mech_data=(flr_num[4]);
			lop_data[lop_flr_num].ds_status=flr_num[5];
			lop_data[lop_flr_num].update_status=flr_num[6];
			if(lop_data[lop_flr_num].booking_request)
			{
				registerHALcall(lop_flr_num);
			}
			lop_timeout[lop_flr_num] = DEVICE_START_TIMER;
	}
}


/**
 * @brief // true - door close false door open
 *
 * @param floorNum
 * @return true
 * @return false
 */
bool readDoorSwitchStatus(uint8_t floorNum)
{
  if(MAX_FLOOR > floorNum)
  {
    return (bool)lop_data[floorNum].door_sensor;
  }
  return false;
}

int readMechanicalLock(uint8_t flr)
{
	if(flr < MAX_FLOOR)
	{
		return lop_data[flr].processed_mech_data;
	}
	return 2;
}

bool setDoorLockState(FLOOR_NUM flr, bool state)
{
  bool sts;
  sts = false;
  if (flr < MAX_FLOOR)
  {
    if (true == state)
    {
      if (readDoorSwitchStatus(flr))
      {
        SET_BIT_n(doorState, flr);
        lockDoorSolenoidAtFloor(flr, state);
        sts = true;
      }
    }
    else
    {
      CLR_BIT(doorState, flr);
      lockDoorSolenoidAtFloor(MAX_FLOOR, state);
      sts = true;
    }
  }
  else
  {
	  doorState = 0;
	  sts = true;
	  lockDoorSolenoidAtFloor(MAX_FLOOR, state);
  }
  return sts;
}

uint8_t doorGetIOstate()
{
//  debug_print_n(" doorGetIOstate \n");
  static uint8_t prevDoorState = 0;
  uint8_t doorIO = false;

  for(uint8_t flr= false ; flr< MAX_FLOOR; flr++)
  {
    doorIO |= readDoorSwitchStatus((FLOOR_NUM)flr)<<flr;
  }
  if(prevDoorState!=doorIO)
  {
     prevDoorState=doorIO;
     for(int i =0;i<MAX_FLOOR; i++)
     {
    	 write_door_led(i, readDoorSwitchStatus((FLOOR_NUM)i)<<i);
     }
     set_door_state(doorIO);
  }
  return doorIO;
}

bool doorClosed()
{
  bool status = false;
  if(doorGetIOstate() == (pow(2, MAX_FLOOR)-1))
  {
    status = true;
  }
  return status;
}

void monitor_lop_timer()
{
//	debug_print_n(" monitor_lop_timer \n");
	static long int print_lop_status  = 0;
	static uint8_t lop_checker = 0;
	if(HAL_GetTick() - lop_checker > 300)
	{
		lop_checker = HAL_GetTick();
		for(int i = 0; i< MAX_FLOOR; i++)
		{
			checkLOPConnection(i);
		}
	}
	if(HAL_GetTick() - print_lop_status > 3000)
	{
		print_lop_status = HAL_GetTick();
		for(int i=0;i<MAX_FLOOR;i++)
		{
			debug_print_n("LOP[%d]:%s\t", i, lop_available[i]? "Ok":"Not Ok");
		}
		debug_print_n("\n");
	}
}

void checkLOPConnection(int lop_floor)
{
	switch(lop_timeout[lop_floor])
	{
		case DEVICE_START_TIMER:
		{
			lop_timer[lop_floor] = HAL_GetTick();
			lop_timeout[lop_floor] = DEVICE_WAIT_UNTIL_COUNTDOWN;
			lop_available[lop_floor] = 1;
			set_network_availability((lop_floor+1),1);
//			debug_print_n("[LOP%d][available]\n", lop_floor);

		}
		break;
		case DEVICE_WAIT_UNTIL_COUNTDOWN:
		{
			if(HAL_GetTick() - lop_timer[lop_floor] >= TIMEOUT_WIFI_DEVICES)
			{
				lop_timer[lop_floor] = HAL_GetTick();
				lop_timeout[lop_floor] = DEVICE_SET_TIMEOUT;
			}
		}
		break;
		case DEVICE_SET_TIMEOUT:
		{
			set_network_availability((lop_floor+1),0);
			lop_available[lop_floor] = 0;
			debug_print_n("[LOP%d][disconnected]\n", lop_floor);
			lop_timeout[lop_floor] = DEVICE_IDLE;
		}
		break;
		case DEVICE_IDLE:
		{
			//do nothing
		}
		break;
		default:
		{
			lop_timeout[lop_floor] = DEVICE_SET_TIMEOUT;
		}
		break;
	}
}

bool lop_disconnected()
{
	int lop_sum = 0;
	bool disconnected_status = false;
	static bool prev_status = false;
	for(int i = 0; i< MAX_FLOOR; i++)
	{
		lop_sum += lop_available[i];
	}
	if(lop_sum != MAX_FLOOR)
	{
//		debug_print_n("lop sum = %d\n", lop_sum);
		disconnected_status = true;
	}
	if(prev_status != disconnected_status)
	{
		prev_status = disconnected_status;
		for(int i=0;i<MAX_FLOOR;i++)
		{
			write_lop_led(i, lop_available[i]);
		}
	}
	return disconnected_status;
}


bool cabin_battery_acceptable()
{
	static uint8_t battery_critical = 0;
	bool cabin_acceptable_battery_level = false;

	if(battery_critical != cabin_acceptable_battery_level)
	{
		battery_critical = cabin_acceptable_battery_level;
	}
	if(cabin_to_shaft[battery_percentage] >= MINIMUM_ALLOWED_BATTERY)
	{
		cabin_acceptable_battery_level = true;
	}
	else
	{
//		debug_print_n("cabin battery low\n");
	}
	return cabin_acceptable_battery_level;
}


void reset_timer()
{
	if(message_changed)
	{
		timer_expired = (HAL_GetTick() + MESSAGE_QUERY_TIMER - RESET_TIMER_OFFSET);
		message_changed = false;
	}
}

bool return_update()
{
	bool update_status = false;
	static bool prev_status = false;
	update_status |= cabin_to_shaft[cabin_update_status];
	for(int i=0;i<MAX_FLOOR;i++)
	{
		update_status |=lop_data[i].update_status;
	}
	if(prev_status != update_status)
	{
		prev_status = update_status;
		set_device_availability(DEVICE_UNDER_OTA, update_status);
	}
	return cabin_to_shaft[cabin_update_status];
}

void send_lidar_value()
{
	autoCalibDataReq=true;
}

void set_device_under_update()
{
	debug_print_n("shaft esp32 under update\n");
	device_ota_progress(SHAFT_OTA_BEGIN);
  set_device_availability(DEVICE_UNDER_OTA, true);
}

void clear_device_under_update()
{
	device_ota_progress(SHAFT_OTA_SUCESS);
	debug_print_n("shaft esp32 update done\n");
	set_device_availability(DEVICE_UNDER_OTA, false);
}

void send_latest_value()
{
	autoCalibDataReq=1;
}

void store_values()
{
	debug_print_n("store configurations in memory\n");
	writeConfigFile(false);
	HAL_Delay(3500);
	device_restart();
}

void device_restart()
{
	debug_print_n("system reset \n");
	HAL_NVIC_SystemReset();
}
