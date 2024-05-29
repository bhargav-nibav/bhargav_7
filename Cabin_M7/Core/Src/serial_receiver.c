/*
 * serial_receiver.c
 *
 *  Created on: Oct 25, 2023
 *      Author: ADMIN
 */

#include "usart.h"
#include "serial_receiver.h"
#include "debug_print.h"
#include "common.h"
#include "android_handler.h"
#include "landing_lever.h"
#include "pwm_devices.h"
#include "linear_actuator.h"

#define MESSAGE_SIZE 64
#define QUEUE_SIZE 3
#define SIREN_TIMEOUT 5000

/*
 * test code to enable queue in c. untested
 * */

//typedef struct {
//	char data[MESSAGE_SIZE];
//} MESSAGE;
//
//typedef struct {
//	MESSAGE messages[QUEUE_SIZE];
//	int begin;
//	int end;
//	int current_load;
//} QUEUE;
//
//void init_queue(QUEUE *queue) {
//	queue->begin = 0;
//	queue->end = 0;
//	queue->current_load = 0;
//	memset(&queue->messages[0], 0, QUEUE_SIZE * sizeof(MESSAGE_SIZE));
//}
//
//bool enque(QUEUE *queue, MESSAGE *message) {
//	if (queue->current_load < QUEUE_SIZE) {
//		if (queue->end == QUEUE_SIZE) {
//			queue->end = 0;
//		}
//		queue->messages[queue->end] = *message;
//		queue->end++;
//		queue->current_load++;
//		return true;
//	} else {
//		return false;
//	}
//}
//
//bool deque(QUEUE *queue, MESSAGE *message) {
//	if (queue->current_load > 0) {
//		*message = queue->messages[queue->begin];
//		memset(&queue->messages[queue->begin], 0, sizeof(MESSAGE));
//		queue->begin = (queue->begin + 1) % QUEUE_SIZE;
//		queue->current_load--;
//		return true;
//	} else {
//		return false;
//	}
//}

//    QUEUE queue;
// init_queue(&queue);
// MESSAGE message1 = {"This is"};
// enque(&queue, &message3);
// MESSAGE rec;
// while (deque(&queue, &rec)) {
//    printf("%s\n", &rec.data[0]);
//}
// MESSAGE rec;

#define BUTTON_PRESS 0x00
#define FLOOR_BUTTON 0X00

#define PWM_SLIDER_USED 0x04
#define PWM_INTENSITY_SETTING 0X00
#define PWM_DURATION_SETTING 0x01


uint8_t android_rx_buffer[8];
uint8_t wifi_rx_buffer[8];

void wifi_process(uint8_t *data_rc, int n);

void monitor_wifi()
{
  static uint8_t reserved_buf[8];
  if(HAL_UART_Receive(&WIFI_UART, wifi_rx_buffer, LENGTH_OF_ARRAY(wifi_rx_buffer),10)==HAL_OK)
  {
      HAL_GPIO_WritePin(ESP32_INDICATION_GPIO_Port, ESP32_INDICATION_Pin, 1);
      uint8_t size_received = (uint8_t) WIFI_UART.RxXferSize;
      memcpy(reserved_buf, wifi_rx_buffer, size_received);
      wifi_process(reserved_buf, size_received);
      memset(reserved_buf, 0, 8);
      HAL_GPIO_WritePin(ESP32_INDICATION_GPIO_Port, ESP32_INDICATION_Pin, 0);
  }
}

void wifi_process(uint8_t *data_rc, int n)
{
	if (data_rc[n - 1] == 0xff) {
		if(data_rc[0] == 0x65 )
		{
			switch(data_rc[1])
			{
			case PWM_SLIDER_USED:
			{
			    if(data_rc[2]==2)
			      {
				debug_print("PWM message received from cabin\n");
				setPWM(LOGO_PWM, data_rc[4], data_rc[3]);
			      }
			    setPWM(data_rc[2], data_rc[4], data_rc[3]);
			}
			break;
			case BUTTON_PRESS:
			{
				switch(data_rc[2])
				{
					case 0x09:
					{
						if(data_rc[3]==0x02)
						{
//							debug_print("ll message received for value is %d\n", data_rc[4]);
							set_landing_lever(CHECK_BIT(data_rc[4], 0));
							setLinearActuator(CHECK_BIT(data_rc[4], 4));
						}
					}
					break;
					case 0x0A:
					{
						//send latest data to android
					}
					break;
					default:
					  {

					  }
					  break;
				}
			}
				break;
			default:
			  {

			  }
			  break;
			}
		}
	}
}


