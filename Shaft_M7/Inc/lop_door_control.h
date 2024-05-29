#include "common.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>



#define DOOR_OPEN_TIMES    30000
//#define DOOR_OPEN_TIME1     30000

typedef enum DOOR_STem
{
  DOOR_IDLE_ST,
  DOOR_OPEN_REQ,
  DOOR_CLOSE_REQ,
  DOOR_WAIT_FOR_OPEN,
  DOOR_OPEN_MSG_TO_CAB,
  DOOR_OPEN_TIMEOUT,
  DOOR_WAIT_FOR_CLOSE,
  DOOR_KEEP_CLOSED,
  DOOR_CLOSE_MSG_TO_CAB,
  DOOR_OPEN_ALARM_ACT,
  // DOOR_OPEN_ALARM_ACT_VALIDATE,
  DOOR_WAIT_ALARM_CLR_STATE,
  DOOR_OPEN_ALARM_CLR,
  // DOOR_OPEN_ALARM_CLR_VALIDATE,
  DOOR_CLOSED_ST
}DOOR_STATE_em;

typedef enum DOOR_STATUS
{
 FIRST_FLOOR_DOOR_OPEN_REQ,
 FIRST_FLOOR_DOOR_OPEN,
 ESP_DOOROPEN_SEND,
 FIRST_FLOOR_DOOR_CLOSE,
 ESP_DOORCLOSE_SEND,
 //SECOND_FLOOR_DOOR_OPEN,
 //SECOND_FLOOR_DOOR_CLOSE,
}DOOR_Stus;

typedef enum ML_STATUS
{
 FIRST_FLOOR_ML_OPEN_REQ,
 FIRST_FLOOR_ML_OPEN,
 ESP_MLOPEN_SEND,
 FIRST_FLOOR_ML_CLOSE,
 ESP_MLCLOSE_SEND,
 //SECOND_FLOOR_DOOR_OPEN,
 //SECOND_FLOOR_DOOR_CLOSE,
}ML_Stus;


void doorInit(uint8_t flr);

void openDoor(FLOOR_NUM floorNumber);

void doorStateProcess();
bool checkAllDoorLockClosed();
void doorKeepClosed();
bool checkDoorAllClosed();
bool checkMLAllClosed();
uint8_t mechLockGetIOstate();
void mechIOState();
bool isDoorAlarmActive();
void closeDoor();
bool doorClosed();
bool checkOtherDoors(FLOOR_NUM flr);
bool keepDoorOpen(FLOOR_NUM flr);
//#ifdef __cplusplus
//}
//#endif /* __cplusplus */


bool checkKeepDoorOpen(FLOOR_NUM flr);
void test_door_inputs();
