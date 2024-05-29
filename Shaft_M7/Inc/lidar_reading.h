 /*
 * lidar_reading.h
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */

#ifndef INC_LIDAR_READING_H_
#define INC_LIDAR_READING_H_

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_UPDATE_TIMER			5 // changed from 5 to 1 by 7436
#define LIDAR_COM_TIMEOUT         	5

typedef enum
{
  CONTROL_NO_OPERATION,
  CONTROL_STARTED,
  CONTROL_UP_DIR,
  CONTROL_DOWN_DIR,
  CONTROL_ACHIVED,
  CONTROL_ERROR

}CONTROL_DIR;

uint16_t getDistance();
uint16_t getSigStrenth();
bool isLIDARWorks();

void setStrength(uint16_t strength);
void setDistance(uint16_t val);
void setLidarWorking(bool status);
void setLidarInMmMode();
bool lidar_available();
void getTFminiData();
void lidarSaveSettings();
void setLidarFrameRate();
void setupLidarDMA();
void updateLidarValue();
void setlidarvalueupdated();
void setLidarITMode();
void setupLidarConfig();
void setLidarBaudRate();
#endif /* INC_LIDAR_READING_H_ */
