/*
 * configuration.h
 *
 *  Created on: Dec 13, 2023
 *      Author: Balamurugan S
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include <stdbool.h>
#include <stdint.h>

#include "common.h"
#define FLASH_START_ADDR               0x080F0000

typedef struct __attribute__((packed,aligned(__alignof__(unsigned int))))
{
  uint32_t numberOfFloors;
  uint32_t LIDAR[MAX_FLOOR];
  uint32_t door_solenoid_open_duration;
  uint32_t lidar_range_check_offset;
  uint32_t speed_mode;
  uint32_t reserved_val2;
  uint32_t crc;
}CONFIG_FILE;

void configFileInit();
CONFIG_FILE *getConfig();
bool writeConfigFile(bool _status);
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *data, uint16_t numberofwords);
bool writeToFlash(uint32_t address, uint32_t *dataBuffer, uint32_t dataSize);


#endif /* INC_CONFIGURATION_H_ */
