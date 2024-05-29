/*
 * configuration.cpp
 *
 *  Created on: Dec 13, 2023
 *      Author: ADMIN
 */


//#include "SPIFFS.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "crc.h"

#include "common.h"
#include "configuration.h"

#include "serial_handler.h"
#include "debug_print.h"
#include "call_serving.h"

// 0x0800C000
#define FLASH_TYPEPROGRAM_WORD        ((uint32_t)0x02U)
#define WORDS_TO_READ                  5
#define FLASHWORD					   8

CONFIG_FILE config_file;// = NULL;
const uint16_t config_size_m = sizeof(config_file) / 4;

bool readConfigFile();
static void configurationInitHardCode();
static uint32_t GetSector(uint32_t Address);
void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);
/**************************************************************************************/

void configFileInit()
{
	bool lidar_state = readConfigFile();
	if (!lidar_state)
	{
		debug_print_n("Flash read failed update new file\r\n");
		configurationInitHardCode();
	}
	else
	{
		debug_print_n("Flash read updated new file\r\n");
		lidar_state=true;
	}
	update_lidar_values(lidar_state);
	if(writeConfigFile(lidar_state))
	{
		debug_print_n("write to memory sucessful\n");
	}
	else
	{
		debug_print_n("write to memory unsucessful\n");
	}
}

bool writeConfigFile(bool _status)
{
	if(!_status)
	{
//		CONFIG_FILE *saving_file = getConfig();
		uint32_t write_data[config_size_m];
		memcpy(write_data, &config_file, config_size_m*4);
//		debug_print_n("config door %x\n",saving_file->door_solenoid_open_duration);
		config_file.crc = HAL_CRC_Calculate(&hcrc, write_data, config_size_m - 1);
		debug_print_n("[crc calculated] %x\n", config_file.crc);
		write_data[config_size_m - 1] = config_file.crc;
		for(int i=0;i<config_size_m;i++)
		{
			debug_print_n("write data %x\n",write_data[i]);
		}
		Flash_Write_Data(FLASH_START_ADDR, write_data, config_size_m);
	}
	return true;
}

void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	static long int_timer_interval = 0;
	int_timer_interval = HAL_GetTick();
	while ((HAL_GetTick() - int_timer_interval < 2000))
	{
		*RxBuf = *(__IO uint32_t*) StartSectorAddress;
		debug_print_n(" StartSectorAddress %X RxBuf %X \n", StartSectorAddress,
				*RxBuf);
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--))

			break;
	}
}

bool readConfigFile()
{
    uint32_t recv_buffer[config_size_m];
    bool read_status = false;
    Flash_Read_Data(FLASH_START_ADDR, recv_buffer, config_size_m);
    if (HAL_CRC_Calculate(&hcrc, recv_buffer, config_size_m - 1) == recv_buffer[config_size_m - 1])
    {
    	debug_print_n("crc match\n");
    	memcpy(&config_file, recv_buffer, config_size_m*4);
//        config_file = (CONFIG_FILE )(&recv_buffer);
        read_status = true;
    }
    else
    {
    	debug_print_n("crc fail\n");
    }
    return read_status;
}

CONFIG_FILE* getConfig() {
	// debug_print_n("accessed now\n");
	return &config_file;
}


/*
 * set default values for all parameters except lidar here
 * */
static void configurationInitHardCode()
{
	CONFIG_FILE *configFile = getConfig();
	configFile->numberOfFloors = MAX_FLOOR;
	configFile->door_solenoid_open_duration 	= DOOR_OPEN_TIMES;
	configFile->lidar_range_check_offset = FLOOR_RANGE_OFFSET_VAL/10;
	configFile->speed_mode = SPEED_LOW;
}

/* The DATA to be written here MUST be according to the List Shown Below

For EXAMPLE:- For H74x/5x, a single data must be 8 numbers of 32 bits word
If you try to write a single 32 bit word, it will automatically write 0's for the rest 7

*          - 256 bits for STM32H74x/5X devices (8x 32bits words)
*          - 128 bits for STM32H7Ax/BX devices (4x 32bits words)
*          - 256 bits for STM32H72x/3X devices (8x 32bits words)
*
*/

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	  uint32_t StartSector = GetSector(StartSectorAddress);
	  uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
	  uint32_t EndSector = GetSector(EndSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;

	  // The the proper BANK to erase the Sector
	  if (StartSectorAddress < 0x08100000)
		  EraseInitStruct.Banks     = FLASH_BANK_1;
	  else EraseInitStruct.Banks    = FLASH_BANK_2;

	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;


	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
		  return HAL_FLASH_GetError ();
	  }

	  /* Program the user Flash area 8 WORDS at a time
	   * (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, StartSectorAddress, (uint32_t ) &data[sofar]) == HAL_OK)
	     {
	    	 StartSectorAddress += 4*FLASHWORD;  //
	    	 sofar+=FLASHWORD;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}



// There are 2 BANKS available for H745, BANK 1 (0x0800 0000 - 0x080F FFFF) and BANK 2 (0x0810 0000 - 0x080F FFFF)
// Both of them have Sectors 0 to 7.
// We will define the sectors in normal way (like Defined below), and later the BANK will be taken care by the HAL

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  /* BANK 1 */
  if((Address >= 0x08000000) && (Address < 0x08020000))
  {
    sector = FLASH_SECTOR_0;
  }

  else if((Address >= 0x08020000) && (Address < 0x08040000))
  {
    sector = FLASH_SECTOR_1;
  }

  else if((Address >= 0x08040000) && (Address < 0x08060000))
  {
    sector = FLASH_SECTOR_2;
  }

  else if((Address >= 0x08060000) && (Address < 0x08080000))
  {
    sector = FLASH_SECTOR_3;
  }

  else if((Address >= 0x08080000) && (Address < 0x080A0000))
  {
    sector = FLASH_SECTOR_4;
  }

  else if((Address >= 0x080A0000) && (Address < 0x080C0000))
  {
    sector = FLASH_SECTOR_5;
  }

  else if((Address >= 0x080C0000) && (Address < 0x080E0000))
  {
    sector = FLASH_SECTOR_6;
  }

  else if((Address >= 0x080E0000) && (Address < 0x08100000))
  {
    sector = FLASH_SECTOR_7;
  }


  /* BANK 2 */
  if((Address >= 0x08100000) && (Address < 0x08120000))
  {
    sector = FLASH_SECTOR_0;
  }

  else if((Address >= 0x08120000) && (Address < 0x08140000))
  {
    sector = FLASH_SECTOR_1;
  }

  else if((Address >= 0x08140000) && (Address < 0x08160000))
  {
    sector = FLASH_SECTOR_2;
  }

  else if((Address >= 0x08160000) && (Address < 0x08180000))
  {
    sector = FLASH_SECTOR_3;
  }

  else if((Address >= 0x08180000) && (Address < 0x081A0000))
  {
    sector = FLASH_SECTOR_4;
  }

  else if((Address >= 0x081A0000) && (Address < 0x081C0000))
  {
    sector = FLASH_SECTOR_5;
  }

  else if((Address >= 0x081C0000) && (Address < 0x081E0000))
  {
    sector = FLASH_SECTOR_6;
  }

  else if((Address >= 0x081E0000) && (Address < 0x08200000))
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}


void printLidarValue()
{
  // print cabinMacId in for loop
  //  for (int i = 0; i < LENGTH_ARRAY(cabinMacId); i++)
  //  {
  //    Serial.printf("%02X ", cabinMacId[i]);
  //  }
}



