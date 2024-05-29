/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define emergency_input_Pin GPIO_PIN_2
#define emergency_input_GPIO_Port GPIOE
#define overload_input_Pin GPIO_PIN_3
#define overload_input_GPIO_Port GPIOE
#define AUTO_CALIB_Pin GPIO_PIN_4
#define AUTO_CALIB_GPIO_Port GPIOE
#define current_sense_input_Pin GPIO_PIN_5
#define current_sense_input_GPIO_Port GPIOE
#define shaft_lidar_rx_Pin GPIO_PIN_6
#define shaft_lidar_rx_GPIO_Port GPIOF
#define shaft_lidar_tx_Pin GPIO_PIN_7
#define shaft_lidar_tx_GPIO_Port GPIOF
#define shift_reg2_data_Pin GPIO_PIN_0
#define shift_reg2_data_GPIO_Port GPIOA
#define shift_reg2_rclk_Pin GPIO_PIN_1
#define shift_reg2_rclk_GPIO_Port GPIOA
#define shift_reg2_srclk_Pin GPIO_PIN_2
#define shift_reg2_srclk_GPIO_Port GPIOA
#define shift_reg3_data_Pin GPIO_PIN_4
#define shift_reg3_data_GPIO_Port GPIOA
#define shift_reg3_rclk_Pin GPIO_PIN_5
#define shift_reg3_rclk_GPIO_Port GPIOA
#define shift_reg3_srclk_Pin GPIO_PIN_6
#define shift_reg3_srclk_GPIO_Port GPIOA
#define shift_reg1_data_Pin GPIO_PIN_4
#define shift_reg1_data_GPIO_Port GPIOC
#define shift_reg1_rclk_Pin GPIO_PIN_5
#define shift_reg1_rclk_GPIO_Port GPIOC
#define shift_reg1_srclk_Pin GPIO_PIN_0
#define shift_reg1_srclk_GPIO_Port GPIOB
#define Zero_Cross_Output_Pin GPIO_PIN_9
#define Zero_Cross_Output_GPIO_Port GPIOE
#define Zero_Cross_Input_Pin GPIO_PIN_11
#define Zero_Cross_Input_GPIO_Port GPIOE
#define External_IO1_Pin GPIO_PIN_12
#define External_IO1_GPIO_Port GPIOE
#define External_IO2_Pin GPIO_PIN_13
#define External_IO2_GPIO_Port GPIOE
#define External_IO3_Pin GPIO_PIN_14
#define External_IO3_GPIO_Port GPIOE
#define External_IO4_Pin GPIO_PIN_15
#define External_IO4_GPIO_Port GPIOE
#define debug_TX_Pin GPIO_PIN_10
#define debug_TX_GPIO_Port GPIOB
#define debug_RX_Pin GPIO_PIN_11
#define debug_RX_GPIO_Port GPIOB
#define UI_RX_Pin GPIO_PIN_12
#define UI_RX_GPIO_Port GPIOB
#define UI_TX_Pin GPIO_PIN_13
#define UI_TX_GPIO_Port GPIOB
#define External_IO5_Pin GPIO_PIN_8
#define External_IO5_GPIO_Port GPIOD
#define Extenal_IO6_Pin GPIO_PIN_9
#define Extenal_IO6_GPIO_Port GPIOD
#define m7_health_Pin GPIO_PIN_7
#define m7_health_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOG
#define ESP32_TX_Pin GPIO_PIN_6
#define ESP32_TX_GPIO_Port GPIOC
#define ESP32_RX_Pin GPIO_PIN_7
#define ESP32_RX_GPIO_Port GPIOC
#define public_health_Pin GPIO_PIN_3
#define public_health_GPIO_Port GPIOD
#define buzzer_Pin GPIO_PIN_4
#define buzzer_GPIO_Port GPIOD
#define power_input_Pin GPIO_PIN_5
#define power_input_GPIO_Port GPIOD
#define evo_relay_Pin GPIO_PIN_7
#define evo_relay_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
