/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
#define TEST_LED_Pin GPIO_PIN_1
#define TEST_LED_GPIO_Port GPIOC
#define RX_ENABLE_Pin GPIO_PIN_4
#define RX_ENABLE_GPIO_Port GPIOA
#define AC_Ctrl_Pin GPIO_PIN_5
#define AC_Ctrl_GPIO_Port GPIOA
#define RUN_LED_Pin GPIO_PIN_7
#define RUN_LED_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_10
#define RS485_DE_GPIO_Port GPIOA
#define RS485_ID0_Pin GPIO_PIN_6
#define RS485_ID0_GPIO_Port GPIOB
#define RS485_ID1_Pin GPIO_PIN_5
#define RS485_ID1_GPIO_Port GPIOB
#define RS485_ID2_Pin GPIO_PIN_4
#define RS485_ID2_GPIO_Port GPIOB
#define RS485_ID3_Pin GPIO_PIN_3
#define RS485_ID3_GPIO_Port GPIOB
#define RS485_ID4_Pin GPIO_PIN_12
#define RS485_ID4_GPIO_Port GPIOC
#define RS485_ID5_Pin GPIO_PIN_11
#define RS485_ID5_GPIO_Port GPIOC
#define RS485_ID6_Pin GPIO_PIN_10
#define RS485_ID6_GPIO_Port GPIOC
#define RS485_ID7_Pin GPIO_PIN_15
#define RS485_ID7_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
