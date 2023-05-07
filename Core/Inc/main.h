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
#include "stm32f1xx_hal.h"

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
#define NTC_2_Pin GPIO_PIN_0
#define NTC_2_GPIO_Port GPIOA
#define NTC_1_Pin GPIO_PIN_1
#define NTC_1_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_2
#define NRF_CE_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_3
#define NRF_IRQ_GPIO_Port GPIOA
#define NRF_NCS_Pin GPIO_PIN_4
#define NRF_NCS_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_5
#define NRF_SCK_GPIO_Port GPIOA
#define NRF_MISO_Pin GPIO_PIN_6
#define NRF_MISO_GPIO_Port GPIOA
#define NRF_MOSI_Pin GPIO_PIN_7
#define NRF_MOSI_GPIO_Port GPIOA
#define OAGP_Pin GPIO_PIN_0
#define OAGP_GPIO_Port GPIOB
#define CURR_Pin GPIO_PIN_1
#define CURR_GPIO_Port GPIOB
#define PWR_3V3_Pin GPIO_PIN_10
#define PWR_3V3_GPIO_Port GPIOB
#define TEMP_OS_Pin GPIO_PIN_7
#define TEMP_OS_GPIO_Port GPIOB
#define TEMP_SCL_Pin GPIO_PIN_8
#define TEMP_SCL_GPIO_Port GPIOB
#define TEMP_SDA_Pin GPIO_PIN_9
#define TEMP_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
