/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define S1_EN_Pin GPIO_PIN_11
#define S1_EN_GPIO_Port GPIOC
#define S6_DIR_Pin GPIO_PIN_6
#define S6_DIR_GPIO_Port GPIOC
#define S6_EN_Pin GPIO_PIN_7
#define S6_EN_GPIO_Port GPIOC
#define S6_STEP_Pin GPIO_PIN_8
#define S6_STEP_GPIO_Port GPIOD
#define S5_DIR_Pin GPIO_PIN_9
#define S5_DIR_GPIO_Port GPIOD
#define S5_STEP_Pin GPIO_PIN_11
#define S5_STEP_GPIO_Port GPIOD
#define S5_EN_Pin GPIO_PIN_15
#define S5_EN_GPIO_Port GPIOD
#define S4_STEP_Pin GPIO_PIN_10
#define S4_STEP_GPIO_Port GPIOA
#define S4_DIR_Pin GPIO_PIN_14
#define S4_DIR_GPIO_Port GPIOA
#define S4_EN_Pin GPIO_PIN_15
#define S4_EN_GPIO_Port GPIOA
#define S4_END5_Pin GPIO_PIN_5
#define S4_END5_GPIO_Port GPIOD
#define S3_DIR_Pin GPIO_PIN_6
#define S3_DIR_GPIO_Port GPIOD
#define S3_STEP_Pin GPIO_PIN_7
#define S3_STEP_GPIO_Port GPIOD
#define S3_EN_Pin GPIO_PIN_10
#define S3_EN_GPIO_Port GPIOF
#define S2_DIR_Pin GPIO_PIN_11
#define S2_DIR_GPIO_Port GPIOF
#define S2_STEP_Pin GPIO_PIN_12
#define S2_STEP_GPIO_Port GPIOF
#define S2_EN_Pin GPIO_PIN_3
#define S2_EN_GPIO_Port GPIOB
#define S1_DIR_Pin GPIO_PIN_4
#define S1_DIR_GPIO_Port GPIOB
#define S1_STEP_Pin GPIO_PIN_2
#define S1_STEP_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
