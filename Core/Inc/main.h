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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIRECTION_1_Pin GPIO_PIN_3
#define DIRECTION_1_GPIO_Port GPIOC
#define CHB_3_Pin GPIO_PIN_2
#define CHB_3_GPIO_Port GPIOA
#define DIRECTION_3_Pin GPIO_PIN_3
#define DIRECTION_3_GPIO_Port GPIOA
#define CHB_2_Pin GPIO_PIN_5
#define CHB_2_GPIO_Port GPIOC
#define CHA_2_Pin GPIO_PIN_8
#define CHA_2_GPIO_Port GPIOC
#define DIRECTION_2_Pin GPIO_PIN_12
#define DIRECTION_2_GPIO_Port GPIOA
#define CHA_1_Pin GPIO_PIN_10
#define CHA_1_GPIO_Port GPIOC
#define CHB_1_Pin GPIO_PIN_12
#define CHB_1_GPIO_Port GPIOC
#define CHA_3_Pin GPIO_PIN_3
#define CHA_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
