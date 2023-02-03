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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_6_Pin GPIO_PIN_0
#define DEBUG_6_GPIO_Port GPIOC
#define DEBUG_5_Pin GPIO_PIN_1
#define DEBUG_5_GPIO_Port GPIOC
#define DEBUG_4_Pin GPIO_PIN_2
#define DEBUG_4_GPIO_Port GPIOC
#define DEBUG_3_Pin GPIO_PIN_3
#define DEBUG_3_GPIO_Port GPIOC
#define DEBUG_2_Pin GPIO_PIN_0
#define DEBUG_2_GPIO_Port GPIOA
#define DEBUG_1_Pin GPIO_PIN_1
#define DEBUG_1_GPIO_Port GPIOA
#define BAT_TEMP_Pin GPIO_PIN_4
#define BAT_TEMP_GPIO_Port GPIOC
#define LVL_SFTR_OE_2_Pin GPIO_PIN_0
#define LVL_SFTR_OE_2_GPIO_Port GPIOB
#define STPR_FLT_2_Pin GPIO_PIN_1
#define STPR_FLT_2_GPIO_Port GPIOB
#define STPR_DIR_2_Pin GPIO_PIN_2
#define STPR_DIR_2_GPIO_Port GPIOB
#define STPR_EN_2_Pin GPIO_PIN_10
#define STPR_EN_2_GPIO_Port GPIOB
#define STPR_PWM_2_Pin GPIO_PIN_11
#define STPR_PWM_2_GPIO_Port GPIOB
#define ID_0_Pin GPIO_PIN_12
#define ID_0_GPIO_Port GPIOB
#define ID_1_Pin GPIO_PIN_13
#define ID_1_GPIO_Port GPIOB
#define ID_2_Pin GPIO_PIN_14
#define ID_2_GPIO_Port GPIOB
#define ID_3_Pin GPIO_PIN_15
#define ID_3_GPIO_Port GPIOB
#define E_STOP_Pin GPIO_PIN_9
#define E_STOP_GPIO_Port GPIOC
#define CAN_HEARTBIT_Pin GPIO_PIN_8
#define CAN_HEARTBIT_GPIO_Port GPIOA
#define STPR_PWM_1_Pin GPIO_PIN_15
#define STPR_PWM_1_GPIO_Port GPIOA
#define STPR_EN_1_Pin GPIO_PIN_10
#define STPR_EN_1_GPIO_Port GPIOC
#define STPR_DIR_1_Pin GPIO_PIN_11
#define STPR_DIR_1_GPIO_Port GPIOC
#define STPR_FLT_1_Pin GPIO_PIN_12
#define STPR_FLT_1_GPIO_Port GPIOC
#define LVL_SFTR_OE_1_Pin GPIO_PIN_2
#define LVL_SFTR_OE_1_GPIO_Port GPIOD
#define BRAKE_IN_Pin GPIO_PIN_9
#define BRAKE_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
