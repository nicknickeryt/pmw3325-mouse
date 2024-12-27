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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi1;
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
#define ENCB_Pin GPIO_PIN_0
#define ENCB_GPIO_Port GPIOA
#define ENCB_EXTI_IRQn EXTI0_1_IRQn
#define ENCA_Pin GPIO_PIN_15
#define ENCA_GPIO_Port GPIOA
#define ENCA_EXTI_IRQn EXTI4_15_IRQn
#define RIGHT_Pin GPIO_PIN_3
#define RIGHT_GPIO_Port GPIOB
#define RIGHT_EXTI_IRQn EXTI2_3_IRQn
#define CENTER_Pin GPIO_PIN_4
#define CENTER_GPIO_Port GPIOB
#define CENTER_EXTI_IRQn EXTI4_15_IRQn
#define LEFT_Pin GPIO_PIN_5
#define LEFT_GPIO_Port GPIOB
#define LEFT_EXTI_IRQn EXTI4_15_IRQn
#define CPI_Pin GPIO_PIN_6
#define CPI_GPIO_Port GPIOB
#define CPI_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
