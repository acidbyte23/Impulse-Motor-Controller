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
#include "stm32f4xx_hal.h"

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
#define Temperature_Pin GPIO_PIN_0
#define Temperature_GPIO_Port GPIOC
#define Voltage_Sense_Pin GPIO_PIN_0
#define Voltage_Sense_GPIO_Port GPIOA
#define Current_Sense_Pin GPIO_PIN_1
#define Current_Sense_GPIO_Port GPIOA
#define Setting_Pw_Frq_Pin GPIO_PIN_2
#define Setting_Pw_Frq_GPIO_Port GPIOA
#define Setting_Delay_Pin GPIO_PIN_3
#define Setting_Delay_GPIO_Port GPIOA
#define High_Side_Pole_Pin GPIO_PIN_0
#define High_Side_Pole_GPIO_Port GPIOB
#define High_Side_Pole_EXTI_IRQn EXTI0_IRQn
#define Low_Side_Pole_Pin GPIO_PIN_1
#define Low_Side_Pole_GPIO_Port GPIOB
#define Low_Side_Pole_EXTI_IRQn EXTI1_IRQn
#define AlarmActive_Pin GPIO_PIN_13
#define AlarmActive_GPIO_Port GPIOB
#define motorEnable_Pin GPIO_PIN_14
#define motorEnable_GPIO_Port GPIOB
#define motorEnableOut_Pin GPIO_PIN_6
#define motorEnableOut_GPIO_Port GPIOC
#define High_Side_Pole_Out_Pin GPIO_PIN_8
#define High_Side_Pole_Out_GPIO_Port GPIOC
#define UartTX_Pin GPIO_PIN_12
#define UartTX_GPIO_Port GPIOC
#define UartRX_Pin GPIO_PIN_2
#define UartRX_GPIO_Port GPIOD
#define Low_Side_Pole_Out_Pin GPIO_PIN_6
#define Low_Side_Pole_Out_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
