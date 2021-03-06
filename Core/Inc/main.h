/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_7
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_0
#define ADC2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_10
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOB
#define MC_4_Pin GPIO_PIN_14
#define MC_4_GPIO_Port GPIOB
#define MC_3_Pin GPIO_PIN_15
#define MC_3_GPIO_Port GPIOB
#define MC_2_Pin GPIO_PIN_8
#define MC_2_GPIO_Port GPIOA
#define MC_1_Pin GPIO_PIN_9
#define MC_1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define BUTTON1_Pin GPIO_PIN_4
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON1_EXTI_IRQn EXTI4_IRQn
#define BUTTON2_Pin GPIO_PIN_5
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON2_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON3_Pin GPIO_PIN_6
#define BUTTON3_GPIO_Port GPIOB
#define BUTTON3_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON4_Pin GPIO_PIN_7
#define BUTTON4_GPIO_Port GPIOB
#define BUTTON4_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
