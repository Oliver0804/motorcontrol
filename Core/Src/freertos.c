/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int speed_flag = 2000;
int timer_conut = 10;
uint32_t ADC_Value[100];
uint32_t ad1, ad2;
uint8_t i;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTaskoutput */
osThreadId_t myTaskoutputHandle;
const osThreadAttr_t myTaskoutput_attributes = {
  .name = "myTaskoutput",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_3: // GPIO_PIN_13 is the Blue Button
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		break;
	case GPIO_PIN_4:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		break;
	case GPIO_PIN_5:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		break;
	case GPIO_PIN_6:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		break;
	}
}

void zheng_zhuan(void) {
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
}
void fan_zhuan(void) {
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
}
void stop_motor(void) {
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
}
void lock_motor(void) {
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
}

void user_pwm_setvalue_1(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}
void user_pwm_setvalue_2(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskoutput */
  myTaskoutputHandle = osThreadNew(StartTask02, NULL, &myTaskoutput_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

	/* Infinite loop */
	for (;;) {
		user_pwm_setvalue_1(1000);
		user_pwm_setvalue_2(1500);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(1);
		//osDelay(speed_flag);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTaskoutput thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_Value, 100);
	/* Infinite loop */
	for (;;) {
		for (i = 0, ad1 = 0, ad2 = 0; i < 100;) {
			ad1 += ADC_Value[i++];
			ad2 += ADC_Value[i++];
		}
		ad1 /= 50;
		ad2 /= 50;
		//HAL_GPIO_TogglePin(GPIOx, GPIO_Pin)
		osDelay(1);
	}
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
