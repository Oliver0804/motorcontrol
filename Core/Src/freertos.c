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
#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f1xx_hal_flash.h"
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
int run_time_n = 0;
int run_time_s = 0;
int run_pwm_n = 2000;
int run_pwm_s = 500;

uint32_t ADC_Value[100];
uint32_t ad1, ad2;
uint32_t real_adc1, real_adc2;
int i = 0;
int buttom_flag[5] = { 0 };

int sys_mode = 0;
int sys_setting = 0;
int motor_dir = 0;
int smooth_mode = 0;
uint8_t rxData[] = { 0 };
uint8_t UartTxBuf[100];

uint32_t writeFlashData = 0x55555555;
uint32_t addr = 0x08007000;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for myTaskoutput */
osThreadId_t myTaskoutputHandle;
const osThreadAttr_t myTaskoutput_attributes = { .name = "myTaskoutput",
		.priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };
/* Definitions for myTask_BUTTON */
osThreadId_t myTask_BUTTONHandle;
const osThreadAttr_t myTask_BUTTON_attributes = { .name = "myTask_BUTTON",
		.priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//FLASH寫入數據測試
void writeFlashTest(void) {
	//1、解鎖FLASH
	HAL_FLASH_Unlock();

	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;
	//設置PageError
	uint32_t PageError = 0;
	//調用擦除函數
	HAL_FLASHEx_Erase(&f, &PageError);

	//3、對FLASH燒寫

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, writeFlashData);
	//4、鎖住FLASH
	HAL_FLASH_Lock();
}

//FLASH讀取數據測試
void printFlashTest(void) {
	uint32_t temp = *(__IO uint32_t*) (addr);

	Usart2DmaPrintf("addr:0x%x, data:0x%x\r\n", addr, temp);
}

void Usart2DmaPrintf(const char *format, ...) {
	uint16_t len;
	va_list args;
	va_start(args, format);
	len = vsnprintf((char*) UartTxBuf, sizeof(UartTxBuf) + 1, (char*) format,
			args);
	va_end(args);
	HAL_UART_Transmit_DMA(&huart2, UartTxBuf, len);
	osDelay(10); //避免資料穿插

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_4: // GPIO_PIN_13 is the Blue Button
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		buttom_flag[1]++;
		break;
	case GPIO_PIN_5:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		buttom_flag[2]++;
		break;
	case GPIO_PIN_6:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		buttom_flag[3]++;
		break;
	case GPIO_PIN_7:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //PC13 Led
		buttom_flag[4]++;
		break;
	}
}

void clean_buttom_flag(void) {
	HAL_UART_Transmit_DMA(&huart2, "clean_buttom_flag\n",
			sizeof("clean_buttom_flag\n") - 1);
	for (i = 0; i <= 5; i++) { //clean flag
		buttom_flag[i] = 0;
	}
}

void zheng_zhuan(void) {
	HAL_UART_Transmit_DMA(&huart2, "zheng_zhuan\n",
			sizeof("zheng_zhuan\n") - 1);
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
}
void fan_zhuan(void) {
	HAL_UART_Transmit_DMA(&huart2, "fan_zhuan\n", sizeof("fan_zhuan\n") - 1);
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
}

void run_motor(int dir, int slow_flag) {
	HAL_UART_Transmit_DMA(&huart2, "run_motor\n", sizeof("run_motor\n") - 1);
	if (dir == 0) {
		fan_zhuan();
	} else if (dir == 1) {
		zheng_zhuan();
	}
}
void point_motor(void) { //上下點動
	HAL_UART_Transmit_DMA(&huart2, "point_motor\n",
			sizeof("point_motor\n") - 1);
	fan_zhuan();
	stop_motor();
	zheng_zhuan();
	stop_motor();
	clean_buttom_flag();
}
void stop_motor(void) {
	HAL_UART_Transmit_DMA(&huart2, "stop_motor\n", sizeof("stop_motor\n") - 1);
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
	user_pwm_setvalue_1(0);
	user_pwm_setvalue_2(0);
	osDelay(100);
}
void lock_motor(void) {
	HAL_UART_Transmit_DMA(&huart2, "lock_motor\n", sizeof("lock_motor\n") - 1);
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
	user_pwm_setvalue_1(0);
	user_pwm_setvalue_2(0);
}
void turn_on_motor(int slow_time, int time, int slow_pwm, int pwm) {
	zheng_zhuan();
	smoothPWM(1, 0, slow_pwm, 10, slow_time);
	//user_pwm_setvalue_1(slow_pwm);
	user_pwm_setvalue_1(pwm);
	osDelay(time);
	smoothPWM(1, slow_pwm, 0, 10, slow_time);
	//user_pwm_setvalue_1(slow_pwm);
	osDelay(slow_time);
	stop_motor();

}

void turn_off_motor(int slow_time, int time, int slow_pwm, int pwm) {
	fan_zhuan();
	smoothPWM(2, 0, slow_pwm, 10, slow_time);
	//user_pwm_setvalue_2(slow_pwm);
	user_pwm_setvalue_2(pwm);
	osDelay(time);
	smoothPWM(2, slow_pwm, 0, 10, slow_time);
	//user_pwm_setvalue_2(slow_pwm);
	osDelay(slow_time);
	stop_motor();

}

void smoothPWM(int channel, int start_val, int end_val, int step_size,
		int step_time) {
	switch (channel) {
	case 1:
		if (start_val < end_val) {
			while (start_val <= end_val) {
				user_pwm_setvalue_1(start_val);
				osDelay(step_time);
				start_val = start_val + step_size;
			}
		} else {
			while (start_val >= end_val) {
				user_pwm_setvalue_1(start_val);
				osDelay(step_time);
				start_val = start_val - step_size;
			}
		}
		break;
	case 2:
		if (start_val < end_val) {
			while (start_val <= end_val) {
				user_pwm_setvalue_2(start_val);
				osDelay(step_time);
				start_val = start_val + step_size;
			}
		} else {
			while (start_val >= end_val) {
				user_pwm_setvalue_2(start_val);
				osDelay(step_time);
				start_val = start_val - step_size;
			}
		}
		break;
	default:
		break;
	}
}

void auto_limits(int setting_mode) {
	HAL_UART_Transmit_DMA(&huart2, "auto_limits\n",
			sizeof("auto_limits\n") - 1);

	switch (setting_mode) {
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		printf("auto limits error\n");
	}
}
void save_limits_set(int save_data) {
	HAL_UART_Transmit_DMA(&huart2, "save_limits_set\n",
			sizeof("save_limits_set\n") - 1);

	switch (save_data) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	default:
		printf("save data mode error\n");
	}
}

void print_sysinfo(int mode) {

	switch (mode) {
	case 0: //All
		Usart2DmaPrintf("\n==============================\n");
		Usart2DmaPrintf("| Sys time : %d\n",xTaskGetTickCount());
		Usart2DmaPrintf("| Mode:\t%d  \t\tSetMode:\t\t%d  \t|\n", sys_mode,
				sys_setting);
		Usart2DmaPrintf("| mdir:\t%d  \t\tSmoothMode:\t%d  \t|\n", motor_dir,
				smooth_mode);
		Usart2DmaPrintf("| TimeN:\t%d  \t\tTimeSlow:\t\t%d  \t|\n", run_time_n,
				run_time_s);
		Usart2DmaPrintf("| pwmN:\t%d  \tPWNSlow:\t%d  \t|\n", run_pwm_n,
				run_pwm_s);
		Usart2DmaPrintf("| Buttom Flag:[%d][%d][%d][%d] \t\t\t|\n",
				buttom_flag[1], buttom_flag[2], buttom_flag[3], buttom_flag[4]);
		Usart2DmaPrintf("| ADC1:\t%d  \tADC2:\t%d \t\t|\n", real_adc1,
				real_adc2);
		Usart2DmaPrintf("==============================\n");
		break;
	case 1:
		break;
	case 2:
		break;
	default:
		printf("save data mode error\n");
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	//stop_motor();
	//printf("Helloworld!\n");
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
	user_pwm_setvalue_1(0);
	user_pwm_setvalue_2(0);
	if (HAL_UART_Receive_DMA(&huart2, (uint8_t*) rxData, sizeof(rxData) - 1)
			!= HAL_OK)	//main函数while(1)前，启动一次DMA接收
			{
		Error_Handler();
	}
	HAL_UART_Transmit_DMA(&huart2, "Sys Run\n", sizeof("Sys Run\n") - 1);//可以通过DMA把数据发出去

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
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of myTaskoutput */
	myTaskoutputHandle = osThreadNew(StartTask02, NULL,
			&myTaskoutput_attributes);

	/* creation of myTask_BUTTON */
	myTask_BUTTONHandle = osThreadNew(StartTask03, NULL,
			&myTask_BUTTON_attributes);

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
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {

		if (buttom_flag[1] > 0) { //按下1按鈕
			/*
			 * turn_on_motor(int slow_time,int time,int slow_pwm,int pwm)
			 * 第一個數值為緩啟動每次變化量時間 	預設50
			 * 第二個數值時間					預設5000 = 5秒
			 * 第三個緩請動最大輸出值			預設500
			 * 第四個正常常模式下功率 			預設100%
			 */
			//turn_on_motor(50, 5000, 500, 1000);
			turn_on_motor(real_adc1 / 50, 5000, real_adc1, 2000);
			//clean_buttom_flag();
		} else if (buttom_flag[2] > 0) { //按下2按鈕

		} else if (buttom_flag[3] > 0) { //按鈕3
			/*
			 * turn_on_motor(int slow_time,int time,int slow_pwm,int pwm)
			 * 第一個數值為緩啟動每次變化量時間 	預設50
			 * 第二個數值時間					預設5000 = 5秒
			 * 第三個緩請動最大輸出值			預設500
			 * 第四個正常常模式下功率 			預設100%
			 */
			//turn_off_motor(50, 5000, 500, 1000);
			turn_off_motor(real_adc1 / 50, 5000, real_adc1, 2000);
			//buttom_flag[2] = 0;
			//clean_buttom_flag();

		} else if (buttom_flag[4] > 0) { //按鈕4
			//clean_buttom_flag();
		}
		osDelay(1);

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
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_Value, 100);
	/* Infinite loop */
	for (;;) {
		for (i = 0, ad1 = 0, ad2 = 0; i < 100;) {
			ad1 += ADC_Value[i++];
			ad2 += ADC_Value[i++];
			osDelay(10);
		}
		real_adc1 = ad1 / 50;
		real_adc2 = ad2 / 50;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		print_sysinfo(0);
	}

	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask_BUTTON thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
/*
		writeFlashData = 0x55555555;
		writeFlashTest();
		printFlashTest();
		writeFlashData = 0xaaaaaaaa;
		writeFlashTest();
		printFlashTest();
*/
		while (1) {                        //for test
			//printFlashTest();
			osDelay(1000);
		}
		// Usart2DmaPrintf("test...%d\n",count++);
		if (buttom_flag[4] > 0) {                        //jump setting
			sys_setting = 0;
			point_motor();
		}

		if (buttom_flag[1] > 0 && buttom_flag[3] > 0) {      //into setting mode
			sys_setting = 1;
			point_motor();
			while (sys_setting > 0) {
				if (buttom_flag[2] > 0 && sys_setting == 1) {           //改變旋轉方向
					point_motor();
					if (motor_dir == 1) {
						motor_dir = 0;
					} else {
						motor_dir = 1;
					}
				}
				if (buttom_flag[2]) {
					save_limits_set(0);
					point_motor();
				}
				if (buttom_flag[4] > 0) {                        //exit setting
					point_motor();
					sys_setting = 0;
					break;
				}
				if (buttom_flag[1] > 0 && buttom_flag[3] > 0) { //Auto set Both limits
					sys_setting = 2;
					auto_limits(1);
					sys_setting = 1;
				} else if (buttom_flag[2] > 0 && buttom_flag[3] > 0) { //Top limit Set by user
					sys_setting = 3;
					run_motor(motor_dir, 0);
					while (1) {                        //keep run
						if (buttom_flag[2] > 0) {
							stop_motor();
							save_limits_set(2);
							sys_setting = 1;
							osDelay(1);
						}
					}
				} else if (buttom_flag[1] > 0 && buttom_flag[2] > 0) { //Bottom limit Set by user
					sys_setting = 4;
					run_motor(abs(motor_dir - 1), 0);
					while (1) {                        //keep run
						if (buttom_flag[2] > 0) {
							stop_motor();
							save_limits_set(3);
							sys_setting = 1;
							osDelay(1);
						}
					}
				}
				osDelay(250);
				//HAL_UART_Transmit_DMA(&huart2,"Setting mode: %d\n",sys_setting,sizeof("stop_motor\n")-1);

			}
		}

		osDelay(250);
	}
	/* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
