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
int timer_conut = 0;
int run_time_n = 5000; 	//全速功率-秒數
int run_time_s = 1000; 	//慢速功率-秒數
int run_pwm_n = 2000;	//全速功率
int run_pwm_s = 100;	//慢速功率
int smooth_mode = 1; 	//緩速模式 0-關閉 1-開啟

uint32_t ADC_Value[100];
uint32_t ad1, ad2;
uint32_t real_adc1, real_adc2;

int i = 0;
int buttom_flag[5] = { 0 };

int sys_mode = 0;
int sys_setting = 0;
int motor_dir = 0;



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
		.priority = (osPriority_t) osPriorityLow - 1, .stack_size = 128 * 4 };

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
	//Usart2DmaPrintf("clena\n");
	for (i = 0; i <= 5; i++) { //clean flag
		buttom_flag[i] = 0;
		osDelay(1);
	}
}

void zheng_zhuan(int pwm) {
	//HAL_UART_Transmit_DMA(&huart2, "zheng_zhuan\n",sizeof("zheng_zhuan\n") - 1);
	Usart2DmaPrintf("zheng_zhuan\n");
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
	user_pwm_setvalue_1(pwm);
}
void fan_zhuan(int pwm) {
	//HAL_UART_Transmit_DMA(&huart2, "fan_zhuan\n", sizeof("fan_zhuan\n") - 1);
	Usart2DmaPrintf("fan_zhuan\n");
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
	user_pwm_setvalue_2(pwm);
}
void stop_motor(void) {
	//HAL_UART_Transmit_DMA(&huart2, "stop_motor\n", sizeof("stop_motor\n") - 1);
	Usart2DmaPrintf("stop_motor\n");
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_RESET);
	user_pwm_setvalue_1(0);
	user_pwm_setvalue_2(0);
	osDelay(100);
}
void run_motor(int dir, int action, int slow_flag) {
	//HAL_UART_Transmit_DMA(&huart2, "run_motor\n", sizeof("run_motor\n") - 1);
	Usart2DmaPrintf("run_motor\n");
	if (action == 0) { //關門
		if (dir == 0) {
			if (slow_flag == 1) {
				zheng_zhuan(run_pwm_s);
			} else {
				zheng_zhuan(run_pwm_n);
			}
		} else if (dir == 1) {
			if (slow_flag == 1) {
				fan_zhuan(run_pwm_s);
			} else {
				fan_zhuan(run_pwm_n);
			}
		}
	} else if (action == 1) { //開門
		if (dir == 0) {
			if (slow_flag == 1) {
				fan_zhuan(run_pwm_s);
			} else {
				fan_zhuan(run_pwm_n);
			}
		} else if (dir == 1) {
			if (slow_flag == 1) {
				zheng_zhuan(run_pwm_s);
			} else {
				zheng_zhuan(run_pwm_n);
			}
		}
	}
	//fan_zhuan(run_pwm_n);

}
void point_motor(void) { //上下點動
	//HAL_UART_Transmit_DMA(&huart2, "point_motor\n",sizeof("point_motor\n") - 1);
	Usart2DmaPrintf("point_motor\n");
	fan_zhuan(run_pwm_n);
	stop_motor();
	zheng_zhuan(run_pwm_n);
	stop_motor();
	clean_buttom_flag();
}

void lock_motor(void) {
	Usart2DmaPrintf("lock_motor\n");
	//HAL_UART_Transmit_DMA(&huart2, "lock_motor\n", sizeof("lock_motor\n") - 1);
	HAL_GPIO_WritePin(MC_1_GPIO_Port, MC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MC_2_GPIO_Port, MC_2_Pin, GPIO_PIN_SET);
	user_pwm_setvalue_1(0);
	user_pwm_setvalue_2(0);
}
void turn_on_motor(int slow_time, int time, int slow_pwm, int pwm) {
	zheng_zhuan(pwm);
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
	fan_zhuan(pwm);
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
	Usart2DmaPrintf("auto_limits\n");
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
	Usart2DmaPrintf("save_limits_set\n");
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
		Usart2DmaPrintf("| Sys time : %d\n", xTaskGetTickCount());
		Usart2DmaPrintf("| Mode:\t%d  \t\tSetMode:\t\t%d  \t|\n", sys_mode,
				sys_setting);
		Usart2DmaPrintf("| m_dir:\t%d  \t\tSmoothMode:\t%d  \t|\n", motor_dir,
				smooth_mode);
		Usart2DmaPrintf("| TimeN:\t%dms\tTimeSlow:\t%dms\t|\n", run_time_n,
				run_time_s);
		Usart2DmaPrintf("| pwmN:\t%d  \tPWNSlow:\t%d  \t|\n", run_pwm_n,
				run_pwm_s);
		Usart2DmaPrintf("| Buttom Flag:[%d] [%d] [%d] [%d] \t\t\t|\n",
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
	//int time_count=0;
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
		while(sys_setting==1){
			osDelay(10);
		}
		if (sys_mode == 1) {
			//clean_buttom_flag();
			timer_conut = 0;
			run_motor(motor_dir, 0, smooth_mode);
			while (timer_conut < run_time_s) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			timer_conut = 0;
			run_motor(motor_dir, 0, 0);
			while (timer_conut < run_time_n) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			timer_conut = 0;
			run_motor(motor_dir, 0, smooth_mode);
			while (timer_conut < run_time_s) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			stop_motor();
		} else if (sys_mode == 2) {
			//clean_buttom_flag();
			stop_motor();
		} else if (sys_mode == 3) {
			//clean_buttom_flag();
			timer_conut = 0;
			run_motor(motor_dir, 1, smooth_mode);
			while (timer_conut < run_time_s) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			timer_conut = 0;
			run_motor(motor_dir, 1, 0);
			while (timer_conut < run_time_n) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			timer_conut = 0;
			run_motor(motor_dir, 1, smooth_mode);
			while (timer_conut < run_time_s) {
				timer_conut++;
				if(buttom_flag[2]>0)break;
				osDelay(1);
			}
			stop_motor();
		}
		clean_buttom_flag();

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
		if (buttom_flag[1] > 0) {                        //up
			sys_mode = 1;
			//run_motor(motor_dir, 1);
		}
		if (buttom_flag[2] > 0) {                        //mind
			sys_mode = 2;
			//stop_motor();
		}
		if (buttom_flag[3] > 0) {                        //down
			sys_mode = 3;
			//run_motor(motor_dir, 1);
		}
		if (buttom_flag[4] > 0) {                        //jump setting
			sys_setting = 0;
			point_motor();
		}

		if (buttom_flag[1] > 0 && buttom_flag[3] > 0) {      //into setting mode
			sys_setting = 1;
			clean_buttom_flag();
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
					run_motor(motor_dir, 1, 0);
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
					run_motor(abs(motor_dir - 1), 0, 0);
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
