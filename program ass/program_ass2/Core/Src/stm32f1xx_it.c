/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "time.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t rxBuffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_WK_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY0_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static int numAccept = 0;
int finish_blink = 0;
static int rightAns = 0;
int delay = 500;
static int Time = 0;

void blink_LED0(int delay_time){
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	HAL_Delay(delay_time);
	finish_blink = 0;
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	HAL_Delay(delay_time);
	finish_blink = 1;
}
void blink_LED1(int delay_time){
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(delay_time);
	finish_blink = 0;
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(delay_time);
	finish_blink = 1;
}

int generateQuestion();

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_Delay(100);
	switch (GPIO_Pin){
	case KEY0_Pin:
		if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET){ // reset the game
			numAccept = 0;
			//HAL_NVIC_SystemReset();
			HAL_UART_Transmit(&huart1, (uint8_t *)"Game start!\r\n", 13, HAL_MAX_DELAY);
			HAL_TIM_Base_Start_IT(&htim3);
			rightAns = generateQuestion();
		}
		break;
	case KEY_WK_Pin:
		if (HAL_GPIO_ReadPin(KEY_WK_GPIO_Port, KEY_WK_Pin) == GPIO_PIN_SET){
			HAL_IWDG_Refresh(&hiwdg);
		}
		break;
	}
}

int generateQuestion(){
	unsigned char exp[1024] = {0};
	int numa = 0, numb = 0, ope = 0, ans = 0;
	int expLen = 0;
	while (ans <= 0 || ans > 100){
		numa = rand() % 100;
		numb = rand() % 100;
		ope = rand() % 4;
		switch (ope){
		case 0: // add
			ans = numa + numb;
			expLen = snprintf((char *)exp, sizeof(exp), "%d + %d = ?\r\n", numa, numb);
			break;
		case 1: // sub
			ans = numa - numb;
			expLen = snprintf((char *)exp, sizeof(exp), "%d - %d = ?\r\n", numa, numb);
			break;
		case 2: // mul
			numa = rand()%10;
			numb = rand()%10;
			ans = numa * numb;
			expLen = snprintf((char *)exp, sizeof(exp), "%d * %d = ?\r\n", numa, numb);
			break;
		case 3: // div
			numb = rand()%15;
			ans = (numb != 0) ? (numa / numb) : 0;
			expLen = snprintf((char *)exp, sizeof(exp), "%d / %d = ?\r\n", numa, numb);
			break;
		default:
			break;
		}
	}
	HAL_UART_Transmit(&huart1, exp, expLen, HAL_MAX_DELAY);
	return ans;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (numAccept == 5 || Time >= 60){
		Time = 0;
		HAL_IWDG_Refresh(&hiwdg);
		return;
	}
	Time++;
	if (Time < 50){
		HAL_IWDG_Refresh(&hiwdg);
	}
	if (Time >= 50 && Time < 60 && numAccept < 5){
		unsigned char info[1024] = {0};
		int infoLen = snprintf((char *)info, sizeof(info), "[Warning] %d seconds left!\r\n", 60 - Time);
		HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
		HAL_IWDG_Refresh(&hiwdg);
	}
	else if (Time == 60 && numAccept < 5){
		unsigned char info[1024] = {0};
		int infoLen = snprintf((char *)info, sizeof(info), "[INFO]Time out. Game over. Please press KEY0 to try again.\r\n");
		HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
		HAL_IWDG_Refresh(&hiwdg);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (numAccept == 5) return;
	unsigned char info[1024] = {0};
	if (huart->Instance == USART1){
		static unsigned char uRx_Data[1024] = {0};
		static unsigned char uLength = 0;
		if (rxBuffer[0] == '\n' || rxBuffer[0] == '\r'){ // user ends input
			uLength = 0;
			int userAns = atoi((char *)uRx_Data);
			int infoLen = snprintf((char *)info, sizeof(info), "Your input data is %d.\r\n", userAns);
			HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
			infoLen = snprintf((char *)info, sizeof(info), "Right answer is %d.\r\n", rightAns);
			HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
			if (userAns == rightAns){
				int infoLen = snprintf((char *)info, sizeof(info), "[INFO] True! Do a good job!\r\n");
				HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
				blink_LED1(delay);
				while (!finish_blink){}
				if (++numAccept == 5){
					int infoLen = snprintf((char *)info, sizeof(info), "[INFO] You pass! Press KEY0 to try again.\r\n");
					HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
					return;
				}
				rightAns = generateQuestion();
			}
			else{
				blink_LED0(delay);
				while (!finish_blink);
				int infoLen = snprintf((char *)info, sizeof(info), "[INFO] Wrong! Please try again!\r\n");
				HAL_UART_Transmit(&huart1, info, infoLen, HAL_MAX_DELAY);
			}
		}
		else{
			uRx_Data[uLength++] = rxBuffer[0];
		}
	}
}
/* USER CODE END 1 */
