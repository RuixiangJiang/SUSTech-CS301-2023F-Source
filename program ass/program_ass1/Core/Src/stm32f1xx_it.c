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
#include "lcd.h"
#include "string.h"
#include "ctype.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

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
int finish_blink = 0;
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
void blink_LED01(int delay_time){
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(delay_time);
	finish_blink = 0;
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(delay_time);
	finish_blink = 1;
}

struct HuffmanEntry {
    char letter;
    char code[15];
};

struct HuffmanEntry huffmanTable[] = {
	{'A', "000"},
	{'B', "101001"},
	{'C', "00101"},
	{'D', "11011"},
	{'E', "011"},
	{'F', "01000"},
	{'G', "001001"},
	{'H', "0011"},
	{'I', "1100"},
	{'J', "00100011"},
	{'K', "0010000"},
	{'L', "11010"},
	{'M', "01001"},
	{'N', "1011"},
	{'O', "1110"},
	{'P', "101011"},
	{'Q', "0010001001"},
	{'R', "0101"},
	{'S', "1000"},
	{'T', "1111"},
	{'U', "10011"},
	{'V', "101000"},
	{'W', "101010"},
	{'X', "001000101"},
	{'Y', "10010"},
	{'Z', "0010001000"}
};

char decodeHuffman(const char* huffmanCode){
    for (int i = 0; i < sizeof(huffmanTable) / sizeof(huffmanTable[0]); i++){
        if (strcmp(huffmanTable[i].code, huffmanCode) == 0){
            return huffmanTable[i].letter;
        }
    }
    return 'f'; // fail
}
char* encodeHuffman(const char huffmanLetter){
	char capitalLetter = (char)((int)huffmanLetter - (int)'a' + (int)'A');
	for (int i = 0; i < sizeof(huffmanTable) / sizeof(huffmanTable[0]); i++){
		if (huffmanTable[i].letter == huffmanLetter){
			return huffmanTable[i].code;
		}
		if (huffmanTable[i].letter == capitalLetter){
			return huffmanTable[i].code;
		}
	}
	return "Error";
}

int cipher_x = 15, cipher_y = 60;
// cipher_x in range [15, 15 + 8 * 25 = 215], each line has 25 characters.
// cipher_y in range [60, 120], there are 4 lines.
int input_str_ptr = 0;
char input_str[105] = "";
int match_ptr = 0;
char match_str[105] = "";

int plaintext_x = 15, plaintext_y = 200;
int output_str[1005];
int output_str_ptr = 0;

int encode_input_num = 0;

void write_to_cipher(char *ch){
	BACK_COLOR = WHITE; POINT_COLOR = RED;
	for (int i = 0; i < strlen(ch); i++){
		char fffff[2];
		fffff[0] = ch[i];
		fffff[1] = '\0';
		LCD_ShowString(cipher_x, cipher_y, 200, 24, 16, (uint8_t*) fffff);
		cipher_x += 8;
		if (cipher_x > 215) cipher_x = 15, cipher_y += 20;
		if (cipher_y > 120) cipher_y = 60;
	}
}
void write_to_plaintext(char *ch){
	BACK_COLOR = WHITE; POINT_COLOR = RED;
	for (int i = 0; i < strlen(ch); i++){
		char fffff[2];
		fffff[0] = ch[i];
		fffff[1] = '\0';
		LCD_ShowString(plaintext_x, plaintext_y, 200, 24, 16, (uint8_t*) fffff);
		plaintext_x += 8;
		if (plaintext_x > 215) plaintext_x = 15, plaintext_y += 20;
		if (plaintext_y > 260) plaintext_y = 200;
	}
}

int mode = -1;
// mode = -1 for decode mode
// mode = 1 for encode mode
// switch by input null message
void init_mode(){
	cipher_x = 15, cipher_y = 60;
	plaintext_x = 15, plaintext_y = 200;
	input_str_ptr = 0;
	for (int i = 0; i < 105; i++) input_str[i] = '\0';
	output_str_ptr = 0;
	for (int i = 0; i < 105; i++) output_str[i] = '\0';
	match_ptr = 0;
	for (int i = 0; i < 105; i++) match_str[i] = '\0';
	encode_input_num = 0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (mode == -1){
		BACK_COLOR = WHITE; POINT_COLOR = BLACK;
		LCD_ShowString(200, 280, 200, 24, 16, (uint8_t*) "D");
	}
	else{
		BACK_COLOR = WHITE; POINT_COLOR = BLACK;
		LCD_ShowString(200, 280, 200, 24, 16, (uint8_t*) "E");
	}
	int delay = 500;
	HAL_Delay(100);
	if (mode == -1){ // decode mode
		switch (GPIO_Pin){
		case KEY0_Pin:
			if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET){
				blink_LED0(delay);
				write_to_cipher("0");
				input_str[input_str_ptr++] = '0';
				HAL_Delay(100);
			}
			break;
		case KEY1_Pin:
			if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){
				blink_LED1(delay);
				write_to_cipher("1");
				input_str[input_str_ptr++] = '1';
				HAL_Delay(100);
			}
			break;
		case KEY_WK_Pin:
			if (HAL_GPIO_ReadPin(KEY_WK_GPIO_Port, KEY_WK_Pin) == GPIO_PIN_SET){
				if (input_str_ptr == 0){ // switch to encode mode
					mode *= -1;
					init_mode();
					break;
				}
				for (int i = 0; i < input_str_ptr;){
					if (input_str[i] == '0'){
						blink_LED0(delay);
					}
					else{
						blink_LED1(delay);
					}

					match_str[match_ptr++] = input_str[i];
					char decodedLetter = decodeHuffman(match_str);
					if (decodedLetter != 'f') {
						char decodeString[2];
						decodeString[0] = decodedLetter;
						decodeString[1] = '\0';
						write_to_plaintext(decodeString);
						for (int i = 0; i < 100; i++) match_str[i] = '\0';
						match_ptr = 0;
						blink_LED01(delay);
					}
					else{
						// haven't match
					}
					if (finish_blink) i++;
				}
				if (match_ptr > 0){ // fail to match
					write_to_plaintext("Decode Error: ");
					write_to_plaintext(match_str);
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
					blink_LED1(delay);
					blink_LED1(delay);
					blink_LED1(delay);
				}
			}
			break;
		default:
			break;
		}
	}




	else{ // encode mode
		switch (GPIO_Pin){
		case KEY0_Pin:
			if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET){
				write_to_plaintext("0");
				input_str[input_str_ptr++] = '0';
				HAL_Delay(100);
			}
			break;
		case KEY1_Pin:
			if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){
				write_to_plaintext("1");
				input_str[input_str_ptr++] = '1';
				HAL_Delay(100);
			}
			break;
		case KEY_WK_Pin:
			if (HAL_GPIO_ReadPin(KEY_WK_GPIO_Port, KEY_WK_Pin) == GPIO_PIN_SET){
				if (input_str_ptr == 0){ // switch to decode mode
					mode *= -1;
					init_mode();
					break;
				}
				for (int i = 0; i < input_str_ptr;){
					encode_input_num = encode_input_num * 2 + (input_str[i] == '1');
					if (i % 8 != 7){
						i++;
						continue;
					}

					// i % 8 == 7 means beginning match

					finish_blink = 1;

					if (((int)'a' <= encode_input_num && encode_input_num <= (int)'z') || ((int)'A' <= encode_input_num && encode_input_num <= (int)'Z')){
						char encode_input_letter[2];
						encode_input_letter[0] = (char)(encode_input_num);
						encode_input_letter[1] = '\0';
						write_to_plaintext(encode_input_letter);
						char* encode_input_code = encodeHuffman(encode_input_letter[0]);
						for (int j = 0; j < strlen(encode_input_code);){
							char encode_output_letter[2];
							encode_output_letter[0] = encode_input_code[j];
							encode_output_letter[1] = '\0';
							write_to_cipher(encode_output_letter);
							if (encode_input_code[j] == '0') blink_LED0(delay); else blink_LED1(delay);
							if (finish_blink) j++;
						}
						blink_LED01(delay);
					}
					else if (encode_input_num == ' '){
						char encode_input_letter[2];
						encode_input_letter[0] = ' ';
						encode_input_letter[1] = '\0';
						write_to_plaintext(encode_input_letter);
						write_to_cipher(encode_input_letter);
					}
					else{ // fail to match
						char encode_input_letter[2];
						encode_input_letter[0] = (char)encode_input_num;
						encode_input_letter[1] = '\0';
						write_to_plaintext("Encode Error: ");
						write_to_plaintext(encode_input_letter);
						write_to_cipher("Encode Error: ");
						write_to_cipher(encode_input_letter);
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
						blink_LED0(delay);
						blink_LED0(delay);
						blink_LED0(delay);
					}
					encode_input_num = 0;
					if (finish_blink) i++;
				}
				if (input_str_ptr % 8 != 0){ // some bits left
					char encode_input_letter[2];
					encode_input_letter[0] = (char)encode_input_num;
					encode_input_letter[1] = '\0';
					write_to_plaintext("Encode Error: ");
					write_to_plaintext(encode_input_letter);
					write_to_cipher("Encode Error: ");
					write_to_cipher(encode_input_letter);
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					blink_LED0(delay);
					blink_LED0(delay);
					blink_LED0(delay);
				}
			}
			break;
		default:
			break;
		}
	}
}
// Hi - 1001000 1101001
// SUS stu - 01010011 01010101 01010011 00100000 01110011 01110100 01110101
// BUS - 01000010 01010101 01010011
// L05E - 01001100 00110000 00110101 01000101
/* USER CODE END 1 */
