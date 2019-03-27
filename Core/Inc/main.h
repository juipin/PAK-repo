/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"												// uinit definition
#include "string.h"
#include "stm32l0xx_nucleo.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define __PAK_APP_ERS_JP_
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USB_USART2_TX_Pin GPIO_PIN_2
#define USB_USART2_TX_GPIO_Port GPIOA
#define USB_USART2_RX_Pin GPIO_PIN_3
#define USB_USART2_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define I2C2_SCL_5V_Pin GPIO_PIN_10
#define I2C2_SCL_5V_GPIO_Port GPIOB
#define I2C2_SDA_5V_Pin GPIO_PIN_11
#define I2C2_SDA_5V_GPIO_Port GPIOB
#define BluetoothModule_USART1_TX_Pin GPIO_PIN_9
#define BluetoothModule_USART1_TX_GPIO_Port GPIOA
#define BluetoothModule_USART1_RX_Pin GPIO_PIN_10
#define BluetoothModule_USART1_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define I2C1_SCL_3_3V_Pin GPIO_PIN_6
#define I2C1_SCL_3_3V_GPIO_Port GPIOB
#define I2C1_SDA_3_3V_Pin GPIO_PIN_7
#define I2C1_SDA_3_3V_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define NUM_OF_ARRAY 21
#define COUNTER_TIMING 10

#define NUM_OF_UART_RECEIVED 20
#define NUM_OF_CALCULATION 6
#define ENTER 13
#define DELETE 8
#define ESC 27

#define FLAG_SET		1
#define FLAG_RESET	0
#define SIZE_BUFFER_TXRX	80

/* Exported types ------------------------------------------------------------*/
typedef struct
{ 
	void 			(*OpFunc)(void);
	uint8_t		OpName[50];
	uint32_t	OpFuncIndex;
} UDRS_OperationTypedef;

typedef struct
{
	uint8_t		timeout1S;					// timing flag for 1 second
	uint8_t		timeout500mS;				// timing flag for 500mS
	uint8_t		timeout200mS;				// timing flag for 200mS
	uint8_t		timeout100mS;				// timing flag for 100mS
	uint8_t		timeoutXms;					// user define
	uint32_t	previousTick_1S;		// uwTick@last 1 second timeout
	uint32_t	previousTick_500mS;	// uwTick@last 500mS timeout
	uint32_t	previousTick_200mS;	// uwTick@last 200mS timeout
	uint32_t	previousTick_100mS;	// uwTick@last 100mS timeout
	uint32_t	previousTick_XmS;		// user define
} UDRS_ProgrammerTypedef;

extern 	uint8_t	g_MenuIndex;
extern	uint8_t	g_TEMPLATE_timeout1S, g_TEMPLATE_timeout500mS;
extern	uint8_t	g_UDRS_timeout1S,	g_UDRS_timeout500mS;
extern	uint8_t	g_JP_timeout1S, g_JP_timeout500mS;

extern uint8_t halfSecondFlag, secondFlag, fiveSecondFlag, errorFlag, enterFlag, exitFlag;  /* global flags */
extern uint8_t i, j, k;  /* general counters */
extern uint8_t autoLogin;
extern uint8_t buffer[100];  /* UART message transmit buffer */
extern uint8_t bufferKeyPressed[100], receiveArray[NUM_OF_UART_RECEIVED+1], receiveInputArray[NUM_OF_UART_RECEIVED+1]; /* UART keyboard receive buffer */
extern uint8_t menuSelection, enterPlacement;  /* general menu selection */

/* Exported macro ------------------------------------------------------------*/
/* Define macro to allow inclusion / removal of operations (menu) 					  */
#define COUNT_OF_OPERATION(x)    (sizeof(x)/sizeof(UDRS_OperationTypedef))

/* USER CODE END Private defines */

#ifdef __cplusplus
extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
