/*
 *	File name:		uart2.h
 *	Author:				AJAY
 *	Modified by:	Er Jui Pin
 *  Date modified:7 Jan 2019
 *
 */

#ifndef UART2_H_
#define UART2_H_

#include "stm32l0xx_hal.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern uint8_t month, date, hour, min;

void ResetBuffer(void);
void API_USR_TransmitMessage(char *message);
void AutoLogin(uint8_t x);
void ReceiveUart(void);
void MainMenu(void);
void SetTime(void);
void SetDate(void);
void SetPSI(void);
void SetExit(void);
void SetContinueOperation(void);
void SelectTopMenuOnUart(void);
void SelectForceSensorMenuOnUart(void);
void SelectPressureSensorMenuOnUart(void);
uint32_t ReceiveUartCalculationValue(void);

extern void SelectColourSensorMenuOnUart(void);

#ifdef __cplusplus
 extern "C" {
#endif
	 
#endif /* UART2_H_ */
