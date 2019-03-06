/*
 *	File name:		uart2.c
 *	Author:				AJAY
 *	Modified by:	Er Jui Pin
 *  Date modified:7 Jan 2019
 *
 */

/**
  ******************************************************************************
  * @file           : uart2.c
  * @brief          : Communitcation using UART2 with PC Hyper-Terminal
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
	
#include "uart2.h"

uint8_t autoLoginCount;
/* Reset buffer to store next new data */
void ResetBuffer(void)
{
	for(i=0;i<100;i++)
	{
		buffer[i]=0;
		bufferKeyPressed[i]=0;
	}
	
	for(i=0;i<=NUM_OF_UART_RECEIVED;i++)
	{
		receiveArray[i]=0;
	}
}


/*****************************************************************/
/* Converts the message into an array                            */
/* Transmit array message using HAL_UART_Transmit function(UART) */
/* Calling ResetBuffer to reset buffer                           */
/*****************************************************************/
void API_USR_TransmitMessage(char *message)
{
	snprintf((char *)buffer, sizeof(buffer), "%s", message);
	if(HAL_UART_Transmit(&huart2,buffer, sizeof(buffer),100) !=HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	ResetBuffer();
}


void DeleteLine(void)
{
	for(i=0;i<NUM_OF_UART_RECEIVED;i++)
	{
		bufferKeyPressed[i]=DELETE;
	}
	if(HAL_UART_Transmit(&huart2,bufferKeyPressed,NUM_OF_UART_RECEIVED,100) !=HAL_OK)
	{
		;
	}
}


void AutoLogin(uint8_t x)
{
	if(1 == secondFlag)
	{
		secondFlag=0;
		autoLoginCount++;

		if((autoLoginCount == COUNTER_TIMING) && (0 == bufferKeyPressed[0]) && (autoLogin != 0))
		{
			autoLoginCount=0;
			API_USR_TransmitMessage("\r\nAuto logging...\r\n\r\n");
			receiveArray[0]=x;
			receiveArray[1]=ENTER;
			autoLogin=0;
		}
	}
}


/*******************************************************************/
/* x is the number of variables you want to receive                */
/* The funtion will continue to loop till user press ENTER key(13) */
/* Sends user's inputs to display                                  */
/*******************************************************************/

void ReceiveUart(void)
{
	ResetBuffer();
	enterFlag=0;
	autoLoginCount=0;
	j=0;

	while(0 == enterFlag) 						/* 13 is ENTER */
	{

		if(HAL_UART_Receive(&huart2,bufferKeyPressed,1,100)==HAL_OK)
		{
			receiveArray[j]=bufferKeyPressed[0];
			j++;
			if(j>NUM_OF_UART_RECEIVED)
				j=0;

			DeleteLine();			/* To avoid printing the input character again */
			if(HAL_UART_Transmit(&huart2,receiveArray,j,100) !=HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}

		if(receiveArray[NUM_OF_UART_RECEIVED] != 0)
		{
			ResetBuffer();
			API_USR_TransmitMessage("\r\n\r\nSorry your input is too long...\r\n\r\n");
		}else
		{
			AutoLogin(autoLogin);
			for(i=0;i<NUM_OF_UART_RECEIVED;i++)
			{
				if(receiveArray[i]==ENTER)
				{
					/* If ENTER is the first key (no data enter), take default selection */
					if(0 == i)
					{
						autoLoginCount=0;
						API_USR_TransmitMessage("\r\nAuto logging...\r\n\r\n");
						receiveInputArray[0]=autoLogin; /* assume autoLogin has only 1 digit */
						receiveInputArray[1]=ENTER;
						autoLogin=0;
						enterPlacement=0;
					}else
					{
						receiveInputArray[i]= ENTER;
						enterPlacement=i-1;
						for(k=0;k<=enterPlacement;k++)
						{
							receiveInputArray[k]=receiveArray[k];
						}
					}
					enterFlag=1;
					/* receiveOneInput=receiveArray[enterPlacement]; */
					ResetBuffer();
					/* reset j counter */
					j = 0;
					break;
				}
			}
		}
		
	}
}


uint32_t ReceiveUartCalculationValue(void)
{
	uint32_t calculateValue;
	ReceiveUart();
	uint8_t x;
	for(x=0;x<NUM_OF_CALCULATION;x++)
	{
		if(0 == receiveInputArray[x])
			receiveInputArray[x]='0';
	}

	calculateValue=(receiveInputArray[enterPlacement]-48);

	if(enterPlacement-1 >= 0 && enterPlacement-1 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-1]-48)*10+calculateValue;

	if(enterPlacement-2 >= 0 && enterPlacement-2 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-2]-48)*100+calculateValue;

	if(enterPlacement-3 >= 0 && enterPlacement-3 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-3]-48)*1000+calculateValue;

	if(enterPlacement-4 >= 0 && enterPlacement-4 <= NUM_OF_CALCULATION)
			calculateValue=(receiveInputArray[enterPlacement-4]-48)*10000+calculateValue;

	if(enterPlacement-5 >= 0 && enterPlacement-5 <= NUM_OF_CALCULATION)
				calculateValue=(receiveInputArray[enterPlacement-5]-48)*100000+calculateValue;

	if(calculateValue > 999999)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is invaild...\r\n\r\n");
		ReceiveUartCalculationValue();
	}

	ResetBuffer();

	return calculateValue;
}


void SetTime(void)
{
	/* Set the HOURS */
	API_USR_TransmitMessage("\r\n\r\nPlease enter the HOUR now in 24 Hours.(HH)\r\n");
	hour = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nhour = %d\r\n",hour);
	API_USR_TransmitMessage((char *)buffer);
	if(hour>23)							/* Check for invaild inputs */
	{
		//API_USR_TransmitMessage((char *)buffer);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetTime();
	}

	ResetBuffer();

	/* Set the MINUTES */
	API_USR_TransmitMessage("\r\n\r\nPlease enter the MINUTES now in 24 Hours.(MM)\r\n");
	min = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nmin = %d\r\n",min);
	API_USR_TransmitMessage((char *)buffer);
	if(min>59)							/* Check for invaild inputs */
	{
		//API_USR_TransmitMessage((char *)buffer);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetTime();
	}
	sTime.Hours=hour;
	sTime.Minutes=min;
	if(HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BCD) != HAL_OK)
	{
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
		API_USR_TransmitMessage("RESTARTING...\r\n");
		SetTime();
	}

	API_USR_TransmitMessage("Setting the Time now...\r\n");
	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe time now is %02d:%02d\r\n",sTime.Hours,sTime.Minutes);
	API_USR_TransmitMessage((char *)buffer);
	HAL_Delay(500);
	API_USR_TransmitMessage("\r\nYour time is all set!\r\n\r\n");
	HAL_Delay(500);

	ResetBuffer();
	MainMenu();
}


void SetDate(void)
{
/*
	API_USR_TransmitMessage("\r\nSetting Date...\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC.\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC..\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC...\r\n");
	HAL_Delay(500);

	if(HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY )
	{
		API_USR_TransmitMessage("\r\nUnable to communicate with RTC\r\n");
	}

	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BCD);

	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe date today is %02d/%02d\r\n",sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
*/
	/* Set the month */
	API_USR_TransmitMessage("\r\n\r\nPlease enter the MONTH now.(MM)\r\n");
	month = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nmonth = %d\r\n",month);
	API_USR_TransmitMessage((char *)buffer);
	if(month>12)							/* Check for invaild inputs */
	{
		//API_USR_TransmitMessage((char *)buffer);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetDate();
	}

	ResetBuffer();
	
	/* Set the date */
	API_USR_TransmitMessage("\r\n\r\nPlease enter the DATE now.(DD)\r\n");
	date = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\ndate = %d\r\n",date);
	API_USR_TransmitMessage((char *)buffer);
	if(date>31)							/* Check for invaild inputs */
	{
		//API_USR_TransmitMessage((char *)buffer);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetDate();
	}
	sDate.Month=month;
	sDate.Date=date;
	if(HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BCD) != HAL_OK)
	{
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
		API_USR_TransmitMessage("RESTARTING...\r\n");
		SetTime();
	}
	API_USR_TransmitMessage("Setting the DATE now...\r\n");
	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe date today is %02d/%02d\r\n",sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	HAL_Delay(500);
	API_USR_TransmitMessage("\r\nYour date is all set!\r\n\r\n");
	HAL_Delay(500);

	ResetBuffer();
	MainMenu();
}


void SetPSI(void)
{
	API_USR_TransmitMessage("\r\nSetting PSI...\r\n");

	API_USR_TransmitMessage("\r\n\r\nPlease enter the The PSI rating now.(NN)\r\n");

	MainMenu();
}


void SetExit(void)
{
	API_USR_TransmitMessage("Exit.\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Exit..\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Exit...\r\n");
	HAL_Delay(500);

	exitFlag=1;
}


void SetContinueOperation(void)
{
	//API_USR_TransmitMessage("Continue Operation.\r\n");
	//HAL_Delay(500);
	//API_USR_TransmitMessage("Continue Operation..\r\n");
	//HAL_Delay(500);
	API_USR_TransmitMessage("Continue Operation...\r\n");
	HAL_Delay(500);

	exitFlag=1;
}


void SetColourSensor(void)
{

	SelectColourSensorMenuOnUart();

}


void SettingBluetooth(void)
{
	//SelectionBluetoothMenu();
/*	//PA8 is connected to the STATE Pin of HC-05
	if(1 == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	{
		API_USR_TransmitMessage("\r\nYour bluetooth is detected!\r\n\r\n");
		HAL_Delay(500);
		SelectionBluetoothMenu();
	}

	else
	{
		API_USR_TransmitMessage("\r\nSorry please check the connection of your bluetooth module\r\n\r\n");
		API_USR_TransmitMessage("\r\nPlease communicate through your bluetooth\r\n");
		SelectTopMenuOnUart();
	}*/

}

void LoadCellMode(void)
{
	API_USR_TransmitMessage("\r\nEntering Load Cell Mode...\r\n");
	//SelectionLoadCellModeMenu();
}

void SelectForceSensorMenuOnUart(void)
{
	SelectTopMenuOnUart();
}

void SelectPressureSensorMenuOnUart(void)
{
	SelectTopMenuOnUart();
}

