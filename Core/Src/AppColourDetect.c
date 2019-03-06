/*
 *	File name:		AppColourDetect.c
 *	Author:				Er Jui Pin
 *  Date created: 7 Jan 2019
 *
 */

/**
  ******************************************************************************
  * @file           : APPColourDetect.c
  * @brief          : Communication with TCS34725 RGBC Colour Sensor
  ******************************************************************************
 */

#include "AppColourDetect.h"
#include "uart2.h"
#include "Adafruit_TCS34725.h"

/* Private variables ---------------------------------------------------------*/
bool _tcs34725Initialised;
bool sensorAvailable[NUM_OF_DATA_COLOURSENSORS];
bool isMultiSensorMode = false;
int8_t maxCount;
int8_t lastSensorIndex = -1;
float col;
tcs34725Gain_t _tcs34725Gain;
tcs34725IntegrationTime_t _tcs34725IntegrationTime;


/* Structs -------------------------------------------------------------------*/
/* Struct containing JP's Device Selection in the menu */
UDRS_OperationTypedef	APP_JP_operation[] = 
{
	{SetDate, "1: Set Date", 0},
	{SetTime, "2: Set Time", 0},
	{SelectForceSensorMenuOnUart, "3: Force Sensor for Bag Full Detection", 0},
	{SelectPressureSensorMenuOnUart, "4: Pressure Sensor", 0},
	{SelectColourSensorMenuOnUart, "5: Colour Sensor for Ammonia Detection", 0},
	{SelectTopMenuOnUart, "6: Select Top Menu", 0},
	{SetExit, "7: Exit to Top Menu", 0},
	
};


/* Using pointer of array of functions achieves the same results as using type struct */
/*
void (* APP_Menu_operation[])(void) = {SetDate, SetTime, SelectForceSensorMenuOnUart, SelectPressureSensorMenuOnUart, SelectColourSensorMenuOnUart, SelectTopMenuOnUart, SetExit};
*/


/* Struct containing Colour Sensor Operations in the menu */
UDRS_OperationTypedef	API_CDT_operation[] = 
{
	{StartColourDetectionTest, "1: Start Colour Detection Test", 0},
	{SelectColourSensorIntegrationTimeMenuOnUart, "2: Set Integration Time", 0},
	{SelectColourSensorGainMenuOnUart, "3: Set Sensor Gain", 0},
	{SetColourSensorDefaultValues, "4: Set Default Integration Time and Gain", 0},
	{AutoCalibrateColourSensor, "5: Auto Calibrate Sensor", 0},
	{StartMultiSensorsTest, "6: Start Multiple Sensors Test", 0},
	{SelectTopMenuOnUart, "7: Return To	Top Menu", 0},
	{SetContinueOperation, "8: Continue the Previous Operation", 0},
	
};

/* Struct containing Colour Sensor Integration Time Selection in the menu */
UDRS_OperationTypedef	API_CDT_integrationTimeSelection[] = 
{
	{SetIntegrationTimeTo2_4MS, "1: 2.4ms", 0},
	{SetIntegrationTimeTo24MS, "2: 24ms", 0},
	{SetIntegrationTimeTo50MS, "3: 50ms", 0},
	{SetIntegrationTimeTo103MS, "4: 103ms", 0},
	{SetIntegrationTimeTo154MS, "5: 154ms", 0},
	{SetIntegrationTimeTo614MS, "6: 614ms", 0},
	{SelectColourSensorMenuOnUart, "7: Return To	Previous Menu", 0},
	{SelectTopMenuOnUart, "8: Return To	Top Menu", 0},
	{SetExit, "9: Exit", 0},
	
};

/* Struct containing Colour Sensor Gain Selection in the menu */
UDRS_OperationTypedef	API_CDT_gainSelection[] = 
{
	{SetGainTo1X, "1: 1X (no gain)", 0},
	{SetGainTo4X, "2: 4X", 0},
	{SetGainTo16X, "3: 16X", 0},
	{SetGainTo60X, "4: 60X", 0},
	{SelectColourSensorMenuOnUart, "5: Return to Previous Menu", 0},
	{SelectTopMenuOnUart, "6: Return To	Top Menu", 0},
	{SetExit, "7: Exit", 0},
	
};


/* Exported variables (extern) -----------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/



/* Function definition -------------------------------------------------------*/
void GetColourSensorTimeGain(void)
{
	/* Sensor values have been set in .begin() */
	/* Get the Integration Time in ms */
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	/* Get the Gain */
  switch(_tcs34725Gain) {
  case TCS34725_GAIN_1X: 
    againx = 1; 
    break;
  case TCS34725_GAIN_4X: 
    againx = 4; 
    break;
  case TCS34725_GAIN_16X: 
    againx = 16; 
    break;
  case TCS34725_GAIN_60X: 
    againx = 60; 
    break;
  }
}


void ShowColourSensorTimeGain(void)
{	
	snprintf((char *)buffer, sizeof(buffer), "\r\n\r\nThe Integration Time is %dms.\r\n\r\n", atime_ms);
	API_USR_TransmitMessage((char *)buffer);

	snprintf((char *)buffer, sizeof(buffer), "All the Channel Gains are %dX.\r\n\r\n", againx);
	API_USR_TransmitMessage((char *)buffer);
}


void SetSensorIntegratonTime(void)
{
	TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
}


void SetSensorGains(void)
{
	TCS34725_setGain(_tcs34725Gain);
}


void TrimOffset(void)
{
	;
}


void CollectColourSensorData(void)
{
	/* Turn On LED and start integration cycle */
	TCS34725_enable();
	
	/* Turn Off LED */
	TCS34725_setInterrupt(true);
	
	/* Read the sensor */
	TCS34725_getRawData(&Red, &Green, &Blue, &Clear);
	
	/* Put RGB readings to 0-255 range */
	/* Use the maximum count as the base instead of sum of RGB */
/*
	switch(_tcs34725IntegrationTime) {
		case TCS34725_INTEGRATIONTIME_24MS:
			maxCount = 10240;
			break;
		case TCS34725_INTEGRATIONTIME_50MS:
			maxCount = 21504;
			break;
		case TCS34725_INTEGRATIONTIME_103MS:
			maxCount = 44032;
			break;
		default:
			maxCount = 65535;
	}
	col = Red; col /= maxCount;	col *= 255;	Red = (uint16_t)(col+0.5);
	col = Green; col /= maxCount; col *= 255; Green = (uint16_t)(col+0.5);
	col = Blue; col /= maxCount; col *= 255; Blue = (uint16_t)(col+0.5);
	col = Clear; col /= maxCount; col *= 255; Clear = (uint16_t)(col+0.5);
*/	
	//////****** To Use Raw or Compensated? ********/////////
	/* DN40 calculations */
	/*
  ir = (Red + Green + Blue > Clear) ? (Red + Green + Blue - Clear) / 2 : 0;
  r_comp = Red - ir;
  g_comp = Green - ir;
  b_comp = Blue - ir;
  c_comp = Clear - ir;   
  cratio = ir / Clear;  	
	*/
	//TCS34725_clearInterrupt();                                
	//return DATA[];
}


/*
void CalculateColourSensorData(void)
{
	Red= Data[1]*256+ Data[0];
	Green= Data[3]*256+ Data[2];
	Blue= Data[5]*256+ Data[4];
	Clear= Data[7]*256+ Data[6];
}
*/


void CollectAndShowAllColourSensorData(void)
{
	/* Turn On LED and start integration cycle */
	TCS34725_enable();
	
	/* Turn Off LED */
	TCS34725_setInterrupt(true);
	
	/* Read the sensor */
	TCS34725_getRawData(&Red, &Green, &Blue, &Clear);
	
	/* DN40 calculations */
	/* If Clear saturates, ir calculation is meaningless. Set ir to 0 */
	/* 50ms:21504, 103ms:44032, 154ms and upwards:65535 */
	if(Clear == 21504 || Clear == 44032 || Clear == 65535 || Clear <= 1024) {
     ir = 0;
  } else {
		ir = (Red + Green + Blue > Clear) ? (Red + Green + Blue - Clear) / 2 : 0;
	}
  r_comp = Red - ir;
  g_comp = Green - ir;
  b_comp = Blue - ir;
  c_comp = Clear - ir;   
  cratio = ir * 1.0 / Clear;

  saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
  saturation75 = (atime_ms < 150) ? (saturation - saturation / 4) : saturation;
  isSaturated = (atime_ms < 150 && Clear > saturation75) ? 1 : 0;
  cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF); 
  maxlux = 65535 / (cpl * 3);

  lux = (TCS34725_R_Coef * r_comp + TCS34725_G_Coef * g_comp + TCS34725_B_Coef * b_comp) / cpl;
  ct = TCS34725_CT_Coef * b_comp / r_comp + TCS34725_CT_Offset;
	
	/* Print sensor information at current setting */
	GetColourSensorTimeGain();	
	snprintf((char *)buffer,sizeof(buffer),"Gain: %2dX		Integration Time: %3d ms (0x%02X)\r\n",againx, atime_ms, atime);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"Raw R: %5d		G: %5d		B: %5d		C: %5d\r\n",Red, Green, Blue, Clear);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"IR: %5d		CRATIO: %2.4f		Sat: %5d		Sat75: %5d		*SATURATED*: %1d\r\n",ir, cratio, saturation, saturation75, isSaturated);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"CPL: %2.4f		Max lux: %5.0f\r\n",cpl, maxlux);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"Compensated R: %5d	G: %5d		B: %5d		C: %5d\r\n",r_comp, g_comp, b_comp, c_comp);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"Luminance: %5.0f	Colour Temperation: %5.0f\r\n\r\n",lux, ct);
	API_USR_TransmitMessage((char *)buffer);
}

/************* NOT USED *********************************
void ArrangeColourSensorArrayValues(void)
{
	uint16_t unknown=0;

		for(int j=0;j<NUM_OF_DATA_COLOURSENSORS;j++)
		{
			for(int i=0;i<(NUM_OF_DATA_COLOURSENSORS-1);i++)
			{
				if(RedColourSensorArray[i]>RedColourSensorArray[i+1])
				{
					unknown=RedColourSensorArray[i];
					RedColourSensorArray[i]=RedColourSensorArray[i+1];
					RedColourSensorArray[i+1]=unknown;
				}

				if(GreenColourSensorArray[i]>GreenColourSensorArray[i+1])
				{
					unknown=GreenColourSensorArray[i];
					GreenColourSensorArray[i]=GreenColourSensorArray[i+1];
					GreenColourSensorArray[i+1]=unknown;
				}

				if(BlueColourSensorArray[i]>BlueColourSensorArray[i+1])
				{
					unknown=BlueColourSensorArray[i];
					BlueColourSensorArray[i]=BlueColourSensorArray[i+1];
					BlueColourSensorArray[i+1]=unknown;
				}

				if(ClearColourSensorArray[i]>ClearColourSensorArray[i+1])
				{
					unknown=ClearColourSensorArray[i];
					ClearColourSensorArray[i]=ClearColourSensorArray[i+1];
					ClearColourSensorArray[i+1]=unknown;
				}

			}
		}
}
*/

void PrereadColourSensorData(void)
{
	/* Show current setting of the colour sensor */
	GetColourSensorTimeGain();
	ShowColourSensorTimeGain();
	/* Collect, calculate and show colour sensor information */
	CollectAndShowAllColourSensorData();
}


/* Displaying default menu */
void MainMenu(void)
{
//	API_USR_TransmitMessage("\n\n\n\n#######################################################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n");
//	API_USR_TransmitMessage("######################           ,#####################\r\n");
//	API_USR_TransmitMessage("########################       .#######################\r\n");
//	API_USR_TransmitMessage("##########################   .#########################\r\n");
//	API_USR_TransmitMessage("############         .#####(######.        ,###########\r\n");
//	API_USR_TransmitMessage("############  .,,,.  .(((######(((   ,,,,  .###########\r\n");
//	API_USR_TransmitMessage("#############(  (###(    *####,   .####. .#############\r\n");
//	API_USR_TransmitMessage("###########(..    .(##*  *####,  ###/     ..###########\r\n");
//	API_USR_TransmitMessage("###########,   /##. ##*  *####,  ##/ /##/   *##########\r\n");
//	API_USR_TransmitMessage("##############  .,  ##*  *####,  ##* ,*  .#############\r\n");
//	API_USR_TransmitMessage("##########/   .###  ##*  *####,  ##* (##,    ##########\r\n");
//	API_USR_TransmitMessage("##########,.   (##  ##*  *####,  ##* /###   .(#########\r\n");
//	API_USR_TransmitMessage("#############.      ##*  *####,  ##(      ,#%##########\r\n");
//	API_USR_TransmitMessage("#########,   .###* ,##*  *####,  ###  ###/    (########\r\n");
//	API_USR_TransmitMessage("########(.   /##(  ###*  *####,  ###/ ,###    ,########\r\n");
//	API_USR_TransmitMessage("############.     *#####.      /#####      *###########\r\n");
//	API_USR_TransmitMessage("#######,          /(################/,          #######\r\n");
//	API_USR_TransmitMessage("#######  */////*,,.     ./####*       .,,*////  /######\r\n");
//	API_USR_TransmitMessage("######, ,##############(,      *##############,  ######\r\n");
//	API_USR_TransmitMessage("#####/  .,,,***/(######################//****,,  .#####\r\n");
//	API_USR_TransmitMessage("#####                 ./#######(*                 #####\r\n");
//	API_USR_TransmitMessage("####################*.     ,      ./###################\r\n");
//	API_USR_TransmitMessage("#########################(.  *#########################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n\n");


//	API_USR_TransmitMessage("/**********************Main Menu**********************/\r\n");
//	API_USR_TransmitMessage("* 1- Set Time                                         *\r\n");
//	API_USR_TransmitMessage("* 2- Set Date                                         *\r\n");
//	API_USR_TransmitMessage("* 3- Set PSI                                          *\r\n");
//	API_USR_TransmitMessage("* 4- Testing Colour Sensor                            *\r\n");
//	API_USR_TransmitMessage("* 5- Exit                                             *\r\n");
//	API_USR_TransmitMessage("/*****************************************************/\r\n\n");
//	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n");

	API_USR_TransmitMessage("/********************* Main Menu ******************************/\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("* 1- Set Date                                                 *\r\n");
	API_USR_TransmitMessage("* 2- Set Time                                                 *\r\n");
	API_USR_TransmitMessage("* 3- Select Force Sensor                                      *\r\n");
	API_USR_TransmitMessage("* 4- Select Pressure Sensor                                   *\r\n");
	API_USR_TransmitMessage("* 5- Select Colour Sensor (default)                           *\r\n");
	API_USR_TransmitMessage("* ----------                                                  *\r\n");
	API_USR_TransmitMessage("* 6- Return to Main Menu                                      *\r\n");
	API_USR_TransmitMessage("* 7- Exit                                                     *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*         Time: %02d:%02d           Date: %02d/%02d                   *\r\n",sTime.Hours,sTime.Minutes,sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("/**************************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}


void ColourSensorMenu(void)
{
	GetColourSensorTimeGain();
	
	API_USR_TransmitMessage("/********************* Colour Sensor Menu *********************/\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("* 1- Start Colour Detection Test                              *\r\n");
	API_USR_TransmitMessage("* 2- Set Integration Time                                     *\r\n");
	API_USR_TransmitMessage("* 3- Set Sensor Gain                                          *\r\n");
	API_USR_TransmitMessage("* 4- Set Default Integration Time and Gain                    *\r\n");
	API_USR_TransmitMessage("* 5- Auto Calibrate Sensor                                    *\r\n");
	API_USR_TransmitMessage("* 6- Start Multiple Sensors Test                              *\r\n");
	API_USR_TransmitMessage("* ----------                                                  *\r\n");
	API_USR_TransmitMessage("* 7- Return To Top Menu                                       *\r\n");
	API_USR_TransmitMessage("* 8- Continue the Previous Operation                          *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*   Integration Time: %3dms	Gain: %2dX                     *\r\n",atime_ms, againx);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*   Red: %5d   Green: %5d   Blue: %5d    Clear: %5d   *\r\n", Red, Green, Blue, Clear);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*              Time: %02d:%02d           Date: %02d/%02d              *\r\n",sTime.Hours, sTime.Minutes, sDate.Date, sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("/**************************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}


void ColourSensorGainSelectionMenu(void)
{
	
	API_USR_TransmitMessage("/************* Colour Sensor Gain Selection Menu **************/\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("* 1- 1X (No gain)                                             *\r\n");
	API_USR_TransmitMessage("* 2- 4X                                                       *\r\n");
	API_USR_TransmitMessage("* 3- 16X                                                      *\r\n");
	API_USR_TransmitMessage("* 4- 60X                                                      *\r\n");
	API_USR_TransmitMessage("* ----------                                                  *\r\n");
	API_USR_TransmitMessage("* 5- Return to Previous Menu                                  *\r\n");
	API_USR_TransmitMessage("* 6- Return to Top Menu                                       *\r\n");
	API_USR_TransmitMessage("* 7- Exit                                                     *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*      Current Gain Set: %2dX                                  *\r\n",againx);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("/**************************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}


void ColourSensorIntegrationTimeSelectionMenu(void)
{
	//atime_ms = ((256 - _tcs34725IntegrationTime) * 2.4);
	API_USR_TransmitMessage("/******* Colour Sensor Integration Time Selection Menu ********/\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("* 1- 2.4ms                                                    *\r\n");
	API_USR_TransmitMessage("* 2- 24ms                                                     *\r\n");
	API_USR_TransmitMessage("* 3- 50ms                                                     *\r\n");
	API_USR_TransmitMessage("* 4- 103ms                                                    *\r\n");
	API_USR_TransmitMessage("* 5- 153ms                                                    *\r\n");
	API_USR_TransmitMessage("* 6- 614ms                                                    *\r\n");
	API_USR_TransmitMessage("* ----------                                                  *\r\n");
	API_USR_TransmitMessage("* 7- Return to Previous Menu                                  *\r\n");
	API_USR_TransmitMessage("* 8- Return to Top Menu                                       *\r\n");
	API_USR_TransmitMessage("* 9- Exit                                                     *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*      Current Integration Time Set: %3dms                    *\r\n",atime_ms);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("/**************************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}


void EnableMultiSensors(void)
/* Enable TCA9548A Multiplexer, return index of last port connecting to a TCS34725 sensor */
{
	uint8_t sensorNumber;
	uint8_t value;
	
	lastSensorIndex = -1;
	
	if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		API_USR_TransmitMessage("I2C1 is not ready\r\n\r\n");
		SelectColourSensorMenuOnUart();
	}
	
	/* Similar to StartColourDetectionTest() for 1 sensor test */
	/* Scan and mark which sensors are available, set Gain and IntegrationTime if connected */
	for (sensorNumber=0; sensorNumber<NUM_OF_DATA_COLOURSENSORS; sensorNumber++)
	{	
		/* Scan, initialize I2C and configures the sensor */		
		value = (1 << sensorNumber); //left shift to select the bit in the Control Register, and send directly
		HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDRESS<<1, &value, 1, 100); /* Scan */
		
		if(TCS34725_begin())		/* Gain and IntegrationTime are set. TCS34725_enable() inside the begin function turns ON LED */
		{
			sensorAvailable[sensorNumber] = true;
			snprintf((char *)buffer, sizeof(buffer),"\r\nPort %1d connected.\r\n\r\n", sensorNumber);
			API_USR_TransmitMessage((char *)buffer);
			lastSensorIndex = sensorNumber;
			//TCS34725_setGain(_tcs34725Gain);
			//TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
		}	else
		{
			sensorAvailable[sensorNumber] = false;
			snprintf((char *)buffer, sizeof(buffer),"Port %1d NOT connected.\r\n\r\n", sensorNumber);
			API_USR_TransmitMessage((char *)buffer);
		}
		//HAL_Delay(100);
	}
	
	/* show sensing data for the last port connected */
	if(lastSensorIndex != -1)
	{
		_tcs34725Initialised = true;
		TCA9548A_select(lastSensorIndex);
	}
}


void StartColourDetectionTest(void)
{
	/* Either one sensor is connected directly to I2C1 port, or */
	/* at least one sensor is connected to the 1-to-8 multiplexer */
	/* If the latter, just process the last sensor port connected */
	
	if(isMultiSensorMode)		/* If true, also indicate there is at least 1 sensor connected */
	{
		TCA9548A_select(lastSensorIndex);
	}
	/* Usually Gain and IntegrationTime should have been set. Just to be safe and for clarity */
	TCS34725_setGain(_tcs34725Gain);
	TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
	GetColourSensorTimeGain();
	ShowColourSensorTimeGain();

	secondCount = 0;				/* reset second counter and start test */

	/* Send Header List */
	snprintf((char *)buffer, sizeof(buffer),"Date,Time,Red,Green,Blue,Clear,B/R,B/G\r\n");
	API_USR_TransmitMessage((char *)buffer);
				
	while(1) /* loop until ESC key is pressed */
	{
		if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
		{
			if(ESC == buffer[0])
				SelectColourSensorMenuOnUart();
		}
		
		if(1 == secondFlag)
		{
			secondFlag = 0;
			CollectColourSensorData();
			snprintf((char *)buffer, sizeof(buffer),"%02d/%02d, %02d:%02d:%02d, %04d, %04d, %04d, %04d, %1.4f, %1.4f\r\n",sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, secondCount, Red, Green, Blue, Clear, (float)Blue/(float)Red, (float)Blue/(float)Green);
			API_USR_TransmitMessage((char *)buffer);
		}
	}
}


void StartMultiSensorsTest(void)
{
	uint8_t sensorNumber;
	
	if(lastSensorIndex == -1) /* no TCS34725 sensor connected to TCA9548A multiplexer */
	{
		API_USR_TransmitMessage("Colour Sensor(I2C1) is not ready\r\n\r\n");
		SelectColourSensorMenuOnUart();
	}

	/* Start multiple sensor test */
	secondCount = 0;				/* reset second counter and start test */

	/* Send Header List */
	snprintf((char *)buffer, sizeof(buffer),"Sensor,Date,Time,Red,Green,Blue,Clear,B/R,B/G\r\n");
	API_USR_TransmitMessage((char *)buffer);
				
	while(1) /* loop until ESC key is pressed */
	{
		if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
		{
			if(ESC == buffer[0])
				SelectColourSensorMenuOnUart();
		}
		
		if(1 == secondFlag)
		{
			secondFlag = 0;
			for (sensorNumber=0; sensorNumber<NUM_OF_DATA_COLOURSENSORS; sensorNumber++)
			{
				if(sensorAvailable[sensorNumber])
				{
					TCA9548A_select(sensorNumber);
					CollectColourSensorData();
					
					snprintf((char *)buffer, sizeof(buffer),"%01d, %02d/%02d, %02d:%02d:%02d, %04d, %04d, %04d, %04d, %1.4f, %1.4f\r\n",sensorNumber, sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, secondCount, Red, Green, Blue, Clear, (float)Blue/(float)Red, (float)Blue/(float)Green);
					API_USR_TransmitMessage((char *)buffer);
				}
			}
			
		}
	}	
}


bool TCA9548A_select(uint8_t i)
{
	if (i > 7) return false;
	
	uint8_t value;
	value = (1 << i); //left shift to select the bit in the Control Register, and send directly
	if(HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDRESS<<1, &value, 1, 100) != HAL_OK)
	{
		snprintf((char *)buffer,sizeof(buffer),"Setting of TCA9548A CONTROL Register went wrong!\r\n");
		API_USR_TransmitMessage((char *)buffer);
		return false;
	}else
		return true;
}


void SetColourSensorDefaultValues(void)
{
	_tcs34725IntegrationTime = DEFAULT_INTEGRATIONTIME;
	_tcs34725Gain = DEFAULT_GAIN;

	/* Set default Integration Time */
	TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
	
	/* Set default Channel Gain */
	TCS34725_setGain(_tcs34725Gain);  

	CollectColourSensorData();
	
	/* Back to show menu */
	SelectColourSensorMenuOnUart();
}


void AutoCalibrateColourSensor(void)
{
	struct tcs_agc {
		
    tcs34725Gain_t ag;
    tcs34725IntegrationTime_t at;
    uint16_t mincnt;
    uint16_t maxcnt;
  };
/*
 Gain/time combinations to use and the min/max limits for hysteresis 
 that avoid saturation. They should be in order from dim to bright.
 So the list is to set maximum gain/time allowable, and reduce until
 the Clear output is below saturation with hysteresis margin. Also
 ensure it is still high enough.
	
 Note that when gain is reduced by 4 times, the Clear output is also
 reduced 4X accordingly.
	
 One of the first factors impacting the integration time decision is 50/60Hz ripple rejection. If the
 programmed integration time is in multiples of 10 ms and 8.3 ms (the half cycle time), both
 frequencies are rejected. An integration time value of 50ms or multiples of 50ms are required to reject
 both 50Hz and 60Hz ripple. In cases requiring faster sampling time, averaging over a 50ms period
 may be needed to reject the fluorescent light and incandescent light ripple.
 However, in our application, we do not worry about the ripple rejection.	

 Also set the first min count and the last max count to 0 to indicate 
 the start and end of the list. 
*/
  static const struct tcs_agc agc_lst[]={
		{ TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_614MS,     0, 20000 },
		{ TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS,  4990, 63000 },
		{ TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS, 16790, 63000 },
		{ TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 63000 },
		{ TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 0 }
	};
/*
 * Another source from https://create.arduino.cc/editor/enickles/779a27d0-fb3a-4527-9be3-7b0c2510d910/preview
	const tcs34725::tcs_agc tcs34725::agc_lst[] = {
		{ TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS,     0, 47566 },
		{ TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS,  3171, 63422 },
		{ TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15855, 63422 },
		{ TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_2_4MS,   248,     0 }
	};
*/	
  uint16_t agc_cur = 0;
	
	/* initialize the sensor */
	isAvailable = TCS34725_begin();
	if(!isAvailable)
	{
		API_USR_TransmitMessage("Colour Sensor(I2C1) is not available!\r\n\r\n");
		SelectTopMenuOnUart();
	}
	API_USR_TransmitMessage("The I2C1 device is TCS34725 Colour Sensor.\r\n\r\n");
	/* Set the gain and integration time */
	TCS34725_setIntegrationTime(agc_lst[agc_cur].at);
	TCS34725_setGain(agc_lst[agc_cur].ag);	
	
	atime = (uint16_t)(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);  
  switch(agc_lst[agc_cur].ag) {
  case TCS34725_GAIN_1X: 
    againx = 1; 
    break;
  case TCS34725_GAIN_4X: 
    againx = 4; 
    break;
  case TCS34725_GAIN_16X: 
    againx = 16; 
    break;
  case TCS34725_GAIN_60X: 
    againx = 60; 
    break;
  }
	
	/* Retrieve data from the sensor and do the calculations */
	//TCS34725_getRawData(&Red, &Green, &Blue, &Clear);
	CollectAndShowAllColourSensorData();
	
	/* read the sensor and autorange if necessary */
	while(1) {
    if (agc_lst[agc_cur].maxcnt && Clear > agc_lst[agc_cur].maxcnt) 
      agc_cur++;
    else if (agc_lst[agc_cur].mincnt && Clear < agc_lst[agc_cur].mincnt)
      agc_cur--;
    else break;

    TCS34725_setIntegrationTime(agc_lst[agc_cur].at);
		TCS34725_setGain(agc_lst[agc_cur].ag);
    HAL_Delay((256 - atime) * 2.4 * 2); 	/* shock absorber */
    CollectAndShowAllColourSensorData();
    //break;    
  }
	
	//CollectAndShowAllColourSensorData();
	/*
	// DN40 calculations //
  ir = (Red + Green + Blue > Clear) ? (Red + Green + Blue - Clear) / 2 : 0;
  r_comp = Red - ir;
  g_comp = Green - ir;
  b_comp = Blue - ir;
  c_comp = Clear - ir;   
  cratio = ir / Clear;

  saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
  saturation75 = (atime_ms < 150) ? (saturation - saturation / 4) : saturation;
  isSaturated = (atime_ms < 150 && Clear > saturation75) ? 1 : 0;
  cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF); 
  maxlux = 65535 / (cpl * 3);

  lux = (TCS34725_R_Coef * r_comp + TCS34725_G_Coef * g_comp + TCS34725_B_Coef * b_comp) / cpl;
  ct = TCS34725_CT_Coef * b_comp / r_comp + TCS34725_CT_Offset;
	*/
	
	/* Save Sensor Gain and Integration Time */
	_tcs34725IntegrationTime = agc_lst[agc_cur].at;
	_tcs34725Gain = agc_lst[agc_cur].ag;
	SetSensorGains();
	SetSensorIntegratonTime();
	CollectColourSensorData();		
		
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo2_4MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_2_4MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo24MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_24MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo50MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_50MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo103MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_103MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_103MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo154MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_154MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetIntegrationTimeTo614MS(void)
{
	TCS34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_614MS);
	_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_614MS;
	atime = (uint16_t)(_tcs34725IntegrationTime);
  atime_ms = ((256 - atime) * 2.4);
	SelectColourSensorMenuOnUart();
}


void SetGainTo1X(void)
{
	TCS34725_setGain(TCS34725_GAIN_1X);
	_tcs34725Gain = TCS34725_GAIN_1X;
	againx = 1;
	SelectColourSensorMenuOnUart();
}


void SetGainTo4X(void)
{
	TCS34725_setGain(TCS34725_GAIN_4X);
	_tcs34725Gain = TCS34725_GAIN_4X;
	againx = 4;
	SelectColourSensorMenuOnUart();
}


void SetGainTo16X(void)
{
	TCS34725_setGain(TCS34725_GAIN_16X);
	_tcs34725Gain = TCS34725_GAIN_16X;
	againx = 16;
	SelectColourSensorMenuOnUart();
}


void SetGainTo60X(void)
{
	TCS34725_setGain(TCS34725_GAIN_60X);
	_tcs34725Gain = TCS34725_GAIN_60X;
	againx = 60;
	SelectColourSensorMenuOnUart();
}


void SelectTopMenuOnUart(void)
{
	MainMenu();
	exitFlag=0;
	autoLogin='5';
	
	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement]; /* for menu, assume number of choices < 10, 1 digit only */
		menuSelection = menuSelection - 48; /* to get integer value from ASCII */
		menuSelection--; 										/*index starts from zero */
		
		if(menuSelection >= COUNT_OF_OPERATION(APP_JP_operation)) /* check range */
		{
			//API_USR_TransmitMessage((char *)buffer);
			API_USR_TransmitMessage("\r\nYour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectTopMenuOnUart();
		}else
		{
			APP_JP_operation[menuSelection].OpFunc();
			/* (APP_Menu_operation[menuSelection])();   *** Same outcome as above ***/
		}
	}
}


void SelectColourSensorMenuOnUart(void)
{
	/* Only do initialization once */
	if (!_tcs34725Initialised)
	{
		/* Initialize variables to default values */
		_tcs34725Gain = DEFAULT_GAIN;
		_tcs34725IntegrationTime = DEFAULT_INTEGRATIONTIME;

		/* Check whether TCA9548A multiplexer is used or TCS34725 sensor is directly connected to I2C1 */
		/* Use part of the TCS34725_begin() function */
		if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
		{
			API_USR_TransmitMessage("I2C is not ready!\r\n");
			return;
		}

		/* Check whether connecting directly to ICS34725 */
		uint8_t x = TCS34725_read8(TCS34725_ID);
		//HAL_Delay(100);
		if (0x44 == x)  /* direct connect, 1 sensor only */
		{
			_tcs34725Initialised = true;
			isMultiSensorMode = false;

			/* Set integration time and gain */
			TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
			TCS34725_setGain(_tcs34725Gain);

			/* Note: by default, the device is in power down mode on bootup */
			TCS34725_enable();
		}
		else						/* possibly connect via the ICA9548A */
		{
			EnableMultiSensors();
			if(lastSensorIndex != -1) {  /* At least 1 TCS34725 sensor is connected to the TCA9548A */
				_tcs34725Initialised = true;
				isMultiSensorMode = true;
			}
			else
			{
				API_USR_TransmitMessage("Colour Sensor(I2C1) is not available or I2C Multipler is not set up!\r\n\r\n");
				exitFlag = 1;	/* set exitFlag so as to exit all the way and pause upon return */
				return;		/* return instead to avoid using up stack memory */
			}
		}
	} else
	{
		/* Just to turn On LED */
		TCS34725_enable();
	}
	
	PrereadColourSensorData(); /* setInterrupt() inside CollectAndShowAllColourSensorData() turns OFF LED */
	ColourSensorMenu();
	exitFlag=0;
	autoLogin=0;	/*Note: It is zero and not '0' for Not to doing autologin */

	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement]; /* for menu, assume number of choices < 10, 1 digit only */		
		menuSelection = menuSelection - 48; /* to get integer value from ASCII */
		menuSelection--; 										/*index starts from zero */
		
		if(menuSelection >= COUNT_OF_OPERATION(API_CDT_operation)) /* check range */
		{
			//API_USR_TransmitMessage((char *)buffer);
			API_USR_TransmitMessage("\r\nYour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectColourSensorMenuOnUart();
		}else
		{
			API_CDT_operation[menuSelection].OpFunc();
		}
	}	
}


void SelectColourSensorIntegrationTimeMenuOnUart(void)
{
	ColourSensorIntegrationTimeSelectionMenu();
	exitFlag=0;
	autoLogin=0;

	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement]; /* for menu, assume number of choices < 10, 1 digit only */		
		menuSelection = menuSelection - 48; /* to get integer value from ASCII */
		menuSelection--; 										/*index starts from zero */
		
		if(menuSelection >= COUNT_OF_OPERATION(API_CDT_integrationTimeSelection)) /* check range */
		{
			//API_USR_TransmitMessage((char *)buffer);
			API_USR_TransmitMessage("\r\nYour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectColourSensorIntegrationTimeMenuOnUart();
		}else
		{
			API_CDT_integrationTimeSelection[menuSelection].OpFunc();
		}
	}

	TCS34725_setIntegrationTime(_tcs34725IntegrationTime);

	SelectColourSensorMenuOnUart();
}


void SelectColourSensorGainMenuOnUart(void)
{
	ColourSensorGainSelectionMenu();
	exitFlag=0;
	autoLogin=0;

	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement]; /* for menu, assume number of choices < 10, 1 digit only */		
		menuSelection = menuSelection - 48; /* to get integer value from ASCII */
		menuSelection--; 										/*index starts from zero */
		
		if(menuSelection >= COUNT_OF_OPERATION(API_CDT_gainSelection)) /* check range */
		{
			//API_USR_TransmitMessage((char *)buffer);
			API_USR_TransmitMessage("\r\nYour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectColourSensorIntegrationTimeMenuOnUart();
		}else
		{
			API_CDT_gainSelection[menuSelection].OpFunc();
		}
	}

   SelectColourSensorMenuOnUart();
}

