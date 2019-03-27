/*
 *	File name:		AppColourDetect.h
 *	Author:				Er Jui Pin
 *  Date created: 7 Jan 2019
 *
 */

#ifndef __APPCOLOURDETECT_H__
#define __APPCOLOURDETECT_H__

#include "stdbool.h"
#include "stm32l0xx_hal.h"


/* Private defines -----------------------------------------------------------*/
#define TCA9548A_ADDRESS (0x70)
#define DEFAULT_GAIN TCS34725_GAIN_16X
#define DEFAULT_INTEGRATIONTIME TCS34725_INTEGRATIONTIME_103MS

#define CALIBRATE_GAIN	TCS34725_GAIN_16X
#define CALIBRATE_INTEGRATIONTIME TCS34725_INTEGRATIONTIME_154MS

/* Some magic numbers for this device from the DN40 application note */
#define TCS34725_R_Coef 0.136 
#define TCS34725_G_Coef 1.000
#define TCS34725_B_Coef -0.444
#define TCS34725_GA 1.0
#define TCS34725_DF 310.0
#define TCS34725_CT_Coef 3810.0
#define TCS34725_CT_Offset 1391.0

#define NUM_OF_DATA_COLOURSENSORS 8


/* Private Variables----------------------------------------------------------*/
bool isAvailable, isSaturated;
uint16_t RedColourSensorArray[NUM_OF_DATA_COLOURSENSORS], GreenColourSensorArray[NUM_OF_DATA_COLOURSENSORS], BlueColourSensorArray[NUM_OF_DATA_COLOURSENSORS], ClearColourSensorArray[NUM_OF_DATA_COLOURSENSORS];
uint16_t Red, Green, Blue, Clear;
uint16_t againx, atime, atime_ms;
uint16_t ir; 
uint16_t r_comp, g_comp, b_comp, c_comp;
uint16_t saturation, saturation75;
float cratio, cpl, ct, lux, maxlux;
//uint8_t Data[8], bufferColourSensor[10], x=0;
//uint16_t address, value, max;


/* Exported variables (extern) -----------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern void _Error_Handler(char *, int);
extern uint16_t secondCount;


/* Private function prototypes -----------------------------------------------*/
void SelectColourSensorMenuOnUart(void);
void SetSensorGains(void);
void SetSensorIntegratonTime(void);
void GetColourSensorTimeGain(void);
void ShowColourSensorTimeGain(void);
void CollectColourSensorData(void);
void CollectAndShowAllColourSensorData(void);
void PrereadColourSensorData(void);
void EnableMultiSensor(void);
bool TCA9548A_select(uint8_t i);
//void TrimOffset(void);
//void CalculateColourSensorData(void);


/* Sharing function prototypes -----------------------------------------------*/
/* API_CDT_operation */                          /****** To Prefix API_CDI_ to API Functions for Sharing ******/
void StartColourDetectionTest(void);
void SelectColourSensorIntegrationTimeMenuOnUart(void);
void SelectColourSensorGainMenuOnUart(void);
void SetColourSensorDefaultValues(void);
void AutoCalibrateColourSensor(void);
void StartMultiSensorsTest(void);

/* API_CDT_gainSelection */
void SetGainTo1X(void);
void SetGainTo4X(void);
void SetGainTo16X(void);
void SetGainTo60X(void);

/* API_CDT_integrationTimeSelection */
void SetIntegrationTimeTo2_4MS(void);
void SetIntegrationTimeTo24MS(void);
void SetIntegrationTimeTo50MS(void);
void SetIntegrationTimeTo103MS(void);
void SetIntegrationTimeTo154MS(void);
void SetIntegrationTimeTo614MS(void);


#endif /* __APPCOLOURDETECT_H__ */
