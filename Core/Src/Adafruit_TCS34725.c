/**************************************************************************/
/*!
    @file     Adafruit_TCS34725.c
    @author   KTOWN (Adafruit Industries)
		@modified by Er Jui Pin (Ported from Adafruit_TCS34725.cpp)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include "Adafruit_TCS34725.h"
#include "stdlib.h"
#include "math.h"
#include "uart2.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t bufferColourSensor[10];

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
/*
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}
*/


/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void TCS34725_write8 (uint8_t reg, uint8_t value)
{
	if(HAL_I2C_Mem_Write(&hi2c1, TCS34725_ADDRESS<<1, (TCS34725_COMMAND_BIT | reg), 1, &value, 1, 100) != HAL_OK)
	{
		//char *message = (char *)("Writing an 8-bit value over I2C to TCS34725 register was not successful!\r\n");
		API_USR_TransmitMessage("Writing an 8-bit value over I2C to TCS34725 register was not successful!\r\n");
	}
}


/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t TCS34725_read8(uint8_t reg)
{
	//uint8_t *pData = 0U;
	if(HAL_I2C_Mem_Read(&hi2c1, TCS34725_ADDRESS<<1, (TCS34725_COMMAND_BIT | reg), 1, bufferColourSensor, 1, 100) == HAL_OK)
	//if(HAL_I2C_Mem_Read(&hi2c1, TCS34725_ADDRESS<<1, (TCS34725_COMMAND_BIT | reg), 1, pData, 1, 100) == HAL_OK)
	{
		return bufferColourSensor[0];
		//return *pData;
	}
	else
	{
		//char *message = (char *)("Reading an 8-bit value over I2C from TCS34725 register was not successful!\r\n");
		API_USR_TransmitMessage("Reading an 8-bit value over I2C from TCS34725 register was not successful!\r\n");
		//HAL_Delay(100);
		return 0;
	}
}


/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t TCS34725_read16(uint8_t reg)
{
  uint16_t x, t;

	if(HAL_I2C_Mem_Read(&hi2c1, TCS34725_ADDRESS<<1, (TCS34725_COMMAND_BIT | reg), 1, bufferColourSensor, 2, 100) == HAL_OK)
	{
		t = bufferColourSensor[0];
		x = bufferColourSensor[1];
		x <<= 8;
		x |= t;
		return x;
	}
	else
	{
		//char *message = (char *)("Reading a 16-bit value over I2C from TCS34725 register was not successful!\r\n");
		API_USR_TransmitMessage("Reading a 16-bit value over I2C from TCS34725 register was not successful!\r\n");
		return 0;
	}
}


/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void TCS34725_enable(void)
{
  TCS34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  TCS34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    /* Set a delay for the integration time.
      This is only necessary in the case where enabling and then
      immediately trying to read values back. This is because setting
      AEN triggers an automatic integration, so if a read RGBC is
      performed too quickly, the data is not yet valid and all 0's are
      returned */
  switch (_tcs34725IntegrationTime)  /* Add 10% to delay in case of inaccuracy in sensor hardware */
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      HAL_Delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      HAL_Delay(26);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      HAL_Delay(55);
      break;
    case TCS34725_INTEGRATIONTIME_103MS:
      HAL_Delay(113);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      HAL_Delay(170);
      break;
    case TCS34725_INTEGRATIONTIME_614MS:
      HAL_Delay(675);
      break;
  }
}


/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void TCS34725_disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = TCS34725_read8(TCS34725_ENABLE);
  TCS34725_write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}


/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
/*
Adafruit_TCS34725::Adafruit_TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain)
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
}
*/


/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool TCS34725_begin(void)
{
	if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		//char *message = (char *)("I2C is not ready!\r\n");
		API_USR_TransmitMessage("I2C is not ready!\r\n");
		return false;
	}

  /* Make sure we're actually connected */
  uint8_t x = TCS34725_read8(TCS34725_ID);
  /* if ((x != 0x44) && (x != 0x10))  */
	if (x != 0x44)
  {
		//char *message = (char *)("The I2C device ID is not 0x44, not TCS34725!\r\n");
		API_USR_TransmitMessage("The I2C device ID is not 0x44, not TCS34725!\r\n");
    return false;
  }
  _tcs34725Initialised = true;

  /* Set integration time and gain */
  TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
  TCS34725_setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  TCS34725_enable();

  return true;
}


/**************************************************************************/
/*!
    Initialize TCA9548A 1-to-8 I2C Multiplexer Breakout
*/
/**************************************************************************/
bool TCA9548A_begin(void)
{
	if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		//char *message = (char *)("I2C is not ready!\r\n");
		API_USR_TransmitMessage("I2C is not ready!\r\n");
		return false;
	}

  /* Make sure we're actually connected */
  uint8_t x = TCS34725_read8(TCS34725_ID);
  /* if ((x != 0x44) && (x != 0x10))  */
	if (x != 0x44)
  {
		//char *message = (char *)("The I2C device ID is not 0x44, not TCS34725!\r\n");
		API_USR_TransmitMessage("The I2C device ID is not 0x44, not TCS34725!\r\n");
    return false;
  }
  _tcs34725Initialised = true;

  /* Set default integration time and gain */
  TCS34725_setIntegrationTime(_tcs34725IntegrationTime);
  TCS34725_setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  TCS34725_enable();

  return true;
}


/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void TCS34725_setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) TCS34725_begin();

  /* Update the timing register */
  TCS34725_write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}


/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void TCS34725_setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) TCS34725_begin();

  /* Update the timing register */
  TCS34725_write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}


/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void TCS34725_getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) TCS34725_begin();

  *c = TCS34725_read16(TCS34725_CDATAL);
  *r = TCS34725_read16(TCS34725_RDATAL);
  *g = TCS34725_read16(TCS34725_GDATAL);
  *b = TCS34725_read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime) /* Add 10% to delay in case of inaccuracy in sensor hardware */
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      HAL_Delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      HAL_Delay(26);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      HAL_Delay(55);
      break;
    case TCS34725_INTEGRATIONTIME_103MS:
      HAL_Delay(113);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      HAL_Delay(170);
      break;
    case TCS34725_INTEGRATIONTIME_614MS:
      HAL_Delay(675);
      break;
  }
}


/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values in
    one-shot mode (e.g., wakes from sleep, takes measurement, enters sleep)
*/
/**************************************************************************/
void TCS34725_getRawDataOneShot (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) TCS34725_begin();

  TCS34725_enable();
  TCS34725_getRawData(r, g, b ,c);
  TCS34725_disable();
}


/**************************************************************************/
/*!
    @brief  Used with Interrupt. TCS34725_getRawData() does a delay(Integration_Time)
    after the sensor readout. We don't need to wait for the next integration cycle
    because we receive an interrupt when the integration cycle is complete.
*/
/**************************************************************************/
/* TCS34725_getRawData() does a delay(Integration_Time) after the sensor readout.
	 We don't need to wait for the next integration cycle because we receive an interrupt
	 when the integration cycle is complete. */
void TCS34725_getRawDataNoDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = TCS34725_read16(TCS34725_CDATAL);
  *r = TCS34725_read16(TCS34725_RDATAL);
  *g = TCS34725_read16(TCS34725_GDATAL);
  *b = TCS34725_read16(TCS34725_BDATAL);
}


/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t TCS34725_calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}


/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin using the algorithm described in DN40 from Taos (now AMS).
*/
/**************************************************************************/
uint16_t TCS34725_calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    /*int rc;                   	 Error return code */
    uint16_t r2, g2, b2;        /* RGB values minus IR component */
    int gl;                     /* Results of the initial lux conversion */
    uint8_t gain_int;           /* Gain multiplier as a normal integer */
    uint16_t sat;               /* Digital saturation level */
    uint16_t ir;                /* Inferred IR content */

    /* Analog/Digital saturation:
     *
     * (a) As light becomes brighter, the clear channel will tend to
     *     saturate first since R+G+B is approximately equal to C.
     * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
     *     time, up to a maximum values of 65535. This means analog
     *     saturation can occur up to an integration time of 153.6ms
     *     (64*2.4ms=153.6ms).
     * (c) If the integration time is > 153.6ms, digital saturation will
     *     occur before analog saturation. Digital saturation occurs when
     *     the count reaches 65535.
     */
    if ((256 - _tcs34725IntegrationTime) > 63) {
        /* Track digital saturation */
        sat = 65535;
    } else {
        /* Track analog saturation */
        sat = 1024 * (256 - _tcs34725IntegrationTime);
    }

    /* Ripple rejection:
     *
     * (a) An integration time of 50ms or multiples of 50ms are required to
     *     reject both 50Hz and 60Hz ripple.
     * (b) If an integration time faster than 50ms is required, you may need
     *     to average a number of samples over a 50ms period to reject ripple
     *     from fluorescent and incandescent light sources.
     *
     * Ripple saturation notes:
     *
     * (a) If there is ripple in the received signal, the value read from C
     *     will be less than the max, but still have some effects of being
     *     saturated. This means that you can be below the 'sat' value, but
     *     still be saturating. At integration times >150ms this can be
     *     ignored, but <= 150ms you should calculate the 75% saturation
     *     level to avoid this problem.
     */
    if ((256 - _tcs34725IntegrationTime) <= 63) {
        /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
        sat -= sat/4;
    }

    /* Check for saturation and mark the sample as invalid if true */
    if (c >= sat) {
        return 0;
    }

    /* AMS RGB sensors have no IR channel, so the IR content must be */
    /* calculated indirectly. */
    ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

    /* Remove the IR component from the raw RGB values */
    r2 = r - ir;
    g2 = g - ir;
    b2 = b - ir;

    /* Convert gain to a usable integer value */
    switch(_tcs34725Gain) {
        case TCS34725_GAIN_4X: /* GAIN 4X */
            gain_int = 4;
            break;
        case TCS34725_GAIN_16X: /* GAIN 16X */
            gain_int = 16;
            break;
        case TCS34725_GAIN_60X: /* GAIN 60X */
            gain_int = 60;
            break;
        case TCS34725_GAIN_1X: /* GAIN 1X */
        default:
            gain_int = 1;
            break;
    }

    /* Calculate the counts per lux (CPL), taking into account the optional
     * arguments for Glass Attenuation (GA) and Device Factor (DF).
     *
     * GA = 1/T where T is glass transmissivity, meaning if glass is 50%
     * transmissive, the GA is 2 (1/0.5=2), and if the glass attenuates light
     * 95% the GA is 20 (1/0.05). A GA of 1.0 assumes perfect transmission.
     *
     * NOTE: It is recommended to have a CPL > 5 to have a lux accuracy
     *       < +/- 0.5 lux, where the digitization error can be calculated via:
     *       'DER = (+/-2) / CPL'.
     */
    float cpl = (((256-_tcs34725IntegrationTime)*2.4f) * gain_int) /
        (1.0f * 310.0f);

    /* Determine lux accuracy (+/- lux) */
    float der = 2.0f / cpl;

    /* Determine the maximum lux value */
    float max_lux = 65535.0f / (cpl * 3.0f);

    /* Lux is a function of the IR-compensated RGB channels and the associated
     * color coefficients, with G having a particularly heavy influence to
     * match the nature of the human eye.
     *
     * NOTE: The green value should be > 10 to ensure the accuracy of the lux
     *       conversions. If it is below 10, the gain should be increased, but
     *       the clear<100 check earlier should cover this edge case.
     */
    gl =  0.136f * (float)r2 +                   /** Red coefficient. */
          1.000f * (float)g2 +                   /** Green coefficient. */
         -0.444f * (float)b2;                    /** Blue coefficient. */

    float lux = gl / cpl;

    /* A simple method of measuring color temp is to use the ratio of blue */
    /* to red light, taking IR cancellation into account. */
    uint16_t cct = (3810 * (uint32_t)b2) /      /** Color temp coefficient. */
                   (uint32_t)r2 + 1391;         /** Color temp offset. */

    return cct;
}


/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to lux
*/
/**************************************************************************/
uint16_t TCS34725_calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


/**************************************************************************/
/*!
    @brief  Sets Interrupt by enabling AIEN bit. Upon completing an
						Integration cycle, INT line goes LOW.
						If asserted, and INT line is connected to LED, the LED
						will be turned OFF.
*/
/**************************************************************************/
void TCS34725_setInterrupt(bool i)
{
  uint8_t r = TCS34725_read8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  TCS34725_write8(TCS34725_ENABLE, r);
}


/**************************************************************************/
/*!
    @brief  Clears Interrupt and brings INT line HIGH
*/
/**************************************************************************/
void TCS34725_clearInterrupt(void)
{
	uint8_t value;
	value = (TCS34725_COMMAND_BIT | 0x66);
	if(HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS<<1, &value, 1, 100) != HAL_OK)
	{
		//char *message = (char *)("Writing an 8-bit value over I2C to TCS34725 register was not successful!\r\n");
		API_USR_TransmitMessage("Writing an 8-bit value over I2C to TCS34725 register was not successful!\r\n");
	}
  /*
	Wire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(TCS34725_COMMAND_BIT | 0x66);
  #else
  Wire.send(TCS34725_COMMAND_BIT | 0x66);
  #endif
  Wire.endTransmission();
	*/
}


/**************************************************************************/
/*!
    @brief  Sets Interrupt Limits
*/
/**************************************************************************/
void TCS34725_setIntLimits(uint16_t low, uint16_t high)
{
   TCS34725_write8(0x04, low & 0xFF);		/* AILTL */
   TCS34725_write8(0x05, low >> 8);			/* AILTH */
   TCS34725_write8(0x06, high & 0xFF);	/* AIHTL */
   TCS34725_write8(0x07, high >> 8);		/* AIHTH */
}
