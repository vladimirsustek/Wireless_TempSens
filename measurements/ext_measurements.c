#include "i2c.h"
#include "ext_measurements.h"
#include "cli.h"

uint16_t LM75AD_ReadTempReg(Measurement_t* measurement)
{

const uint32_t MAX_TIMEOUT = 100;
const uint32_t MAX_ATTEMPT = 1;
/* All three LM42AD's ADR pins HIGH => 0b01001111
 * STM32 HAL driver needs address 'shift << 1' left*/
const uint16_t BOARD_LM75AD_ADR = 0b01001111 << 1;

int32_t temperature = INT32_MIN;

	  if(HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1,
			  BOARD_LM75AD_ADR, MAX_TIMEOUT, MAX_ATTEMPT))
	  {
		  uint8_t raw_temperature[2];

		  if(HAL_OK == HAL_I2C_Master_Receive(&hi2c1,
				  BOARD_LM75AD_ADR, raw_temperature,
				  2, MAX_TIMEOUT));
		  /* Get two 8-bit values */
		  temperature = ((uint32_t)raw_temperature[0] << 8) |
				  raw_temperature[1];

		  /* Omit last 5-bits as they are not used*/
		  temperature = temperature >> 5;

		  if(temperature & (1 << 11))
		  {
			  temperature = (temperature << 7) - (temperature << 1) - (temperature);
		  }
		  else
		  {
			  /* Todo finish minus temperature*/
			  temperature = (temperature << 7) - (temperature << 1) - (temperature);
		  }
	  }

	  DEBUG_PRINT("TMPE %ld\n", temperature);
	  measurement->tmpe = temperature;

	  return temperature;
}
