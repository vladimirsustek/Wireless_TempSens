#include "i2c.h"
#include "ext_measurements.h"
#include "cli.h"

uint16_t ExtMeas_LM75AD_GetTemp(Measurement_t* measurement)
{

/* Function based on the datasheet Docs/LM75A.pdf.
 * I2C-Reading of 2-bytes reads temperature in raw 12-bit format
 *
 * For temperature above 0°C (MSB = 0)
 *
 * temperature[°C] = raw * 0.125
 *
 * For temperature below 0°C (MSB = 1)
 *
 * temperature[°C) = -raw * 0.125
 *
 * */
const uint32_t MAX_TIMEOUT = 10;
const uint32_t MAX_ATTEMPT = 3;
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
				  2, MAX_TIMEOUT))

		  {
			  /* Get two 8-bit values - temperature register*/
			  temperature = ((uint32_t)raw_temperature[0] << 8) |
					  raw_temperature[1];

			  /* Omit last 5-bits as they are not used*/
			  temperature = temperature >> 5;

			  /* MSB signalizes sign */
			  if(temperature & (1 << 10))
			  {
				  /* e.g.          0b11100111000 = -25000 m°C
				   * 12-bit format 0b10000000000 = -1024
				   * remove MSB => 0b01100111000 =  824
				   * -1024 + 824 = -200
				   * -200*125 = -25000m°C
				   * -(200 << 7) -(200 << 1) -200 = -25000m°C
				   */

				  temperature = -1024 + ((~(1 << 10)) &temperature);
				  /* Optimized way of 125x multiplication => temperature in m°C*/
				  temperature = (temperature << 7) - (temperature << 1) - (temperature);
			  }
			  else
			  {
				  /* 000 1100 1000 = 25000 m°C*/
				  /* Optimized way of 125x multiplication => temperature in m°C*/
				  temperature = (temperature << 7) - (temperature << 1) - (temperature);
			  }
		  }
		  else
		  {
			  UART_PRINT("LM75AD read temperature error\n");
		  }

	  }
	  else
	  {
		  UART_PRINT("LM75AD not ready\n");
	  }

	  UART_PRINT("TMPE %ld\n", temperature);
	  measurement->tmpe = temperature;

	  return temperature;
}
