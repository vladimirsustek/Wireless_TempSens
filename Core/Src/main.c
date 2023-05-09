/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <assert.h>
#include <stdbool.h>

#include "cli.h"
#include "measurement_defines.h"
#include "nrf24l01p_driver.h"
#include "int_measurements.h"
#include "ext_measurements.h"

#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef Measurement_t Payload_t;

typedef struct NVMdata
{
	char tx_adr[6];
	char rx_adr[6];
	uint8_t stop_period;
	const uint8_t padding[3];
}NVMdata_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PARAMETRIZATION_TIMEOUT	(uint32_t)(10*1000)
#define TMPE_ATTEMPT_TIMEOUT_MS (uint32_t)(100)
#define STOP_PERIOD_MAX			(uint8_t)(255)
#define COMMAND_LNG				(uint32_t)(16)
#define TX_ADR_OFFSET 			(uint32_t)(0)
#define RX_ADR_OFFSET 			(uint32_t)(6)
#define ADR_LNG					(uint32_t)(5)
#define DLMT_1_IDX				(uint32_t)(5)
#define DLMT_2_IDX				(uint32_t)(11)
#define UNITS_IDX 				(uint32_t)(14)
#define TENS_IDX 				(uint32_t)(13)
#define HUNDREDS_IDX 			(uint32_t)(12)
#define DELIMITER				(char)(' ')
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Board_3V3PWR_Up();
void Board_3V3PWR_Down();
void Board_SetNextAlarm(uint8_t seconds);
void NVM_Write(uint32_t*data, uint32_t length);
void NVM_Read(uint32_t*data, uint32_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/* Maximal length of the payload for the NRF24L01+ is PAYLOAD_MAX*/
	static_assert(sizeof(Payload_t) <= PAYLOAD_MAX);
	/* Padding 4 for better uint32_t 'pointing'*/
	static_assert(sizeof(NVMdata_t) % 4 == 0);
	Payload_t payload = {0};
	NVMdata_t data = {0};
	uint8_t raw_data[COMMAND_LNG] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* Get NVM data */
  NVM_Read((uint32_t*)&data, sizeof(NVMdata_t)/4);

  UART_PRINT("LISTENING FOR PARAMETRIZATION %ld ms\n", PARAMETRIZATION_TIMEOUT);
  UART_PRINT("FORMAT: %s %s %03d\n", data.tx_adr, data.rx_adr, data.stop_period);

  /* Listen for 10 seconds and try to parse the command. This Timeout also
   * prevents from MCU going into STOP MODE immediately after initialization*/
  if (HAL_OK == HAL_UART_Receive(&huart1, raw_data, COMMAND_LNG, PARAMETRIZATION_TIMEOUT) &&
	  (raw_data[DLMT_1_IDX] == DELIMITER &&
			  raw_data[DLMT_2_IDX] == DELIMITER  &&
			  isdigit(raw_data[UNITS_IDX]) &&
			  isdigit(raw_data[TENS_IDX]) &&
			  isdigit(raw_data[HUNDREDS_IDX])))
	  {
	      /* Copy the address */
		  memcpy(data.tx_adr, raw_data + TX_ADR_OFFSET, ADR_LNG);
		  memcpy(data.rx_adr, raw_data + RX_ADR_OFFSET, ADR_LNG);
		  /* Terminate strings */
		  data.tx_adr[ADR_LNG] = '\0';
		  data.rx_adr[ADR_LNG] = '\0';

		  /* Covert string to a numbers*/
		  uint32_t raw = (raw_data[HUNDREDS_IDX] - '0')*100 +
				  (raw_data[TENS_IDX] - '0')*10 +
				  (raw_data[UNITS_IDX] - '0');

		  /* Evaluate maximal period length */
		  data.stop_period = (raw <= STOP_PERIOD_MAX) ? raw : data.stop_period;

		  /* Notify about extracted parameters */
		  UART_PRINT("TX_ADR %s\n", data.tx_adr);
		  UART_PRINT("RX_ADR %s\n", data.rx_adr);
		  UART_PRINT("STOP_PERIOD %d\n", data.stop_period);

		  /* Save to the dedicated place */
		  NVM_Write((uint32_t*)&data, sizeof(NVMdata_t)/4);
	  }
  else
  {
	  UART_PRINT("NO CHANGES APPLIED\n");
  }

  UART_PRINT("NORMAL MODE STARTED\n");

  /* Suspend tick as it is natively source of 1ms the ISR
   * and enter the most low-power STOP mode */
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Return from the STOP mode needs re-start the all clocks*/
	  SystemClock_Config();

	  /* Power up all external peripherals (NTCs, LM75AD, NRF24L01+, OAs)*/
	  Board_3V3PWR_Up();

	  /* Resume the tick necessary for HAL polling functions
	   * tick comes from the ARM-core */
	  HAL_ResumeTick();

	  /* Start measurements (Automatic DMA, TIM3 triggered) */
	  IntMeas_Open();

	  /* Power up NRF24L01+ chip */

	  /* Wait some time to:
	   * 1) 103ms boot up time for the NRF24L01+
	   * 2) 1/50Hz*48 = 960ms ... see IntMeas implementation */
	  HAL_Delay(960);

	  /* Configure NRF as a transmitter */
	  NRF_configure(true);
	  NRF_setTX_ADDR((uint8_t*)data.tx_adr, ADR_LNG);
	  NRF_setRX_ADDR_P0((uint8_t*)data.rx_adr, ADR_LNG);

	  /* Get the external measurements results */
	  /* Set the tmpe as an error value and try to read the .tmpe
	   * maximally for the TMPE_ATTEMPT_TIMEOUT, if any valid readout
	   * is reached, the tmpe will be simply tmpe */
	  payload.tmpe = INT32_MIN;
	  uint32_t tick  = HAL_GetTick();
	  while((payload.tmpe == INT32_MIN) &&
			  (tick + TMPE_ATTEMPT_TIMEOUT_MS > HAL_GetTick()))
	  {
		  ExtMeas_LM75AD_GetTemp(&payload);
	  }

	  /* Get the internal measurements */
	  IntMeas_Get(&payload);

	  /* Prepare NRF24L01+ payload*/
	  NRF_setW_TX_PAYLOAD((uint8_t*)&payload, sizeof(Payload_t));

	  /* Send payload and check*/
	  NRF_CEactivate();
	  HAL_Delay(100);
	  NRF_CEdeactivate();
	  UART_PRINT("NRF STATUS: 0x%02x\r\n", NRF_getSTATUS());

	  /* Disable internal measurement */
	  IntMeas_Close();
	  /* Disable all external peripherals*/
	  Board_3V3PWR_Down();
	  /* Enable wake-up timer*/
	  Board_SetNextAlarm(data.stop_period);
	  /* Disable tick to prevent wakeup - tick comes from the ARM-core*/
	  HAL_SuspendTick();
	  /* Enter low-power mode*/
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Power up the 3V3 dependent external parts*/
void Board_3V3PWR_Up()
{
	HAL_GPIO_WritePin(PWR_3V3_GPIO_Port, PWR_3V3_Pin, GPIO_PIN_RESET);
}
/* Power down the 3V3 dependent external parts*/
void Board_3V3PWR_Down()
{
	HAL_GPIO_WritePin(PWR_3V3_GPIO_Port, PWR_3V3_Pin, GPIO_PIN_SET);
}

/* The empty RTC Callback, used to hit a breakpoint when the RTC interrupt fires */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	(void)(hrtc);
}

/* Set a new alarm, algorithm just checks the RTC's time
 * and using an appropriate seconds/minutes/hours math
 * specifies the next time-point to wakeup based on the argument */
void Board_SetNextAlarm(uint8_t seconds)
{
	  RTC_DateTypeDef date = {0};
	  RTC_TimeTypeDef time = {0};
	  RTC_AlarmTypeDef alarm = {0};

	  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	  time.Seconds += seconds;
	  if(time.Seconds>=60)
	  {
	  time.Minutes++;
	  time.Seconds= time.Seconds -60;
	  }
	  if(time.Minutes >= 60)
	  {
	  time.Hours++;
	  time.Minutes = time.Minutes - 60;
	  }
	  if(time.Hours > 23)
	  {
	  time.Hours = 0;
	  }
	  alarm.Alarm = RTC_ALARM_A;
	  alarm.AlarmTime.Hours = time.Hours;
	  alarm.AlarmTime.Minutes = time.Minutes;
	  alarm.AlarmTime.Seconds = time.Seconds;

	  HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
}


void NVM_Write(uint32_t*data, uint32_t length)
{
	/* RM0008 Reference manual (STM32F103C8T6 datasheet), page 55*/
	/* [Flash-Page 127 0x0801FC00 - 0x0801FFFF] 1KB/page */
	const uint32_t PAGE_127_ADR = 0x800FC00;
	const uint32_t MAX_LENGTH = 1024;
	uint32_t adr = PAGE_127_ADR;

	FLASH_EraseInitTypeDef erase = {0};
	uint32_t error = 0;

	assert(data != NULL && length <= MAX_LENGTH);

	assert(HAL_OK == HAL_FLASH_Unlock());
	assert(HAL_OK == HAL_FLASH_OB_Unlock());

	erase.NbPages = 1;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = PAGE_127_ADR;

	assert(HAL_OK == HAL_FLASHEx_Erase(&erase, &error));

	for(uint32_t idx = 0; idx < length; idx++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adr, (uint32_t)*data);
		(uint32_t*)data++;
		adr += 4;
	}

	assert(HAL_OK == HAL_FLASH_Lock());
	assert(HAL_OK == HAL_FLASH_OB_Lock());
}

void NVM_Read(uint32_t*data, uint32_t length)
{
	/* RM0008 Reference manual (STM32F103C8T6 datasheet), page 55*/
	/* [Flash-Page 127 0x0801FC00 - 0x0801FFFF] 1KB/page */
	const uint32_t PAGE_127_ADR = 0x800FC00;
	const uint32_t MAX_LENGTH = 1024;

	assert(data != NULL && length <= MAX_LENGTH);

	memcpy(data, (uint32_t*)PAGE_127_ADR, length*4);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
