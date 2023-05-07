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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef Measurement_t Payload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_MODE_LENGTH (uint8_t)(58)
#define TMPE_ATTEMPT_TIMEOUT (uint32_t)(100)
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
	static_assert(sizeof(Payload_t) <= PAYLOAD_MAX);
	Payload_t payload = {0};
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

  /* It seems like the STOP mode prevents from a
   * new connecting to the MCU thus this timeout
   * shall help when attempting to flash/debug */
  HAL_Delay(3000);
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

	  /* Get the external measurements results */
	  /* Set the tmpe as an error value and try to read the .tmpe
	   * maximally for the TMPE_ATTEMPT_TIMEOUT, if any valid readout
	   * is reached, the tmpe will be simply tmpe */
	  payload.tmpe = INT32_MIN;
	  uint32_t tick  = HAL_GetTick();
	  while((payload.tmpe == INT32_MIN) &&
			  (tick + TMPE_ATTEMPT_TIMEOUT > HAL_GetTick()))
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
	  DEBUG_PRINT("NRF STATUS: 0x%02x\r\n", NRF_getSTATUS());

	  /* Disable internal measurement */
	  IntMeas_Close();
	  /* Disable all external peripherals*/
	  Board_3V3PWR_Down();
	  /* Enable wake-up timer*/
	  Board_SetNextAlarm(STOP_MODE_LENGTH);
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
