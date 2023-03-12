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
#include "nrf24l01p_driver.h"
#include "measurements.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct payload
{
	uint32_t vdda;
	uint32_t temp_ntc;
	uint32_t temp_sens;
}payload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
	static_assert(sizeof(payload_t) <= PAYLOAD_MAX);
	payload_t payload = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_SuspendTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Suspend tick as it is natively source of 1ms the ISR*/
  HAL_SuspendTick();

  /* Start a timer waking up the MCU*/
  HAL_TIM_Base_Start_IT(&htim1);

  /* Go to the sleep mode*/
  HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	  /* Disable and reset timer which woke up the processor */
	  WakeUp_TIM_Stop();
	  /* Start ms timer for blocking delay purposes */
	  HAL_ResumeTick();
	  /* wait until ARM and external sensor boots up */
	  HAL_Delay(5);
	  /* Start measurements (Automatic DMA, TIM3 triggered) */
	  measurements_open();
#ifdef EXT_SENSOR
	  /* TODO: add external sensor*/
	  ext_measurements_open();
#endif
	  /* Power up NRF24L01+ chip */
	  NRF_powerUp();

	  /* Wait some time to:
	   * 1) 103ms boot up time for the NRF24L01+
	   * 2) 1/50Hz*16 = 320ms */
	  HAL_Delay(350);

	  /* Configure NRF as a transmitter */
	  NRF_configure(true);

	  /* Get measurement when ready*/

#ifdef EXT_SENSOR
	  /* TODO: add external sensor*/
	  ext_measurements_get();
#endif

	  printf("-------------------\r\n");
	  if(measurement_get(&payload.temp_ntc, &payload.vdda))
	  {
		  payload.temp_sens++;

		  DEBUG_PRINT("\r\nch0 %ld\n"
				  "vrefint %ld\n"
				  "cycle num %ld\r\n",
				  payload.temp_ntc,
				  payload.vdda,
				  payload.temp_sens);

		  /* Prepare payload for transmitting */
		  NRF_setW_TX_PAYLOAD((uint8_t*)&payload, sizeof(payload_t));
	  }
	  else
	  {
		  DEBUG_PRINT("Measurement not ready\r\n");

		  payload.temp_ntc = 0xDEADBEEF;
		  payload.vdda = 0xDEADBEEF;
		  payload.temp_sens = 0xDEADBEEF;

		  /* Prepare payload for transmitting - error DEADBEEF constant */
		  NRF_setW_TX_PAYLOAD((uint8_t*)&payload, sizeof(payload_t));

	  }

	  NRF_CEactivate();
	  HAL_Delay(100);
	  NRF_CEdeactivate();
	  printf("NRF STATUS: 0x%02x\r\n", NRF_getSTATUS());

	  /* Disable unneeded devices to reduce consumption */
	  measurements_close();
#ifdef EXT_SENSOR
	  /* TODO: add external sensor*/
	  ext_measurements_clos();
#endif
	  HAL_SuspendTick();
	  NRF_powerDown();

	  /* Enable wake-up timer and go to the sleep */
	  WakeUp_TIM_Start();
	  HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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
