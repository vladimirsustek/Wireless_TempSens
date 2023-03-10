/*
 * measurements.c
 *
 *  Created on: Mar 3, 2023
 *      Author: 42077
 */
#include <assert.h>

#include "measurements.h"

#include "adc.h"
#include "tim.h"

#include "stm32f1xx.h"

#define ADC_BUFFER_SIZE 16

#define ADC1_RANK_1 0
#define ADC1_RANK_2 1

const uint32_t VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;

/* Calculated from known VDAA comparing the VDAA calc result */
const uint32_t EST_VREFINT = 1158;

static volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t conv_done = 0;



#ifndef STM32F1
#error VALID ONLY FOR STM32F103C8T6
#endif

void measurements_open()
{
	/* Start the timer and start a TIM-triggered endless DMA conversion */
	assert(HAL_OK == HAL_TIM_Base_Start(&htim3));
	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE));
}

void measurements_close()
{
	/* Stop timer and the ADC */
	assert(HAL_OK == HAL_TIM_Base_Stop(&htim3));
	assert(HAL_OK == HAL_ADC_Stop(&hadc1));
}

bool measurement_get(uint32_t* ch, uint32_t* vreft_int)
{
	if(!conv_done)
	{
		return false;
	}
	else
	{
		HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
		conv_done = 0;
	}

	assert(ch != NULL);
	assert(vreft_int != NULL);

	uint32_t aux_ch = 0, aux_vrefint = 0;

	/* Accumulating */
	for(uint32_t idx = 0; idx <= ADC_BUFFER_SIZE; idx+=2)
	{
		aux_ch += adc_buffer[idx + ADC1_RANK_1];
		aux_vrefint += adc_buffer[idx + ADC1_RANK_2];
	}

	/* Averaging (when ADC_BUFFER_SIZE is 8)*/
	aux_ch = aux_ch >> 3;
	aux_vrefint = aux_vrefint >> 3;

	*ch = (aux_ch*VDDA_NOM)/ADC_MAX;
	*vreft_int = (ADC_MAX*EST_VREFINT)/(aux_vrefint);

	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	return true;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
			conv_done = 1;
	}
}





